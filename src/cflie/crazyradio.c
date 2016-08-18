// Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cflie/crazyradio.h>
#include <stdlib.h>
#include <stdio.h>

struct crazyradio * crazyradio_alloc(const char *strRadioIdentifier) {

  struct crazyradio *radio = (struct crazyradio*)malloc(sizeof(struct crazyradio));

  if(radio) {
    crazyradio_init(radio, strRadioIdentifier);
  }

  return radio;
}

void crazyradio_init(struct crazyradio *radio, const char *strRadioIdentifier) {
  radio->m_strRadioIdentifier = strdup(strRadioIdentifier);
  radio->m_enumPower = P_M18DBM;

  radio->m_ctxContext = NULL;
  radio->m_hndlDevice = NULL;

  radio->m_bAckReceived = false;
  radio->m_lstLoggingPacketsCount = 0;

  /*int nReturn = */libusb_init(&radio->m_ctxContext);

  // Do error checking here.
}

void crazyradio_destroy(struct crazyradio *radio) {
  crazyradio_closeDevice(radio);
  free((void*)radio->m_strRadioIdentifier);

  // TODO(winkler): Free all remaining packets in radio->m_lstLoggingPackets.

  if(radio->m_ctxContext) {
    libusb_exit(radio->m_ctxContext);
  }
}

void crazyradio_free(struct crazyradio *radio) {
  crazyradio_destroy(radio);
  free(radio);
}

void crazyradio_closeDevice(struct crazyradio *radio) {
  if(radio->m_hndlDevice) {
    libusb_close(radio->m_hndlDevice);
    libusb_unref_device(radio->m_devDevice);

    radio->m_hndlDevice = NULL;
    radio->m_devDevice = NULL;
  }
}

libusb_device ** crazyradio_listDevices(struct crazyradio *radio, int nVendorID, int nProductID) {
#define MAX_NUM_DEVICES 20
  libusb_device **lstDevices = (libusb_device **)calloc(sizeof(libusb_device*), (MAX_NUM_DEVICES + 1));
  int lstDevicesIndex = 0;
  ssize_t szCount;
  libusb_device **ptDevices;

  szCount = libusb_get_device_list(radio->m_ctxContext, &ptDevices);
  for(unsigned int unI = 0; unI < szCount; unI++) {
    libusb_device *devCurrent = ptDevices[unI];
    struct libusb_device_descriptor ddDescriptor;

    libusb_get_device_descriptor(devCurrent, &ddDescriptor);

    if(ddDescriptor.idVendor == nVendorID && ddDescriptor.idProduct == nProductID) {
      libusb_ref_device(devCurrent);
      if(lstDevicesIndex == MAX_NUM_DEVICES) {
        fprintf(stderr, "Filled lstDevicesIndex\n");
        exit(EXIT_FAILURE);
      }
      lstDevices[lstDevicesIndex++] = devCurrent;
    }
  }

  if(szCount > 0) {
    libusb_free_device_list(ptDevices, 1);
  }

  return lstDevices;
}

bool crazyradio_openUSBDongle(struct crazyradio *radio) {
  crazyradio_closeDevice(radio);
  libusb_device** lstDevices = crazyradio_listDevices(radio, 0x1915, 0x7777);

  if(lstDevices[0] != NULL) {
    // For now, just take the first device. Give it a second to
    // initialize the system permissions.
    sleep(1.0);

    libusb_device *devFirst = lstDevices[0];
    int lstDevicesIndex;
    int nError = libusb_open(devFirst, &radio->m_hndlDevice);

    if(nError == 0) {
      // Opening device OK. Don't free the first device just yet.
      radio->m_devDevice = devFirst;
    }

    for(lstDevicesIndex = 1;
        lstDevices[lstDevicesIndex] != NULL;
        lstDevicesIndex++) {
      libusb_unref_device(lstDevices[lstDevicesIndex]);
    }

    return !nError;
  }

  return false;
}

bool crazyradio_startRadio(struct crazyradio *radio) {
  if(crazyradio_openUSBDongle(radio)) {
    int nDongleNBR;
    int nRadioChannel;
    int nDataRate;
    char cDataRateType;

    if(sscanf(radio->m_strRadioIdentifier, "radio://%d/%d/%d%c",
              &nDongleNBR, &nRadioChannel, &nDataRate,
              &cDataRateType) != EOF) {

      char strDataRate[256];
      sprintf(strDataRate, "%d%c", nDataRate, cDataRateType);

      // Read device version
      struct libusb_device_descriptor ddDescriptor;
      libusb_get_device_descriptor(radio->m_devDevice, &ddDescriptor);
      char tmpStr[256];
      sprintf(tmpStr, "%d.%d", ddDescriptor.bcdDevice >> 8, ddDescriptor.bcdDevice & 0x0ff);
      sscanf(tmpStr, "%f", &radio->m_fDeviceVersion);

      if(radio->m_fDeviceVersion < 0.3) {
        return false;
      }

      // Set active configuration to 1
      libusb_set_configuration(radio->m_hndlDevice, 1);

      // Claim interface
      if(crazyradio_claimInterface(radio, 0)) {
        // Set power-up settings for dongle (>= v0.4)
        crazyradio_setDataRate(radio, "2M");
        crazyradio_setChannel(radio, 2);

        if(radio->m_fDeviceVersion >= 0.4) {
          crazyradio_setContCarrier(radio, false);
          char cAddress[5];
          cAddress[0] = 0xe7;
          cAddress[1] = 0xe7;
          cAddress[2] = 0xe7;
          cAddress[3] = 0xe7;
          cAddress[4] = 0xe7;
          crazyradio_setAddress(radio, cAddress);
          crazyradio_setPower(radio, P_0DBM);
          crazyradio_setARC(radio, 3);
          crazyradio_setARDBytes(radio, 32);
        }

        // Initialize device
        if(radio->m_fDeviceVersion >= 0.4) {
          crazyradio_setARC(radio, 10);
        }

        crazyradio_setChannel(radio, nRadioChannel);
        crazyradio_setDataRate(radio, strDataRate);

        return true;
      }
    }
  }

  return false;
}

struct crtppacket *crazyradio_writeData(struct crazyradio *radio, void *vdData, int nLength) {
  struct crtppacket *crtpPacket = NULL;

  int nActuallyWritten;
  int nReturn = libusb_bulk_transfer(radio->m_hndlDevice, (0x01 | LIBUSB_ENDPOINT_OUT), (unsigned char*)vdData, nLength, &nActuallyWritten, 1000);

  if(nReturn == 0 && nActuallyWritten == nLength) {
    crtpPacket = crazyradio_readACK(radio);
  }

  return crtpPacket;
}

bool crazyradio_readData(struct crazyradio *radio, void *vdData, int *nMaxLength) {
  int nActuallyRead;
  int nReturn = libusb_bulk_transfer(radio->m_hndlDevice, (0x81 | LIBUSB_ENDPOINT_IN),
                                     (unsigned char*)vdData, *nMaxLength,
                                     &nActuallyRead, 50);

  if(nReturn == 0) {
    *nMaxLength = nActuallyRead;

    return true;
  } else {
    switch(nReturn) {
    case LIBUSB_ERROR_TIMEOUT:
      printf("USB timeout\n");
      break;

    default:
      break;
    }
  }

  return false;
}

bool crazyradio_writeControl(struct crazyradio *radio, void *vdData, int nLength, uint8_t u8Request, uint16_t u16Value, uint16_t u16Index) {
  int nTimeout = 1000;

  /*int nReturn = */libusb_control_transfer(radio->m_hndlDevice, LIBUSB_REQUEST_TYPE_VENDOR, u8Request, u16Value, u16Index, (unsigned char*)vdData, nLength, nTimeout);

  // if(nReturn == 0) {
  //   return true;
  // }

  // Hack.
  return true;
}

void crazyradio_setARC(struct crazyradio *radio, int nARC) {
  radio->m_nARC = nARC;
  crazyradio_writeControl(radio, NULL, 0, 0x06, nARC, 0);
}

void crazyradio_setChannel(struct crazyradio *radio, int nChannel) {
  radio->m_nChannel = nChannel;
  crazyradio_writeControl(radio, NULL, 0, 0x01, nChannel, 0);
}

void crazyradio_setDataRate(struct crazyradio *radio, const char *strDataRate) {
  radio->m_strDataRate = strDataRate;
  int nDataRate = -1;

  if(strcmp(radio->m_strDataRate, "250K") == 0) {
    nDataRate = 0;
  } else if(strcmp(radio->m_strDataRate, "1M") == 0) {
    nDataRate = 1;
  } else if(strcmp(radio->m_strDataRate, "2M") == 0) {
    nDataRate = 2;
  }

  crazyradio_writeControl(radio, NULL, 0, 0x03, nDataRate, 0);
}

void crazyradio_setARDTime(struct crazyradio *radio, int nARDTime) { // in uSec
  radio->m_nARDTime = nARDTime;

  int nT = (int)((nARDTime / 250) - 1);
  if(nT < 0) {
    nT = 0;
  } else if(nT > 0xf) {
    nT = 0xf;
  }

  crazyradio_writeControl(radio, NULL, 0, 0x05, nT, 0);
}

void crazyradio_setARDBytes(struct crazyradio *radio, int nARDBytes) {
  radio->m_nARDBytes = nARDBytes;

  crazyradio_writeControl(radio, NULL, 0, 0x05, 0x80 | nARDBytes, 0);
}

enum Power crazyradio_power(struct crazyradio *radio) {
  return radio->m_enumPower;
}

void crazyradio_setPower(struct crazyradio *radio, enum Power enumPower) {
  radio->m_enumPower = enumPower;

  crazyradio_writeControl(radio, NULL, 0, 0x04, enumPower, 0);
}

void crazyradio_setAddress(struct crazyradio *radio, char *cAddress) {
  radio->m_cAddress = cAddress;

  crazyradio_writeControl(radio, cAddress, 5, 0x02, 0, 0);
}

void crazyradio_setContCarrier(struct crazyradio *radio, bool bContCarrier) {
  radio->m_bContCarrier = bContCarrier;

  crazyradio_writeControl(radio, NULL, 0, 0x20, (bContCarrier ? 1 : 0), 0);
}

bool crazyradio_claimInterface(struct crazyradio *radio, int nInterface) {
  return libusb_claim_interface(radio->m_hndlDevice, nInterface) == 0;
}

struct crtppacket *crazyradio_sendPacket(struct crazyradio *radio,
                                         struct crtppacket *crtpSend,
                                         bool bDeleteAfterwards) {
  struct crtppacket *crtpPacket = NULL;

  char *cSendable = crtppacket_sendableData(crtpSend);
  crtpPacket = crazyradio_writeData(radio, cSendable, crtppacket_sendableDataLength(crtpSend));

  free(cSendable);

  if(crtpPacket) {
    char *cData = crtppacket_data(crtpPacket);
    int nLength = crtppacket_dataLength(crtpPacket);

    if(nLength > 0) {
      short sPort = (cData[0] & 0xf0) >> 4;
      crtppacket_setPort(crtpPacket, sPort);
      short sChannel = cData[0] & 0b00000011;
      crtppacket_setChannel(crtpPacket, sChannel);

      switch(sPort) {
      case 0: { // Console
        char cText[nLength];
        memcpy(cText, &cData[1], nLength - 1);
        cText[nLength - 1] = '\0';

      } break;

      case 5: { // Logging
        if(crtppacket_channel(crtpPacket) == 2) {
          struct crtppacket *crtpLog = crtppacket_alloc_with_data(cData, nLength, crtppacket_channel(crtpPacket));
          crtppacket_setChannel(crtpLog, crtppacket_channel(crtpPacket));
          crtppacket_setPort(crtpLog, crtppacket_port(crtpPacket));

          if(radio->m_lstLoggingPacketsCount == MAX_LIST_LOGGING_PACKETS) {
            fprintf(stderr, "radio->m_lstLoggingPacketsCount reached %d\n",
                    MAX_LIST_LOGGING_PACKETS);
            exit(EXIT_FAILURE);
          }
          radio->m_lstLoggingPackets[radio->m_lstLoggingPacketsCount++] = crtpLog;
        }
      } break;
      }
    }
  }

  if(bDeleteAfterwards) {
    crtppacket_free(crtpSend);
  }

  return crtpPacket;
}

struct crtppacket *crazyradio_readACK(struct crazyradio *radio) {
  struct crtppacket *crtpPacket = NULL;

  int nBufferSize = 64;
  char cBuffer[nBufferSize];
  int nBytesRead = nBufferSize;

  if(crazyradio_readData(radio, cBuffer, &nBytesRead)) {
    if(nBytesRead > 0) {
      // Analyse status byte
      radio->m_bAckReceived = true;//cBuffer[0] & 0x1;
      //bool bPowerDetector = cBuffer[0] & 0x2;
      //int nRetransmissions = cBuffer[0] & 0xf0;

      // TODO(winkler): Do internal stuff with the data received here
      // (store current link quality, etc.). For now, ignore it.

      crtpPacket = crtppacket_alloc(0);

      if(nBytesRead > 1) {
        crtppacket_setData(crtpPacket, &cBuffer[1], nBytesRead);
      }
    }
    else {
      radio->m_bAckReceived = false;
    }
  }

  return crtpPacket;
}

bool crazyradio_ackReceived(struct crazyradio *radio) {
  return radio->m_bAckReceived;
}

bool crazyradio_usbOK(struct crazyradio *radio) {
  struct libusb_device_descriptor ddDescriptor;
  return (libusb_get_device_descriptor(radio->m_devDevice,
                                       &ddDescriptor) == 0);
}

struct crtppacket *crazyradio_waitForPacket(struct crazyradio *radio) {
  bool bGoon = true;
  struct crtppacket *crtpReceived = NULL;
  struct crtppacket *crtpDummy = crtppacket_alloc(0);
  crtppacket_setIsPingPacket(crtpDummy, true);

  while(bGoon) {
    crtpReceived = crazyradio_sendPacket(radio, crtpDummy, false);
    bGoon = (crtpReceived == NULL);
  }

  crtppacket_free(crtpDummy);
  return crtpReceived;
}

struct crtppacket *crazyradio_sendAndReceive(struct crazyradio *radio,
                                       struct crtppacket *crtpSend,
                                       bool bDeleteAfterwards) {
  return crazyradio_sendAndReceive2(radio, crtpSend,
                                    crtppacket_port(crtpSend),
                                    crtppacket_channel(crtpSend),
                                    bDeleteAfterwards,
                                    10, 100);
}

struct crtppacket *
crazyradio_sendAndReceive2(struct crazyradio *radio, struct crtppacket *crtpSend,
                           int nPort, int nChannel, bool bDeleteAfterwards,
                           int nRetries, int nMicrosecondsWait) {
  bool bGoon = true;
  int nResendCounter = 0;
  struct crtppacket *crtpReturnvalue = NULL;
  struct crtppacket *crtpReceived = NULL;

  while(bGoon) {
    if(nResendCounter == 0) {
      crtpReceived = crazyradio_sendPacket(radio, crtpSend, false);
      nResendCounter = nRetries;
    } else {
      nResendCounter--;
    }

    if(crtpReceived) {
      if(crtppacket_port(crtpReceived) == nPort &&
         crtppacket_channel(crtpReceived) == nChannel) {
        crtpReturnvalue = crtpReceived;
        bGoon = false;
      }
    }

    if(bGoon) {
      if(crtpReceived) {
        crtppacket_free(crtpReceived);
      }

      usleep(nMicrosecondsWait);
      crtpReceived = crazyradio_waitForPacket(radio);
    }
  }

  if(bDeleteAfterwards) {
    crtppacket_free(crtpSend);
  }

  return crtpReturnvalue;
}

struct crtppacket** crazyradio_popLoggingPackets(struct crazyradio *radio, int *count) {
  struct crtppacket** lstPackets =
    (struct crtppacket**)malloc(sizeof(*lstPackets) * radio->m_lstLoggingPacketsCount);
  if(lstPackets == NULL) {
    fprintf(stderr, "Malloc in crazyradio_popLoggingPackets failed\n");
    exit(EXIT_FAILURE);
  }
  memcpy(lstPackets, radio->m_lstLoggingPackets,
         sizeof(*lstPackets) * radio->m_lstLoggingPacketsCount);
  *count = radio->m_lstLoggingPacketsCount;
  radio->m_lstLoggingPacketsCount = 0;

  return lstPackets;
}

bool crazyradio_sendDummyPacket(struct crazyradio *radio) {
  struct crtppacket *crtpReceived = NULL;
  struct crtppacket *crtpDummy = crtppacket_alloc(0);
  crtppacket_setIsPingPacket(crtpDummy, true);

  crtpReceived = crazyradio_sendPacket(radio, crtpDummy, true);
  if(crtpReceived) {
    crtppacket_free(crtpReceived);
    return true;
  }

  return false;
}
