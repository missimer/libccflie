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


#include <cflie/CTOC.h>

struct toc * toc_alloc(struct crazyradio *crRadio, int nPort) {
  struct toc * toc = (struct toc *)malloc(sizeof(struct toc));
  if(toc) {
    toc_init(toc, crRadio, nPort);
  }
  return toc;
}

void toc_init(struct toc *toc, struct crazyradio *crRadio, int nPort) {
  toc->m_crRadio = crRadio;
  toc->m_nPort = nPort;
  toc->m_nItemCount = 0;
  toc->m_lstTOCElementsCount = 0;
  toc->m_lstLoggingBlocksCount = 0;
}

void toc_destroy(struct toc *toc) {
  for(int i = 0; i < toc->m_lstTOCElementsCount; i++) {
    free((void *)toc->m_lstTOCElements[i].strGroup);
    free((void *)toc->m_lstTOCElements[i].strIdentifier);
  }
  for(int i = 0; i < toc->m_lstLoggingBlocksCount; i++) {
    free((void*)toc->m_lstLoggingBlocks[i].strName);
  }
}

void toc_free(struct toc *toc) {
  toc_destroy(toc);
  free(toc);
}

bool toc_sendTOCPointerReset(struct toc *toc) {
  struct crtppacket* crtpPacket = crtppacket_alloc(0x00, 0);
  crtppacket_setPort(crtpPacket, toc->m_nPort);
  struct crtppacket* crtpReceived = crazyradio_sendPacket(toc->m_crRadio, crtpPacket, true);

  if(crtpReceived) {
    crtppacket_free(crtpReceived);
    return true;
  }

  return false;
}

bool toc_requestMetaData(struct toc *toc) {
  bool bReturnvalue = false;

  struct crtppacket* crtpPacket = crtppacket_alloc(0x01, 0);
  crtppacket_setPort(crtpPacket, toc->m_nPort);
  struct crtppacket* crtpReceived = crazyradio_sendAndReceive(toc->m_crRadio, crtpPacket);

  if(crtppacket_data(crtpReceived)[1] == 0x01) {
    toc->m_nItemCount = crtppacket_data(crtpReceived)[2];
    bReturnvalue = true;
  }

  crtppacket_free(crtpReceived);
  return bReturnvalue;
}

bool toc_requestInitialItem(struct toc *toc) {
  return toc_requestItem(toc, 0, true);
}

bool toc_requestItem(struct toc *toc, int nID) {
  return toc_requestItem(toc, nID, false);
}

bool toc_requestItem(struct toc *toc, int nID, bool bInitial) {
  bool bReturnvalue = false;

  char cRequest[2];
  cRequest[0] = 0x0;
  cRequest[1] = nID;

  struct crtppacket* crtpPacket = crtppacket_alloc(cRequest,
                                            (bInitial ? 1 : 2),
                                            0);
  crtppacket_setPort(crtpPacket, toc->m_nPort);
  struct crtppacket* crtpReceived = crazyradio_sendAndReceive(toc->m_crRadio, crtpPacket);

  bReturnvalue = toc_processItem(toc, crtpReceived);

  crtppacket_free(crtpReceived);
  return bReturnvalue;
}

bool toc_requestItems(struct toc *toc) {
  for(int nI = 0; nI < toc->m_nItemCount; nI++) {
    toc_requestItem(toc, nI);
  }

  return true;
}

bool toc_processItem(struct toc *toc, struct crtppacket* crtpItem) {
  if(crtppacket_port(crtpItem) == toc->m_nPort) {
    if(crtppacket_channel(crtpItem) == 0) {
      char* cData = crtppacket_data(crtpItem);
      int nLength = crtppacket_dataLength(crtpItem);

      if(cData[1] == 0x0) { // Command identification ok?
        int nID = cData[2];
        int nType = cData[3];

        int strGroupIndex = 0;
        char strGroup[256];
        int nI;
        memset(strGroup, 0, sizeof(strGroup));
        for(nI = 4; cData[nI] != '\0'; nI++) {
          if(strGroupIndex == (sizeof(strGroup) - 1)) {
            fprintf(stderr, "strGroup too small\n");
            exit(EXIT_FAILURE);
          }
          strGroup[strGroupIndex++] = cData[nI];
        }

        nI++;
        int strIdentifierIndex = 0;
        char strIdentifier[256];
        memset(strIdentifier, 0, sizeof(strIdentifier));
        for(; cData[nI] != '\0'; nI++) {
          if(strIdentifierIndex == (sizeof(strIdentifier) - 1)) {
            fprintf(stderr, "strIndentifier too small\n");
            exit(EXIT_FAILURE);
          }
          strIdentifier[strIdentifierIndex++] = cData[nI];
        }

        struct TOCElement teNew;
        teNew.strIdentifier = strdup(strIdentifier);
        teNew.strGroup = strdup(strGroup);
        teNew.nID = nID;
        teNew.nType = nType;
        teNew.bIsLogging = false;
        teNew.dValue = 0;

        if(toc->m_lstTOCElementsCount == MAX_LST_TOC_ELEMENTS) {
          fprintf(stderr, "toc->m_lstTOCElementsCount == MAX_LST_TOC_ELEMENTS\n");
          exit(EXIT_FAILURE);
        }
        toc->m_lstTOCElements[toc->m_lstTOCElementsCount++] = teNew;

        // NOTE(winkler): For debug purposes only.
        //std::cout << strGroup << "." << strIdentifier << std::endl;

        return true;
      }
    }
  }

  return false;
}

struct TOCElement toc_elementForName(struct toc *toc, const char *strName, bool *bFound) {
  for(int i = 0; i < toc->m_lstTOCElementsCount; i++) {
    struct TOCElement teCurrent = toc->m_lstTOCElements[i];

    char *strTempFullname = (char *)malloc(strlen(teCurrent.strGroup) +
                                           strlen(teCurrent.strIdentifier) + 2);
    if(strTempFullname == NULL) {
      fprintf(stderr, "malloc in CTOC::elementForName failed\n");
      exit(EXIT_FAILURE);
    }
    strTempFullname[0] = '\0';
    strcat(strTempFullname, teCurrent.strGroup);
    strcat(strTempFullname, ".");
    strcat(strTempFullname, teCurrent.strIdentifier);
    if(strcmp(strName, strTempFullname) == 0) {
      *bFound = true;
      free((void*)strTempFullname);
      return teCurrent;
    }
    free((void*)strTempFullname);
  }

  *bFound = false;
  struct TOCElement teEmpty;

  return teEmpty;
}

struct TOCElement toc_elementForID(struct toc *toc, int nID, bool *bFound) {
  for(int i = 0; i < toc->m_lstTOCElementsCount; i++) {
    struct TOCElement teCurrent = toc->m_lstTOCElements[i];

    if(nID == teCurrent.nID) {
      *bFound = true;
      return teCurrent;
    }
  }

  *bFound = false;
  struct TOCElement teEmpty;

  return teEmpty;
}

int toc_idForName(struct toc *toc, char *strName) {
  bool bFound;

  struct TOCElement teResult = toc_elementForName(toc, strName, &bFound);

  if(bFound) {
    return teResult.nID;
  }

  return -1;
}

int toc_typeForName(struct toc *toc, char *strName) {
  bool bFound;

  struct TOCElement teResult = toc_elementForName(toc, strName, &bFound);

  if(bFound) {
    return teResult.nType;
  }

  return -1;
}

bool toc_startLogging(struct toc *toc, const char *strName, const char *strBlockName) {
  bool bFound;
  struct LoggingBlock lbCurrent = toc_loggingBlockForName(toc, strBlockName, &bFound);

  if(bFound) {
    struct TOCElement teCurrent = toc_elementForName(toc, strName, &bFound);
    if(bFound) {
      char cPayload[5] = {0x01, lbCurrent.nID, teCurrent.nType, teCurrent.nID};
      struct crtppacket* crtpLogVariable = crtppacket_alloc(cPayload, 4, 1);
      crtppacket_setPort(crtpLogVariable, toc->m_nPort);
      crtppacket_setChannel(crtpLogVariable, 1);
      struct crtppacket* crtpReceived = crazyradio_sendAndReceive(toc->m_crRadio, crtpLogVariable, true);

      char* cData = crtppacket_data(crtpReceived);
      bool bCreateOK = false;
      if(cData[1] == 0x01 &&
         cData[2] == lbCurrent.nID &&
         cData[3] == 0x00) {
        bCreateOK = true;
      } else {
        printf("%c\n", cData[3]);
      }

      if(crtpReceived) {
        crtppacket_free(crtpReceived);
      }

      if(bCreateOK) {
        toc_addElementToBlock(toc, lbCurrent.nID, teCurrent.nID);

        return true;
      }
    }
  }

  return false;
}

bool toc_addElementToBlock(struct toc *toc, int nBlockID, int nElementID) {
  for(int i = 0; i < toc->m_lstLoggingBlocksCount; i++) {
    struct LoggingBlock lbCurrent = toc->m_lstLoggingBlocks[i];

    if(lbCurrent.nID == nBlockID) {
      if(toc->m_lstLoggingBlocks[i].lstElementIDsCount == MAX_LST_ELEMENT_IDS) {
        fprintf(stderr, "(*itBlock).lstElementIDsCount reached MAX_LST_ELEMENT_IDS");
        exit(EXIT_FAILURE);
      }
      toc->m_lstLoggingBlocks[i].lstElementIDs[toc->m_lstLoggingBlocks[i].lstElementIDsCount++] = nElementID;

      return true;
    }
  }

  return false;
}

bool toc_stopLogging(struct toc *toc, const char *strName) {
  // TODO: Implement me.
}

bool toc_isLogging(struct toc *toc, const char *strName) {
  // TODO: Implement me.
}

double toc_doubleValue(struct toc *toc, const char *strName) {
  bool bFound;

  struct TOCElement teResult = toc_elementForName(toc, strName, &bFound);

  if(bFound) {
    return teResult.dValue;
  }

  return 0;
}

struct LoggingBlock toc_loggingBlockForName(struct toc *toc, const char *strName, bool *bFound) {
  for(int i = 0; i < toc->m_lstLoggingBlocksCount; i++) {
    struct LoggingBlock lbCurrent = toc->m_lstLoggingBlocks[i];

    if(strcmp(strName, lbCurrent.strName) == 0) {
      *bFound = true;
      return lbCurrent;
    }
  }

  *bFound = false;
  struct LoggingBlock lbEmpty;
  lbEmpty.lstElementIDsCount = 0;

  return lbEmpty;
}

struct LoggingBlock toc_loggingBlockForID(struct toc *toc, int nID, bool *bFound) {
  for(int i = 0; i < toc->m_lstLoggingBlocksCount; i++) {
    struct LoggingBlock lbCurrent = toc->m_lstLoggingBlocks[i];

    if(nID == lbCurrent.nID) {
      *bFound = true;
      return lbCurrent;
    }
  }

  *bFound = false;
  struct LoggingBlock lbEmpty;
  lbEmpty.lstElementIDsCount = 0;

  return lbEmpty;
}

bool toc_registerLoggingBlock(struct toc *toc, const char *strName, double dFrequency) {
  int nID = 0;
  bool bFound;

  if(dFrequency > 0) { // Only do it if a valid frequency > 0 is given
    toc_loggingBlockForName(toc, strName, &bFound);
    if(bFound) {
      toc_unregisterLoggingBlock(toc, strName);
    }

    do {
      toc_loggingBlockForID(toc, nID, &bFound);

      if(bFound) {
        nID++;
      }
    } while(bFound);

    toc_unregisterLoggingBlockID(toc, nID);

    double d10thOfMS = (1 / dFrequency) * 1000 * 10;
    char cPayload[4] = {0x00, nID, d10thOfMS};

    struct crtppacket* crtpRegisterBlock = crtppacket_alloc(cPayload, 3, 1);
    crtppacket_setPort(crtpRegisterBlock, toc->m_nPort);
    crtppacket_setChannel(crtpRegisterBlock, 1);

    struct crtppacket* crtpReceived = crazyradio_sendAndReceive(toc->m_crRadio, crtpRegisterBlock, true);

    char* cData = crtppacket_data(crtpReceived);
    bool bCreateOK = false;
    if(cData[1] == 0x00 &&
       cData[2] == nID &&
       cData[3] == 0x00) {
      bCreateOK = true;
      printf("Registered logging block `%s'\n", strName);
    }

    if(crtpReceived) {
      crtppacket_free(crtpReceived);
    }

    if(bCreateOK) {
      struct LoggingBlock lbNew;
      lbNew.lstElementIDsCount = 0;
      lbNew.strName = strdup(strName);
      lbNew.nID = nID;
      lbNew.dFrequency = dFrequency;

      if(toc->m_lstLoggingBlocksCount == MAX_LST_LOGGING_BLOCKS) {
        fprintf(stderr, "toc->m_lstLoggingBlocksCont == MAX_LST_LOGGING_BLOCKS");
        exit(EXIT_FAILURE);
      }
      toc->m_lstLoggingBlocks[toc->m_lstLoggingBlocksCount++] = lbNew;

      return toc_enableLogging(toc, strName);
    }
  }

  return false;
}

bool toc_enableLogging(struct toc *toc, const char *strBlockName) {
  bool bFound;

  struct LoggingBlock lbCurrent = toc_loggingBlockForName(toc, strBlockName, &bFound);
  if(bFound) {
    double d10thOfMS = (1 / lbCurrent.dFrequency) * 1000 * 10;
    char cPayload[3] = {0x03, lbCurrent.nID, d10thOfMS};

    struct crtppacket* crtpEnable = crtppacket_alloc(cPayload, 3, 1);
    crtppacket_setPort(crtpEnable, toc->m_nPort);
    crtppacket_setChannel(crtpEnable, 1);

    struct crtppacket* crtpReceived = crazyradio_sendAndReceive(toc->m_crRadio, crtpEnable);
    crtppacket_free(crtpReceived);

    return true;
  }

  return false;
}

bool toc_unregisterLoggingBlock(struct toc *toc, const char *strName) {
  bool bFound;

  struct LoggingBlock lbCurrent = toc_loggingBlockForName(toc, strName, &bFound);
  if(bFound) {
    return toc_unregisterLoggingBlockID(toc, lbCurrent.nID);
  }

  return false;
}

bool toc_unregisterLoggingBlockID(struct toc *toc, int nID) {
  char cPayload[2] = {0x02, nID};

  struct crtppacket* crtpUnregisterBlock = crtppacket_alloc(cPayload, 2, 1);
  crtppacket_setPort(crtpUnregisterBlock, toc->m_nPort);
  crtppacket_setChannel(crtpUnregisterBlock, 1);

  struct crtppacket* crtpReceived = crazyradio_sendAndReceive(toc->m_crRadio, crtpUnregisterBlock, true);

  if(crtpReceived) {
    crtppacket_free(crtpReceived);
    return true;
  }

  return false;
}

void toc_processPackets(struct toc *toc, struct crtppacket** lstPackets, int count) {
  int i;
  if(count > 0) {
    for(i = 0; i < count; i++) {
      struct crtppacket* crtpPacket = lstPackets[i];

      char* cData = crtppacket_data(crtpPacket);
      float fValue;
      memcpy(&fValue, &cData[5], 4);

      char* cLogdata = &cData[5];
      int nOffset = 0;
      int nIndex = 0;
      int nAvailableLogBytes = crtppacket_dataLength(crtpPacket) - 5;

      int nBlockID = cData[1];
      bool bFound;
      struct LoggingBlock lbCurrent = toc_loggingBlockForID(toc, nBlockID, &bFound);

      if(bFound) {
        while(nIndex < lbCurrent.lstElementIDsCount) {
          int nElementID = toc_elementIDinBlock(toc, nBlockID, nIndex);
          bool bFound;
          struct TOCElement teCurrent = toc_elementForID(toc, nElementID, &bFound);

          if(bFound) {
            int nByteLength = 0;

            // NOTE(winkler): We just copy over the incoming bytes in
            // their according data structures and afterwards assign
            // the value to fValue. This way, we let the compiler to
            // the magic of conversion.
            float fValue = 0;

            switch(teCurrent.nType) {
            case 1: { // UINT8
              nByteLength = 1;
              uint8_t uint8Value;
              memcpy(&uint8Value, &cLogdata[nOffset], nByteLength);
              fValue = uint8Value;
            } break;

            case 2: { // UINT16
              nByteLength = 2;
              uint16_t uint16Value;
              memcpy(&uint16Value, &cLogdata[nOffset], nByteLength);
              fValue = uint16Value;
            } break;

            case 3: { // UINT32
              nByteLength = 4;
              uint32_t uint32Value;
              memcpy(&uint32Value, &cLogdata[nOffset], nByteLength);
              fValue = uint32Value;
            } break;

            case 4: { // INT8
              nByteLength = 1;
              int8_t int8Value;
              memcpy(&int8Value, &cLogdata[nOffset], nByteLength);
              fValue = int8Value;
            } break;

            case 5: { // INT16
              nByteLength = 2;
              int16_t int16Value;
              memcpy(&int16Value, &cLogdata[nOffset], nByteLength);
              fValue = int16Value;
            } break;

            case 6: { // INT32
              nByteLength = 4;
              int32_t int32Value;
              memcpy(&int32Value, &cLogdata[nOffset], nByteLength);
              fValue = int32Value;
            } break;

            case 7: { // FLOAT
              nByteLength = 4;
              memcpy(&fValue, &cLogdata[nOffset], nByteLength);
            } break;

            case 8: { // FP16
              // NOTE(winkler): This is untested code (as no FP16
              // variable gets advertised yet). This has to be tested
              // and is to be used carefully. I will do that as soon
              // as I find time for it.
              nByteLength = 2;
              char cBuffer1[nByteLength];
              char cBuffer2[4];
              memcpy(cBuffer1, &cLogdata[nOffset], nByteLength);
              cBuffer2[0] = cBuffer1[0] & 0b10000000; // Get the sign bit
              cBuffer2[1] = 0;
              cBuffer2[2] = cBuffer1[0] & 0b01111111; // Get the magnitude
              cBuffer2[3] = cBuffer1[1];
              memcpy(&fValue, cBuffer2, 4); // Put it into the float variable
            } break;

            default: { // Unknown. This hopefully never happens.
            } break;
            }

            toc_setFloatValueForElementID(toc, nElementID, fValue);
            nOffset += nByteLength;
            nIndex++;
          } else {
            fprintf(stderr, "Didn't find element ID %d in block ID %d while "
                    "parsing incoming logging data.\n"
                    "This REALLY shouldn't be happening!\n",
                    nElementID, nBlockID);
            exit(EXIT_FAILURE);
          }
        }
      }

      crtppacket_free(crtpPacket);
    }
  }
}

int toc_elementIDinBlock(struct toc *toc, int nBlockID, int nElementIndex) {
  bool bFound;

  struct LoggingBlock lbCurrent = toc_loggingBlockForID(toc, nBlockID, &bFound);
  if(bFound) {
    if(nElementIndex < lbCurrent.lstElementIDsCount) {
      return lbCurrent.lstElementIDs[nElementIndex];
    }
  }

  return -1;
}

bool toc_setFloatValueForElementID(struct toc *toc, int nElementID, float fValue) {
  int nIndex = 0;
  for(int i = i; i < toc->m_lstTOCElementsCount; i++) {
    struct TOCElement teCurrent = toc->m_lstTOCElements[i];

    if(teCurrent.nID == nElementID) {
      teCurrent.dValue = fValue; // We store floats as doubles
      toc->m_lstTOCElements[i] = teCurrent;
      // std::cout << fValue << std::endl;
      return true;
    }
  }

  return false;
}
