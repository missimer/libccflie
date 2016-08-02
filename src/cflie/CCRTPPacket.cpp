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


#include <cflie/CCRTPPacket.h>
#include <stdlib.h>


CCRTPPacket::CCRTPPacket(int nPort) {
  crtppacket_basicSetup(&this->packet);
  crtppacket_setPort(&this->packet, nPort);
}

CCRTPPacket::CCRTPPacket(char *cData, int nDataLength, int nPort) {
  crtppacket_basicSetup(&this->packet);
  crtppacket_setPort(&this->packet, nPort);

  crtppacket_setData(&this->packet, cData, nDataLength);
}

CCRTPPacket::CCRTPPacket(char cData, int nPort) {
  crtppacket_basicSetup(&this->packet);
  crtppacket_setPort(&this->packet, nPort);

  crtppacket_setData(&this->packet, &cData, 1);
}

CCRTPPacket::~CCRTPPacket() {
  crtppacket_clearData(&this->packet);
}

void crtppacket_basicSetup(struct crtppacket *packet) {
  packet->m_cData = NULL;
  packet->m_nDataLength = 0;
  packet->m_nPort = 0;
  packet->m_nChannel = 0;
  packet->m_bIsPingPacket = false;
}

void crtppacket_setData(struct crtppacket *packet, char *cData, int nDataLength) {
  crtppacket_clearData(packet);

  packet->m_cData = (char *)malloc(sizeof(char) * nDataLength);
  memcpy(packet->m_cData, cData, nDataLength);
  packet->m_nDataLength = nDataLength;
}

char *crtppacket_data(struct crtppacket *packet) {
  return packet->m_cData;
}

int crtppacket_dataLength(struct crtppacket *packet) {
  return packet->m_nDataLength;
}

void crtppacket_clearData(struct crtppacket *packet) {
  if(packet->m_cData != NULL) {
    free(packet->m_cData);
    packet->m_cData = NULL;
    packet->m_nDataLength = 0;
  }
}

char *crtppacket_sendableData(struct crtppacket *packet) {
  char *cSendable =
    (char *)malloc(sizeof(char) * crtppacket_sendableDataLength(packet));

  if(packet->m_bIsPingPacket) {
    cSendable[0] = 0xff;
  } else {
    // Header byte
    cSendable[0] = (packet->m_nPort << 4) | 0b00001100 | (packet->m_nChannel & 0x03);

    // Payload
    memcpy(&cSendable[1], packet->m_cData, packet->m_nDataLength);

    // Finishing byte
    //cSendable[packet->m_nDataLength + 1] = 0x27;
  }

  return cSendable;
}

int crtppacket_sendableDataLength(struct crtppacket *packet) {
  if(packet->m_bIsPingPacket) {
    return 1;
  } else {
    return packet->m_nDataLength + 1;//2;
  }
}

void crtppacket_setPort(struct crtppacket *packet, int nPort) {
  packet->m_nPort = nPort;
}

int crtppacket_port(struct crtppacket *packet) {
  return packet->m_nPort;
}

void crtppacket_setChannel(struct crtppacket *packet, int nChannel) {
  packet->m_nChannel = nChannel;
}

int crtppacket_channel(struct crtppacket *packet) {
  return packet->m_nChannel;
}

void crtppacket_setIsPingPacket(struct crtppacket *packet, bool bIsPingPacket) {
  packet->m_bIsPingPacket = bIsPingPacket;
}

bool crtppacket_isPingPacket(struct crtppacket *packet) {
  return packet->m_bIsPingPacket;
}
