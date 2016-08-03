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


/* \author Jan Winkler */


#ifndef __C_TOC_H__
#define __C_TOC_H__


// System
#include <string.h>

// Private
#include "crazyradio.h"
#include "crtppacket.h"


/*! \brief Storage element for logged variable identities */
struct TOCElement {
  /*! \brief The numerical ID of the log element on the copter's
      internal table */
  int nID;
  /*! \brief The (ref) type of the log element */
  int nType;
  /*! \brief The string group name of the log element */
  const char* strGroup;
  /*! \brief The string identifier of the log element */
  const char* strIdentifier;
  bool bIsLogging;
  double dValue;
};

#define MAX_LST_ELEMENT_IDS 128

struct LoggingBlock {
  const char *strName;
  int nID;
  double dFrequency;
  int lstElementIDsCount;
  int lstElementIDs[MAX_LST_ELEMENT_IDS];
};

#define MAX_LST_TOC_ELEMENTS 64
#define MAX_LST_LOGGING_BLOCKS 64

struct toc {
  int m_nPort;
  struct crazyradio *m_crRadio;
  int m_nItemCount;
  int m_lstTOCElementsCount;
  struct TOCElement m_lstTOCElements[MAX_LST_TOC_ELEMENTS];
  int m_lstLoggingBlocksCount;
  struct LoggingBlock m_lstLoggingBlocks[MAX_LST_LOGGING_BLOCKS];
};

struct toc * toc_alloc(struct crazyradio *crRadio, int nPort);
void toc_init(struct toc *toc, struct crazyradio *crRadio, int nPort);
void toc_destroy(struct toc *toc);
void toc_free(struct toc *toc);


bool toc_requestInitialItem(struct toc *toc);
bool toc_requestItem(struct toc *toc, int nID, bool bInitial); //bInitial default is false;
bool toc_processItem(struct toc *toc, struct crtppacket* crtpItem);

bool toc_sendTOCPointerReset(struct toc *toc);
bool toc_requestMetaData(struct toc *toc);
bool toc_requestItems(struct toc *toc);

struct TOCElement toc_elementForName(struct toc *toc, const char *strName, bool *bFound);
struct TOCElement toc_elementForID(struct toc *toc, int nID, bool *bFound);
int toc_idForName(struct toc *toc, char *strName);
int toc_typeForName(struct toc *toc, char *strName);

// For loggable variables only
bool toc_registerLoggingBlock(struct toc *toc, const char *strName, double dFrequency);
bool toc_unregisterLoggingBlock(struct toc *toc, const char *strName);
struct LoggingBlock toc_loggingBlockForName(struct toc *toc, const char *strName, bool *bFound);
struct LoggingBlock toc_loggingBlockForID(struct toc *toc, int nID, bool *bFound);

bool toc_startLogging(struct toc *toc, const char *strName, const char *strBlockName);
bool toc_stopLogging(struct toc *toc, const char *strName);
bool toc_isLogging(struct toc *toc, const char *strName);

double toc_doubleValue(struct toc *toc, const char *strName);

bool toc_enableLogging(struct toc *toc, const char *strBlockName);

void toc_processPackets(struct toc *toc, struct crtppacket** lstPackets, int count);

int toc_elementIDinBlock(struct toc *toc, int nBlockID, int nElementIndex);
bool toc_setFloatValueForElementID(struct toc *toc, int nElementID, float fValue);
bool toc_addElementToBlock(struct toc *toc, int nBlockID, int nElementID);
bool toc_unregisterLoggingBlockID(struct toc *toc, int nID);

#endif /* __C_TOC_H__ */
