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


#include <cflie/CCrazyflie.h>

struct crazyflie * crazyflie_alloc(CCrazyRadio *crRadio) {

  struct crazyflie *cf = (struct crazyflie *)malloc(sizeof(struct crazyflie));

  if(cf) {
    crazyflie_init(cf, crRadio);
  }

  return cf;
}

void crazyflie_init(struct crazyflie *cf, CCrazyRadio *crRadio) {
  cf->m_crRadio = crRadio;

  // Review these values
  cf->m_fMaxAbsRoll = 45.0f;
  cf->m_fMaxAbsPitch = cf->m_fMaxAbsRoll;
  cf->m_fMaxYaw = 180.0f;
  cf->m_nMaxThrust = 60000;
  cf->m_nMinThrust = 0;//15000;

  cf->m_fRoll = 0;
  cf->m_fPitch = 0;
  cf->m_fYaw = 0;
  cf->m_nThrust = 0;

  cf->m_bSendsSetpoints = false;

  cf->m_tocParameters = toc_alloc(cf->m_crRadio, 2);
  cf->m_tocLogs = toc_alloc(cf->m_crRadio, 5);

  cf->m_enumState = STATE_ZERO;

  cf->m_dSendSetpointPeriod = 0.01; // Seconds
  cf->m_dSetpointLastSent = 0;
}

void crazyflie_destroy(struct crazyflie *cf) {
  crazyflie_stopLogging(cf);
}

void crazyflie_free(struct crazyflie *cf) {
  crazyflie_destroy(cf);
  free(cf);
}

bool crazyflie_readTOCParameters(struct crazyflie *cf) {
  if(toc_requestMetaData(cf->m_tocParameters)) {
    if(toc_requestItems(cf->m_tocParameters)) {
      return true;
    }
  }

  return false;
}

bool crazyflie_readTOCLogs(struct crazyflie *cf) {
  if(toc_requestMetaData(cf->m_tocLogs)) {
    if(toc_requestItems(cf->m_tocLogs)) {
      return true;
    }
  }

  return false;
}

bool crazyflie_sendSetpoint(struct crazyflie *cf, float fRoll, float fPitch, float fYaw, short sThrust) {
  fPitch = -fPitch;

  int nSize = 3 * sizeof(float) + sizeof(short);
  char cBuffer[nSize];
  memcpy(&cBuffer[0 * sizeof(float)], &fRoll, sizeof(float));
  memcpy(&cBuffer[1 * sizeof(float)], &fPitch, sizeof(float));
  memcpy(&cBuffer[2 * sizeof(float)], &fYaw, sizeof(float));
  memcpy(&cBuffer[3 * sizeof(float)], &sThrust, sizeof(short));

  CCRTPPacket *crtpPacket = new CCRTPPacket(cBuffer, nSize, 3);
  CCRTPPacket *crtpReceived = cf->m_crRadio->sendPacket(crtpPacket);

  delete crtpPacket;
  if(crtpReceived != NULL) {
    delete crtpReceived;
    return true;
  } else {
    return false;
  }
}

void crazyflie_setThrust(struct crazyflie *cf, int nThrust) {
  cf->m_nThrust = nThrust;

  if(cf->m_nThrust < cf->m_nMinThrust) {
    cf->m_nThrust = cf->m_nMinThrust;
  } else if(cf->m_nThrust > cf->m_nMaxThrust) {
    cf->m_nThrust = cf->m_nMaxThrust;
  }
}

int crazyflie_thrust(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "stabilizer.thrust");
}

bool crazyflie_cycle(struct crazyflie *cf) {
  double dTimeNow = crazyflie_currentTime(cf);

  switch(cf->m_enumState) {
  case STATE_ZERO: {
    cf->m_enumState = STATE_READ_PARAMETERS_TOC;
  } break;

  case STATE_READ_PARAMETERS_TOC: {
    if(crazyflie_readTOCParameters(cf)) {
      cf->m_enumState = STATE_READ_LOGS_TOC;
    }
  } break;

  case STATE_READ_LOGS_TOC: {
    if(crazyflie_readTOCLogs(cf)) {
      cf->m_enumState = STATE_START_LOGGING;
    }
  } break;

  case STATE_START_LOGGING: {
    if(crazyflie_startLogging(cf)) {
      cf->m_enumState = STATE_ZERO_MEASUREMENTS;
    }
  } break;

  case STATE_ZERO_MEASUREMENTS: {
    int count;
    CCRTPPacket **packets = cf->m_crRadio->popLoggingPackets(&count);
    toc_processPackets(cf->m_tocLogs, packets, count);

    // NOTE(winkler): Here, we can do measurement zero'ing. This is
    // not done at the moment, though. Reason: No readings to zero at
    // the moment. This might change when altitude becomes available.

    cf->m_enumState = STATE_NORMAL_OPERATION;
  } break;

  case STATE_NORMAL_OPERATION: {
    // Shove over the sensor readings from the radio to the Logs TOC.
    int count;
    CCRTPPacket **packets = cf->m_crRadio->popLoggingPackets(&count);
    toc_processPackets(cf->m_tocLogs, packets, count);

    if(cf->m_bSendsSetpoints) {
      // Check if it's time to send the setpoint
      if(dTimeNow - cf->m_dSetpointLastSent > cf->m_dSendSetpointPeriod) {
        // Send the current set point based on the previous calculations
        crazyflie_sendSetpoint(cf, cf->m_fRoll, cf->m_fPitch, cf->m_fYaw, cf->m_nThrust);
        cf->m_dSetpointLastSent = dTimeNow;
      }
    } else {
      // Send a dummy packet for keepalive
      cf->m_crRadio->sendDummyPacket();
    }
  } break;

  default: {
  } break;
  }

  if(cf->m_crRadio->ackReceived()) {
    cf->m_nAckMissCounter = 0;
  } else {
    cf->m_nAckMissCounter++;
  }

  return cf->m_crRadio->usbOK();
}

bool crazyflie_copterInRange(struct crazyflie *cf) {
  return cf->m_nAckMissCounter < cf->m_nAckMissTolerance;
}

void crazyflie_setRoll(struct crazyflie *cf, float fRoll) {
  cf->m_fRoll = fRoll;

  if(fabs(cf->m_fRoll) > cf->m_fMaxAbsRoll) {
    cf->m_fRoll = copysign(cf->m_fMaxAbsRoll, cf->m_fRoll);
  }
}

float crazyflie_roll(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "stabilizer.roll");
}

void crazyflie_setPitch(struct crazyflie *cf, float fPitch) {
  cf->m_fPitch = fPitch;

  if(fabs(cf->m_fPitch) > cf->m_fMaxAbsPitch) {
    cf->m_fPitch = copysign(cf->m_fMaxAbsPitch, cf->m_fPitch);
  }
}

float crazyflie_pitch(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "stabilizer.pitch");
}

void crazyflie_setYaw(struct crazyflie *cf, float fYaw) {
  cf->m_fYaw = fYaw;

  if(fabs(cf->m_fYaw) > cf->m_fMaxYaw){
      cf->m_fYaw = copysign(cf->m_fMaxYaw, cf->m_fYaw);
  }
}

float crazyflie_yaw(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "stabilizer.yaw");
}

double crazyflie_currentTime(struct crazyflie *cf) {
  struct timespec tsTime;
  clock_gettime(CLOCK_MONOTONIC, &tsTime);

  return tsTime.tv_sec + double(tsTime.tv_nsec) / NSEC_PER_SEC;
}

bool crazyflie_isInitialized(struct crazyflie *cf) {
  return cf->m_enumState == STATE_NORMAL_OPERATION;
}

bool crazyflie_startLogging(struct crazyflie *cf) {
  // Register the desired sensor readings
  crazyflie_enableStabilizerLogging(cf);
  crazyflie_enableGyroscopeLogging(cf);
  crazyflie_enableAccelerometerLogging(cf);
  crazyflie_enableBatteryLogging(cf);
  crazyflie_enableMagnetometerLogging(cf);
  crazyflie_enableAltimeterLogging(cf);

  return true;
}

bool crazyflie_stopLogging(struct crazyflie *cf) {
  crazyflie_disableStabilizerLogging(cf);
  crazyflie_disableGyroscopeLogging(cf);
  crazyflie_disableAccelerometerLogging(cf);
  crazyflie_disableBatteryLogging(cf);
  crazyflie_disableMagnetometerLogging(cf);
  crazyflie_disableAltimeterLogging(cf);

  return true;
}

void crazyflie_setSendSetpoints(struct crazyflie *cf, bool bSendSetpoints) {
  cf->m_bSendsSetpoints = bSendSetpoints;
}

bool crazyflie_sendsSetpoints(struct crazyflie *cf) {
  return cf->m_bSendsSetpoints;
}

double crazyflie_sensorDoubleValue(struct crazyflie *cf, const char *strName) {
  return toc_doubleValue(cf->m_tocLogs, strName);
}

void crazyflie_disableLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "high-speed");
  toc_unregisterLoggingBlock(cf->m_tocLogs, "low-speed");
}

void crazyflie_enableStabilizerLogging(struct crazyflie *cf) {
  toc_registerLoggingBlock(cf->m_tocLogs, "stabilizer", 1000);

  toc_startLogging(cf->m_tocLogs, "stabilizer.roll", "stabilizer");
  toc_startLogging(cf->m_tocLogs, "stabilizer.pitch", "stabilizer");
  toc_startLogging(cf->m_tocLogs, "stabilizer.yaw", "stabilizer");
}

void crazyflie_enableGyroscopeLogging(struct crazyflie *cf) {
  toc_registerLoggingBlock(cf->m_tocLogs, "gyroscope", 1000);

  toc_startLogging(cf->m_tocLogs, "gyro.x", "gyroscope");
  toc_startLogging(cf->m_tocLogs, "gyro.y", "gyroscope");
  toc_startLogging(cf->m_tocLogs, "gyro.z", "gyroscope");
}

float crazyflie_gyroX(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "gyro.x");
}

float crazyflie_gyroY(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "gyro.y");
}

float crazyflie_gyroZ(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "gyro.z");
}

void crazyflie_enableAccelerometerLogging(struct crazyflie *cf) {
  toc_registerLoggingBlock(cf->m_tocLogs, "accelerometer", 1000);

  toc_startLogging(cf->m_tocLogs, "acc.x", "accelerometer");
  toc_startLogging(cf->m_tocLogs, "acc.y", "accelerometer");
  toc_startLogging(cf->m_tocLogs, "acc.z", "accelerometer");
  toc_startLogging(cf->m_tocLogs, "acc.zw", "accelerometer");
}

float crazyflie_accX(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "acc.x");
}

float crazyflie_accY(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "acc.y");
}

float crazyflie_accZ(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "acc.z");
}

float crazyflie_accZW(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "acc.zw");
}

void crazyflie_disableStabilizerLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "stabilizer");
}

void crazyflie_disableGyroscopeLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "gyroscope");
}

void crazyflie_disableAccelerometerLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "accelerometer");
}

void crazyflie_enableBatteryLogging(struct crazyflie *cf) {
  toc_registerLoggingBlock(cf->m_tocLogs, "battery", 1000);

  toc_startLogging(cf->m_tocLogs, "pm.vbat", "battery");
  toc_startLogging(cf->m_tocLogs, "pm.state", "battery");
}

double crazyflie_batteryLevel(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "pm.vbat");
}

float crazyflie_batteryState(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "pm.state");
}

void crazyflie_disableBatteryLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "battery");
}

void crazyflie_enableMagnetometerLogging(struct crazyflie *cf) {
  toc_registerLoggingBlock(cf->m_tocLogs, "magnetometer", 1000);

  toc_startLogging(cf->m_tocLogs, "mag.x", "magnetometer");
  toc_startLogging(cf->m_tocLogs, "mag.y", "magnetometer");
  toc_startLogging(cf->m_tocLogs, "mag.z", "magnetometer");
}
float crazyflie_magX(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "mag.x");
}
float crazyflie_magY(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "mag.y");
}
float crazyflie_magZ(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "mag.z");
}
void crazyflie_disableMagnetometerLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "magnetometer");
}

void crazyflie_enableAltimeterLogging(struct crazyflie *cf) {
  toc_registerLoggingBlock(cf->m_tocLogs, "altimeter", 1000);
  toc_startLogging(cf->m_tocLogs, "alti.asl", "altimeter");
  toc_startLogging(cf->m_tocLogs, "alti.aslLong", "altimeter");
  toc_startLogging(cf->m_tocLogs, "alti.pressure", "altimeter");
  toc_startLogging(cf->m_tocLogs, "alti.temperature", "altimeter");
}

float crazyflie_asl(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "alti.asl");
}
float crazyflie_aslLong(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "alti.aslLong");
}
float crazyflie_pressure(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "alti.pressure");
}
float crazyflie_temperature(struct crazyflie *cf) {
  return crazyflie_sensorDoubleValue(cf, "alti.temperature");
}

void crazyflie_disableAltimeterLogging(struct crazyflie *cf) {
  toc_unregisterLoggingBlock(cf->m_tocLogs, "altimeter");
}
