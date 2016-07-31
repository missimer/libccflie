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


CCrazyflie::CCrazyflie(CCrazyRadio *crRadio) {
  cf.m_crRadio = crRadio;

  // Review these values
  cf.m_fMaxAbsRoll = 45.0f;
  cf.m_fMaxAbsPitch = cf.m_fMaxAbsRoll;
  cf.m_fMaxYaw = 180.0f;
  cf.m_nMaxThrust = 60000;
  cf.m_nMinThrust = 0;//15000;

  cf.m_fRoll = 0;
  cf.m_fPitch = 0;
  cf.m_fYaw = 0;
  cf.m_nThrust = 0;

  cf.m_bSendsSetpoints = false;

  cf.m_tocParameters = toc_alloc(cf.m_crRadio, 2);
  cf.m_tocLogs = toc_alloc(cf.m_crRadio, 5);

  cf.m_enumState = STATE_ZERO;

  cf.m_dSendSetpointPeriod = 0.01; // Seconds
  cf.m_dSetpointLastSent = 0;
}

CCrazyflie::~CCrazyflie() {
  this->stopLogging();
}

bool CCrazyflie::readTOCParameters() {
  if(toc_requestMetaData(cf.m_tocParameters)) {
    if(toc_requestItems(cf.m_tocParameters)) {
      return true;
    }
  }

  return false;
}

bool CCrazyflie::readTOCLogs() {
  if(toc_requestMetaData(cf.m_tocLogs)) {
    if(toc_requestItems(cf.m_tocLogs)) {
      return true;
    }
  }

  return false;
}

bool CCrazyflie::sendSetpoint(float fRoll, float fPitch, float fYaw, short sThrust) {
  fPitch = -fPitch;

  int nSize = 3 * sizeof(float) + sizeof(short);
  char cBuffer[nSize];
  memcpy(&cBuffer[0 * sizeof(float)], &fRoll, sizeof(float));
  memcpy(&cBuffer[1 * sizeof(float)], &fPitch, sizeof(float));
  memcpy(&cBuffer[2 * sizeof(float)], &fYaw, sizeof(float));
  memcpy(&cBuffer[3 * sizeof(float)], &sThrust, sizeof(short));

  CCRTPPacket *crtpPacket = new CCRTPPacket(cBuffer, nSize, 3);
  CCRTPPacket *crtpReceived = cf.m_crRadio->sendPacket(crtpPacket);

  delete crtpPacket;
  if(crtpReceived != NULL) {
    delete crtpReceived;
    return true;
  } else {
    return false;
  }
}

void CCrazyflie::setThrust(int nThrust) {
  cf.m_nThrust = nThrust;

  if(cf.m_nThrust < cf.m_nMinThrust) {
    cf.m_nThrust = cf.m_nMinThrust;
  } else if(cf.m_nThrust > cf.m_nMaxThrust) {
    cf.m_nThrust = cf.m_nMaxThrust;
  }
}

int CCrazyflie::thrust() {
  return this->sensorDoubleValue("stabilizer.thrust");
}

bool CCrazyflie::cycle() {
  double dTimeNow = this->currentTime();

  switch(cf.m_enumState) {
  case STATE_ZERO: {
    cf.m_enumState = STATE_READ_PARAMETERS_TOC;
  } break;

  case STATE_READ_PARAMETERS_TOC: {
    if(this->readTOCParameters()) {
      cf.m_enumState = STATE_READ_LOGS_TOC;
    }
  } break;

  case STATE_READ_LOGS_TOC: {
    if(this->readTOCLogs()) {
      cf.m_enumState = STATE_START_LOGGING;
    }
  } break;

  case STATE_START_LOGGING: {
    if(this->startLogging()) {
      cf.m_enumState = STATE_ZERO_MEASUREMENTS;
    }
  } break;

  case STATE_ZERO_MEASUREMENTS: {
    int count;
    CCRTPPacket **packets = cf.m_crRadio->popLoggingPackets(&count);
    toc_processPackets(cf.m_tocLogs, packets, count);

    // NOTE(winkler): Here, we can do measurement zero'ing. This is
    // not done at the moment, though. Reason: No readings to zero at
    // the moment. This might change when altitude becomes available.

    cf.m_enumState = STATE_NORMAL_OPERATION;
  } break;

  case STATE_NORMAL_OPERATION: {
    // Shove over the sensor readings from the radio to the Logs TOC.
    int count;
    CCRTPPacket **packets = cf.m_crRadio->popLoggingPackets(&count);
    toc_processPackets(cf.m_tocLogs, packets, count);

    if(cf.m_bSendsSetpoints) {
      // Check if it's time to send the setpoint
      if(dTimeNow - cf.m_dSetpointLastSent > cf.m_dSendSetpointPeriod) {
        // Send the current set point based on the previous calculations
        this->sendSetpoint(cf.m_fRoll, cf.m_fPitch, cf.m_fYaw, cf.m_nThrust);
        cf.m_dSetpointLastSent = dTimeNow;
      }
    } else {
      // Send a dummy packet for keepalive
      cf.m_crRadio->sendDummyPacket();
    }
  } break;

  default: {
  } break;
  }

  if(cf.m_crRadio->ackReceived()) {
    cf.m_nAckMissCounter = 0;
  } else {
    cf.m_nAckMissCounter++;
  }

  return cf.m_crRadio->usbOK();
}

bool CCrazyflie::copterInRange() {
  return cf.m_nAckMissCounter < cf.m_nAckMissTolerance;
}

void CCrazyflie::setRoll(float fRoll) {
  cf.m_fRoll = fRoll;

  if(fabs(cf.m_fRoll) > cf.m_fMaxAbsRoll) {
    cf.m_fRoll = copysign(cf.m_fMaxAbsRoll, cf.m_fRoll);
  }
}

float CCrazyflie::roll() {
  return this->sensorDoubleValue("stabilizer.roll");
}

void CCrazyflie::setPitch(float fPitch) {
  cf.m_fPitch = fPitch;

  if(fabs(cf.m_fPitch) > cf.m_fMaxAbsPitch) {
    cf.m_fPitch = copysign(cf.m_fMaxAbsPitch, cf.m_fPitch);
  }
}

float CCrazyflie::pitch() {
  return this->sensorDoubleValue("stabilizer.pitch");
}

void CCrazyflie::setYaw(float fYaw) {
  cf.m_fYaw = fYaw;

  if(fabs(cf.m_fYaw) > cf.m_fMaxYaw){
      cf.m_fYaw = copysign(cf.m_fMaxYaw, cf.m_fYaw);
  }
}

float CCrazyflie::yaw() {
  return this->sensorDoubleValue("stabilizer.yaw");
}

double CCrazyflie::currentTime() {
  struct timespec tsTime;
  clock_gettime(CLOCK_MONOTONIC, &tsTime);

  return tsTime.tv_sec + double(tsTime.tv_nsec) / NSEC_PER_SEC;
}

bool CCrazyflie::isInitialized() {
  return cf.m_enumState == STATE_NORMAL_OPERATION;
}

bool CCrazyflie::startLogging() {
  // Register the desired sensor readings
  this->enableStabilizerLogging();
  this->enableGyroscopeLogging();
  this->enableAccelerometerLogging();
  this->enableBatteryLogging();
  this->enableMagnetometerLogging();
  this->enableAltimeterLogging();

  return true;
}

bool CCrazyflie::stopLogging() {
  this->disableStabilizerLogging();
  this->disableGyroscopeLogging();
  this->disableAccelerometerLogging();
  this->disableBatteryLogging();
  this->disableMagnetometerLogging();
  this->disableAltimeterLogging();

  return true;
}

void CCrazyflie::setSendSetpoints(bool bSendSetpoints) {
  cf.m_bSendsSetpoints = bSendSetpoints;
}

bool CCrazyflie::sendsSetpoints() {
  return cf.m_bSendsSetpoints;
}

double CCrazyflie::sensorDoubleValue(const char *strName) {
  return toc_doubleValue(cf.m_tocLogs, strName);
}

void CCrazyflie::disableLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "high-speed");
  toc_unregisterLoggingBlock(cf.m_tocLogs, "low-speed");
}

void CCrazyflie::enableStabilizerLogging() {
  toc_registerLoggingBlock(cf.m_tocLogs, "stabilizer", 1000);

  toc_startLogging(cf.m_tocLogs, "stabilizer.roll", "stabilizer");
  toc_startLogging(cf.m_tocLogs, "stabilizer.pitch", "stabilizer");
  toc_startLogging(cf.m_tocLogs, "stabilizer.yaw", "stabilizer");
}

void CCrazyflie::enableGyroscopeLogging() {
  toc_registerLoggingBlock(cf.m_tocLogs, "gyroscope", 1000);

  toc_startLogging(cf.m_tocLogs, "gyro.x", "gyroscope");
  toc_startLogging(cf.m_tocLogs, "gyro.y", "gyroscope");
  toc_startLogging(cf.m_tocLogs, "gyro.z", "gyroscope");
}

float CCrazyflie::gyroX() {
  return this->sensorDoubleValue("gyro.x");
}

float CCrazyflie::gyroY() {
  return this->sensorDoubleValue("gyro.y");
}

float CCrazyflie::gyroZ() {
  return this->sensorDoubleValue("gyro.z");
}

void CCrazyflie::enableAccelerometerLogging() {
  toc_registerLoggingBlock(cf.m_tocLogs, "accelerometer", 1000);

  toc_startLogging(cf.m_tocLogs, "acc.x", "accelerometer");
  toc_startLogging(cf.m_tocLogs, "acc.y", "accelerometer");
  toc_startLogging(cf.m_tocLogs, "acc.z", "accelerometer");
  toc_startLogging(cf.m_tocLogs, "acc.zw", "accelerometer");
}

float CCrazyflie::accX() {
  return this->sensorDoubleValue("acc.x");
}

float CCrazyflie::accY() {
  return this->sensorDoubleValue("acc.y");
}

float CCrazyflie::accZ() {
  return this->sensorDoubleValue("acc.z");
}

float CCrazyflie::accZW() {
  return this->sensorDoubleValue("acc.zw");
}

void CCrazyflie::disableStabilizerLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "stabilizer");
}

void CCrazyflie::disableGyroscopeLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "gyroscope");
}

void CCrazyflie::disableAccelerometerLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "accelerometer");
}

void CCrazyflie::enableBatteryLogging() {
  toc_registerLoggingBlock(cf.m_tocLogs, "battery", 1000);

  toc_startLogging(cf.m_tocLogs, "pm.vbat", "battery");
  toc_startLogging(cf.m_tocLogs, "pm.state", "battery");
}

double CCrazyflie::batteryLevel() {
  return this->sensorDoubleValue("pm.vbat");
}

float CCrazyflie::batteryState() {
  return this->sensorDoubleValue("pm.state");
}

void CCrazyflie::disableBatteryLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "battery");
}

void CCrazyflie::enableMagnetometerLogging() {
  toc_registerLoggingBlock(cf.m_tocLogs, "magnetometer", 1000);

  toc_startLogging(cf.m_tocLogs, "mag.x", "magnetometer");
  toc_startLogging(cf.m_tocLogs, "mag.y", "magnetometer");
  toc_startLogging(cf.m_tocLogs, "mag.z", "magnetometer");
}
float CCrazyflie::magX() {
  return this->sensorDoubleValue("mag.x");
}
float CCrazyflie::magY() {
  return this->sensorDoubleValue("mag.y");
}
float CCrazyflie::magZ() {
  return this->sensorDoubleValue("mag.z");
}
void CCrazyflie::disableMagnetometerLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "magnetometer");
}

void CCrazyflie::enableAltimeterLogging() {
  toc_registerLoggingBlock(cf.m_tocLogs, "altimeter", 1000);
  toc_startLogging(cf.m_tocLogs, "alti.asl", "altimeter");
  toc_startLogging(cf.m_tocLogs, "alti.aslLong", "altimeter");
  toc_startLogging(cf.m_tocLogs, "alti.pressure", "altimeter");
  toc_startLogging(cf.m_tocLogs, "alti.temperature", "altimeter");
}

float CCrazyflie::asl() {
  return this->sensorDoubleValue("alti.asl");
}
float CCrazyflie::aslLong() {
  return this->sensorDoubleValue("alti.aslLong");
}
float CCrazyflie::pressure() {
  return this->sensorDoubleValue("alti.pressure");
}
float CCrazyflie::temperature() {
  return this->sensorDoubleValue("alti.temperature");
}

void CCrazyflie::disableAltimeterLogging() {
  toc_unregisterLoggingBlock(cf.m_tocLogs, "altimeter");
}
