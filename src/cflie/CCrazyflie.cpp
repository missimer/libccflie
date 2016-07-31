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
  m_crRadio = crRadio;

  // Review these values
  m_fMaxAbsRoll = 45.0f;
  m_fMaxAbsPitch = m_fMaxAbsRoll;
  m_fMaxYaw = 180.0f;
  m_nMaxThrust = 60000;
  m_nMinThrust = 0;//15000;

  m_fRoll = 0;
  m_fPitch = 0;
  m_fYaw = 0;
  m_nThrust = 0;

  m_bSendsSetpoints = false;

  m_tocParameters = toc_alloc(m_crRadio, 2);
  m_tocLogs = toc_alloc(m_crRadio, 5);

  m_enumState = STATE_ZERO;

  m_dSendSetpointPeriod = 0.01; // Seconds
  m_dSetpointLastSent = 0;
}

CCrazyflie::~CCrazyflie() {
  this->stopLogging();
}

bool CCrazyflie::readTOCParameters() {
  if(toc_requestMetaData(m_tocParameters)) {
    if(toc_requestItems(m_tocParameters)) {
      return true;
    }
  }

  return false;
}

bool CCrazyflie::readTOCLogs() {
  if(toc_requestMetaData(m_tocLogs)) {
    if(toc_requestItems(m_tocLogs)) {
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
  CCRTPPacket *crtpReceived = m_crRadio->sendPacket(crtpPacket);

  delete crtpPacket;
  if(crtpReceived != NULL) {
    delete crtpReceived;
    return true;
  } else {
    return false;
  }
}

void CCrazyflie::setThrust(int nThrust) {
  m_nThrust = nThrust;

  if(m_nThrust < m_nMinThrust) {
    m_nThrust = m_nMinThrust;
  } else if(m_nThrust > m_nMaxThrust) {
    m_nThrust = m_nMaxThrust;
  }
}

int CCrazyflie::thrust() {
  return this->sensorDoubleValue("stabilizer.thrust");
}

bool CCrazyflie::cycle() {
  double dTimeNow = this->currentTime();

  switch(m_enumState) {
  case STATE_ZERO: {
    m_enumState = STATE_READ_PARAMETERS_TOC;
  } break;

  case STATE_READ_PARAMETERS_TOC: {
    if(this->readTOCParameters()) {
      m_enumState = STATE_READ_LOGS_TOC;
    }
  } break;

  case STATE_READ_LOGS_TOC: {
    if(this->readTOCLogs()) {
      m_enumState = STATE_START_LOGGING;
    }
  } break;

  case STATE_START_LOGGING: {
    if(this->startLogging()) {
      m_enumState = STATE_ZERO_MEASUREMENTS;
    }
  } break;

  case STATE_ZERO_MEASUREMENTS: {
    int count;
    CCRTPPacket **packets = m_crRadio->popLoggingPackets(&count);
    toc_processPackets(m_tocLogs, packets, count);

    // NOTE(winkler): Here, we can do measurement zero'ing. This is
    // not done at the moment, though. Reason: No readings to zero at
    // the moment. This might change when altitude becomes available.

    m_enumState = STATE_NORMAL_OPERATION;
  } break;

  case STATE_NORMAL_OPERATION: {
    // Shove over the sensor readings from the radio to the Logs TOC.
    int count;
    CCRTPPacket **packets = m_crRadio->popLoggingPackets(&count);
    toc_processPackets(m_tocLogs, packets, count);

    if(m_bSendsSetpoints) {
      // Check if it's time to send the setpoint
      if(dTimeNow - m_dSetpointLastSent > m_dSendSetpointPeriod) {
        // Send the current set point based on the previous calculations
        this->sendSetpoint(m_fRoll, m_fPitch, m_fYaw, m_nThrust);
        m_dSetpointLastSent = dTimeNow;
      }
    } else {
      // Send a dummy packet for keepalive
      m_crRadio->sendDummyPacket();
    }
  } break;

  default: {
  } break;
  }

  if(m_crRadio->ackReceived()) {
    m_nAckMissCounter = 0;
  } else {
    m_nAckMissCounter++;
  }

  return m_crRadio->usbOK();
}

bool CCrazyflie::copterInRange() {
  return m_nAckMissCounter < m_nAckMissTolerance;
}

void CCrazyflie::setRoll(float fRoll) {
  m_fRoll = fRoll;

  if(fabs(m_fRoll) > m_fMaxAbsRoll) {
    m_fRoll = copysign(m_fMaxAbsRoll, m_fRoll);
  }
}

float CCrazyflie::roll() {
  return this->sensorDoubleValue("stabilizer.roll");
}

void CCrazyflie::setPitch(float fPitch) {
  m_fPitch = fPitch;

  if(fabs(m_fPitch) > m_fMaxAbsPitch) {
    m_fPitch = copysign(m_fMaxAbsPitch, m_fPitch);
  }
}

float CCrazyflie::pitch() {
  return this->sensorDoubleValue("stabilizer.pitch");
}

void CCrazyflie::setYaw(float fYaw) {
  m_fYaw = fYaw;

  if(fabs(m_fYaw) > m_fMaxYaw){
      m_fYaw = copysign(m_fMaxYaw, m_fYaw);
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
  return m_enumState == STATE_NORMAL_OPERATION;
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
  m_bSendsSetpoints = bSendSetpoints;
}

bool CCrazyflie::sendsSetpoints() {
  return m_bSendsSetpoints;
}

double CCrazyflie::sensorDoubleValue(const char *strName) {
  return toc_doubleValue(m_tocLogs, strName);
}

void CCrazyflie::disableLogging() {
  toc_unregisterLoggingBlock(m_tocLogs, "high-speed");
  toc_unregisterLoggingBlock(m_tocLogs, "low-speed");
}

void CCrazyflie::enableStabilizerLogging() {
  toc_registerLoggingBlock(m_tocLogs, "stabilizer", 1000);

  toc_startLogging(m_tocLogs, "stabilizer.roll", "stabilizer");
  toc_startLogging(m_tocLogs, "stabilizer.pitch", "stabilizer");
  toc_startLogging(m_tocLogs, "stabilizer.yaw", "stabilizer");
}

void CCrazyflie::enableGyroscopeLogging() {
  toc_registerLoggingBlock(m_tocLogs, "gyroscope", 1000);

  toc_startLogging(m_tocLogs, "gyro.x", "gyroscope");
  toc_startLogging(m_tocLogs, "gyro.y", "gyroscope");
  toc_startLogging(m_tocLogs, "gyro.z", "gyroscope");
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
  toc_registerLoggingBlock(m_tocLogs, "accelerometer", 1000);

  toc_startLogging(m_tocLogs, "acc.x", "accelerometer");
  toc_startLogging(m_tocLogs, "acc.y", "accelerometer");
  toc_startLogging(m_tocLogs, "acc.z", "accelerometer");
  toc_startLogging(m_tocLogs, "acc.zw", "accelerometer");
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
  toc_unregisterLoggingBlock(m_tocLogs, "stabilizer");
}

void CCrazyflie::disableGyroscopeLogging() {
  toc_unregisterLoggingBlock(m_tocLogs, "gyroscope");
}

void CCrazyflie::disableAccelerometerLogging() {
  toc_unregisterLoggingBlock(m_tocLogs, "accelerometer");
}

void CCrazyflie::enableBatteryLogging() {
  toc_registerLoggingBlock(m_tocLogs, "battery", 1000);

  toc_startLogging(m_tocLogs, "pm.vbat", "battery");
  toc_startLogging(m_tocLogs, "pm.state", "battery");
}

double CCrazyflie::batteryLevel() {
  return this->sensorDoubleValue("pm.vbat");
}

float CCrazyflie::batteryState() {
  return this->sensorDoubleValue("pm.state");
}

void CCrazyflie::disableBatteryLogging() {
  toc_unregisterLoggingBlock(m_tocLogs, "battery");
}

void CCrazyflie::enableMagnetometerLogging() {
  toc_registerLoggingBlock(m_tocLogs, "magnetometer", 1000);

  toc_startLogging(m_tocLogs, "mag.x", "magnetometer");
  toc_startLogging(m_tocLogs, "mag.y", "magnetometer");
  toc_startLogging(m_tocLogs, "mag.z", "magnetometer");
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
  toc_unregisterLoggingBlock(m_tocLogs, "magnetometer");
}

void CCrazyflie::enableAltimeterLogging() {
  toc_registerLoggingBlock(m_tocLogs, "altimeter", 1000);
  toc_startLogging(m_tocLogs, "alti.asl", "altimeter");
  toc_startLogging(m_tocLogs, "alti.aslLong", "altimeter");
  toc_startLogging(m_tocLogs, "alti.pressure", "altimeter");
  toc_startLogging(m_tocLogs, "alti.temperature", "altimeter");
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
  toc_unregisterLoggingBlock(m_tocLogs, "altimeter");
}
