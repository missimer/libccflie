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


// System
#include <stdio.h>
#include <signal.h>
#include <ncurses.h>
#include <stdlib.h>

// libcflie
#include <cflie/crazyflie.h>

void my_handler(int s){
  endwin();
  exit(1);
}

#define ROLL  0
#define PITCH 1
#define YAW   2

int main(int argc, char **argv) {
  struct sigaction sigIntHandler;
  float k_p = 1.0f;
  float k_i = 1.0f;
  float k_d = 1.0f;
  float errors[3]   = {0.0f, 0.0f, 0.0f};
  float sums[3]     = {0.0f, 0.0f, 0.0f};
  float change[3]   = {0.0f, 0.0f, 0.0f};
  float previous[3] = {0.0f, 0.0f, 0.0f};
  float ratios[3]   = {1.0f, 1.0f, 1.0f};
  struct crazyradio *crRadio = crazyradio_alloc("radio://0/80/250K");

  initscr();                  /* Start curses mode */
  printw("Hello World !!!");  /* Print Hello World */
  refresh();                  /* Print it on to the real screen */

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  if(crazyradio_startRadio(crRadio)) {
    struct crazyflie *cflieCopter = crazyflie_alloc(crRadio);
    //crazyflie_setThrust(cflieCopter, 36801);

    // Enable sending the setpoints. This can be used to temporarily
    // stop updating the internal controller setpoints and instead
    // sending dummy packets (to keep the connection alive).
    crazyflie_setSendSetpoints(cflieCopter, true);

    while(crazyflie_cycle(cflieCopter)) {

      move(0, 0);
      clear();

      //printf("\n\n");
      // Main loop. Currently empty.

      errors[ROLL]  = 0.0f - crazyflie_roll(cflieCopter);
      errors[PITCH] = 0.0f - crazyflie_pitch(cflieCopter);
      errors[YAW]   = 0.0f - crazyflie_yaw(cflieCopter);

      sum[ROLL]  += errors[ROLL];
      sum[PITCH] += errors[PITCH];
      sum[YAW]   += errors[YAW];

      change[ROLL]  = errors[ROLL] - previous[ROLL];
      change[YAW]   = errors[YAW] - previous[YAW];
      change[PITCH] = errors[PITCH] - previous[PITCH];

      crazyflie_setRoll(cflieCopter, 0);
      crazyflie_setPitch(cflieCopter, 0);
      crazyflie_setYaw(cflieCopter, 0);

      printw("accX: %f\n", crazyflie_accX(cflieCopter));
      printw("accY: %f\n", crazyflie_accY(cflieCopter));
      printw("accZ: %f\n", crazyflie_accZ(cflieCopter));
      printw("accZW: %f\n", crazyflie_accZW(cflieCopter));
      printw("gyroX: %f\n", crazyflie_gyroX(cflieCopter));
      printw("gyroY: %f\n", crazyflie_gyroY(cflieCopter));
      printw("gyroZ: %f\n", crazyflie_gyroZ(cflieCopter));

      previous[ROLL]  = errors[ROLL];
      previous[YAW]   = errors[YAW];
      previous[PITCH] = errors[PITCH];

      refresh();                  /* Print it on to the real screen */


      /* Examples to set thrust and RPY:

      // Range: 10001 - (approx.) 60000
      crazyflie_setThrust(cflieCopter, 10001);

      // All in degrees. R/P shouldn't be over 45 degree (it goes
      // sidewards really fast!). R/P/Y are all from -180.0deg to 180.0deg.
      crazyflie_setRoll(cflieCopter, 20);
      crazyflie_setPitch(cflieCopter, 15);
      crazyflie_setYaw(cflieCopter, 140); */
    }

    crazyflie_free(cflieCopter);
  } else {
    fprintf(stderr, "Could not connect to dongle. Did you plug it in?");
  }

  crazyradio_free(crRadio);

  endwin();

  return 0;
}
