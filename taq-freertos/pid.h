/*
 * Copyright (c) 2015-2016 Antmicro Ltd.
 *
 * This work is based on Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com>
 * https://github.com/br3ttb/Arduino-PID-Library/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef PID_H
#define PID_H
#include <stdbool.h>
#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT  0
#define REVERSE  1

typedef struct {
    float dispKp;               // * we'll hold on to the tuning parameters in user-entered
    float dispKi;               //   format for display purposes
    float dispKd;               //
    float kp;                   // * (P)roportional Tuning Parameter
    float ki;                   // * (I)ntegral Tuning Parameter
    float kd;                   // * (D)erivative Tuning Parameter
    int controllerDirection;
    float *myInput;             // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;            //   This creates a hard link between the variables and the
    float *mySetpoint;          //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.
    unsigned long lastTime;
    double ITerm, lastInput;
    unsigned long SampleTime;
    float outMin, outMax;bool inAuto;
} PIDState;

void PID_Init(PIDState* state, float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd,
        int ControllerDirection);
void SetOutputLimits(PIDState* state, float Min, float Max);
bool Compute(PIDState* state);
void SetSampleTime(PIDState* state, int NewSampleTime);
void SetTunings(PIDState* state, float Kp, float Ki, float Kd);
void SetMode(PIDState* state, int Mode);
void Initialize(PIDState* state);
void SetControllerDirection(PIDState* state, int Direction);

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double GetKp(PIDState* state) {
    return state->dispKp;
}
double GetKi(PIDState* state) {
    return state->dispKi;
}
double GetKd(PIDState* state) {
    return state->dispKd;
}
int GetMode(PIDState* state) {
    return state->inAuto ? AUTOMATIC : MANUAL;
}
int GetDirection(PIDState* state) {
    return state->controllerDirection;
}
#endif
