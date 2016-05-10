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

#include <math.h>
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug_console_imx.h"

static long GetMiliseconds(void) {
    return (xTaskGetTickCount());
}

void PID_Init(PIDState* state, float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd,
        int ControllerDirection) {
    state->myOutput = Output;
    state->myInput = Input;
    state->mySetpoint = Setpoint;
    state->inAuto = false;

    SetOutputLimits(state, 0, 255);               //default output limit corresponds to

    state->SampleTime = 100;                      //default Controller Sample Time is 0.1 seconds

    SetControllerDirection(state, ControllerDirection);
    SetTunings(state, Kp, Ki, Kd);

    state->lastTime = GetMiliseconds() - state->SampleTime;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool Compute(PIDState* state) {
    if (!state->inAuto)
        return false;
    unsigned long now = GetMiliseconds();
    unsigned long timeChange = (now - state->lastTime);
    if (timeChange >= state->SampleTime) {
        /*Compute all the working error variables*/
        double input = *(state->myInput);
        double error = *(state->mySetpoint) - input;
        state->ITerm += (state->ki * error);
        if (state->ITerm > state->outMax)
            state->ITerm = state->outMax;
        else if (state->ITerm < state->outMin)
            state->ITerm = state->outMin;
        double dInput = (input - state->lastInput);

        /*Compute PID Output*/
        double output = state->kp * error + state->ITerm - state->kd * dInput;

        if (output > state->outMax)
            output = state->outMax;
        else if (output < state->outMin)
            output = state->outMin;
        *(state->myOutput) = output;

        /*Remember some variables for next time*/
        state->lastInput = input;
        state->lastTime = now;
        return true;
    } else
        return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(PIDState* state, float Kp, float Ki, float Kd) {
    if (Kp < 0 || Ki < 0 || Kd < 0)
        return;

    state->dispKp = Kp;
    state->dispKi = Ki;
    state->dispKd = Kd;

    double SampleTimeInSec = ((double) state->SampleTime) / 1000;
    state->kp = Kp;
    state->ki = Ki * SampleTimeInSec;
    state->kd = Kd / SampleTimeInSec;

    if (state->controllerDirection == REVERSE) {
        state->kp = (0 - state->kp);
        state->ki = (0 - state->ki);
        state->kd = (0 - state->kd);
    }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed  
 ******************************************************************************/
void SetSampleTime(PIDState* state, int NewSampleTime) {
    if (NewSampleTime > 0) {
        double ratio = (double) NewSampleTime / (double) state->SampleTime;
        state->ki *= ratio;
        state->kd /= ratio;
        state->SampleTime = (unsigned long) NewSampleTime;
    }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void SetOutputLimits(PIDState* state, float Min, float Max) {
    if (Min >= Max)
        return;
    state->outMin = Min;
    state->outMax = Max;

    if (state->inAuto) {
        if (*(state->myOutput) > state->outMax)
            *(state->myOutput) = state->outMax;
        else if (*(state->myOutput) < state->outMin)
            *(state->myOutput) = state->outMin;

        if (state->ITerm > state->outMax)
            state->ITerm = state->outMax;
        else if (state->ITerm < state->outMin)
            state->ITerm = state->outMin;
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void SetMode(PIDState* state, int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto == !state->inAuto) { /*we just went from manual to auto*/
        Initialize(state);
    }
    state->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Initialize(PIDState* state) {
    state->ITerm = *(state->myOutput);
    state->lastInput = *(state->myInput);
    if (state->ITerm > state->outMax)
        state->ITerm = state->outMax;
    else if (state->ITerm < state->outMin)
        state->ITerm = state->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void SetControllerDirection(PIDState* state, int Direction) {
    if (state->inAuto && Direction != state->controllerDirection) {
        state->kp = (0 - state->kp);
        state->ki = (0 - state->ki);
        state->kd = (0 - state->kd);
    }
    state->controllerDirection = Direction;
}
