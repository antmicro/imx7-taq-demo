/*
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

#include "kalman.h"

// object state variables
float angle;           // angle
float angularAccel;    // angular acceleration
float mAngle;          // measured angle
float mAnglualAccel;   // measured angular acceleration

// Kalman state variables
float Q;
float R;               // variance
float kAngle;          // angle
float kAngularAccel;   // angular acceleration
float gOffset;         // gyro offset
float P11;
float P13;
float P21;
float P31;
float P33;
float K1;
float K2;
float K3;

void Kalman_SetInitialValues(float initAngle, float initAccel) {
    angle = 0;
    angularAccel = 0;
    mAngle = 0;
    mAnglualAccel = 0;
    kAngle = initAngle;
    kAngularAccel = initAccel;
    gOffset = 0;
    Q = 0.0001;
    R = 1;
    P11 = R;
    P13 = 0;
    P21 = 0;
    P31 = 0;
    P33 = 0;
    K1 = 0;
    K2 = 0;
    K3 = 0;
}

float Kalman_Filter(float angle, float accel, float dt) {
    //Read sensor values
    mAngle = angle;
    mAnglualAccel = accel;

    // Prediction
    kAngle = kAngle + (mAnglualAccel - gOffset) * dt;
    kAngularAccel = mAnglualAccel - gOffset;

    P11 = P11 - P31 * dt + P33 * dt * dt - P13 * dt + Q;
    P13 = P13 - P33 * dt;
    P21 = P33 * dt - P31;
    P31 = P31 - P33 * dt;
    P33 = P33 + Q;

    // Correction
    K1 = P11 * (1 / (P11 + R));
    K2 = P21 * (1 / (P11 + R));
    K3 = P31 * (1 / (P11 + R));

    kAngle = kAngle + K1 * (mAngle - kAngle);
    kAngularAccel = kAngularAccel + K2 * (mAngle - kAngle);
    gOffset = gOffset + K3 * (mAngle - kAngle);

    P11 = (1 - K1) * P11;
    P13 = (1 - K1) * P13;
    P21 = P21 - P11 * K2;
    P31 = P31 - P11 * K3;
    P33 = P33 - P13 * K3;

    // Assign new values
    angle = kAngle;
    angularAccel = kAngularAccel;

    return kAngle;
}
