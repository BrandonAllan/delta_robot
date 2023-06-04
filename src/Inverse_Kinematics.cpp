#include <math.h>
#include <stdio.h>

// Constants
const float f = 70;     // Base length
const float e = 50;     // End effector length
const float rf = 200;   // Forearm length
const float re = 460;   // Upper arm length
const float sqrt3 = 1.73205;   // sqrt(3.0)
const float pi = 3.141592653;  // PI
const float tan30 = 0.57735;   // tan(30 degrees)

struct DeltaAngles {
    float theta1;
    float theta2;
    float theta3;
};

DeltaAngles calculateInverseKinematics(float x0, float y0, float z0) {
    DeltaAngles angles;

    // Calculate intermediate variables
    float y1 = -tan30 * (f + e);
    y0 -= e;

    // Delta equations
    float a1 = 2 * rf * (y0 - y1);
    float b1 = 2 * rf * sqrt3 * x0;
    float c1 = pow(x0, 2) + pow(y0 - y1, 2) + pow(z0, 2) + pow(rf, 2) - pow(re, 2);

    // Calculate theta angles
    angles.theta1 = (-b1 + sqrt(pow(b1, 2) - 4 * a1 * c1)) / (2 * a1);
    angles.theta2 = (-b1 - sqrt(pow(b1, 2) - 4 * a1 * c1)) / (2 * a1);
    angles.theta3 = (-2 * z0) / (rf + re);

    return angles;
}

