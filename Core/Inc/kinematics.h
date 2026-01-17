#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

// Structura care tine unghiurile calculate
typedef struct {
    float theta1; // Baza
    float theta2; // Umar
    float theta3; // Cot
    float theta4; // Wrist Roll
    float theta5; // Wrist Pitch
    int error;    // 0 = OK, 1 = Tinta imposibila
} RobotAngles;

// Prototipul functiei
RobotAngles calculateIK(float x, float y, float z, float pitch_deg, float roll_deg);

#endif
