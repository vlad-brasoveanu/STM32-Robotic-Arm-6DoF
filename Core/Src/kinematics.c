#include "kinematics.h"

// --- DIMENSIUNI ROBOT (mm) ---
#define L1  68.073f
#define L2  120.000f
#define L3  120.880f
#define L4  120.000f

#define PI 3.14159265359f
#define RAD_TO_DEG 57.2957795f

RobotAngles calculateIK(float x, float y, float z, float pitch_deg, float roll_deg) {
    RobotAngles sol;
    sol.error = 0;
    sol.theta4 = roll_deg; // Roll-ul e pass-through

    // 1. Unghi Baza (Theta 1)
    sol.theta1 = atan2f(y, x) * RAD_TO_DEG;

    // 2. Calcul Pozitie Incheietura (Wrist Center)
    float pitch_rad = pitch_deg / RAD_TO_DEG;
    float r_target = sqrtf(x*x + y*y);

    float r_w = r_target - L4 * cosf(pitch_rad);
    float z_w = z - L4 * sinf(pitch_rad) - L1;

    // 3. Verificare Triunghi Brat
    float D = sqrtf(r_w*r_w + z_w*z_w);
    if (D > (L2 + L3) || D < fabsf(L2 - L3) || D == 0.0f) {
        sol.error = 1;
        return sol;
    }

    // 4. Legea Cosinusului (Cot - Theta 3)
    float cos_angle3 = (L2*L2 + L3*L3 - D*D) / (2 * L2 * L3);
    if (cos_angle3 > 1.0f) cos_angle3 = 1.0f;
    if (cos_angle3 < -1.0f) cos_angle3 = -1.0f;

    float angle3_rad = acosf(cos_angle3);
    sol.theta3 = (PI - angle3_rad) * RAD_TO_DEG;

    // 5. Unghi Umar (Theta 2)
    float alpha = atan2f(z_w, r_w);
    float cos_angle2 = (L2*L2 + D*D - L3*L3) / (2 * L2 * D);
    if (cos_angle2 > 1.0f) cos_angle2 = 1.0f;
    float beta = acosf(cos_angle2);

    sol.theta2 = (alpha + beta) * RAD_TO_DEG;

    // 6. Unghi Incheietura Pitch (Theta 5)
    sol.theta5 = pitch_deg - (sol.theta2 - 90 + sol.theta3);

    return sol;
}
