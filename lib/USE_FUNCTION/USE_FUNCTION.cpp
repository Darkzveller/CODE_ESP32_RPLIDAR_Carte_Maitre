#include "Variable.h"
#include "USE_FUNCTION.h"

#include <mat.h>

int16_t fusion_octet(int octet0, int octet1)
{

    int16_t octet16 = (octet0 << 8) | octet1;
    return octet16;
}


float normaliser_angle_rad(float angle_rad)
{
    // return fmod(angle_rad + M_PI / 2, 2.0 * M_PI) - (M_PI / 2.0);
    while (angle_rad > M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
}
