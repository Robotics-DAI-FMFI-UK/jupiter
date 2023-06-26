#include <stdio.h>

void transform_camera_to_arm_coordinates(double camera_x, double camera_y, double camera_z, double *arm_x, double *arm_y, double *arm_z)
{
  *arm_x = camera_x;
  *arm_z = camera_z - 0.226;
  *arm_y = camera_y - 0.180;
  printf("transformed camera [%.2lf,%.2lf,%.2lf] to arm [%.2lf,%.2lf,%.2lf]\n", camera_x, camera_y, camera_z,
           *arm_x, *arm_y, *arm_z);
}

