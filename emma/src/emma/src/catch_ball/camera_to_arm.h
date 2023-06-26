#ifndef CAMERA_TO_ARM
#define CAMERA_TO_ARM


// translates a point [x,y,z] from camera coordinates to arm coordinates
void transform_camera_to_arm_coordinates(double camera_x, double camera_y, double camera_z, double *arm_x, double *arm_y, double *arm_z);


#endif
