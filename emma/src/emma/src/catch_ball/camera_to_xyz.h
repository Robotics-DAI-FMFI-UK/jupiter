#ifndef CAMERA_TO_XYZ
#define CAMERA_TO_XYZ

void init_calculate_position();


// calculates the position of a round object in visual field of camera
// the object was found at pixel coordinates [pixel_x, pixel_y], has a given diameter
// the function calculates the position of the object in 3D coordinates in front
// of the robot,  
//    x=perpendicular to robot heading, increasing to the right
//    y=in the direction of robot heading
//    z=vertical, increasing upwards
void calculate_position(int pixel_x, int pixel_y, int ball_width, int ball_height, double *x, double *y, double *z);


#endif
