#ifndef __MOVE_ARM_TO_XYZ__
#define __MOVE_ARM_TO_XYZ__

void init_move_arm_to_xyz(ros::NodeHandle n);

void move_arm_to_xyz(double x, double y, double z, double gamma);

void close_hand();

void open_hand();

void rossleep(int n);

#endif
