
// calculates the position of a round object in visual field of camera
// the object was found at pixel coordinates [pixel_x, pixel_y], has a given diameter
// the function calculates the position of the object in 3D coordinates in front
// of the robot,  
//    x=perpendicular to robot heading, increasing to the right
//    y=in the direction of robot heading
//    z=vertical, increasing upwards

#include <stdio.h>
#include <math.h>

#include "find_brick.h"    //sirka,vyska

#define VERTICAL_FOV 49.5  // deg
#define HORIZONTAL_FOV 60  // deg

#define HALF_BALL_DIAMETER 0.033

static double TAN_NU_HALF, TAN_RHO_HALF;

void init_calculate_position()
{
  TAN_NU_HALF = tan(HORIZONTAL_FOV / 2.0 / 180.0 * M_PI);
  TAN_RHO_HALF = tan(VERTICAL_FOV / 2.0 / 180.0 * M_PI);
}


void calculate_position(int pixel_x, int pixel_y, int ball_width, int ball_height, double *x, double *y, double *z)
{
  pixel_x = pixel_x - sirka / 2;
  pixel_y = vyska / 2 - pixel_y;


  double radius = ball_width / 2;
  double relative_distance_from_center = fabs(pixel_x)/(sirka/2);  // 0..1
  relative_distance_from_center -= 0.3;   // we want to make center larger, and periphery smaller

  double shrinking_ratio = relative_distance_from_center * 0.14;   // approx 1/7 too wide at the periphery

  // shrink the radious of ball if it is located in the periphery (camera causes dilatation away from center)
  radius -= radius * shrinking_ratio;

  printf("modified radius = %.2lf\n", radius);

  double ksi = HORIZONTAL_FOV * radius / (sirka / 2);

  printf("ksi = %.2lf\n", ksi);

  *y = HALF_BALL_DIAMETER / tan(ksi * M_PI / 180.0 / 2);
  *x = 2 * (*y) * TAN_NU_HALF * pixel_x / sirka;
  *z = 2 * (*y) * TAN_RHO_HALF * pixel_y / vyska;

  printf("point [%d,%d] with ball [%d,%d]  calculated as [%.2lf,%.2lf,%.2lf]\n", pixel_x, pixel_y, ball_width, ball_height, *x, *y, *z);
}

