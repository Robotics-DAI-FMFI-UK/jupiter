#include <iostream>
#include <cmath>
#include <tuple>
#include <stdio.h>

using namespace std;

tuple<double, double, double, double> findArmAngles(double x, double y, double z, double l1, double l2, double l3, double gamma) {
    double alpha = atan2(x, y);// Uhol, o ktory sa musi rameno natocit k objektu

    //nove suradnice x, y
    double d = sqrt(x*x + y*y);
    y = z;
    x = d;
    
    double m = l3 * sin(gamma);
    double n = l3 * cos(gamma);

    //dalej pracujeme v rovine x, y

    double cx, cy;
    cx = x - m;
    cy = y + n; 
    
    double r = sqrt(cx * cx + cy * cy);

    //vyjadrene z kosinusovej vety 
    double phi = acos((l1 * l1 + r * r - l2 * l2) / (2 * l1 * r));

    double omega = asin(cy / r); 
    double theta2 = M_PI/2 - phi - omega;

    double epsilon = acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));
    double beta = M_PI - epsilon;

    double delta = M_PI - theta2 - beta - gamma;   

    //double delta = M_PI - psi - gamma;

    printf("calculations: d=%.2lf, [cx,cy]=[%.2lf,%.2lf], r=%.2lf\n", d, cx, cy, r);

    return make_tuple(alpha, theta2, beta, delta);
}


