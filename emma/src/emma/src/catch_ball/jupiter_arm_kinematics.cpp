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
    double n = l3 * cos(gamma);//sqrt(l1 * l1 - m * m);

    //dalej pracujeme v rovine x, y

    double cx, cy;
    cx = x - m;
    cy = y + n; 
    
    double r = sqrt(cx * cx + cy * cy);

    //vyjadrene z kosinusovej vety (c^2 = a^2 + b^2 - 2ab * cos(C))
    //2ab * cos(C) = a^2 + b^2 - c^2                 
    //cos(C) = (a^2 + b^2 - c^2) / 2ab
    // C = acos((a^2 + b^2 - c^2) / 2ab)
    double phi = acos((l1 * l1 + r * r - l2 * l2) / (2 * l1 * r));

    double omega = asin(cy / r);  //acos((cx * cx + r * r - cy * cy) / (2 * cx * r));
    double theta2 = M_PI/2 - phi - omega;

    double epsilon = acos((l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2));
    double beta = M_PI - epsilon;

    double delta = M_PI - theta2 - beta - gamma;    //psi = 2*M_PI - alpha - epsilon - omega - M_PI/2;

    //double delta = M_PI - psi - gamma;

    printf("calculations: d=%.2lf, [cx,cy]=[%.2lf,%.2lf], r=%.2lf\n", d, cx, cy, r);

    return make_tuple(alpha, theta2, beta, delta);
}


