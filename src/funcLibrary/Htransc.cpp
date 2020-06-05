// HOMEGENOUS TRANSFORMATION FOR KUKA LBR IIWA R820
#include <iostream>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "funcHeaders.h"

using namespace Eigen;
using namespace std;

typedef Matrix<double, 4, 4> Matrix4d;

Matrix4d Htransc(double th[]){

//DH Parameters
double d1 = 0.360;
double d2 = 0.0;
double d3 = 0.420;
double d4 = 0.0;
double d5 = 0.400;
double d6 = 0.0;
double d7 = 0.126;
//double dtool = 0.260;
double dtool = 0.24;
double d[] = {d1,d2,d3,d4,d5,d6,d7+dtool};
       
Matrix4d HT(4,4);

HT << 0,0,0,0,
      0,0,0,0,
      0,0,0,0,
      0,0,0,1;

// from standard method
/*HT(0,0) = sin(th[6])*(sin(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))) - cos(th[6])*(sin(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))));
HT(0,1) = cos(th[6])*(sin(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))) + sin(th[6])*(sin(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))));
HT(0,2) = cos(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) - sin(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])));
HT(0,3) = d[6]*(cos(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) - sin(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])))) + d[4]*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) + d[2]*cos(th[0])*sin(th[1]);
HT(1,0) = cos(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])))) - sin(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])));
HT(1,1) = - cos(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))) - sin(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))));
HT(1,2) = - cos(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))) - sin(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))));
HT(1,3) = sin(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))) - cos(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1]));
HT(2,0) = cos(th[6])*(cos(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) - sin(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3]))) - sin(th[6])*(sin(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) - cos(th[4])*sin(th[1])*sin(th[2]));
HT(2,1) = - cos(th[6])*(sin(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) - cos(th[4])*sin(th[1])*sin(th[2])) - sin(th[6])*(cos(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) - sin(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3])));
HT(2,2) = sin(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) + cos(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3]));
HT(2,3) = d[0] + d[4]*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3])) + d[6]*(sin(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) + cos(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3]))) + d[2]*cos(th[1]);
*/
// from modified method
HT(0,0) = sin(th[6])*(sin(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))) - cos(th[6])*(sin(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))));
HT(0,1) = cos(th[6])*(sin(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))) + sin(th[6])*(sin(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2]))));
HT(0,2) = cos(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) - sin(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])));
HT(0,3) = d[6]*(cos(th[5])*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) - sin(th[5])*(cos(th[4])*(cos(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) - cos(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[2])*sin(th[0]) + cos(th[0])*cos(th[1])*sin(th[2])))) + d[4]*(sin(th[3])*(sin(th[0])*sin(th[2]) - cos(th[0])*cos(th[1])*cos(th[2])) + cos(th[0])*cos(th[3])*sin(th[1])) + d[2]*cos(th[0])*sin(th[1]);
HT(1,0) = cos(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])))) - sin(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2])));
HT(1,1) = - cos(th[6])*(sin(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) - cos(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))) - sin(th[6])*(sin(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) + cos(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))));
HT(1,2) = sin(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))) - cos(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1]));
HT(1,3) = d[2]*sin(th[0])*sin(th[1]) - d[4]*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) - d[6]*(cos(th[5])*(sin(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) - cos(th[3])*sin(th[0])*sin(th[1])) - sin(th[5])*(cos(th[4])*(cos(th[3])*(cos(th[0])*sin(th[2]) + cos(th[1])*cos(th[2])*sin(th[0])) + sin(th[0])*sin(th[1])*sin(th[3])) + sin(th[4])*(cos(th[0])*cos(th[2]) - cos(th[1])*sin(th[0])*sin(th[2]))));
HT(2,0) = cos(th[6])*(cos(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) - sin(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3]))) - sin(th[6])*(sin(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) - cos(th[4])*sin(th[1])*sin(th[2]));
HT(2,1) = - cos(th[6])*(sin(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) - cos(th[4])*sin(th[1])*sin(th[2])) - sin(th[6])*(cos(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) - sin(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3])));
HT(2,2) = sin(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) + cos(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3]));
HT(2,3) = d[0] + d[4]*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3])) + d[6]*(sin(th[5])*(cos(th[4])*(cos(th[1])*sin(th[3]) - cos(th[2])*cos(th[3])*sin(th[1])) + sin(th[1])*sin(th[2])*sin(th[4])) + cos(th[5])*(cos(th[1])*cos(th[3]) + cos(th[2])*sin(th[1])*sin(th[3]))) + d[2]*cos(th[1]);
 
return HT;
}
