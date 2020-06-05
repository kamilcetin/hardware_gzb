// funcHeaders.h
#include <iostream>
#include <math.h>
#include <cmath>
#include <stdio.h>
#include <vector>
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense> 

#define PI 3.14159265

using namespace Eigen;
using namespace std;

//typedef Matrix<double, 6, 6> Matrix6d;

// Robots DH parameters
extern double    r1;
extern double    r2;
extern double    p1;
extern double    p2;

//Matrix6d J_stewart(double p[]);

Eigen::Matrix<double, 6, 6> J_stewart(const Eigen::Matrix<double, Eigen::Dynamic, 1> &x);
