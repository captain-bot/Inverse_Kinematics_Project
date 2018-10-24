/* 
 * File:   functions.h
 * Author: nobug-ros
 * Topic: Includes all functions
 * Created on October 22, 2018, 3:13 PM
 */

#ifndef IKFUNCTIONS_H
#define IKFUNCTIONS_H

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <math.h>
#include <string>
#include <armadillo>

using namespace std;
using namespace arma;

class solveBaxIK{
private:
    bool start;
public:
    solveBaxIK();
    solveBaxIK(bool val);
    void computeBaxIK(double* th1, mat* des_p, mat* des_R, double* end_lnk);
};

// Other functions
bool comp_angle(mat* sol, int* iter_val, double* tol, double* th1, mat* pd, mat* rd, mat* ak, mat* dk, mat* alpk, mat* bax_base, mat* jl_min, mat* jl_max, mat* wrist_gbl, double* L1, double* L2, vec* dcrt_vec, int* npts);
mat bax_tran(double a, double d, double alpha, double th, int idx);
mat find_ang45(int* sol_exist, mat* ak, mat* dk, mat* pt_W, mat* jl_min, mat* jl_max);
mat rotan(mat* wp, double* len);
#endif	// IKFUNCTIONS_H

