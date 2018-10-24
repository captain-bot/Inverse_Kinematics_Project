/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "IKfunctions.h"

// Define constructor
solveBaxIK::solveBaxIK() : start(true) {};
solveBaxIK::solveBaxIK(bool val) : start(val) {};

// Computes local DH transformation of Baxter robot
mat bax_tran(double a, double d, double alpha, double th, int idx){
    if (idx == 1) th += M_PI_2;
    mat trans =   {{std::cos(th), -std::cos(alpha)*std::sin(th), std::sin(alpha)*std::sin(th),  a*std::cos(th)},
              {std::sin(th), std::cos(alpha)*std::cos(th), -std::sin(alpha)*std::cos(th),  a*std::sin(th)},
              {0, std::sin(alpha), std::cos(alpha), d},
              {0, 0, 0, 1}};
    return trans;
}

// Rotates points from its original position on to Z3 axis
mat rotan(mat* wp, double* len) {
    mat PP2 = {*len, 0, 0};
    mat pr_vec = cross(*wp, PP2);
    pr_vec /= norm(pr_vec);
    mat om_hat = {{0, -pr_vec.at(2), pr_vec.at(1)}, 
                  {pr_vec.at(2), 0, -pr_vec.at(0)},
                  {-pr_vec.at(1), pr_vec.at(0), 0}};
    double th = std::acos(wp->at(0)/(*len));
    return eye<mat>(3,3) + std::sin(th)*om_hat + (1 - std::cos(th))*(om_hat*om_hat);
}

// computes IK solutions for Baxter robot
void solveBaxIK::computeBaxIK(double* th1, mat* des_p, mat* des_R, double* end_lnk) {
    wall_clock timer;
    
    /*Solution tolerance*/
    double tol = 1e-1;
    
    /*User defined control parameters*/
    double c_ang_res = tol * 1e-2;
    double th1_res = 0.01;
    
    /*Discretize redundancy circle*/
    int npts = ceil(2*M_PI/c_ang_res);
    vec discrete_ang = linspace<vec>(0,2*M_PI,npts);
    
    /*DH parameters of Baxter robot*/    
    mat ak = {0.069, 0.0, 0.069, 0.0, 0.010, 0.0, 0.0};
    mat dk = {0.27035, 0.0, 0.36435, 0.0, 0.37429, 0.0, 0.254525+(*end_lnk)};
    mat alp = {-M_PI_2, M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, 0};

    /*Base transformation*/
    mat bax_base = {{0.7071, -0.7071, 0.0, 0.0640},
                    {0.7071, 0.7071, 0.0, 0.2590},
                    {0.0, 0.0, 1.0, 0.1296},
                    {0.0, 0.0, 0.0, 1.0}};
    
    /*Joint limits*/
    mat jl_min = {-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059};
    mat jl_max = {1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094, 3.059};
    
    /*Wrist*/
    mat DTpose = des_p->t();
    mat TRz = des_R->col(2);
    mat wrist_gbl = DTpose - TRz*dk.col(6);
    
    double L1 = sqrt(pow(dk.at(2),2) + pow(ak.at(2),2));
    double L2 = sqrt(pow(dk.at(4),2) + pow(ak.at(4),2));
    
    /* Call compIK method*/
    int iter_val;
    bool sol_flag;
    mat sol_vec = zeros(1,7);
    sol_vec.at(0) = *th1;
    timer.tic();
    for (int i = 0; i < 4; i++) {
        sol_flag = comp_angle(&sol_vec, &iter_val, &tol, &*th1, &*des_p, &*des_R, &ak, &dk, &alp, &bax_base, &jl_min, &jl_max, &wrist_gbl, &L1, &L2, &discrete_ang, &npts);
        if(sol_flag) {
            tol *= 0.1;
            discrete_ang = linspace<vec>(discrete_ang.at(iter_val-100),discrete_ang.at(iter_val+100),npts);        
        }
        else if (!sol_flag) {
            /*To do*/
            /*You may want to increase the search range or make search grid denser*/
            cout << "no progress made" << endl;
        }
        else {
            cout << "error occurred" << endl;
        }
    }
    double elapsedt = timer.toc();
    sol_vec.print();
    cout << "time elapsed: " << elapsedt << endl;
}

// Compute angle 2 and angle 3
mat find_ang23(int* sol_exist, mat* ak, mat* dk, mat* pt_C, mat* jl_min, mat* jl_max){
    double s3 = (pt_C->at(2) - dk->at(1))/ak->at(2);
    mat ang_set = {0, 0};
    if (fabs(s3) < 1) {
        double th3_1 = std::asin(s3);
        if (th3_1 < jl_max->at(2) && th3_1 > jl_min->at(2)) {
            double square_val = pow(ak->at(2)*std::cos(th3_1)+ak->at(1),2) + pow(dk->at(2),2);
            double s2_1 = (dk->at(2)*pt_C->at(1) - (ak->at(2)*std::cos(th3_1) + ak->at(1))*pt_C->at(0))/square_val;
            double c2_1 = (dk->at(2)*pt_C->at(0) + (ak->at(2)*std::cos(th3_1) + ak->at(1))*pt_C->at(1))/square_val;
            double th2_1 = atan2(s2_1,c2_1);
            if (th2_1 < jl_max->at(1) && th2_1 > jl_min->at(1)) {
                ang_set = {th2_1, th3_1};
                *sol_exist += 1;
            }
        }
        double th3_2 = M_PI - th3_1;
        if (th3_2 < jl_max->at(2) && th3_2 > jl_min->at(2)) {
            double square_val = pow(ak->at(2)*std::cos(th3_2)+ak->at(1),2) + pow(dk->at(2),2);
            double s2_2 = (dk->at(2)*pt_C->at(1) - (ak->at(2)*std::cos(th3_2) + ak->at(1))*pt_C->at(0))/square_val;
            double c2_2 = (dk->at(2)*pt_C->at(0) + (ak->at(2)*std::cos(th3_2) + ak->at(1))*pt_C->at(1))/square_val;
            double th2_2 = atan2(s2_2,c2_2);
            if (th2_2 < jl_max->at(1) && th2_2 > jl_min->at(1)) {
                mat ang_set2 = {th2_2, th3_2};
                ang_set = join_vert(ang_set, ang_set2);
                *sol_exist += 1;
            }
        }
        return ang_set;
    }
}

// Compute angle 4 and angle 5
mat find_ang45(int* sol_exist, mat* ak, mat* dk, mat* pt_W, mat* jl_min, mat* jl_max){
    double s5 = (pt_W->at(2) - dk->at(3))/ak->at(4);
    mat ang_set = {0, 0};
    if (fabs(s5) < 1) {
        double th5_1 = std::asin(s5);
        if (th5_1 < jl_max->at(4) && th5_1 > jl_min->at(4)) {
            double square_val = pow(ak->at(4)*std::cos(th5_1)+ak->at(3),2) + pow(dk->at(4),2);
            double s4_1 = (dk->at(4)*pt_W->at(0) + (ak->at(4)*std::cos(th5_1) + ak->at(3))*pt_W->at(1))/square_val;
            double c4_1 = ((ak->at(4)*std::cos(th5_1) + ak->at(3))*pt_W->at(0) - dk->at(4)*pt_W->at(1))/square_val;
            double th4_1 = atan2(s4_1,c4_1);
            if (th4_1 < jl_max->at(3) && th4_1 > jl_min->at(3)) {
                ang_set = {th4_1, th5_1};
                *sol_exist += 1;
            }
        }
        double th5_2 = M_PI - th5_1;
        if (th5_2 < jl_max->at(4) && th5_2 > jl_min->at(4)) {
            double square_val = pow(ak->at(4)*std::cos(th5_2)+ak->at(3),2) + pow(dk->at(4),2);
            double s4_2 = (dk->at(4)*pt_W->at(0) + (ak->at(4)*std::cos(th5_2) + ak->at(3))*pt_W->at(1))/square_val;
            double c4_2 = ((ak->at(4)*std::cos(th5_2) + ak->at(3))*pt_W->at(0) - dk->at(4)*pt_W->at(1))/square_val;
            double th4_2 = atan2(s4_2,c4_2);
            if (th4_2 < jl_max->at(3) && th4_2 > jl_min->at(3)) {
                mat ang_set2 = {th4_2, th5_2};
                ang_set = join_vert(ang_set, ang_set2);
                *sol_exist += 1;
            }
        }
        return ang_set;
    }
}



// Computes angle 2 to 7
bool comp_angle(mat* solv, int* iter_val, double* tol, double* th1, mat* pd, mat* rd, mat* ak, mat* dk, mat* alpk, mat* bax_base, mat* jl_min, mat* jl_max, mat* wrist_gbl, double* L1, double* L2, vec* dcrt_vec, int* npts) {
    mat T_af = (*bax_base) * bax_tran(ak->at(0), dk->at(0), alpk->at(0), *th1, 0);
    mat DWpose_af = T_af(span(0,2),span(0,2)).t() * (*wrist_gbl) - T_af(span(0,2),span(0,2)).t()*T_af(span(0,2),span(3,3));
    double d = norm(DWpose_af, 2);
    
    /*Check reachability to desired position*/
    if (d >= (*L1)+(*L2)) {
        cout << "Desired configuration can not be reached for th1 = " << th1 << endl;
        cout << "Going to check next theta1 value" << endl;
        return false;
    }
    
    /*Compute parameter values of the triangle formed*/
    double alpha_2 = std::acos((pow(d,2) + pow(*L2,2) - pow(*L1,2))/(2*d*(*L2)));
    double alpha_1 = std::asin((*L2)*std::sin(alpha_2)/(*L1));
    double d_c = (*L1)*std::cos(alpha_1);
    double R_c = (*L1)*std::sin(alpha_1);
    mat R_norm = rotan(&DWpose_af, &d);
    
    /*Compute theta2 through theta7*/
    for (int i_iter = 0; i_iter < *npts; i_iter++) {
        mat pt_C_fr1 = R_norm.t()*mat({d_c, -R_c*std::sin(dcrt_vec->at(i_iter)), R_c*std::cos(dcrt_vec->at(i_iter))}).t();
        //Compute th2 & th3
        int sol_exist1 = 0;
        mat sol23 = find_ang23(&sol_exist1, ak, dk, &pt_C_fr1, jl_min, jl_max);
        if (sol_exist1 != 0) {
            for (int j_iter = 0; j_iter < sol_exist1; j_iter++) {
               mat T3 = T_af * bax_tran(ak->at(1), dk->at(1), alpk->at(1), sol23(j_iter, 0), 1) * bax_tran(ak->at(2), dk->at(2), alpk->at(2), sol23(j_iter, 1), 2);
               mat wrist_lcl = T3(span(0,2),span(0,2)).t() * (*wrist_gbl - T3(span(0,2),span(3,3)));
               //Compute th4 & th5
               int sol_exist2 = 0;
               mat sol45 = find_ang45(&sol_exist2, ak, dk, &wrist_lcl, jl_min, jl_max);
               if (sol_exist2 != 0) {
                   for (int k_iter = 0; k_iter < sol_exist2; k_iter++){
                       mat T5 = T3 * bax_tran(ak->at(3), dk->at(3), alpk->at(3), sol45(k_iter, 0), 3) * bax_tran(ak->at(4), dk->at(4), alpk->at(4), sol45(k_iter, 1), 4);
                       if (dot(T5(span(0,2),span(2,2)), rd->col(2)) < *tol) {
                           mat T6_temp = T5 * bax_tran(ak->at(5), dk->at(5), alpk->at(5), 0, 5);
                           double th6_1 = std::acos(dot(T6_temp(span(0,2), span(2,2)),rd->col(2)));
                           double th6_2 = -th6_1;
                           mat th6 = {th6_1, th6_2};
                           for (int l_iter=0; l_iter < 2; l_iter++) {
                               if (th6.at(l_iter) < jl_max->at(5) && th6.at(l_iter) > jl_min->at(5)){
                                   mat T6 = T5*bax_tran(ak->at(5), dk->at(5), alpk->at(5), th6.at(l_iter), 5);
                                   double th7 = -atan2(dot(T6(span(0,2),span(0,0)),rd->col(1)), dot(T6(span(0,2),span(0,0)),rd->col(0)));
                                   if (th7 < jl_max->at(6) && th7 > jl_min->at(6)){
                                       mat T7 = T6 * bax_tran(ak->at(6), dk->at(6), alpk->at(6), th7, 7);
                                       if (norm(T7(span(0,2),span(3,3))-(pd->t())) < *tol && norm(eye<mat>(3,3) - (*rd)*T7(span(0,2),span(0,2)).t()) < *tol){
                                           *iter_val = i_iter;                                           
                                           T7.print();
                                           solv->at(1) = sol23(j_iter, 0); 
                                           solv->at(2) = sol23(j_iter, 1); 
                                           solv->at(3) = sol45(k_iter, 0); 
                                           solv->at(4) = sol45(k_iter, 1);
                                           solv->at(5) = th6.at(l_iter);
                                           solv->at(6) = th7;
                                           return true;
                                       }
                                   }
                               }
                           }
                       }
                   }
               }
            }
        }
    }
    return false;
}