/* 
 * File:   my_baxter_IK.cpp
 * Author: nobug-ros
 * Topic: BaxterIK
 * Created on October 13, 2018, 6:18 PM
 */

#include "IKfunctions.h"

int main(int argc, char** argv) {
    
    if (argc==15) {
        /*Desired End Effector Configuration*/
        mat des_R = {{atof(argv[1]),atof(argv[2]),atof(argv[3])},
                     {atof(argv[4]),atof(argv[5]),atof(argv[6])}, 
                     {atof(argv[7]),atof(argv[8]),atof(argv[9])}};

        mat des_p = {atof(argv[10]),atof(argv[11]),atof(argv[12])};

        /*Set desired theta1 value*/
        double theta_1 = atof(argv[13]);

        /*Set end frame*/
        string end_frame = argv[14];
        cout << "end_frame: " << end_frame << endl;
        
        /*Frame about which desired SE(3) configuration is given*/
        double exten_link = 0;
        if (end_frame.compare("left_gripper")==0) {
            exten_link = 0.150;
        }
        else if (end_frame.compare("left_gripper_base")==0) {
            exten_link = 0.0;
        }
        else {
            cout << "No match for frame: " << end_frame << endl;
            cout << "Exit . . ." << endl;
        }
        
        /*Call computeBaxIK()*/
        solveBaxIK a(true);
        a.computeBaxIK(&theta_1, &des_p, &des_R, &exten_link);
    }
    else {
        std::cout << "Not enough input argument provided\n" << std::endl;
    }    
    return 0;
}