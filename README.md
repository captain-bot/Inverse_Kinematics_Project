# Inverse_Kinematics_Project
I have created a C++ implementation of Baxter Inverse Kinematics(IK). In order to solve IK for a given end effector configuration one needs to do the following after cloning the repository.
```
$ cd ~/.../.../C++_Implementation
$ make
$ ./sol_baxik -0.988025628538199  0.149789985589697   0.036990776806313  0.147173250638263   0.986939845736761  -0.065496375423884  -0.046318372685826   -0.059268044628937  -0.997166940505751  0.87894511  0.10325123   -0.0384456   -1.1625   left_gripper
```
Notice that I have provided 14 arguments after the executable named **sol_bax_ik**. Below are the descriptions of inputs.
* First 9 are rotation matrix in row major format
* Next 3 are position vector
* Guessed value of ![](https://latex.codecogs.com/gif.latex?%5Ctheta_1)
* Next is the name of end effector frame of the configuration provided *left\_gripper*

After following the instruction given above, one should see the following output,
```
-1.1625  -0.0180   1.3570   0.2989  -1.4235   1.4831   0.0685
time elapsed: 0.0568677
```
