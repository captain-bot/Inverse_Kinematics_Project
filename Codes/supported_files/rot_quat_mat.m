clc
clear all
close all

%grip_base_orient1 =[0.302256 0.553927 -0.199433 0.749688];
%grip_base_orient1 =[0.58472 -0.16545 -0.4797 0.6329];
grip_base_orient1 =[0.607702 0.378835 -0.444104 0.538473];

grip_base_quat = grip_base_orient1;

qx = grip_base_orient1(1, 1);
qy = grip_base_orient1(1, 2);
qz = grip_base_orient1(1, 3);
qw = grip_base_orient1(1, 4);

% qx = -0.0011842; qy = -0.00020302; qz = 0.38042; qw = 0.92481;
% rot_quat = [qx; qy; qz; qw];

rot_mat(1, 1) = 1 - 2*qy^2 - 2*qz^2;
rot_mat(1, 2) = 2*qx*qy - 2*qz*qw;
rot_mat(1, 3) = 2*qx*qz + 2*qy*qw;
rot_mat(2, 1) = 2*qx*qy + 2*qz*qw;
rot_mat(2, 2) = 1 - 2*qx^2 - 2*qz^2;
rot_mat(2, 3) = 2*qy*qz - 2*qx*qw;
rot_mat(3, 1) = 2*qx*qz - 2*qy*qw;
rot_mat(3, 2) = 2*qy*qz + 2*qx*qw;
rot_mat(3, 3) = 1 - 2*qx^2 - 2*qy^2;

fprintf('The rotation matrix is as following: ')
rot_mat

% rotation matrix to quaternion
quat_vec(4, 1) = 0.5*sqrt(1 + rot_mat(1, 1) + rot_mat(2, 2) + rot_mat(3, 3));
quat_vec(1, 1) = (rot_mat(3, 2) - rot_mat(2, 3))/(4*quat_vec(4, 1));
quat_vec(2, 1) = (rot_mat(1, 3) - rot_mat(3, 1))/(4*quat_vec(4, 1));
quat_vec(3, 1) = (rot_mat(2, 1) - rot_mat(1, 2))/(4*quat_vec(4, 1));
fprintf('Check rotation to quaternion: ')
quat_vec