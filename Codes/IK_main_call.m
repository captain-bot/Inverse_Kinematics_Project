clc
clear
close all

%% Add function files path
addpath supported_files

%% Solution tolerance
tol = 1e-4;

%% Control parameters of the code
%npts = 50;               % number of discrete theta1
c_ang_res = tol*1e-2;     % resolution of phi angle
th1_res = 0.01;           % resolution of theta1 angle

%% Desired configuration
% gst_d = load('read_write_files/ee_configuration5.txt');     % desired tool frame configuration

p = [0.675, 0.225, 0.130];  % IKFast fails
R = quat2rotm([0, 0, 1, 0]);
gst_d = [R p'; 0 0 0 1];

%% Opening a file to write joint solutions
filename = sprintf('read_write_files/%s_%d.txt','IKsol',30);
fileID = fopen(filename, 'w');
fclose(fileID);

%% Slove forward kinematics
% DH parameters
ak = [0.069 0 0.069 0 0.010 0 0];
% dk = [0.27035 0 0.36435 0 0.37429 0 0.254525+0.150];  % add 0.15 to match left_gripper transform
dk = [0.27035 0 0.36435 0 0.37429 0 0.2295+0.150];
alp = pi*[-1/2 1/2 -1/2 1/2 -1/2 1/2 0];

% base transformations
bax_base = [0.7071   -0.7071         0    0.0640;
            0.7071    0.7071         0    0.2590;
                 0         0    1.0000    0.1296;
                 0         0         0    1.0000];

% joint limits
jl_min = [-1.7016 -2.147 -3.0541 -0.05 -3.059 -1.5707 -3.059];
jl_max = [1.7016 1.047 3.0541 2.618 3.059 2.094 3.059];

% wrist
DTpose = gst_d(1:3,4);           % desired tool position
TRz = gst_d(1:3,3);              % desired tool z axis
wrist_gbl = DTpose - TRz*dk(7);  % desired wrist position (torso frame)

L1 = sqrt(dk(3)^2 + ak(3)^2);
L2 = sqrt(dk(5)^2 + ak(5)^2);

% range of theta1
wrist_bax_base = bax_base(1:3,1:3)'*wrist_gbl - bax_base(1:3,1:3)'*bax_base(1:3,4);

A_til = wrist_bax_base(1)*ak(1);
B_til = wrist_bax_base(2)*ak(1);
D_til = 0.5*(wrist_bax_base(1)^2 + wrist_bax_base(2)^2 + wrist_bax_base(3)^2 ...
    + ak(1)^2 + dk(1)^2 - 2*wrist_bax_base(3)*dk(1) - (L1 + L2)^2);
Xp = sqrt(A_til^2 + B_til^2);
alph_ang = atan2(A_til, B_til);
th1_min = asin(D_til/Xp) - alph_ang;
th1_max = pi - asin(D_til/Xp) + alph_ang;

if th1_min < jl_min(1)
    th1_min = jl_min(1);
end

if th1_max > jl_max(1)
    th1_max = jl_max(1);
end

fprintf('Solution will be found for %6.4f < theta_1 < %6.4f\n\n', th1_min, th1_max);
fprintf('Starting IK routine for the above range for theta1 . . . .\n\n');
fprintf('------------------------------------------------------------\n');

% th1_min = -0.45; th1_max = 1.0009;
% th1_min = -1.15; th1_max = -0.7;
% th1_min = -1.6; th1_max = -0.085;   % counter IKFast
% th1_range = linspace(th1_min, th1_max, ceil((th1_max - th1_min)/th1_res));
th1_range = -0.5867;
tic;
for i = 1 : length(th1_range)
%     comp_angle(tol, -1.1525, c_ang_res, gst_d, ak, dk, alp, bax_base, jl_min, jl_max, wrist_gbl, L1, L2, filename);
   comp_angle(tol, th1_range(i), c_ang_res, gst_d, ak, dk, alp, bax_base, jl_min, jl_max, wrist_gbl, L1, L2, filename);
end
toc;