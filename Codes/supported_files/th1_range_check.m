clc
clear all
close all

% DH parameters
ak = [0.069 0 0.069 0 0.010 0 0];
dk = [0.27035 0 0.36435 0 0.37429 0 0.254525];
alp = pi*[-1/2 1/2 -1/2 1/2 -1/2 1/2 0];

L1 = sqrt(dk(3)^2 + ak(3)^2);
L2 = sqrt(dk(5)^2 + ak(5)^2);

bax_base = [0.7071   -0.7071         0    0.0640;
            0.7071    0.7071         0    0.2590;
                 0         0    1.0000    0.1196;
                 0         0         0    1.0000];

gst_d = load('baxfkin.txt');     % desired tool frame configuration

DTpose = gst_d(1:3,4);           % desired tool position
TRz = gst_d(1:3,3);              % desired tool z axis
wrist_gbl = DTpose - TRz*dk(7);  % desired wrist position (torso frame)
wrist_bax_base = bax_base(1:3,1:3)'*wrist_gbl - bax_base(1:3,1:3)'*bax_base(1:3,4);

A_til = wrist_bax_base(1)*ak(1);
B_til = wrist_bax_base(2)*ak(1);
D_til = 0.5*(wrist_bax_base(1)^2 + wrist_bax_base(2)^2 + wrist_bax_base(3)^2 ...
    + ak(1)^2 + dk(1)^2 - 2*wrist_bax_base(3)*dk(1) - (L1 + L2)^2);
Xp = sqrt(A_til^2 + B_til^2);
%Xn = - Xp;
alph_ang = atan2(A_til, B_til);
th1_min = asin(D_til/Xp) - alph_ang;
th1_max = pi - asin(D_til/Xp) - alph_ang;

fprintf('solution will be found for %6.4f < theta_1 < %6.4f\n', th1_min, th1_max)


