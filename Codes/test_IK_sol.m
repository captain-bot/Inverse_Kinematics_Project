clc
clear
close all

%% Slove forward kinematics
qpart = [0.5 -0.5 0.5 0.5 0.5 0.5 0.5];

% DH parameters
ak = [0.069 0 0.069 0 0.010 0 0];
dk = [0.27035 0 0.36435 0 0.37429 0 0.254525];
alp = pi*[-1/2 1/2 -1/2 1/2 -1/2 1/2 0];
bax_base = [0.7071   -0.7071         0    0.0640;
            0.7071    0.7071         0    0.2590;
                 0         0    1.0000    0.1196;
                 0         0         0    1.0000];

% DH transformations / Forward kinematics
T1 = bax_base * bax_tran(ak(1), dk(1), alp(1), qpart(1), 1);
T2 = T1*bax_tran(ak(2), dk(2), alp(2), qpart(2), 2);
T3 = T2*bax_tran(ak(3), dk(3), alp(3), qpart(3), 3);
T4 = T3*bax_tran(ak(4), dk(4), alp(4), qpart(4), 4);
T5 = T4*bax_tran(ak(5), dk(5), alp(5), qpart(5), 5);
T6_temp = T5*bax_tran(ak(6), dk(6), alp(6), 0, 6);
T6 = T5*bax_tran(ak(6), dk(6), alp(6), qpart(6), 6);
T7 = T6*bax_tran(ak(7), dk(7), alp(7), qpart(7), 7);

%% Testing
% Suppose correct position and orientation of C is known and T1 is also 
% known. We want to see whether we get back correct theta_2 and theta_3
pt_C_gbl = T3(1:3,4);
pt_C_lcl = T1(1:3,1:3)'*pt_C_gbl - T1(1:3,1:3)'*T1(1:3,4);  % point C in frame 2

% Compute theta2 and theta3
[th2, th3] = find_ang23(ak, dk, pt_C_lcl);

% Suppose correct position of D (wrist) is known and T3 is also known
% We want to see whether we get back correct theta_4 and theta_5
wrist_gbl = T5(1:3,4);
wrist_lcl = T3(1:3,1:3)'*wrist_gbl - T3(1:3,1:3)'*T3(1:3,4); % wrist point in frame 4

% Compute theta4 and theta5
[th4, th5] = find_ang45(ak, dk, wrist_lcl);

% Suppose angles theta4 and theta5 are known. Then T5 is known also.
% We can also compute T6_temp.
th6 = acos(dot(T6_temp(1:3,3), T7(1:3,3)));

% Supoose we know theta6. T6 is also known. T7 is end effector transform.
th7 = acos(dot(T6(1:3,1), T7(1:3,1)));

% equations for th2 and th3
%--------------------------------
fprintf('\nChecking validity for th2 and th3\n\n')
if abs(-(ak(3)*cos(th3(1))+ak(2))*sin(th2(1)) + dk(3)*cos(th2(1)) - pt_C_lcl(1,1)) < 1e-5
    fprintf('1st eq satisfied\n')
end

if abs((ak(3)*cos(th3(1))+ak(2))*cos(th2(1)) + sin(th2(1))*dk(3) - pt_C_lcl(2,1)) < 1e-5
    fprintf('2nd eq satisfied\n');
end

if abs(ak(3)*sin(th3(1)) + dk(2) - pt_C_lcl(3,1)) < 1e-5
    fprintf('3rd eq satisfied\n');
end

fprintf('\nChecking validity for th4 and th5\n\n')
if abs(dk(5)*sin(th4(1)) + (ak(5)*cos(th5(1)) + ak(4))*cos(th4(1)) - wrist_lcl(1,1)) < 1e-5
    fprintf('1st eq satisfied\n');
end

if abs((ak(5)*cos(th5(1)) + ak(4))*sin(th4(1)) - dk(5)*cos(th4(1)) - wrist_lcl(2,1)) < 1e-5
    fprintf('2nd eq satisfied\n')
end

if abs(ak(5)*sin(th5(1)) + dk(4) - wrist_lcl(3,1)) < 1e-5
    fprintf('3rd eq satisfied\n');
end

fprintf('\nChecking the joint angles\n\n')
if abs(th2(1) - qpart(2)) < 1e-4
    fprintf('Computed theta2 correctly\n')
end

if abs(th3(1) - qpart(3)) < 1e-4
    fprintf('Computed theta3 correctly\n')
end

if abs(th4(1) - qpart(4)) < 1e-4
    fprintf('Computed theta4 correctly\n')
end

if abs(th5(1) - qpart(5)) < 1e-4
    fprintf('Computed theta5 correctly\n')
end

if abs(th6 - qpart(6)) < 1e-4
    fprintf('Computed theta6 correctly\n')
end

if abs(th7 - qpart(7)) < 1e-4
    fprintf('Computed theta7 correctly\n')
end