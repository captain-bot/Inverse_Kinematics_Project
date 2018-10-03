 clc
clear
close all

%% Solution tolerance
tol = 1e-4;

%% Control parameters of the code
th1 = 0.5;
ang_res = 0.00001;
npts1 = 100000;
% npts2 = 0.5*npts1;

%% Desired configuration
gst_d = load('baxfkin.txt');     % desired tool frame configuration

%% Slove forward kinematics
% DH parameters
ak = [0.069 0 0.069 0 0.010 0 0];
dk = [0.27035 0 0.36435 0 0.37429 0 0.254525];
alp = pi*[-1/2 1/2 -1/2 1/2 -1/2 1/2 0];

% base transformations
bax_base = [0.7071   -0.7071         0    0.0640;
            0.7071    0.7071         0    0.2590;
                 0         0    1.0000    0.1196;
                 0         0         0    1.0000];
% joint limits
jl_min = [-1.7016 -2.147 -3.0541 -0.05 -3.059 -1.5707 -3.059];
jl_max = [1.7016 1.047 3.0541 2.618 3.059 2.094 3.059];

%% Start of Inverse Kinematics
DTpose = gst_d(1:3,4);           % desired tool position
TRz = gst_d(1:3,3);              % desired tool z axis
wrist_gbl = DTpose - TRz*dk(7);  % desired wrist position (torso frame)

L1 = sqrt(dk(3)^2 + ak(3)^2);
L2 = sqrt(dk(5)^2 + ak(5)^2);

% dealing with the artificial frame
% The wrist with respect to the Artificial frame
qpart = [th1 0];
T_af_multi = zeros(4,4,3);
T_af_multi(:,:,1) = bax_base;

for i = 1:2
    T_af_multi(:,:,i+1) = T_af_multi(:,:,i)*bax_tran(ak(i), dk(i), alp(i), qpart(i), i);
end

T_af = T_af_multi(:,:,2);               % transformation of artificial frame in global frame

DWpose_af = T_af(1:3,1:3)'*wrist_gbl - T_af(1:3,1:3)'*T_af(1:3,4);
d = norm(DWpose_af);

% Check reachability for given theta1
if d >= L1 + L2
    fprintf('Desired configuration can not be reached\n');
    fprintf('\nProgram exitting . . . . \n');
    return
end

alph_2 = acos((d^2 + L2^2 - L1^2)/(2*d*L2));
alph_1 = asin(L2*sin(alph_2)/(L1));
d_c = L1*cos(alph_1);
R_c = L1*sin(alph_1);
R_norm = rotan(DWpose_af);
[ang] = find_phirange(R_norm, R_c, d_c, ak, ang_res);
if isempty(ang) == 1
    fprintf('No solution exists for theta1 = %6.4f\n\n', th1);
    fprintf('Program exiting . . . . \n')
    return
end
    
ang = linspace(2.0664, 2*pi, npts1);
tic;

for i_iter = 1:length(ang) % 16447
    pt_C_fr1 = R_norm'*[d_c -R_c*sin(ang(i_iter)) R_c*cos(ang(i_iter))]';% parametrize point C in artificial frame
    % Compute theta2 and theta3
    [th2, th3] = find_ang23(ak, dk, pt_C_fr1, jl_min, jl_max);
    if isempty(th2) == 0
        for j_iter = 1 : length(th2)
            %fprintf('th2 = %6.4f,\tth3 = %6.4f\n\n', th2(1), th3(1));
            T2 = T_af_multi(:, :, 2)*bax_tran(ak(2), dk(2), alp(2), th2(j_iter), 2);
            T3 = T2*bax_tran(ak(3), dk(3), alp(3), th3(j_iter), 3);
            %wrist_gbl = DWpose;                                         % wrist in global frame
            wrist_lcl = T3(1:3,1:3)'*wrist_gbl - T3(1:3,1:3)'*T3(1:3,4); % wrist point in frame 4
            % Compute theta4 and theta5
            [th4, th5] = find_ang45(ak, dk, wrist_lcl, jl_min, jl_max);
            if isempty(th4) == 0
                for k_iter = 1 : length(th4)
                    T4 = T3*bax_tran(ak(4), dk(4), alp(4), th4(k_iter), 4);
                    T5 = T4*bax_tran(ak(5), dk(5), alp(5), th5(k_iter), 5);
                    %fprintf('achieved wrist pose'); T5(1:3,4);
                    T6_temp = T5*bax_tran(ak(6), dk(6), alp(6), 0, 6);
                    th6_1 = acos(dot(T6_temp(1:3,3), gst_d(1:3,3)));
                    %th6_2 = -th6_1;
                    th6 = [th6_1 -th6_1];
                    for l_iter = 1 : length(th6)
                        if th6(l_iter) <= jl_max(6) && th6(l_iter) >= jl_min(6)
                            T6 = T5*bax_tran(ak(6), dk(6), alp(6), th6(l_iter), 6);
                            th7_1 = acos(dot(T6(1:3,1), gst_d(1:3,1)));
                            th7_2 = -th7_1;
                            th7 = [th7_1 th7_2];
                            T7 = T6*bax_tran(ak(7), dk(7), alp(7), th7(1), 7);
                            if norm(T7(1:3, 4) - gst_d(1:3, 4)) < tol
                                fprintf('Solution found : \n');
                                fprintf('[%6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f]\n',...
                                    th1, th2(1), th3(1), th4(1), th5(1), th6(1), th7(1));
                                fprintf('\nTolerance : % 6.4f\n', tol);
                                fprintf('\nAchieved tool config : '); T7
                                fprintf('\nDesired tool config :'); gst_d
                                toc;
                                fprintf('-----------------------------------------------------------\n');
                                return
                            end
                        end
                    end
                end
            end
        end
    end
end