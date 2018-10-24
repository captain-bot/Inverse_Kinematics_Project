function [] = comp_angle(tol, th1, ang_res, gst_d, ak, dk, alp, bax_base, jl_min, jl_max, wrist_gbl, L1, L2, filename)
%% Start of Inverse Kinematics

% dealing with the artificial frame
% The wrist with respect to the Artificial frame

qpart = th1;
T_af_multi = zeros(4,4,2);
T_af_multi(:,:,1) = bax_base;

for i = 1:1
    T_af_multi(:,:,i+1) = T_af_multi(:,:,i)*bax_tran(ak(i), dk(i), alp(i), qpart(i), i);
end

T_af = T_af_multi(:,:,2);               % transformation of artificial frame in global frame

DWpose_af = T_af(1:3,1:3)'*wrist_gbl - T_af(1:3,1:3)'*T_af(1:3,4);
d = norm(DWpose_af);
% Check reachability for given theta1
if d >= L1 + L2
    fprintf('\nDesired configuration can not be reached for th1 = %6.4f\n', th1);
    fprintf('Program exitting . . . . \n');
    fprintf('---------------------------------------------------------\n');
    return
end

alph_2 = acos((d^2 + L2^2 - L1^2)/(2*d*L2));
alph_1 = asin(L2*sin(alph_2)/(L1));
d_c = L1*cos(alph_1);
R_c = L1*sin(alph_1);
R_norm = rotan(DWpose_af);
% [phi_max, phi_min] = find_phirange(R_norm, R_c, d_c, ak);
% ang = find_phirange(R_norm, R_c, d_c, ak, ang_res, th1);
% if isempty(ang) == 1
%     return;
% end

%ang = linspace(0, 2*pi, ceil(2*pi / ang_res));
ang = linspace(0, 2*pi, ceil((2*pi-1.3)/ang_res));

% Generate point C and rotate using R_norm in artificial frame
% if phi_max > phi_min
%     ang = linspace(phi_min, phi_max, ceil((phi_max-phi_min)/ang_res));
% else
%     ang1 = linspace(phi_min, 2*pi, ceil((2*pi - phi_min)/ang_res));
%     ang2 = linspace(0, phi_max, ceil(phi_max/ang_res));
%     ang = [ang1 ang2];
% end

% ii = 1;
    
for i_iter = 1 : length(ang) % 16447
    pt_C_fr1 = R_norm'*[d_c -R_c*sin(ang(i_iter)) R_c*cos(ang(i_iter))]';% parametrize point C in artificial frame
    % Compute theta2 and theta3
    [th2, th3] = find_ang23(ak, dk, pt_C_fr1, jl_min, jl_max);
    if isempty(th2) == 0
        for j_iter = 1 : length(th2)
            T2 = T_af_multi(:, :, 2)*bax_tran(ak(2), dk(2), alp(2), th2(j_iter), 2);
            T3 = T2*bax_tran(ak(3), dk(3), alp(3), th3(j_iter), 3);                                                   
            wrist_lcl = T3(1:3,1:3)'*wrist_gbl - T3(1:3,1:3)'*T3(1:3,4); % wrist point in frame 4
            % Compute theta4 and theta5
            [th4, th5] = find_ang45(ak, dk, wrist_lcl, jl_min, jl_max);
            if isempty(th4) == 0
                for k_iter = 1 : length(th4)
                    T4 = T3*bax_tran(ak(4), dk(4), alp(4), th4(k_iter), 4);
                    T5 = T4*bax_tran(ak(5), dk(5), alp(5), th5(k_iter), 5);                    
                    if dot(T5(1:3, 3), gst_d(1:3, 3)) < tol
                        T6_temp = T5*bax_tran(ak(6), dk(6), alp(6), 0, 6);
                        th6_1 = acos(dot(T6_temp(1:3,3), gst_d(1:3,3)));
                        th6_2 = -th6_1;
                        th6 = [th6_1 th6_2];
                        for l_iter = 1 : length(th6)
                            if th6(l_iter) <= jl_max(6) && th6(l_iter) >= jl_min(6)
                                T6 = T5*bax_tran(ak(6), dk(6), alp(6), th6(l_iter), 6);
                                th7_cos_comp = dot(T6(1:3,1), gst_d(1:3,1));
                                th7_sin_comp = dot(T6(1:3,1), gst_d(1:3,2));
                                th7 = -atan2(th7_sin_comp, th7_cos_comp);   % why -ve sign ? because of the relative orientations of the frames 6 and 7
                                %th7 = [th7_1 th7_2];
                                T7 = T6*bax_tran(ak(7), dk(7), alp(7), th7, 7);
                                if norm(T7(1:3, 4) - gst_d(1:3, 4)) < tol && norm(eye(3, 3) - gst_d(1:3, 1:3) * T7(1:3, 1:3)') < tol 
                                    fprintf('Solution found : \n');
                                    fprintf('[%6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f]\n',...
                                    th1, th2(j_iter), th3(j_iter), th4(k_iter), th5(k_iter), th6(l_iter), th7);
                                    fprintf('\nTolerance : % 6.4f\n', tol);
                                    fprintf('\nAchieved tool config : '); T7
                                    fprintf('\nDesired tool config :'); gst_d
                                    fprintf('\nAngle phi = %6.4f\n', ang(i_iter));
                                    fprintf('-----------------------------------------------------------\n');
                                    %break;
                                    fileID = fopen(filename, 'a');
                                    fprintf(fileID,'%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n',...
                                       th1, th2(j_iter), th3(j_iter), th4(k_iter), th5(k_iter), th6(l_iter), th7);
                                    fclose(fileID);
                                    return
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

end