function [ang] = find_phirange(R_norm, R_c, d_c, ak, ang_res, th1)

    %phi_max = [];
    %phi_min = [];
    
    ang = [];
    
    A1 = sqrt(R_norm(2,3)^2 + R_norm(3,3)^2);
    B1 = (ak(3) - d_c*R_norm(1,3))/R_c;
    B2 = (ak(3) + d_c*R_norm(1,3))/R_c;
    
    if abs(B1/A1) < 1 && abs(B2/A1) < 1
        a_var_p = acos(B1/A1);
        b_var_p = atan2(R_norm(2,3), R_norm(3,3));
        
        phi_str_p = a_var_p - b_var_p;
        phi_end_p = -(a_var_p + b_var_p);
        
        if phi_end_p < 0
            phi_end_p = 2*pi + phi_end_p;
        end
    
        if phi_str_p < 0
            phi_str_p = 2*pi + phi_str_p;
        end
        
        a_var_n = acos(-B2/A1);
        b_var_n = atan2(R_norm(2,3), R_norm(3,3));
        
        phi_end_n = a_var_n - b_var_n;
        phi_str_n = -(a_var_n + b_var_n);
        
        if phi_end_n < 0
            phi_end_n = 2*pi + phi_end_n;
        end
    
        if phi_str_n < 0
            phi_str_n = 2*pi + phi_str_n;
        end
        
        if phi_str_p < phi_end_p
            if phi_str_n < phi_end_n
                if phi_str_n < phi_end_p && phi_str_n > phi_str_p && phi_end_n < phi_end_p && phi_end_n > phi_str_p
                    ang = linspace(phi_str_p, phi_end_p, ceil((phi_end_p-phi_str_p)/ang_res));
                elseif phi_str_n < phi_end_p && phi_str_n > phi_str_p && phi_end_n > phi_end_p
                    ang = linspace(phi_str_p, phi_end_n, ceil((phi_end_n-phi_str_p)/ang_res));
                elseif phi_end_n > phi_str_p && phi_end_n < phi_end_p && phi_str_n < phi_str_p
                    ang = linspace(phi_str_n, phi_end_p, ceil((phi_end_p-phi_str_n)/ang_res));
                end
            elseif phi_str_n > phi_end_n
                if phi_str_n < phi_end_p && phi_str_n > phi_str_p && phi_end_n < phi_end_p && phi_end_n > phi_str_p
                    ang = linspace(0, 2*pi, ceil(2*pi/ang_res));
                elseif phi_str_n < phi_end_p && phi_str_n > phi_str_p && phi_end_n < phi_str_p
                    ang1 = linspace(phi_str_p, 2*pi, ceil((2*pi - phi_str_p)/ang_res));
                    ang2 = linspace(0, phi_end_n, cleil(phi_end_n/ang_res));
                    ang = [ang1 ang2];
                elseif phi_end_n < phi_end_p && phi_end_n > phi_str_p && phi_str_n > phi_end_p
                    ang1 = linspace(phi_str_n, 2*pi, ceil((2*pi-phi_str_n)/ang_res));
                    ang2 = linspace(0, phi_end_p, ceil(phi_end_p/ang_res));
                    ang = [ang1 ang2];
                end
            end
        elseif phi_str_p > phi_end_p
            if phi_str_n < phi_end_n
                if phi_str_n < phi_str_p && phi_str_n < phi_end_p && phi_end_n > phi_str_p
                    %ang1 = linspace(phi_str_p, 2*pi, ceil((2*pi-phi_str_p)/ang_res));
                    %ang2 = linspace(0, phi_end_p, ceil(phi_end/ang_res));
                    %ang = [ang1 ang2];
                    ang = linspace(0, 2*pi, ceil(2*pi/ang_res));
                elseif phi_str_n < phi_str_p && phi_str_n < phi_end_p && phi_end_n < phi_str_p && phi_end_n > phi_end_p
                    ang1 = linspace(phi_str_p, 2*pi, ceil((2*pi-phi_str_p)/ang_res));
                    ang2 = linspace(0, phi_end_n, ceil(phi_end_n/ang_res));
                    ang = [ang1 ang2];
                elseif phi_str_n > phi_str_p && phi_str_n > phi_end_p && phi_end_n > phi_str_p && phi_end_n > phi_end_p
                    ang1 = linspace(phi_str_p, 2*pi, ceil(2*pi-phi_str_p)/ang_res);
                    ang2 = linspace(0, phi_end_p, ceil(phi_end_p/ang_res));
                    ang = [ang1 ang2];
                end
            elseif phi_str_n > phi_end_n
                if phi_str_n > phi_end_p && phi_str_n > phi_str_p && phi_end_n < phi_str_p && phi_end_n < phi_end_p
                    ang1 = linspce(phi_str_p, 2*pi, ceil((2*pi - phi_str_n)/ang_res));
                    ang2 = linspace(0, phi_end_p, ceil(phi_end_p/ang_res));
                    ang = [ang1 ang2];
                elseif phi_str_n > phi_str_p && phi_str_n > phi_end_p && phi_end_n < phi_str_p && phi_end_n > phi_end_p
                    ang1 = linspce(phi_str_p, 2*pi, ceil((2*pi - phi_str_p)/ang_res));
                    ang2 = linspace(0, phi_end_n, ceil(phi_end_n/ang_res));
                    ang = [ang1 ang2];
                elseif phi_str_n < phi_str_p && phi_str_n > phi_end_p && phi_end_n < phi_str_p && phi_end_n < phi_end_p
                    ang1 = linspace(phi_str_p_n, 2*pi, ceil((2*pi - phi_str_n)/ang_res));
                    ang2 = linspace(0, phi_end_p, ceil(phi_end_p/ang_res));
                    ang = [ang1 ang2];
                end
            end
         end
 
    elseif abs(B1/A1) < 1    
        a_var = acos(B1/A1);
        b_var = atan2(R_norm(2,3), R_norm(3,3));
        
        phi_min = a_var - b_var;
        phi_max = -(a_var + b_var);
    
        if phi_max < 0
            phi_max = 2*pi + phi_max;
        end
    
        if phi_min < 0
            phi_min = 2*pi + phi_min;
        end
        
        if phi_max > phi_min
          ang = linspace(phi_min, phi_max, ceil((phi_max-phi_min)/ang_res));
        
        else
          ang1 = linspace(phi_min, 2*pi, ceil((2*pi - phi_min)/ang_res));
          ang2 = linspace(0, phi_max, ceil(phi_max/ang_res));
          ang = [ang1 ang2];
        end
        
    elseif abs(B2/A1) < 1
        a_var = acos(-B2/A1);
        b_var = atan2(R_norm(2,3), R_norm(3,3));
        
        phi_max = a_var - b_var;
        phi_min = -(a_var + b_var);
    
        if phi_max < 0
            phi_max = 2*pi + phi_max;
        end
    
        if phi_min < 0
            phi_min = 2*pi + phi_min;
        end
        
        if phi_max > phi_min
          ang = linspace(phi_min, phi_max, ceil((phi_max-phi_min)/ang_res));
        
        else
          ang1 = linspace(phi_min, 2*pi, ceil((2*pi - phi_min)/ang_res));
          ang2 = linspace(0, phi_max, ceil(phi_max/ang_res));
          ang = [ang1 ang2];
        end
        
    else
%         fprintf('\nNo phi range found for theta1 = %6.4f\n', th1);
%         fprintf('Program exiting . . . . \n')
%         fprintf('--------------------------------------------------\n');
          ang = [];
    end
end
