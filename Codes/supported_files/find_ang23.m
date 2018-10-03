function [th2, th3] = find_ang23(ak, dk, pt_C, jl_min, jl_max)
    th2 = [];
    th3 = [];

    cx = pt_C(1, 1);
    cy = pt_C(2, 1);
    cz = pt_C(3, 1);
    
%     r = (cx + cy)/2;

    s3 = (cz - dk(2))/ak(3);
    
    if abs(s3) < 1
        th3_1 = asin(s3);
        if th3_1 < jl_max(3) && th3_1 > jl_min(3)
            th3(1) = th3_1;
            s2_1 = (dk(3)*cy - (ak(3)*cos(th3_1) + ak(2))*cx)/((ak(3)*cos(th3_1)+ak(2))^2 + dk(3)^2);
            c2_1 = (dk(3)*cx + (ak(3)*cos(th3_1) + ak(2))*cy)/((ak(3)*cos(th3_1)+ak(2))^2 + dk(3)^2);
            th2_1 = atan2(s2_1, c2_1);
            if th2_1 < jl_max(2) && th2_1 > jl_min(2)
                th2(1) = th2_1;
            end
        end
        
        th3_2 = pi - th3_1;
        if th3_2 < jl_max(3) && th3_2 > jl_min(3)
            th3(2) = th3_2;
            s2_2 = (dk(3)*cy - (ak(3)*cos(th3_2) + ak(2))*cx)/((ak(3)*cos(th3_2)+ak(2))^2 + dk(3)^2);
            c2_2 = (dk(3)*cx + (ak(3)*cos(th3_2) + ak(2))*cy)/((ak(3)*cos(th3_2)+ak(2))^2 + dk(3)^2);
            th2_2 = atan2(s2_2, c2_2);
            if th2_2 < jl_max(2) && th2_2 > jl_min(2)
                th2(2) = th2_2;
            end
        end
    end
    
    % fprintf('theta2:'); th2
    % fprintf('theta3:'); th3
    
end