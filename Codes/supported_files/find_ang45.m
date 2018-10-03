function [th4, th5] = find_ang45(ak, dk, pt_W, jl_min, jl_max)
    th4 = [];
    th5 = [];
    
    wx = pt_W(1, 1);
    wy = pt_W(2, 1);
    wz = pt_W(3, 1);
    
    s5 = (wz-dk(4))/ak(5);
    
    if abs(s5) < 1        
        th5_1 = asin(s5);
        if th5_1 < jl_max(5) && th5_1 > jl_min(5)
            th5(1) = th5_1;
            s4_1 = (wx*dk(5) + wy*(ak(5)*cos(th5_1) + ak(4)))/((ak(5)*cos(th5_1) + ak(4))^2 + dk(5)^2);
            c4_1 = (wx*(ak(5)*cos(th5_1) + ak(4)) - wy*dk(5))/((ak(5)*cos(th5_1) + ak(4))^2 + dk(5)^2);
            th4_1 = atan2(s4_1, c4_1);
            if th4_1 < jl_max(4) && th4_1 > jl_min(4)
                th4(1) = th4_1;
            end
        end
        
        th5_2 = pi - th5_1;
        if th5_2 < jl_max(5) && th5_2 > jl_min(5)
            th5(2) = th5_2;
            s4_2 = (wx*dk(5) + wy*(ak(5)*cos(th5_2) + ak(4)))/((ak(5)*cos(th5_2) + ak(4))^2 + dk(5)^2);
            c4_2 = (wx*(ak(5)*cos(th5_2) + ak(4)) - wy*dk(5))/((ak(5)*cos(th5_2) + ak(4))^2 + dk(5)^2);
            th4_2 = atan2(s4_2, c4_2);
            if th4_2 < jl_max(4) && th4_2 > jl_min(4)
                th4(2) = th4_2;
            end
        end   
    end   
end