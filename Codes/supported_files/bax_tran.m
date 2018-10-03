function[trans_mat] = bax_tran(a, d, alp, ang, i)
if i == 2
    ang = ang + pi/2;
end
trans_mat = [cos(ang) -cos(alp)*sin(ang) sin(alp)*sin(ang)  a*cos(ang);
             sin(ang) cos(alp)*cos(ang) -sin(alp)*cos(ang)  a*sin(ang);
             0 sin(alp) cos(alp) d;
             0 0 0 1];
end