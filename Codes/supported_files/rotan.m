function [Rot_mat] = rotan(wristpose)

    % ROTAN is the function returns the rotation matrix that rotates points
    % from its original position on to z3 axis

    PP1 = wristpose;                      % initial vector   
%     PP2 = [0; 0; norm(PP1)];            % rotated vector
    PP2 = [norm(PP1); 0; 0];

    pr_vec = cross(PP1, PP2);             % vector perp to PP1 and PP2
    pr_vec = pr_vec/norm(pr_vec);         % unit vector perp to PP1 and PP2
    om_hat = [0 -pr_vec(3) pr_vec(2);     % axis of rotation (skew symmetric matrix form)
              pr_vec(3) 0 -pr_vec(1);
              -pr_vec(2) pr_vec(1) 0];

    aval = dot(PP1, PP2);      
    bval = norm(PP1)*norm(PP2);

    th = acos(aval/bval);                 % angle of rotation

    Rot_mat = eye(3,3) + sin(th)*om_hat + (1 - cos(th))*(om_hat^2); % rotation matrix
    
end
