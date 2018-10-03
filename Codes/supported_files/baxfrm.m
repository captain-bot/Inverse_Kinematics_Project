function [] = baxfrm()
% This function is useful for generating new desired configurations and
% to be written into a text file which can be loaded to the main file of
% inverse kinematics code.

% All baxter robot frames can be drawn by calling this function

% The frames are drawn using matlab's QUIVER function

% The baxter robot model is drawn using matlab's robotics toolbox
%%  DH parameters
% the DH parameters
ak = [0.069 0 0.069 0 0.010 0 0];
alpk = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];
dk = [0.27035 0 0.36435 0 0.37429 0 0.254525];
% R = [0.7106 -0.7036 -0.0013; 0.7036 0.7106 0.0020; -0.0005 -0.0023 1.000];
% p = [0.025447; 0.21903; 0.10798];
% n = 500;
th1 = 0.5;

%% Defining the links for toolbox
% Define the robot model using standard (not modified) DH parameters.
% Left Arm

%             Theta d     a             alpha r/p  theta offset
Ll(1) = Link ([0    dk(1)  ak(1)      -pi/2  0    0]);           % start at joint s0 and move to joint s1
Ll(2) = Link ([0    0        0         pi/2  0    pi/2]);        % start at joint s1 and move to joint e0
Ll(3) = Link ([0    dk(3)  ak(3)      -pi/2  0    0]);           % start at joint e0 and move to joint e1
Ll(4) = Link ([0    0        0         pi/2  0    0]);           % start at joint e1 and move to joint w0
Ll(5) = Link ([0    dk(5)  ak(5)      -pi/2  0    0]);           % start at joint w0 and move to joint w1
Ll(6) = Link ([0    0        0         pi/2  0    0]);           % start at joint w1 and move to joint w2
Ll(7) = Link ([0    dk(7)    0           0   0    0]);           % start at joint w2 and move to end-effector

%% Create Robots
% Define the base of the left arm of the robot
% Create the Robots Baxter_L(left arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.108588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038))
% Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
%                      trotz(pi/4)*transl(0.056, 0, 0)*transl(0, 0, 0.011))

%% Define qs
% Vectors that contain joint angles to test the Baxter models
%q1     = [th1        -pi/6      pi/12        pi/4       pi/6         pi/4        pi/4];
%q1     = [th1        -pi/6      pi/6        pi/6       pi/6         pi/6        pi/6];
%q1 = [th1 -0.6513 -0.0286 0.3225 -0.5375 0 0];
%q1 = [0.5 -0.4905 -0.1358 0 0 0 0];
q1 = [th1 -0.5 0.5 0.5 0.5 0.5 0.5];
%q1 = zeros(1,7);
%q1 =[-0.730174 -0.668815 -0.014189 1.043490 -0.033747 1.012810 -0.171422];
% determine which q is being used
q_l = q1;
fprintf('Forward kinematics sloved using toolbox')

%% Forward kinematics using toolbox
% this is for understanding the rvc module
Bax_fkin = Baxter_l.fkine(q_l)

%% writing the forward kinematics matrix in a txt file
%fileID = fopen('~/Desktop/ICRA2017Code/baxfkin.txt','w');
fileID = fopen('baxfkin.txt','w');
fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f\n', Bax_fkin');
fclose(fileID);

%% Plot Baxter
% Only one arm can be plotted with the current update. A ticket needs to be
% made to fix this.
figure(1)
clf;
hold on
Baxter_l.plot(q_l)
view(57,23)

%% DH Transformations
% make the Transformation matricies that will tranform from the DH frame to
% the Base frame. They are the same for both the left and right
DOF = Baxter_l.n;
T_dh_to_b = NaN(4,4,DOF + 1);
T_dh_to_b(:,:,1) = Baxter_l.base;
rz = [cos(-pi/2) -sin(-pi/2) 0 0; sin(-pi/2) cos(-pi/2) 0 0; 0 0 1 0; 0 0 0 1];  

for i = 1:DOF
   T_dh_to_b(:,:,i+1) = T_dh_to_b(:,:,i) * Ll(i).A(q_l(i));
end
%T_art = rz*T_dh_to_b(:,:,3);
T_art = T_dh_to_b(:,:,3);          % transformation of artificial frame in global frame
T_art(1:3,1) = T_dh_to_b(1:3,2,3);
T_art(1:3,2) = -T_dh_to_b(1:3,1,3);
C_lcl = T_art\T_dh_to_b(:,4,4);
%norm(C_lcl(1:3,1))
Baxter_l.n;
fprintf('Forward kinematics sloved mechanically')
% T_dh_to_b
forward = T_dh_to_b(:,:,DOF+1)

%% Plot the frames for i = 1:7
figure(2)

for i = 1:DOF+1
     T_dh_to_b(:,:,i);
     quiver3(T_dh_to_b(1,4,i), T_dh_to_b(2,4,i), T_dh_to_b(3,4,i), T_dh_to_b(1,1,i), T_dh_to_b(2,1,i), T_dh_to_b(3,1,i),0.15,'r','filled','Linewidth',1);
     quiver3(T_dh_to_b(1,4,i), T_dh_to_b(2,4,i), T_dh_to_b(3,4,i), T_dh_to_b(1,2,i), T_dh_to_b(2,2,i), T_dh_to_b(3,2,i),0.15,'g','filled','Linewidth',1);
     quiver3(T_dh_to_b(1,4,i), T_dh_to_b(2,4,i), T_dh_to_b(3,4,i), T_dh_to_b(1,3,i), T_dh_to_b(2,3,i), T_dh_to_b(3,3,i),0.15,'b','filled','Linewidth',1);
     hold on
end

% plot the torso frame
quiver3(0,0,0,1,0,0,0.15,'r','filled','Linewidth',1);
quiver3(0,0,0,0,1,0,0.15,'g','filled','Linewidth',1);
quiver3(0,0,0,0,0,1,0.15,'b','filled','Linewidth',1);

% plot artificial frame
% quiver3(T_art(1,4), T_art(2,4), T_art(3,4), T_art(1,1), T_art(2,1), T_art(3,1),0.15,'r','filled','Linewidth',1);
% quiver3(T_art(1,4), T_art(2,4), T_art(3,4), T_art(1,2), T_art(2,2), T_art(3,2),0.15,'g','filled','Linewidth',1);
% quiver3(T_art(1,4), T_art(2,4), T_art(3,4), T_art(1,3), T_art(2,3), T_art(3,3),0.15,'b','filled','Linewidth',1);

view(95,90)
xlim([-1.5 1.5])
ylim([-1.5 1.5])
zlim([-2 2])
title('Baxter robot frames (Torso - Tool)')
xlabel('x')
ylabel('y')
zlabel('z')
hold on
end


%     if i == 3
%         T_dh_to_b(:,:,i+1) = rz*T_dh_to_b(:,:,i);
%         continue
%     end