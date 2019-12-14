% MATLAB code to create a model of Universal Robotics UR5 6 DOF manipulator
% This file uses the function for UR5 from The Robotics Toolbox for MATLAB (RTB).
clc; clear; %clear the command window and the workspace 
% Use run to load the Robotics Toolbox into Matlab's path
run('C:\Robotics Toolbox Matlab\robot-10.3.1\rvctools\startup_rvc.m')
% Use symbolic variables to define joint variables.
syms theta1 theta2 theta3 theta4 theta5 theta6

% Define variables for link lengths
%SI units of metres for length and radians for angle are used.
% a2 and a3 are taken to be negative because the poses were
% defined accordingly in moveit and then the DH table was made
d1 = 0.08916; a2 = -0.425; a3 = -0.39225;
d4 = 0.10915; d5 = 0.09456; d6 = 0.0823;
% Defining poses
rest_pose = ([0 -1.5708 0 -1.5708 0 1.5708]); % rest pose
object_pickup_pose = ([4.6851 -pi/2 -pi/2 -pi/2 0 pi/2]);
object_drop_pose = ([-0.1090 -pi/2 -pi/2 -pi/2 0.8353 -pi/2]);
milk_pick_pose = ([0 -pi/2 -pi/2 -pi/2 0 -pi/2]);
milk_drink_pose = ([-3.8182 -pi/2 -1.7977 -pi/2 0 -pi/2]);

% Enter the DH parameters as a vector, in case your joint is prismatic, use Link([....],'p')
% By default, the joint is revolute.
% L(1) means Link 1
L(1) = Link([0  d1   0   0]);
%L(1).qlim = [0 pi];  %In case you want to set limits on the link motion
L(2) = Link([0  0   0    pi/2]); 
L(3) = Link([0  0   a2    0]);
L(4) = Link([0  d4   a3   0]);
L(5) = Link([0  d5   0  pi/2]);
L(6) = Link([0  d6   0    -pi/2]);

% Connect all links of the robot using the command serial link.
% Pass the vector L to serial link. We have the object "UR5_662" on the LHS.
UR5_arm = SerialLink(L);
% In order to name this robot, we use:-
UR5_arm.name = 'UR5 simulation in MATLAB';
% To see the DH table of the arm in the command window:-
fprintf('DH table of the Universal Robot 5 arm:- \n')
UR5_arm

% To plot a single pose of the robot, uncomment and use:-
% UR5_arm.plot(object_drop_pose)
% UR5_arm.plot([0 0 0 0 0 0])
% UR5_arm.plot(object_pickup_pose)

% To save images to a folder
%saveas(gcf,'C:\Robotics Toolbox Matlab\ur5_imgs\ur5_initial_pose.png')
%saveas(gcf,'C:\Robotics Toolbox Matlab\ur5_imgs\ur5_object_pickup_pose.png')
saveas(gcf,'C:\Robotics Toolbox Matlab\ur5_imgs\teach.png')


% For the general relation: -
% Uncomment General_FK if you need it because it takes a long time to
% compute it
% fprintf('General Homogeneous transform of UR5')
% General_FK = UR5_arm.fkine([theta1 theta2 theta3 theta4 theta5 theta6])

T_rest_pose = UR5_arm.fkine(rest_pose);
fprintf('Homogeneous transform of the pose for picking up an object using the UR5 arm:-')
T_object_pickup_pose = UR5_arm.fkine(object_pickup_pose)
T_object_drop_pose = UR5_arm.fkine(object_drop_pose);
T_milk_pick_pose = UR5_arm.fkine(milk_pick_pose);
T_milk_drink_pose = UR5_arm.fkine(milk_drink_pose);

% For inverse kinematics, we first find the homogeneous transform using fk,
% We comment this section because the IK solver of matlab gives an error
% It fails to converge and asks for a different set of initial joint
% values.
% T_inv = UR5_arm.fkine(object_pickup_pose)
% q_inv = UR5_arm.ikine(T_inv)

% Uncomment to use GUI interface for physical insight and to configure poses:-
 UR5_arm.teach

% Animation of the moving robot using for loop and pause for easy visualization
% Animation to go from rest_pose = ([0 -1.5708 0 -1.5708 0 1.5708])
% to object_pickup_pose = ([4.6851 -pi/2 -pi/2 -pi/2 0 pi/2])
% Set initial pose, t=theta, i=initial
ti1 = 0;
ti2 = -1.5708;
ti3 = 0;
ti4 = -1.5708;
ti5 = 0;
ti6 = 1.5708;
UR5_arm.plot([ti1 ti2 ti3 ti4 ti5 ti6])

% Set final pose, t=theta, f=final
fi1 = 4.6851;
fi2 = -pi/2;
fi3 = -pi/2;
fi4 = -pi/2;
fi5 = 0;
fi6 = pi/2;

if ti1-fi1<0
    sign1 = 1; %change sign for step size of for loop to positive
    %when fi1>ti1    
else sign1 = -1;%change sign for step size of for loop to negative
    %when fi1<ti1
end

if ti2-fi2<0
    sign2 = 1; %change sign for step size of for loop to positive
    %when fi2>ti2    
else sign2 = -1;%change sign for step size of for loop to negative
    %when fi2<ti2
end

if ti3-fi3<0
    sign3 = 1; %change sign for step size of for loop to positive
    %when fi3>ti3  
else sign3 = -1;%change sign for step size of for loop to negative
    %when fi3<ti3
end

if ti4-fi4<0
    sign4 = 1; %change sign for step size of for loop to positive
    %when fi4>ti4  
else sign4 = -1;%change sign for step size of for loop to negative
    %when fi4<ti4
end

if ti5-fi5<0
    sign5 = 1; %change sign for step size of for loop to positive
    %when fi5>ti5  
else sign5 = -1;%change sign for step size of for loop to negative
    %when fi5<ti5
end

if ti6-fi6<0
    sign6 = 1; %change sign for step size of for loop to positive
    %when fi6>ti6  
else sign6 = -1;%change sign for step size of for loop to negative
    %when fi6<ti6
end
    
 
% Using for loops to go from initial pose to final pose
% The animation can be made smoother by decreasing the step size in the for
% loop. The sign of the step size has to be changed according to situation
for th1 = ti1: sign1*0.06: fi1
    UR5_arm.plot([th1 ti2 ti3 ti4 ti5 ti6]);
    pause(0.25)
end
pause(0.25)
for th2 = ti2: sign2*0.06: fi2
    UR5_arm.plot([fi1 th2 ti3 ti4 ti5 ti6]);
    pause(0.1)
end
pause(0.25)
for th3 = ti3: sign3*0.06: fi3
    UR5_arm.plot([fi1 fi2 th3 ti4 ti5 ti6]);
    pause(0.1)
end
pause(0.25)
for th4 = ti4:  sign4*0.06: fi4
    UR5_arm.plot([fi1 fi2 fi3 th4 ti5 ti6]);
    pause(0.1)
end
pause(0.25)
for th5 = ti5: sign5*0.06: fi5
    UR5_arm.plot([fi1 fi2 fi3 fi4 th5 ti6]);
    pause(0.1)
end
pause(0.25)
for th6 = ti6: sign6*0.06: fi6
    UR5_arm.plot([fi1 fi2 fi3 fi4 fi5 th6]);
    pause(0.1)
end

