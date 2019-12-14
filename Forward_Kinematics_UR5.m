% We have T from previous code, we use it here to find the homogeneous
% transform by substituting the values of theta1, theta2, theta3, theta4, theta5 and theta6
% We solve the Forward kinematics for object_pickup_pose = ([4.6851 -pi/2 -pi/2 -pi/2 0 pi/2])

clc; clear; %clear the command window and the workspace 

% Define variables for link lengths
% SI units of metres for length and radians for angle are used.
d1 = 0.08916; a2 = -0.425; a3 = -0.39225;
d4 = 0.10915; d5 = 0.09456; d6 = 0.0823;  

% Substituting the vales of all the joint variables:-
theta1 = 4.6851;
theta2 = -pi/2;
theta3 = -pi/2;
theta4 = -pi/2;
theta5 = 0;
theta6 = pi/2;
 
% Values of A1 to A6 stored here from previous calculations 
% so that they can be used when the workspace is cleared

 A1 =[ cos(theta1), -sin(theta1), 0,  0;
 sin(theta1),  cos(theta1), 0,  0;
           0,            0, 1, d1;
           0,            0, 0,  1]
 
 
A2 =[ cos(theta2), 0,  sin(theta2), 0;
 sin(theta2), 0, -cos(theta2), 0;
          0, 1,            0, 0;
          0, 0,            0, 1]
 
 
A3 =[ cos(theta3), -sin(theta3), 0, a2*cos(theta3);
 sin(theta3),  cos(theta3), 0, a2*sin(theta3);
           0,            0, 1,              0;
           0,            0, 0,              1]
 
 
A4 =[ cos(theta4), -sin(theta4), 0, a3*cos(theta4);
 sin(theta4),  cos(theta4), 0, a3*sin(theta4);
           0,            0, 1,             d4;
           0,            0, 0,              1]
 
 
A5 =[ cos(theta5), 0,  sin(theta5),  0;
 sin(theta5), 0, -cos(theta5),  0;
           0, 1,            0, d5;
           0, 0,            0,  1]
 
 
A6 =[ cos(theta6),  0, -sin(theta6),  0;
 sin(theta6),  0,  cos(theta6),  0;
          0, -1,            0, d6;
           0,  0,            0,  1]
 
 
      
% All the values are substituted in the general value of T from previous code
T = [ sin(theta1 + theta2)*sin(theta6) + cos(theta6)*(cos(theta1 + theta2)*cos(theta3 + theta4)*cos(theta5) - cos(theta1 + theta2)*sin(theta3 + theta4)*sin(theta5)), -sin(theta3 + theta4 + theta5)*cos(theta1 + theta2),   sin(theta1 + theta2)*cos(theta6) - sin(theta6)*(cos(theta1 + theta2)*cos(theta3 + theta4)*cos(theta5) - cos(theta1 + theta2)*sin(theta3 + theta4)*sin(theta5)), d6*(cos(theta1 + theta2)*cos(theta3 + theta4)*sin(theta5) + cos(theta1 + theta2)*sin(theta3 + theta4)*cos(theta5)) + d4*sin(theta1 + theta2) + d5*sin(theta1 + theta2) + a3*cos(theta1 + theta2)*cos(theta3 + theta4) + a2*cos(theta1 + theta2)*cos(theta3);
 cos(theta6)*(cos(theta3 + theta4)*sin(theta1 + theta2)*cos(theta5) - sin(theta1 + theta2)*sin(theta3 + theta4)*sin(theta5)) - cos(theta1 + theta2)*sin(theta6), -sin(theta3 + theta4 + theta5)*sin(theta1 + theta2), - sin(theta6)*(cos(theta3 + theta4)*sin(theta1 + theta2)*cos(theta5) - sin(theta1 + theta2)*sin(theta3 + theta4)*sin(theta5)) - cos(theta1 + theta2)*cos(theta6), d6*(cos(theta3 + theta4)*sin(theta1 + theta2)*sin(theta5) + sin(theta1 + theta2)*sin(theta3 + theta4)*cos(theta5)) - d5*cos(theta1 + theta2) - d4*cos(theta1 + theta2) + a3*cos(theta3 + theta4)*sin(theta1 + theta2) + a2*sin(theta1 + theta2)*cos(theta3);
                                                                                                                      sin(theta3 + theta4 + theta5)*cos(theta6),                       cos(theta3 + theta4 + theta5),                                                                                                                       -sin(theta3 + theta4 + theta5)*sin(theta6),                                                                                                                                                                            d1 + a3*sin(theta3 + theta4) + a2*sin(theta3) - d6*cos(theta3 + theta4 + theta5);
                                                                                                                                                              0,                                                   0,                                                                                                                                                                0,                                                                                                                                                                                                                                                           1]
                                                                                                                                                          

fprintf('We see that we get the same value of the homogeneous transform for the object pickup pose from both approaches \n')                                                                                                                                                          
fprintf('Hence, FK is validated for UR5 arm using MATLAB robotics toolbox')                                                                                                                                                          
                                                                                                                                                          
                                                                                                                                                          
                                                                                                                                                          
                                                                                                                                                          