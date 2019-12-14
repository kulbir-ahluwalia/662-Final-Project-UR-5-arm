%the code is run for finding every Ai matrix after making the necessary changes to theta, a ,d and alpha
%A1, A2, A3... are stored in the workspace as we find each Ai
%then in the end, we just find the final T by multiplying all Ai

syms  alpha d a theta 
syms theta1 theta2 theta3 theta4 theta5 theta6
syms d4 d5 d6 d1 a2 a3   %to create symbolic variables

rot_z = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 0; 0 0 0 1];      %Initialising homogeneous transforms
trans_z = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
trans_x = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
rot_x = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];
final = (rot_z*trans_z*trans_x*rot_x);

theta = theta6;   
% theta5 = 0;
% theta6 = deg2rad(theta5);
% %theta =  theta6+theta3;
% theta =  theta6+0;

d = d6;
a = 0;

alpha = -90;
alpha = deg2rad(alpha);

final = subs(final); %updates values of variables
A6 = simplify(final);
%disp(A6);

% Following code can be uncommented when we have all Ais and want to see
% them and calculate T
% A1  %to see the final values of Ai matrices
% A2
% A3
% A4
% A5
% A6

% T = simplify(A1*A2*A3*A4*A5*A6)