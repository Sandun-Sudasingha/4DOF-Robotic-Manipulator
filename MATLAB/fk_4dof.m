
function [x, y, z, phi] = fk_4dof(q1, q2, q3, q4)
%#codegen
% 4-DOF Robot Forward Kinematics
%
% Inputs: Joint angles (rad)
% Outputs: Position (mm), pitch (rad)

% Robot dimensions (mm)
R_offset = 90;
H_base = 288;
L1 = 350;
L2 = 335;
L3 = 225;

% Cumulative pitch angles
t1 = q2;
t2 = q2 + q3;
t3 = q2 + q3 + q4;

% Radial position
r = R_offset ...
    + L1*cos(t1) ...
    + L2*cos(t2) ...
    + L3*cos(t3);

% Vertical position
z = H_base ...
    + L1*sin(t1) ...
    + L2*sin(t2) ...
    + L3*sin(t3);

% Cartesian coordinates
x = r*cos(q1);
y = r*sin(q1);

% Tool orientation
phi = t3;

end
