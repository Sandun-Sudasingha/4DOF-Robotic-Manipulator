
function [q1, q2, q3, q4, flag] = ik_4dof(xd, yd, zd, phi)
%#codegen
% 4-DOF Robot Inverse Kinematics
%
% Inputs:
% xd, yd, zd : Desired position (mm)
% phi        : Desired tool pitch (rad)
%
% Outputs:
% q1, q2, q3, q4 : Joint angles (rad)
% flag            : 1 = reachable, 0 = unreachable

% Robot dimensions (mm)
R_offset = 90;
H_base   = 288;
L1 = 350;
L2 = 335;
L3 = 225;

% Initialize outputs
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
flag = 0;

% Base rotation
q1 = atan2(yd, xd);

% Convert to vertical plane coordinates
r = sqrt(xd^2 + yd^2) - R_offset;
z = zd - H_base;

% Wrist centre
wx = r - L3*cos(phi);
wz = z - L3*sin(phi);

% Cosine rule
D = (wx^2 + wz^2 - L1^2 - L2^2)/(2*L1*L2);

% Workspace check
if D < -1 || D > 1
    return;
end

% Numerical protection
D = min(1, max(-1, D));

% Elbow-down configuration
q3 = atan2(sqrt(max(0,1-D^2)), D);

% Shoulder angle
q2 = atan2(wz,wx) ...
    - atan2(L2*sin(q3),L1+L2*cos(q3));

% Wrist angle
q4 = phi-q2-q3;

% Successful solution
flag = 1;

end
