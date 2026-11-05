
function [X_ref,Y_ref,Z_ref,...
          Vx_ref,Vy_ref,Vz_ref] = ...
          Cartesian_Trajectory_Planner(P0,Pf,t,T,t_start)

% 3D Cartesian Cubic Trajectory Planner
%
% P0       = Initial position [X;Y;Z] (mm)
% Pf       = Target position [X;Y;Z] (mm)
% t        = Simulation time (seconds)
% T        = Movement duration (seconds)
% t_start  = Time when movement begins (seconds)
%
% Outputs:
% X_ref,Y_ref,Z_ref = Desired positions (mm)
% Vx_ref,Vy_ref,Vz_ref = Desired velocities (mm/s)

% Initialize outputs at starting position
X_ref = P0(1);
Y_ref = P0(2);
Z_ref = P0(3);

Vx_ref = 0;
Vy_ref = 0;
Vz_ref = 0;

% Validate inputs
if T <= 0
    return;
end

% Hold initial position before movement
if t <= t_start
    return;
end

% Calculate elapsed movement time
tau = min(t - t_start,T);

% Normalized trajectory time
u = tau/T;

% Cubic interpolation
s = 3*u^2 - 2*u^3;

% Derivative of cubic interpolation
sdot = (6*u - 6*u^2)/T;

% Cartesian displacement
dP = Pf - P0;

% Desired Cartesian position
P_ref = P0 + dP*s;

% Desired Cartesian velocity
V_ref = dP*sdot;

% Assign position outputs
X_ref = P_ref(1);
Y_ref = P_ref(2);
Z_ref = P_ref(3);

% Assign velocity outputs
Vx_ref = V_ref(1);
Vy_ref = V_ref(2);
Vz_ref = V_ref(3);

% Hold final position after movement
if t >= (t_start + T)

    X_ref = Pf(1);
    Y_ref = Pf(2);
    Z_ref = Pf(3);

    Vx_ref = 0;
    Vy_ref = 0;
    Vz_ref = 0;

end

end
