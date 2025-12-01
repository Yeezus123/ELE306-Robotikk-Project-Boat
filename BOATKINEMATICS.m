clear
clc

% DH-parameters (standard) for a typical marine manipulator
% Adjust lengths if needed
% L1: vertikal mast fra bakken opp
L1 = Link([0      0.50   0       pi/2], 'standard');   % Base
L2 = Link([0      0      0.35    0   ], 'standard');   % Skulder
L3 = Link([0      0      0.25   -pi/2], 'standard');   % Albue
L4 = Link([0      0      0.20    0   ], 'standard');   % Håndledd

MarineArm = SerialLink([L1 L2 L3 L4], 'name', 'BOATARM');

% Straight/stretched arm (all angles = 0)
q0 = [0 0 0 0];
T0 = MarineArm.fkine(q0)

% Safe "drive" position (folded)
q_drive = [pi/2  -pi/3   pi/4   0];
Tdrive = MarineArm.fkine(q_drive)

%% === EXAMPLE WORK POSITION ===
qn = [0.3, 0.6, -1.2, 0.5];   % example joint angles
MarineArm.plot(qn);

T3 = MarineArm.fkine(qn)

%% === INVERSE KINEMATICS ===
% 4 DOF = cannot control full 6D pose → use mask = [1 1 1 0 0 0]
mask = [1 1 1 0 0 0];

qi = MarineArm.ikine(T3, 'mask', mask)

MarineArm.plot(qi);

%% === JACOBIAN ===
J = MarineArm.jacob0(qn)

joint_vel = [0.1; 0.1; -0.1; 0.2];  % 4×1
eef_vel = J * joint_vel

