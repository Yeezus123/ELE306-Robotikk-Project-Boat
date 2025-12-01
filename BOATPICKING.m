clear; clc; close all;

L1 = Link([0      0.50   0       pi/2], 'standard');   
L2 = Link([0      0      0.35    0   ], 'standard');   
L3 = Link([0      0      0.25   -pi/2], 'standard');   
L4 = Link([0      0      0.20    0   ], 'standard'); 

L1.qlim = [-pi       pi];
L2.qlim = [-pi/2     pi/2];     
L3.qlim = [-3*pi/4   3*pi/4];
L4.qlim = [-pi       pi];

boatArm = SerialLink([L1 L2 L3 L4], 'name', 'BoatPickingArm');


x_target = 0.5;
y_target = 0.0;
z_over  = 0.8;  
z_pick  = 0.2;   

maskPos = [1 1 1 0 0 0];


T_over = transl(x_target, y_target, z_over);
T_pick = transl(x_target, y_target, z_pick);

q0 = [0 0 0 0];  

q_over = boatArm.ikine(T_over, q0, 'mask', maskPos);

q_pick = boatArm.ikine(T_pick, q_over, 'mask', maskPos);


T_over_check = boatArm.fkine(q_over);
T_pick_check = boatArm.fkine(q_pick);
disp('Over-posisjon [x y z]:');  disp(T_over_check.t');
disp('Plukk-posisjon [x y z]:'); disp(T_pick_check.t');

n = 50;

traj_down  = jtraj(q_over, q_pick, n);   
traj_up    = jtraj(q_pick, q_over, n);   

ws = [-1 1 -1 1 0 1.5];   


boatArm.plot(q_over, 'workspace', ws);
drawnow;

for i = 1:n
    boatArm.plot(traj_down(i,:), 'workspace', ws);
    drawnow;
end

for i = 1:n
    boatArm.plot(traj_up(i,:), 'workspace', ws);
    drawnow;
end
