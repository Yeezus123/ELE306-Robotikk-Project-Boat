clear; clc; close all;

R = 0.1;    
L = 0.5;    
dd = DifferentialDrive(R, L); 


startposition = [0 0 0];
checkpoint_1 = [10 0 0];
checkpoint_2 = [10 0 pi/2];
checkpoint_3 = [10 10 pi/2];
checkpoint_4 = [10 10 pi];
checkpoint_5 = [0 10 pi];


map = binaryOccupancyMap(20, 20, 1);
setOccupancy(map, [5, 5], 1); 


for y = 0:12
    setOccupancy(map, [12, y], 1); 
end

for x = 0:12
    setOccupancy(map, [x, 12], 1);  
end

lp = Lattice(map); 
lp.plan('iterations', 20);  
viz = Visualizer2D;
viz.hasWaypoints = true;


sampleTime = 0.1;               
tVec = 0:sampleTime:50;         
pose = zeros(3, numel(tVec));   
pose(:,1) = startposition';     

controller = controllerPurePursuit;
controller.LookaheadDistance = 0.35;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1000000;


disp('Navigating to checkpoint 1...');
path_1 = lp.query(startposition, checkpoint_1);  
controller.Waypoints = path_1(:, [1, 2]);        

figure;
hold on;
show(map);  
plot(path_1(:,1), path_1(:,2), 'r--', 'LineWidth', 2);  


r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; 
    vel = bodyToWorld(velB, pose(:,idx-1));  

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    plot(pose(1,idx), pose(2,idx), 'bo');  

    
    if norm(pose(1:2, idx) - checkpoint_1(1:2)') < 0.1
        disp('Reached checkpoint 1.');
        break;  
    end

    waitfor(r);
end


pose(:,1) = checkpoint_1';


disp('Navigating to checkpoint 2...');
path_2 = lp.query(checkpoint_1, checkpoint_2);  
controller.Waypoints = path_2(:, [1, 2]);       


plot(path_2(:,1), path_2(:,2), 'g--', 'LineWidth', 2);  


for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; 
    vel = bodyToWorld(velB, pose(:,idx-1));  

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    plot(pose(1,idx), pose(2,idx), 'bo');  

    
    if norm(pose(1:2, idx) - checkpoint_2(1:2)') < 0.1
        disp('Reached checkpoint 2.');
        break;  
    end

    waitfor(r);
end


pose(:,1) = checkpoint_2';


disp('Navigating to checkpoint 3...');
path_3 = lp.query(checkpoint_2, checkpoint_3);  
controller.Waypoints = path_3(:, [1, 2]);       


plot(path_3(:,1), path_3(:,2), 'g--', 'LineWidth', 2); 


for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; 
    vel = bodyToWorld(velB, pose(:,idx-1));  

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    
    plot(pose(1,idx), pose(2,idx), 'bo');  

   
    if norm(pose(1:2, idx) - checkpoint_3(1:2)') < 0.1
        disp('Reached checkpoint 3.');
        break;  
    end

    waitfor(r);
end


pose(:,1) = checkpoint_3';


disp('Navigating to checkpoint 4...');
path_4 = lp.query(checkpoint_3, checkpoint_4);  
controller.Waypoints = path_4(:, [1, 2]);       


plot(path_4(:,1), path_4(:,2), 'g--', 'LineWidth', 2);  


for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; 
    vel = bodyToWorld(velB, pose(:,idx-1));  

    
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    
    plot(pose(1,idx), pose(2,idx), 'bo');  

    
    if norm(pose(1:2, idx) - checkpoint_4(1:2)') < 0.1
        disp('Reached checkpoint 4.');
        break;  
    end

    waitfor(r);
end


pose(:,1) = checkpoint_4';


disp('Navigating to checkpoint 5...');
path_5 = lp.query(checkpoint_4, checkpoint_5);  
controller.Waypoints = path_5(:, [1, 2]);       


plot(path_5(:,1), path_5(:,2), 'g--', 'LineWidth', 2);  


for idx = 2:numel(tVec)
    [vRef, wRef] = controller(pose(:,idx-1));
    [wL, wR] = inverseKinematics(dd, vRef, wRef);

    
    [v, w] = forwardKinematics(dd, wL, wR);
    velB = [v; 0; w]; 
    vel = bodyToWorld(velB, pose(:,idx-1));  

    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    plot(pose(1,idx), pose(2,idx), 'bo');  

    
    if norm(pose(1:2, idx) - checkpoint_5(1:2)') < 0.1
        disp('Reached checkpoint 5.');
        break;  
    end

    waitfor(r);
end


hold off;