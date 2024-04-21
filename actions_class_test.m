% Pick and place

%% Connect to ROS (use your own masterhost IP address)
clc
clear
% clearvars;
% clearvars –global

pause(2);       % Check if more down time helps diminish connection errors

%% 00 Define classes and static variables

masterhostIP    = "192.168.64.129";
rosshutdown;
rosinit(masterhostIP);

%% Create ROS class with all communication objects
r = rosClassHandle;

% Optional argments
                 % Type of global dictionary with all options to facilitate passing of options
keys   = ["debug", "toolFlag", "traj_steps", "z_offset", "traj_duration", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
values = {      0,          0,            1,       0.09,               2,                     1,                    1,            0.165,         r};
optns = dictionary(keys,values);

% optns("debug")               = 0;     % If set to true visualize traj before running  
% optns("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
% optns("traj_steps")          = 1;     % Num of traj steps
% optns("z_offset")            = 0.09;  % Vertical offset for top-down approach
% optns("traj_duration")       = 2;     % Traj duration (secs)   
% 
% optns("frameAdjustmentFlag") = 1;
% optns("toolAdjustmentFlag")  = 1;
% optns("toolAdjustment")      = 0.165; % Distance from tool0 to gripper_tip_link
% optns("rHandle")             = r; % contains ros objects
% 


%% 02 Go Home
goHome('qr', optns);
resetWorld;
type = 'manual'; % gazebo, ptcloud, cam, manual
disp('Getting goal...')
%moveToQ([pi/2,0,pi,pi/2,0,0],optns)
% % via gazebo
% models = getModels;
% 
% 
% via manual
rx = [0.295290, -0.077456, 0.699540];
ry = [0.502748, -0.667331, 0.032219];
rz = [0.573223, 0.690198, 0.573224];

rCan1 = [-ry(1,1), rx(1,1)+0.1, rz(1,1)-0.44, -pi/2, -pi, 0];
rCan2 = [-ry(1,2), rx(1,2)+0.1, rz(1,2)-0.44, -pi/2, -pi, 0];
rCan3 = [-ry(1,3), rx(1,3)+0.1, rz(1,3)-0.44, -pi/2, -pi, 0];
% framework for vision ai later on
x = [0.679967,0.306365,0.073349,-0.170184];
y = [-0.160081,0.150915,0.230001,0.613552];
z = [0.613571,0.613600,0.613600,0.613552];
xtest = -0.0829;
ytest = -0.0373;
ztest = 0.5507;
test = [xtest, ytest, ztest, -pi/2, -pi, 0];

roll = [-pi/2, 0];
pitch = [-pi, 0];
yaw = [0, 0];

yBottle2 = [-y(1,1), x(1,1)+0.1, z(1,1)-0.37, roll(1,1), pitch(1,1), yaw(1,1)];
yBottle3 = [-y(1,2), x(1,2)+0.1, z(1,2)-0.37, roll(1,1), pitch(1,1), yaw(1,1)];
bBottle2 = [-y(1,3), x(1,3)+0.1, z(1,3)-0.37, roll(1,1), pitch(1,1), yaw(1,1)];
bBottle1 = [-y(1,4), x(1,4)+0.1, z(1,4)-0.37, roll(1,1), pitch(1,1), yaw(1,1)];
model_pos = [yBottle2;yBottle3;bBottle2;bBottle1];
goal = [rCan1;rCan2;rCan3];
%Z = -.01

rate = rosrate(10);
n = 3;
% 
for i=1:n
    % 03 Get Pose
    %mat_R_T_M = set_manual_goal(model_pos(i,1:6));
    mat_R_T_M = set_manual_goal(goal(i,1:6));

    % 04 Pick Model
    strategy = 'topdown'; % Assign strategy: topdown, direct
    ret = pick(strategy, mat_R_T_M, optns); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
    

    % 05 Place
    if ~ret
        disp('Attempting place...')
        greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi, 0];
        place_pose = set_manual_goal(greenBin);
        strategy = 'topdown';
        fprintf('Moving to bin...');
        ret = moveToBin(strategy,mat_R_T_M,place_pose,optns);
    end

    % Return to home
    if ~ret
        ret = moveToQ('qr',optns);
    end

    % Control loop
    waitfor(rate);
end
