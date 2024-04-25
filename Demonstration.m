%% ROS setup
masterhostIP = "192.168.64.129";
rosshutdown;
rosinit(masterhostIP)

r = rosClassHandle;

% for the pouch:
% keys   = ["debug", "toolFlag", "traj_steps", "x_offset", "y_offset", "z_offset", 'gripPos', "traj_duration", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
% values = {      0,          0,            1,          0,          0,     0.1672,     0.608,               2,                     1,                    1,            0.165,         r};
% for the can:
keys   = ["debug", "toolFlag", "traj_steps", "x_offset", "y_offset", "z_offset", 'gripPos', "traj_duration", "frameAdjustmentFlag", "toolAdjustmentFlag", "toolAdjustment", "rHandle"];
values = {      0,          0,            1,          0,          0,        0.2,      0.23,               2,                     1,                    1,            0.165,         r};
optns = dictionary(keys,values);

disp('Going home...');
goHome('qtest',optns);    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld      % reset models through a gazebo service

%% Call Class to Create Subscriber
disp("Calling class to create subscriber...")

%% Take picture
disp("Taking picture...")
rosImg  = receive(r.rgb_sub);
myImg   = readImage(rosImg);
counter = 1;

%% Pick which object you're picking
 for i = 1:counter
    if(i == 1)
        GT_xyz = [0.80,-0.03,0.15]; %Can
        nm = 'rCan3';
    else
        GT_xyz = [0.62,0.04,-0.09]; %Pouch
        nm = 'pouch4';
    end
%GT_xyz = [0.62,0.04,-0.09]; % Pouch
%GT_xyz = [0.80,-0.03,0.15]; % Can

%% Bounding Boxes
disp("Computing bounding boxes, scores, and labels...")
if(i == 1)
    pretrained = load('can_detector_20240412.mat');
    trainedYoloNet = pretrained.detector;
    [bboxes,scores,labels] = detect(trainedYoloNet,myImg); %Can
else
    pretrained = load('data.mat');
    trainedYoloNet = pretrained.detector;
    [bboxes,scores,labels] = detectObjectsYoloNet(trainedYoloNet,myImg); %Pouch
end
%[bboxes,scores,labels] = detectObjectsYoloNet(trainedYoloNet,myImg);

%% Visualize the detected objects' bounding boxes
disp("Drawing bounding boxes...")
annotatedImage = insertObjectAnnotation(im2uint8(myImg), 'Rectangle',...
    bboxes, string(labels)+":"+string(scores),'Color','cyan');
figure, imshow(annotatedImage);

%% Specify percentage of acceptable bounding box
disp("Updating valid bounding boxes...")
valid_idx = scores > 0.4;
bboxes = bboxes(valid_idx, :);
scores = scores(valid_idx);
labels = labels(valid_idx);
numObjects = size(bboxes,1);

%% Decide which object to detect
n = 1; % item index you want to use
bboxes = bboxes(n,:);
labels = labels(n,:);
scores = scores(n,:);
% do we need to recall this?
numObjects = size(bboxes,1);

%% Redraw updated bounding boxes on image
disp("Redrawing updated bounding boxes...")

% visualize the detected object bounding box
reannotatedImage = insertObjectAnnotation(im2uint8(myImg), 'Rectangle',...
    bboxes, string(labels)+":"+string(scores),'Color','cyan');
figure, imshow(reannotatedImage);

%% Do new ptcloud transform
ptCloud = pcread('allObj.ply');
disp("Creating point cloud...")
ds = r.pt_cloud_sub;
transformedPtCloud = transform_ptcloud(ds,optns);

%% Create point cloud visually **NewPtCloud (npc)
disp("Plotting point cloud separately...")
planeThickness = .001;
normalVector = [0,0,1];
maxPlaneTilt = 5;
[param_npc, planeIdx_npc, nonPlaneIdx_npc] = pcfitplane(transformedPtCloud, planeThickness, normalVector, maxPlaneTilt);
plane_npc = select(transformedPtCloud, planeIdx_npc);
nonPlane_npc = select(transformedPtCloud, nonPlaneIdx_npc);

% show plane
figure,pcshow(plane_npc,'ViewPlane','XY');axis on;
% show items
figure,pcshow(nonPlane_npc,'ViewPlane','XY');axis on;

%% Create non-plane mask
% get size of image
[m,n,~] = size(myImg);

% create variable with dimensions of image
nonPlaneMask_npc = zeros(m,n);
% make variable only a column
nonPlaneMask_npc =nonPlaneMask_npc(:);
% use nonPlaneIdx to fill nonPlaneMask with useful data
nonPlaneMask_npc(nonPlaneIdx_npc)= 1;

%% Estimate object pose
disp("Finding object pose...")
gridDownsample = 0.02;
[xyz_npc,theta_npc,ptCloud_vec_npc,scene_pca_vec_npc] = findObjectPoses(transformedPtCloud, myImg, bboxes, gridDownsample, nonPlaneMask_npc);

%% Plots point on point cloud
disp("Plotting guess point on point cloud...")

% show items & show point
figure;
hold on;
pcshow(nonPlane_npc,'ViewPlane','XY');axis on;
plot3(xyz_npc(1),xyz_npc(2),xyz_npc(3),'-o','color','r');

%% Replot with new ground truth
% manually input location
%GT_xyz = [0.62,0.04,-0.09];
% plot
hold on;
disp("Plotting new point using ground truth...")
pcshow(nonPlane_npc,'ViewPlane','XY');axis on;
plot3(GT_xyz(1),GT_xyz(2),GT_xyz(3),'-o', 'color','cyan');

% PICK UP OBJECTS
rate = rosrate(1);
fprintf('Picking up the pouch');
[mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(nm,0,optns);

% Pick Model
% Assign strategy: topdown, direct
strategy = 'topdown';
ret = pick(strategy, mat_R_T_M, optns); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
pause(2);

% Place
if ~ret
    disp('Attempting place...')
    
    %Set bin goals
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0]; % left
    blueBin  = [-0.4,  0.45, 0.25, -pi/2, -pi 0]; % right

    goal_pose = greenBin;

    place_pose = set_manual_goal(goal_pose);

    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose,optns);
end
goHome('qr',optns);
end