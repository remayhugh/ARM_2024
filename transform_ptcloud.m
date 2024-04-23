function transformedPtCloud = transform_ptcloud(optns)
%--------------------------------------------------------------------------
% Fix point cloud
% Input:

% Output: 
%--------------------------------------------------------------------------

%% Get pointcloud2
r = optns('rHandle');
r = r{1};
ptcloud = receive(r.ds,2);

%% Get base_link to camera_depth_link transform
% tftree       = rostf('DataFormat','struct');     
base         = 'base_link';
end_effector = ptcloud.Header.FrameId; %'camera_depth_link';
% Compute the ROS/Gazebo transform from base_link to camera_depth_link
waitForTransform(r.tftree, base, end_effector);
p = getTransform(r.tftree, base, end_effector, rostime('now'),'Timeout', r.tf_listening_time);

%% Get position and orientation -> Homogeneous Transform
% Currently in ROS/Gazebo settings. No need to change anything.
pos = [ p.Transform.Translation.X, ... 
        p.Transform.Translation.Y, ...
        p.Transform.Translation.Z]; 
% This result makes sense
    
q = UnitQuaternion(p.Transform.Rotation.W, ...
                   [p.Transform.Rotation.X, ...
                    p.Transform.Rotation.Y, ...
                    p.Transform.Rotation.Z]);

% Compute the 4x4
%Check q.T. 
q.T
T = transl(pos) * q.T;

%----------------------------------
% I expect a 
% 0 0 1  
% 0 1 0
%-1 0 0
% But not getting that...
%----------------------------------
% R = [0 0 1; 
%      0 1 0; 
%      -1 0 0];
% qq = UnitQuaternion(R);
%T = transl(pos) * qq.T;

%% Transform the pt cloud (i.e. trying to do what you see in https://www.mathworks.com/help/releases/R2023b/vision/ref/pctransform.html)
% Extract Nx3 matrix of XYZ points
xyz = rosReadXYZ(ptcloud);         

% Remove NaNs
xyz = xyz(~isnan(xyz(:,1)),:);

% n rows by 3 cols with (x,y,z) tuples
[n,c] = size(xyz); 

% Transform each row of points, need homogeneous coords P=[x,y,z,1]'
pointsHomogeneous = [xyz, ones(size(xyz, 1), 1)];

% Apply the transformation
transformedPointsHomogeneous = pointsHomogeneous * T';

% Remove the homogeneous coordinate
transformedPoints = transformedPointsHomogeneous(:, 1:3);

% Recreate the pointCloud object if needed for visualization or further processing
% The pointCloud object creates point cloud data from a set of points in 3-D coordinate system. The points generally represent the x,y, and z geometric coordinates for samples on a surface or of an environment. Each point can also be represented with additional information, such as the RGB color. The point cloud data is stored as an object with the properties listed in Properties. Use Object Functions to retrieve, select, and remove desired points from the point cloud data.
transformedPtCloud = pointCloud(transformedPoints);

%% Visualize
figure2 = figure;
axes2 = axes(Parent=figure2);
pcshow(transformedPtCloud,Parent=axes2,AxesVisibility='on');
xlabel('X');
ylabel('Y');
zlabel('Z');
title({'Transformed Point Cloud'},FontSize=14)

