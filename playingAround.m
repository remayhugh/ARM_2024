function playingAround(optns)
    %% subscribing
    disp("Getting required data...")
    r = optns('rHandle');
    r = r{1};
    % this should be gotten from rosClassHandle
    r.rgb_sub;
    r.pt_cloud_sub;
    %pause(3)
    
    %% create readable data from subscriber
    disp("Reading image data...")
    % for image
    rgb_sub_lm = r.rgb_sub.LatestMessage;
    read_image = readImage(rgb_sub_lm);
    
    % for point cloud
    pt_cloud_sub_lm = r.pt_cloud_sub.LatestMessage;
    % read_ptCloud = readImage(pt_cloud_sub_lm);
    
    %pause(3)
    
    %% read image and point cloud data
    rgbImage1 = read_image;
    ptCloud = pt_cloud_sub_lm;
    
    %pause(3)
    
    %% to see image & point cloud that we are working with
    imshow(rgbImage1)
    scatter3(ptCloud)
    
    %pause(3)
    
    %% set flag
    showPartialOutputs = true;
    
    %pause(3)
    
    %% load pretrained YOLO v4 network
    disp("Loading detector data...")
    % pretrained = load('trainedYOLOv4Detector_cuboid.mat'); % we will need this .mat (gives detector, elapsed time, & info)
    % pretrained = load('can_detector_20240412.mat');
    %pretrained1 = load('data.mat');
    pretrained2 = load('can_detector_20240412.mat');
    %trainedYoloNet = pretrained1.detector; %& pretrained2.detector;
    trainedYoloNet = pretrained2.detector;
    %pause(3)
    
    %% compute bounding boxes (need this function)
    disp("Computing bounding boxes, scores, and labels...")
    [bboxes,scores,labels] = detect(trainedYoloNet,rgbImage1);
    %[bboxes,scores,labels] = detectObjectsYoloNet(trainedYoloNet,rgbImage1);
    % manually update for testing (delete when fixed)
    %bboxes = [230, 20, 90, 110];
    % bboxes = bboxes(1,:);
    % labels = labels(1,:);
    % scores = scores(1,:);
    %pause(3)
    
    %% draw bounding boxes on image
    disp("Drawing bounding boxes...")
    if(showPartialOutputs)
        % visualize the detected object bounding box
        annotatedImage = insertObjectAnnotation(im2uint8(rgbImage1), 'Rectangle',...
            bboxes, string(labels)+":"+string(scores),'Color','r');
        figure, imshow(annotatedImage);
    end
    
    %pause(3)
    
    %% specify percentage of acceptable bounding box
    disp("Updating valid bounding boxes...")
    valid_idx = scores > 0.71;
    bboxes = bboxes(valid_idx, :);
    scores = scores(valid_idx);
    labels = labels(valid_idx);
    numObjects = size(bboxes,1);
    
    %pause(3)
    
    %% redraw updated bounding boxes on image
    disp("Redrawing updated bounding boxes...")
    if(showPartialOutputs)
        % visualize the detected object bounding box
        reannotatedImage = insertObjectAnnotation(im2uint8(rgbImage1), 'Rectangle',...
            bboxes, string(labels)+":"+string(scores),'Color','red');
        figure, imshow(reannotatedImage);
    end
    
    %pause(3)
    
    %% making ptCloud a pointCloud instead of pointCloud2
    disp("Creating updated point cloud...")
    ptCloud1 = pointCloud(rosReadXYZ(ptCloud));
    ptCloud1.Color = readRGB(ptCloud);
    ptCloud1.Normal = pcnormals(ptCloud1);
    a = rosReadXYZ(ptCloud);
    ptCloud1.Intensity = a(:,1);
    % ptCloud1.RangeData = sqrt(sum(a.^2,2))
    % ptCloud1.RangeData = readField(ptCloud,'Data');
    % ptCloud1.Intensity = readField(ptCloud,'Intensity');
    
    %pause(3)
    
    %% create inverted point cloud visually
    transformedPtCloud = transform_ptcloud(optns);
    disp("Plotting updated point cloud...")
    planeThickness = .001;
    normalVector = [0,0,1];
    maxPlaneTilt = 5;
    % [param, planeIdx, nonPlaneIdx] = pcfitplane(ptCloud1, planeThickness, normalVector, maxPlaneTilt);
    [param, planeIdx, nonPlaneIdx] = pcfitplane(transformedPtCloud, planeThickness, normalVector, maxPlaneTilt);
    plane = select(transformedPtCloud, planeIdx);
    nonPlane = select(transformedPtCloud, nonPlaneIdx);
    % show plane
    if(showPartialOutputs)
        figure,pcshow(plane,'ViewPlane','XY');axis on;
    end
    
    % show items
    if(showPartialOutputs)
        figure,pcshow(nonPlane,'ViewPlane','XY');axis on;
    end
    
    %pause(3)
    
    %% create non-plane mask
    % get size of image
    [m,n,~] = size(rgbImage1);
    % create variable with dimensions of image
    nonPlaneMask = zeros(m,n);
    % make variable only a column
    nonPlaneMask =nonPlaneMask(:);
    % use nonPlaneIdx to fill nonPlaneMask with useful data
    nonPlaneMask(nonPlaneIdx)= 1;
    
    %pause(3)
    
    %% estimate object pose (need this function)
    disp("Finding object pose...")
    gridDownsample = 0.02;
    % [xyz,theta,ptCloud_vec,scene_pca_vec] = findObjectPoses(ptCloud1,rgbImage1, bboxes, gridDownsample, nonPlaneMask);
    [xyz,theta,ptCloud_vec,scene_pca_vec] = findObjectPoses(transformedPtCloud, rgbImage1, bboxes, gridDownsample, nonPlaneMask);
    
    %pause(3)
    
    %% Plots velocity vector?
    disp("Plotting...")
    if(showPartialOutputs)
        figure;
        for idx = 1: numObjects
            U = scene_pca_vec{idx}.UVW(:,1);
            V = scene_pca_vec{idx}.UVW(:,2);
            W = scene_pca_vec{idx}.UVW(:,3);
            center = scene_pca_vec{idx}.centroid;
            nexttile;
            pcshow(ptCloud_vec{idx},'ViewPlane','XY');
            hold on;
            quiver3(center(1), center(2), center(3), U(1), V(1), W(1), 'r');
            quiver3(center(1), center(2), center(3), U(2), V(2), W(2), 'g');
            quiver3(center(1), center(2), center(3), U(3), V(3), W(3), 'b');
            hold off;
        end
    end
    
    %% Plots velocity vector? on ptCloud
    disp("Plotting...")
    if(showPartialOutputs)
        hold on
        pcshow(nonPlane,'ViewPlane','XY');axis on;
        for idx = 1: numObjects
            U = scene_pca_vec{idx}.UVW(:,1);
            V = scene_pca_vec{idx}.UVW(:,2);
            W = scene_pca_vec{idx}.UVW(:,3);
            center = scene_pca_vec{idx}.centroid;
            pcshow(ptCloud_vec{idx},'ViewPlane','XY');
            hold on;
            quiver3(center(1), center(2), center(3), U(1), V(1), W(1), 'r');
            quiver3(center(1), center(2), center(3), U(2), V(2), W(2), 'g');
            quiver3(center(1), center(2), center(3), U(3), V(3), W(3), 'b');
            hold off;
        end
    end
    
    %% compute object orientation wrt robot coord system
    mat_R_T_C = robToCam;
    rotationFromRobotToCam = mat_R_T_C(1:3,1:3);
    
    thetaNew = zeros(numObjects, 1);
    for idx = 1:numObjects
    %     U = ptCloudParameterVector{idx}.UVW(:,1);
    %     V = ptCloudParameterVector{idx}.UVW(:,2);
    %     W = ptCloudParameterVector{idx}.UVW(:,3);
        U = scene_pca_vec{idx}.UVW(:,1);
        V = scene_pca_vec{idx}.UVW(:,2);
        W = scene_pca_vec{idx}.UVW(:,3);
        majorAxis = [U(1), V(1), W(1)];
        majorAxis = (rotationFromRobotToCam*majorAxis')';
        % This calculates the angle between the positive y axis ([0 1 0]) and
        % the major axis of the object in an anti-clockwise direction
        thetaNew(idx) = atan2d(dot([0 0 1],cross([ 0 1 0],majorAxis)),dot([ 0 1 0],majorAxis));
        if (thetaNew(idx) < 0)
            thetaNew(idx) = 180 + thetaNew(idx);
        end
    end
    
    %% use parameter rotation matrix and camera translation to compute the final part ground truth for motion planning
    partGT = zeros(numObjects,4);
    posDepthCam = mat_R_T_C(1:3,4);
    posDepthCam2 = [0 0 -mat_R_T_C(3,4)];
    
    mat_R_T_C_X = mat_R_T_C(1,4); % -0.5473
    mat_R_T_C_Y = mat_R_T_C(2,4); % 0.1337
    mat_R_T_C_Z = mat_R_T_C(3,4); % 0.4874
    endEffectorAdjustment = 0.165;
    
    MatrixMult = [1.5577 0.1777 -0.0444 -0.2511; -0.6888 1.3112 0.0222 -0.0389; -0.5777 -0.0777 1.0777 -0.0389; 0 0 0 1];
    
    for i = 1:length(scene_pca_vec)
        % partGT(1,1:3) = rotationFromRobotToCam*xyz(i,:)' = [0.2834,0.4834,-0.1242]
        % partGT(i,1:3) = rotationFromRobotToCam*xyz(i,:)' + posDepthCam2';
        InitialGuess = rotationFromRobotToCam*xyz(i,:)';
        InitialGuess(4,1) = 1;
        %partGT(i,1:3) = MatrixMult.*InitialGuess;
        something = MatrixMult*InitialGuess;
        partGT(i,1:4) = something';
        partGT(i,4) = thetaNew(i);
    end
    
    %% replot with new ground truth
    xxyyzz = partGT(1,1:3);
    disp("Plotting...")
    if(showPartialOutputs)
        hold on
        pcshow(nonPlane,'ViewPlane','XY');axis on;
        plot3(xxyyzz(1),xxyyzz(2),xxyyzz(3),'-o');
    end
    
    %% get pose from robot to cam and gripper
    mat_R_T_C = robToCam;
    mat_R_T_G = robToGrip;
    
    %% Move robot to object
    sending_goals(partGT(1), partGT(2), partGT(3));
    %[partGT(1), partGT(2), partGT(3)]

end