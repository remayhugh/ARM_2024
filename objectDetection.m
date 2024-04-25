% read image and point cloud data
disp("Reading image data...")
rgbImage = imread("sample_rgbImage.png"); % we will need this .png
ptCloud = pcread("sample_ptCloud.ply"); % we will need this .ply
% to see image that we are working with
imshow(rgbImage)

% set flag
showPartialOutputs = true;

% load pretrained YOLO v4 network
disp("Loading detector data...")
pretrained = load('trainedYOLOv4Detector_cuboid.mat'); % we will need this .mat (gives detector, elapsed time, & info)
trainedYoloNet = pretrained.detector;

% compute bounding boxes (need this function)
[bboxes,scores,labels] = detectObjectsYoloNet(trainedYoloNet,rgbImage);

% draw bounding boxes on image
disp("Drawing bounding boxes...")
if(showPartialOutputs)
    % visualize the detected object bounding box
    annotatedImage = insertObjectAnnotation(im2uint8(rgbImage), 'Rectangle',...
        bboxes, string(labels)+":"+string(scores),'Color','r');
    figure, imshow(annotatedImage);
end

% specify percentage of acceptable bounding box
disp("Updating valid bounding boxes...")
valid_idx = scores > 0.9997;
bboxes = bboxes(valid_idx, :);
scores = scores(valid_idx);
labels = labels(valid_idx);
numObjects = size(bboxes,1);

% redraw updated bounding boxes on image
disp("Redrawing updated bounding boxes...")
if(showPartialOutputs)
    % visualize the detected object bounding box
    reannotatedImage = insertObjectAnnotation(im2uint8(rgbImage), 'Rectangle',...
        bboxes, string(labels)+":"+string(scores),'Color','b');
    figure, imshow(reannotatedImage);
end

% create inverted point cloud visually
planeThickness = 0.02;
normalVector = [0,0,1];
maxPlaneTilt = 5;
[param, planeIdx, nonPlaneIdx] = pcfitplane(ptCloud, planeThickness, normalVector, maxPlaneTilt);
plane = select(ptCloud, planeIdx);
nonPlane = select(ptCloud, nonPlaneIdx);
if(showPartialOutputs)
    figure,pcshow(plane,'ViewPlane','XY');axis on;
end

% create non-plane mask
% get size of image
[m,n,~] = size(rgbImage);
% create variable with dimensions of image
nonPlaneMask = zeros(m,n);
% make variable only a column
nonPlaneMask =nonPlaneMask(:);
% use nonPlaneIdx to fill nonPlaneMask with useful data
nonPlaneMask(nonPlaneIdx)= 1;

% estimate object pose (need this function)
gridDownsample = 0.001;
[xyz,theta,ptCloud_vec,scene_pca_vec] = findObjectPoses(ptCloud,rgbImage, bboxes, gridDownsample, nonPlaneMask);

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



