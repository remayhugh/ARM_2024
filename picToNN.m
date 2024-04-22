function [bboxes,scores,labels] = picToNN(x,y,z)
% Takes photo and sends to neural network
%   Takes x,y,z coordinate input, robot moves to that location and takes a
%   picture. 
%   Function determines bounding boxes, scores, and labels
%   Function outputs annotated image with boxes, labels, and scores

%% 1. Call Class to Create Subscriber
r = rosClassHandle;

%% 2. Send Robot to Photo Location
% this part doesn't work yet
xyz_pos = [x,y,z];
pos = transl(xyz_pos);

    optns = dictionary();                 % Type of global dictionary with all options to facilitate passing of options
    optns("debug")               = 0;     % If set to true visualize traj before running  
    optns("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    optns("traj_steps")          = 1;     % Num of traj steps
    optns("z_offset")            = 0.09;  % Vertical offset for top-down approach
    optns("traj_duration")       = 2;     % Traj duration (secs)   
    
    optns("frameAdjustmentFlag") = 1;
    optns("toolAdjustmentFlag")  = 1;
    optns("toolAdjustment")      = 0.165; % Distance from tool0 to gripper_tip_link

traj_result = moveTo_no(pos,optns);

%% 2. Take Picture
rosImg  = receive(r.rgb_sub);
myImg   = readImage(rosImg);

% If need to, can switch to rosReadImage which will read a structure - but
% will need to change subscriber message type in the rosClassHandle code
%% 4. Attempt Bounding Boxes
%rgbImage = imread(myImg); % we will need this .png
imshow(myImg)

pretrained = load('trainedYOLOv4Detector_cuboid.mat'); % we will need this .mat (gives detector, elapsed time, & info)
trainedYoloNet = pretrained.detector;

[bboxes,scores,labels] = detectObjectsYoloNet(trainedYoloNet,myImg);

%% 5. visualize the detected object bounding box
annotatedImage = insertObjectAnnotation(im2uint8(myImg), 'Rectangle',...
    bboxes, string(labels)+":"+string(scores),'Color','r');
figure, imshow(annotatedImage);

end