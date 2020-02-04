% Generate Arbitrary Error Curve for Depth Error

clear;
close all;

addpath(genpath('Images/')); % Add folders recursively

% image = imageDatastore(fullfile(toolboxdir('vision'),'visiondata','calibration','slr'));
% Get set of images in a struct

images = imageDatastore('Images/Colour');

image1 = imread('RGB1_Color.png');
image2 = imread('RGB2_Color.png');

depth1 = imread('Depth1_Depth.png');
depth2 = imread('Depth2_Depth.png');

[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);
squareSize = 20;
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

I = readimage(images,1);
imageSize = [size(I,1), size(I,2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);

% showMatchedFeatures(image1,image2,imagePoints(:,:,1), imagePoints(:,:,2),'montage');
% Show the correspondences between the checkerboards

%% Getting the pixel co-ords of the corners of the image in the colour images

% pointsToWorld - is the 2D world co-ords, we want 3D solution

% Getting x,y points and depth from them manually by corners:
% Test 1:
% Top left 

[M,pos]=min(imagePoints(:,:,1),[],1);
depthLocTop = imagePoints(pos(1),:,1);

[M,pos]=max(imagePoints(:,:,1),[],1);
depthLocBot = imagePoints(pos(1),:,1);

% THIS DOES NOT WORK - IR CAMERA NOT ALIGNED WITH RGB CAMERA, CANNOT
% COMPARE CO-ORDS - USE RS-ALIGN EXAMPLE TO DO
% However, we still use the above for the calculations of co-ord
% transformation

%% Now do error reprojection using depth info

% For image 1, top left corner distance = 0.31m, bottom right = 0.35m
% For image 2, top left = 0.32m, bottom right = 0.26m


% Try one of the used images first
% [img1Undist,newOrigin] = undistortImage(image1,cameraParams,'OutputView','full');
    % Doing an undistortion - skip for now
    
% Extract Rotation, Translation Matrices
rotMat_1 = cameraParams.RotationMatrices(:,:,1);
rotMat_2 = cameraParams.RotationMatrices(:,:,2);

transVec_1 = cameraParams.TranslationVectors(1,:);
transVec_2 = cameraParams.TranslationVectors(2,:);
    
% Compute and display camera pose
close all;
figure
hold on

% Camera 1
[orient_1, loc_1] = extrinsicsToCameraPose(rotMat_1, transVec_1);

plotCamera('Location', loc_1, 'Orientation', orient_1, 'Size', 20);
pcshow([worldPoints,zeros(size(worldPoints,1),1)], 'VerticalAxisDir','down','MarkerSize',40);

% Camera 2
[orient_2, loc_2] = extrinsicsToCameraPose(rotMat_2, transVec_2);

plotCamera('Location', loc_2, 'Orientation', orient_2, 'Size', 20);

a = 0;

% test = pointsToWorld(cameraParams, rotMat_1, transVec_1, imagePoints(:,:,1));

%% Using pinhole camera model formula: (see Comp. Vision week 5 Lecture)

% TESTING: TO SET THE Z DISTANCE ON THE TRANSLATION VECTOR TO TRY AND
% CONVERT TO MM

% Transvec_2 z value = 317 pixels
%ratio = transVec_1(3) / transVec_2(3);
%transVec_2(3) = 2;
%transVec_1(3) = transVec_2(3) * ratio;

% Testing 2: convert the depth reading into pixels
depth_1 = 310; %310mm
depth_2 = 320; %320mm
% depth_1 = 20000; % say that 640px ~ 1cm

% Camera Intrinsics: keep square as we need to invert
intrinsicMat = [cameraParams.IntrinsicMatrix',zeros(3,1)];
% Extrinsic Matrices
extMat_1 = [rotMat_1',transVec_1';0,0,0,1]; % NEED TO TRANSPOSE ALL VALUES
extMat_2 = [rotMat_2',transVec_2';0,0,0,1];

% To get world co-ords, have to undo Xc = RXw +t: Xw = (Xc - t)R', and we
% also have to undo the intrinsic parameters first on Xc

% For image 1, top left corner distance = 0.31m, bottom right = 0.35m
% For image 2, top left = 0.32m, bottom right = 0.26m

%depth_1 = 450;
topVector = [depthLocTop,1]; 
% Convert to world co-ordinates
% Might be incorrect due to distance units being wrong? Pixels vs cm

temp1 = (cameraParams.IntrinsicMatrix') \ topVector';

depth_1 = 352;
temp2 = temp1 .* depth_1; % Replace z value with depth to first image

temp2(4) = 1; % Add 1 to make a 4x1 vector
temp3 = extMat_1 \ temp2;

%temp2 = temp1 - transVec_1';
%temp3 = temp2' / rotMat_1;

topWorldPoint = temp3;
% topWorldPoint = [0;120;0;1]; %Actual Position

% Then convert into camera 2's frame of reference
topPoint_2 = intrinsicMat * extMat_2 * topWorldPoint;
topPoint_1 = intrinsicMat * extMat_1 * topWorldPoint;

% Scale so that z value = 1
topPoint_2 = topPoint_2 ./ topPoint_2(3);
topPoint_1 = topPoint_1 ./ topPoint_1(3);

%% Potential Depth Error: Generating Points
% According to Ahn et al., variance at 1m measuring distance = 0.01m
% RMS error = (square) root mean square error, i.e. standard deviation
% RMS error at <1m is bound to under 5mm, averaging around 4mm

% For standard deviations, within 1 = 67%, within 2 = 95%, within 3=99.7%
% ASSUMING THE SHAPE OF THE PROBABILITY CURVE IS NORMAL (which in this case
% it is not - better fits a gaussian distribution)

% Fixed error range using 4mm, 3std deviations each way
errorRange = [[-3:1:3] * 4];

% Smaller Error Range
%errorRange = [0, [-.01:0.001:0.01] * depth_1];

% Larger Error Range
%errorRange = [0, [-.1:0.005:0.1] * depth_1];

% Potential depth offsets in the world point
topWorldPoint = zeros(4,size(errorRange,2));
topWorldPoint(1,:) = 0; % 0 for top left
topWorldPoint(2,:) = 120; % 120 for top left
topWorldPoint(3,:) = errorRange;
topWorldPoint(4,:) = 1;

% Then convert into camera 2's frame of reference
topPoint_2 = intrinsicMat * extMat_2 * topWorldPoint;
topPoint_2 = topPoint_2 ./ topPoint_2(3,:);


%% Plotting

% Plot on images
close all

imshow([image1,image2],'InitialMagnification',200); % Show images side by side
hold on

plot(imagePoints(:,1,1),imagePoints(:,2,1),'r*'); % Show original detected checkerboard points
plot(imagePoints(:,1,2)+640,imagePoints(:,2,2),'r*');

% Plot original point
plot(topVector(1),topVector(2),'o-','MarkerSize',20,'MarkerFaceColor','Green'); 

centrePoint = ceil(size(topPoint_2,2)/2);
% Plot base reprojected point
plot(topPoint_2(1,centrePoint)+640,topPoint_2(2,centrePoint),'o-',...
    'MarkerSize',10,'MarkerFaceColor','Green'); 

pointsToPlot = [centrePoint+1,centrePoint-1];
% Plot 1 std. dev. away
plot(topPoint_2(1,pointsToPlot)+640,topPoint_2(2,pointsToPlot),'o-',...
    'MarkerSize',10,'MarkerFaceColor','Yellow');

pointsToPlot = [centrePoint+2,centrePoint-2];
% Plot 1 std. dev. away
plot(topPoint_2(1,pointsToPlot)+640,topPoint_2(2,pointsToPlot),'o-',...
    'MarkerSize',10,'MarkerFaceColor','Magenta');

pointsToPlot = [centrePoint+3,centrePoint-3];
% Plot 1 std. dev. away
plot(topPoint_2(1,pointsToPlot)+640,topPoint_2(2,pointsToPlot),'o-',...
    'MarkerSize',10,'MarkerFaceColor','Red');

%{
 Old

topWorldPoint = (inv(cameraParams.IntrinsicMatrix') * topVector' - transVec_1')' * rotMat_1';

%topWorldPoint = [120, 120, 0];



% Then convert into camera 2's frame of reference


topPoint_2 = intrinsicMat * extMat_2 * [topWorldPoint,1]';

%disp(topPoint_2);


% Using Matlab's format - horizontal world point vector, Rt vertically
% stacked, K transposed
K = cameraParams.IntrinsicMatrix;
matlabExtMat_2 = [cameraParams.RotationMatrices(:,:,2);cameraParams.TranslationVectors(2,:)];
matlabTopPoint_2 = [topWorldPoint,1]*matlabExtMat_2*K;
% matlabTopPoint_2 = matlabTopPoint_2 ./ matlabTopPoint_2(3); % Scale down
% so z in camera coord = 1

disp(matlabTopPoint_2);

% Using inbuilt functions - estimating the fundamental matrix and then
% calculating using epipolar geometry E =
% estimateEssentialMatrix(imagePoints(:,:,1),imagePoints(:,:,2),cameraParams);

% OPTIONAL - SKIP A TRANSFORM USING RELATIVE CAMERA TRANSFORMATION Get
% relative orientation and translation [relOrient,relTrans] =
% relativeCameraPose(E, cameraParams,
% imagePoints(:,:,1),imagePoints(:,:,2));

% Do manually using p = rq + t to project to world co-ords and back
% relOrient_Manual = rotMat_2' * rotMat_1; % Similar but not the same
%relTrans_Manual = (rotMat_2' * (transVec_1 - transVec_2)')';

%}