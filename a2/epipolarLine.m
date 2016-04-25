function lines = epipolarLine(f,pts)
% Compute epipolar lines for stereo images.
%   Assuming that the fundamental matrix, F, maps points in image I1 to
%   epipolar lines in image I2,
% 
%   LINES = epipolarLine(F,PTS) computes the epipolar lines in I2
%   corresponding to the points, PTS, in I1. 
% 
%   LINES = epipolarLine(F',PTS) computes the epipolar lines in I1
%   corresponding to the points, PTS, in I2. 
% 
%     PTS is a 2-by-N matrix, where each column contains the row
%     and column coordinates of a point. N is the number of points.
% 
%     F is a 3-by-3 fundamental matrix. If P1, a point in I1, corresponds
%     to P2, a point in I2, then [P2;1]' * F * [P1;1] = 0. 
% 
%     LINES is a 3-by-N matrix where each column has the format [A;B;C]
%     which defines a line as A * row + B * col + C = 0.
% 
%   Class Support
%   -------------
%   F must be double or single. PTS must be double, single, or integer.
%
%   Example
%   -------
%   % Use the Least Median of Squares method to find inliers and to
%   % compute the fundamental matrix. The points, matched_points1 and
%   % matched_points2, have been putatively matched.
%   load stereoPointPairs
%   [fLMedS, inliers] = estimateFundamentalMatrix(matched_points1, ...
%     matched_points2, 'NumTrials', 4000);
% 
%   % Show the inliers in the first image.
%   I1 = imread('viprectification_deskLeft.png');
%   figure; 
%   subplot(121); imshow(I1); title('First Image'); hold on;
%   plot(matched_points1(2,inliers), matched_points1(1,inliers), 'go')
% 
%   % Compute the epipolar lines in the first image.
%   epiLines = epipolarLine(fLMedS', matched_points2(:, inliers));
% 
%   % Compute the intersection points of the lines and the image border.
%   pts = lineToBorderPoints(epiLines, size(I1));
% 
%   % Note that Computer Vision System Toolbox uses coordinate system
%   % centered at [0,0]. To use plot, adjust the points accordingly.
%   pts = pts + 1;
%   line([pts(2,:); pts(4,:)], [pts(1,:); pts(3,:)]);
%
%   % Show the inliers in the second image.
%   I2 = imread('viprectification_deskRight.png');
%   subplot(122); imshow(I2); title('Second Image'); hold on;
%   plot(matched_points2(2,inliers), matched_points2(1,inliers), 'go')
% 
%   % Compute and show the epipolar lines in the second image.
%   epiLines = epipolarLine(fLMedS, matched_points1(:, inliers));
%   pts = lineToBorderPoints(epiLines, size(I2));
%   pts = pts + 1;
%   line([pts(2,:); pts(4,:)], [pts(1,:); pts(3,:)]);
%   truesize;
%
% See also estimateFundamentalMatrix, lineToBorderPoints, line, 
% vision.ShapeInserter
%
% References:
%   R. Hartley, A. Zisserman, "Multiple View Geometry in Computer Vision,"
%   Cambridge University Press, 2003.

% Copyright 2010 The MathWorks, Inc.
% $Revision: 1.1.8.2 $  $Date: 2010/11/01 23:11:52 $

checkInputs(f, pts);
nPts = size(pts, 2);
outputClass = class(f);
lines = f * [cast(pts, outputClass); ones(1, nPts, outputClass)];
end

%========================================================================== 
function checkInputs(f, pts)
%--------------------------------------------------------------------------
% Check F
%--------------------------------------------------------------------------
validateattributes(f, {'double', 'single'}, ...
  {'2d', 'nonsparse', 'real'},...
  mfilename, 'F');

if any(size(f) ~= [3, 3])
  error(getMsgID('F'), 'Expected F to be a 3-by-3 matrix.');
end

%--------------------------------------------------------------------------
% Check PTS
%--------------------------------------------------------------------------
validateattributes(pts, {'single', 'double', 'int8', 'int16', ...
  'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64'}, ...
  {'2d', 'nonsparse', 'nonempty', 'real'},...
  mfilename, 'Pts');

if size(pts, 1) ~= 2
  error(getMsgID('Pts'), 'Expected PTS to be a 2-by-N matrix.');
end
end

%========================================================================== 
function id = getMsgID(name)
id = ['vipblks:epipolarLine:invalid', name];
end
%eof