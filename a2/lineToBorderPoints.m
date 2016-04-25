function points = lineToBorderPoints(lines,imageSize)
%lineToBorderPoints Compute the intersection points of lines and image border.
%   POINTS = lineToBorderPoints(LINES,IMAGESIZE) computes the intersection
%   points between one or more lines with the image border.
% 
%     LINES is a 3-by-N matrix where each column has the format of [A;B;C]
%     which defines a line as A * row + B * col + C = 0. N is the number of
%     lines.
%
%     IMAGESIZE is the size of the image, and is in the format returned by
%     the function SIZE.
%
%     POINTS is a 4-by-N matrix where each column has the format of
%     [r1;c1;r2;c2], where [r1;c1] and [r2;c2] are the two intersection
%     points. When a line and the image border do not intersect, the
%     function returns [-1;-1;-1;-1].
%
%   Class Support
%   -------------
%   LINES must be double or single. IMAGESIZE must be double, single, or
%   integer.
%
%   Example
%   -------
%   % Load and display an image.
%   I = imread('rice.png');
%   figure; imshow(I); hold on;
%   % Define a line: 2 * row + col - 300 = 0
%   aLine = [2;1;-300];
%   % Compute the intersection points of the line and the image border.
%   points = lineToBorderPoints(aLine, size(I));
%   % Note that Computer Vision System Toolbox uses coordinate system
%   % centered at [0,0]. To use plot, adjust the points accordingly.
%   points = points + 1;
%   line([points(2); points(4)], [points(1); points(3)]);
%
% See also epipolarLine, line, size, vision.ShapeInserter.

% Copyright 2010 The MathWorks, Inc.
% $Revision: 1.1.8.2 $  $Date: 2010/11/01 23:11:59 $

checkInputs(lines, imageSize);
points = cvalgLineToBorderPoints(lines, imageSize);

%========================================================================== 
function checkInputs(lines, imageSize)
%--------------------------------------------------------------------------
% Check LINES
%--------------------------------------------------------------------------
validateattributes(lines, {'double', 'single'}, ...
  {'2d', 'nonsparse', 'nonempty', 'real'},...
  mfilename, 'LINES');

if size(lines, 1) ~= 3
  error('vision:lineToBorderPoints:invalidLINES',...
    'Expected LINES to be a 3-by-N matrix.');
end

%--------------------------------------------------------------------------
% Check IMAGESIZE
%--------------------------------------------------------------------------
validateattributes(imageSize, {'single', 'double', 'int8', 'int16', ...
  'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64'}, ...
  {'vector', 'nonsparse', 'nonempty', 'real', 'positive', 'integer'},...
  mfilename, 'IMAGESIZE');

if length(imageSize) < 2
  error('vision:lineToBorderPoints:invalidIMAGESIZE', ...
    'Expected IMAGESIZE to be a vector of two or more elements.');
end
