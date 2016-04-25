function pts = cvalgLineToBorderPoints(lines,imageSize)
% Algorithm for computing the intersection points of lines and image border

% Copyright 2010 The MathWorks, Inc.
% $Revision: 1.1.6.2 $  $Date: 2010/11/17 13:12:13 $
%#codegen

integerClass = 'int32';
outputClass = class(lines);
nPts = cast(size(lines, 2), integerClass);
pts = coder.nullcopy(-ones(4, nPts, outputClass));

% The border of the image is defined as
%   row = -0.5
%   row = imageSize(1) - 0.5
%   col = -0.5
%   col = imageSize(2) - 0.5
firstRow = cast(-0.5, outputClass);
firstCol = cast(-0.5, outputClass);
lastRow = firstRow + cast(imageSize(1), outputClass);
lastCol = firstCol + cast(imageSize(2), outputClass);

% Loop through all lines and compute the intersection points of the lines
% and the image border.
for iLine = 1: nPts
  a = lines(1, iLine);
  b = lines(2, iLine);
  c = lines(3, iLine);
  iPoint = ones(1, integerClass);
  % Check for the intersections with the left and right image borders
  % unless the line is vertical.
  if abs(a) > eps(outputClass)
    % Compute and check the intersection of the line and the left image
    % border. 
    row = - (b * firstCol + c) / a;
    if row>=firstRow && row<=lastRow
      pts(iPoint:iPoint+1, iLine) = [row; firstCol];
      iPoint = iPoint + 2;
    end

    % Compute and check the intersection of the line and the right image
    % border. 
    row = - (b * lastCol + c) / a;
    if row>=firstRow && row<=lastRow
      pts(iPoint:iPoint+1, iLine) = [row; lastCol];
      iPoint = iPoint + 2;
    end
  end

  % Check for the intersections with the top and bottom image borders
  % unless the line is horizontal.
  if abs(b) > eps(outputClass)
    % If we have not found two intersection points, compute and check the
    % intersection of the line and the top image border. 
    if iPoint < 4
      col = - (a * firstRow + c) / b;
      if col>=firstCol && col<=lastCol
        pts(iPoint:iPoint+1, iLine) = [firstRow; col];
        iPoint = iPoint + 2;
      end
    end

    % If we have not found two intersection points, compute and check the
    % intersection of the line and the bottom image border. 
    if iPoint < 4
      col = - (a * lastRow + c) / b;
      if col>=firstCol && col<=lastCol
        pts(iPoint:iPoint+1, iLine) = [lastRow; col];
        iPoint = iPoint + 2;
      end
    end
  end

  % If the line does not intersect with the image border, set the
  % intersection to -1; 
  for iPoint = iPoint: 4
    pts(iPoint, iLine) = -1;
  end
end
end
%eof