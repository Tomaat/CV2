% Assignment 4 Computer Vision 2016
% author: Ysbrand Galama, 10262067
% Code copied from my previous experience with convolutions:
%   BSc KI cource Beeldverwerken, assignment 4 (2013)
%   and my improvements assisting the same course (2014)

% calculate a transformation matrix from the given from and to points
% so uv=projMatrix*xy
function projMatrix = estimateProjectionMatrix(xy, uv)
    % set the points
    x = xy (:, 1);
    y = xy (:, 2);
    u = uv (:, 1);
    v = uv (:, 2);
    o = ones ( size (x));
    z = zeros ( size (x));
    % make the matrix and calculate Am = 0
    Aoddrows = [x, y, o , z, z, z, -u.*x, -u.*y, -u];
    Aevenrows = [z, z, z, x, y, o, -v.*x, -v.*y, -v];
    A = [ Aoddrows ; Aevenrows ];
    [~, ~, V] = svd(A);
    m = V(:, end );
    % reshape m into the 3x4 projection matrix M
    projMatrix = reshape (m, 3, 3)';
end