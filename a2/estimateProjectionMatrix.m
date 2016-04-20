% Assignment 4 Computer Vision 2016
% author: Ysbrand Galama, 10262067
% Code copied from my previous experience with convolutions:
%   BSc KI cource Beeldverwerken, assignment 4 (2013)
%   and my improvements assisting the same course (2014)

% calculate a transformation matrix from the given from and to points
% so 0=uv'*projMatrix*xy
function projMatrix = estimateProjectionMatrix(xy, uv)
    o = ones(1,size(xy,2));
    % normalise
    [xy,T1] = normalise(xy,o);
    [uv,T2] = normalise(uv,o);
    % set the points
    x = xy(1, :);
    y = xy(2, :);
    u = uv(1, :);
    v = uv(2, :);
    
    % make the matrix and calculate xAu = 0
    A = [x.*u; x.*v; x; y.*u; y.*v; y; u; v; o;]';
    [~, ~, V] = svd(A);
    m = V(:, end );
    % reshape m into the 3x3 projection matrix M
    projMatrix = reshape (m, 3, 3)';
    [U,D,V] = svd(projMatrix);
    D(end,end) = 0;
    projMatrix = U*D*V';
    projMatrix = T2'*projMatrix*T1;
end

function [xy,T] = normalise(xy,o)
    m = mean(xy,2);
    d = mean(sqrt(sum((xy - repmat(m,1,size(xy,2)) ).^2, 1)));
    sq2d = sqrt(2)/d;
    T = [sq2d 0 -m(1)*sq2d; 0 sq2d -m(2)*sq2d; 0 0 1];
    xy = T*[xy;o];
end