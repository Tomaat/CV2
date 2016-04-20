% Assignment 4 Computer Vision 2016
% author: Ysbrand Galama, 10262067
% Code copied from my previous experience with convolutions:
%   BSc KI cource Beeldverwerken, assignment 4 (2013)
%   and my improvements assisting the same course (2014)

% This function uses the RANSAC algorithm on the given Data
% with given error, iterations k and part of the first step p
% it returns the median m value and the inliers IN
function [T, bestIN] = RANSAC(Data,eps,k)
    [~,x] = size(Data);
    bestIN = [];
    for i=1:k % loop iteratins
        
        perm = randperm(x); %
        IN = Data(:,perm(1:4)); % get a random permutatio of 4 points
        other = Data(:,perm(5:end));
        M = createProjectionMatrix(IN(1:2,:)',IN(3:4,:)');
        a = M*[other(1:2,:);ones(1,x-4)];
        a = a ./ repmat(a(3,:),3,1);
        
        dis = sqrt(sum( (a(1:2,:)-other(3:4,:)).^2 ));
        
        
        truePos = other(:,dis < eps); % threshhold
        
        if size(truePos,2) > size(bestIN,2)
            bestIN = truePos;
        end
    end %end for loop iteration
    T = estimateProjectionMatrix(bestIN(1:2,:)',bestIN(3:4,:)');
end % end function