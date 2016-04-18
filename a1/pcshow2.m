function pcshow2(f,varargin)
    for i=1:length(varargin)
        A = varargin{i};
        if numel(A) < f
            continue
        end
        
        if size(A,1) == 4
            C = A(4,1:f:end);
        else
            C = zeros(size(A(1,1:f:end)));
        end
        S = ones(size(C))*10;

        scatter3(A(1,1:f:end),A(3,1:f:end),-A(2,1:f:end),S);%,C)
        hold on
    end
    hold off
    %axis([-.5 .5 .2 1.2 -.5 .5])
end