function [stdPos, stdAng] = motionModelParams(bias)

if ~exist('bias', 'var')
    bias = 1;
end

%Motion noise
stdPos = 0.08;
stdAng = 0.02 * sqrt(bias);
