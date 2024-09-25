%Uniform sampling and reconsturction
%Authors - Shashank Sridhara, Eduardo Pavez
%ICASSP-2022 : Chroma subsampling on point clouds

% Function implements uniform sampling for point clouds [1]
%   References:
%       [1] S. N. Sridhara, E. Pavez, A. Ortega, R. Watanabe, and K. Nonaka, “Point
%       cloud attribute compression via chroma subsampling," in IEEE-ICASSP (ICASSP), 2022, pp. 2579–2583.
%
%   Inputs:
%       V - coordinates of 3D point cloud (N x 3), where N is the number of points.
%       m (int) - Sampling rate (1/m)
%      
%
%   Output:
%       blue - Sampling set, red - Complement of the sampling set

function [red, blue, tEnd] = uniform_sampling(V, m)
    tStart = tic;
    N = size(V,1);
    blue  =(mod(V(:,1) + V(:,2),m) == mod( V(:,3),m));
    red = ~blue;
    tEnd=   toc(tStart);
end
