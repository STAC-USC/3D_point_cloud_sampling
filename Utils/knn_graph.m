% Graph-based scalable sampling and reconstruction for point clouds
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California


% Function to construct a k-NN graph on 3D point cloud
% Reference:
% [1] 
%
%   Inputs:
%       V - coordinates of 3D point cloud (N x 3), where N is the number of points.
%       k (int) - number of neighbors (k=5)
%      
%
%   Output:
%       A - Adjacency matrix (N x N)

function [edges, idx,D, distanceVector, A] = knn_graph(V, K)

    [idx, D] = knnsearch(V, V, 'K',K+1);
    edges = zeros(size(idx,1)*(K+1),2);
    distanceVector = D(:);

    indexList = idx(:,1); 
    N = size(V,1);

    for k=2:K+1
        starti = (k-1)*N +1;
        endi = starti + N-1;
        edges(starti:endi,:) = [indexList, idx(:,k)];
    end

    %Remove repeated edges and their corresponding distances
    [edges, ia, ~] = unique(edges,'rows');
    distanceVector = distanceVector(ia);

    edges = edges(2:size(edges,1),:);
    distanceVector = distanceVector(2: size(distanceVector,1),:);

    %Construct Adjacency matrix
    sigma = mean(distanceVector)/3;
    weights = exp(-distanceVector.^2/(2*sigma*sigma));
    A = sparse(edges(:,1), edges(:,2), weights, N,N);
    A = (A + A')/2;

end
