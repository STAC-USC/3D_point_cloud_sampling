% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

%Function to compute self-loop weights for each block

function self_loop_mat = edge_weight_self_loops(A, block_logical)
    % Computes the self loop matrix based on the edge weights going outside
    % the block defined by the vertices between first_point and last_point

    npoints = size(A, 1);
    E_edges = A(block_logical,:);
    vec = sparse(npoints, 1);
    vec(~block_logical, :) = 1;

    self_loop_mat = diag(E_edges*vec);
end
