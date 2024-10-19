% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

%Function to compute self-loop weights for each block (zero self loop weights in this case)


function loop_wts_mat = no_self_loops(A, block_logical)
    n_block = nnz(block_logical);
    loop_wts_mat = sparse(n_block, n_block);
end
