% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Function implements block-based sampling of a large graph (eg. Point cloud). 
% 
% Inputs:
%       pc - .ply file of the point cloud
%       srate - sampling rate (srate = num_samples/N)
%       self_loop_fn - Function handle (@no_self_loops or @edge_weight_self_loops)
%       bsize - Block size (bsize = 2^k)
%       p - optional p value. if not given the algorithm calculates p from srate

%Output:
%       S - N x 1 logical array indicating the sampled points


function [A, Sampling_set, tEnd] = block_based_PC_sampling(pc, srate, self_loop_fn, bsize, p)


    %calculate/set p(series_approx)
    if nargin == 5, series_approx= p;end
    if nargin < 5, series_approx = ceil(1/(2*srate));end

    %Get coordinates and attributes of PC
    V = double(pc.Location);
    npoints = size(V,1);
    
    %% construct a k-NN graph on the entire point cloud
    K = 5;
    [~, ~,~, ~, A] = knn_graph(V, K);
    n_verts = size(A, 1);


    %% define block size and partition the entire point cloud
    b = bsize;
    start_indices = block_indices(V, b);
    end_indices = [start_indices(2:end)-1;npoints];
    ni = end_indices - start_indices +1;
    to_change = find(ni ~= 0);
   
    tStart = tic;
    %% Initialize the sampling set
    Sampling_set = zeros(npoints,1);
  
   
   
   % Sample blocks independently
   for currblock =1:length(to_change)
        first_point = start_indices(to_change(currblock));
        last_point  = end_indices(to_change(currblock));

        Adj = A(first_point:last_point, first_point: last_point);
        block_logical = index_bounds_to_logical_arr(n_verts, [first_point last_point]);

        self_loop_mat = self_loop_fn(A, block_logical);
        [sampled_vertices] = sampl_func(Adj, srate, self_loop_mat, series_approx);
        Sampling_set(first_point:last_point,:) = sampled_vertices;
    end
    Sampling_set = logical(Sampling_set);
    tEnd=   toc(tStart);
end





