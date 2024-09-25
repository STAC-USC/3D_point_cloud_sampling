% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Function to sample a large graph (eg. Point cloud). The sampling set is optimized globally on the entire graph without partitioning
%
% Inputs:
%       A - Adjacency matrix (N x N) of a connected (irreducible) graph, where N is the number of points.
%       self_loop_matrix - Diagonal matrix indicating node weight. Input
%                          sparse(N,N) is there is no self-loops
%       num_samples - sampling budget
%       p - series approx parameter. Recommended: p = ceil(1/(2*srate))
%
%Output:
%       S - N x 1 logical array indicating the sampled points

function [S,tEnd] = Recon_aware_global_sampling(A, self_loop_mat, num_samples, p)
    tStart = tic;
    N = size(A,1);
    %Variation operator
    D = diag(sum(A,2)) + self_loop_mat;
    D_inv = diag(diag(D).^(-1));
    Z = 0.5*(speye(N, N) + D_inv*A);
    LPF = Z;
    for l = 2:p
        LPF = LPF + Z^(l);
    end
    T = LPF'*LPF;%Pre-compute
    vert_imp = diag(T);
    S = false(N,1);
    cum_proj = zeros(N,1);
    %Start sampling
    for j = 1:num_samples
        proj = vert_imp - cum_proj;
        [~, ind] = max(proj(~S));
        s_cur = find(~S);
        S(s_cur(ind)) = true;
        n_vert = s_cur(ind);

        %computing projections
        new_proj = T(:,n_vert);
        cum_proj = cum_proj + (new_proj.^2)/new_proj(n_vert);
    end
    tEnd=   toc(tStart);
end
