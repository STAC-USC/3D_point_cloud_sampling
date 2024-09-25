% Graph-based scalable sampling and reconstruction for point clouds
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California


% Function to reconstruct from samples using label propagation [1]
% Reference:
% [1] X. Zhu and Z. Ghahramani, “Learning from labeled and unlabeled data
%     with label propagation," 2002
%
% Inputs:
%       Lrw - Random-walk Laplacian (N x N), where N is the number of points.
%       sampled_vertices - N x 1 logical array indicating the sampled points
%       Ys - sampled original signal
%
%Output:
%       Y_recon - N x 1 reconstructed signal

function [Y_recon] = label_propagation(Lrw, sampled_vertices, Ys)
    % One-hop low-pass filter based on random-walk Laplacian
    N = size(Lrw,1);
    LPF = (speye(N, N) - 0.5*(Lrw));
    dim = size(Ys, 2);
    %Initialize
    Y_du = zeros(N, dim); Y_du(sampled_vertices,:) = Ys;
    Y_recon = LPF*Y_du;
    num_itr = 1500;
    for i = 1:num_itr
        % Sample consisent step
        err_s = (Y_du-Y_recon);
        err_s(~sampled_vertices,:) = 0; % error on the known set
    
        % Low-pass filtering step
        Y_temp = LPF*(Y_recon + err_s);
        Y_recon = Y_temp;
    end
    %Final sample consistent step
    Y_recon(sampled_vertices,:) = Ys;
end

