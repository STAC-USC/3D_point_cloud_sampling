% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Script to demonstrate proposed Rreconstruction-aware global sampling (RAGS, Algorithm 1 in the paper).


clear
clc
close all

extra_package_paths = [genpath("../Utils")];
addpath(extra_package_paths);

point_cloud_path = "./longdress_vox10_1121.ply";
pc = pcread(point_cloud_path);
coordinates = double(pc.Location);
npoints = size(V,1);
  
%% construct a k-NN graph on the entire point cloud
K = 5;
[~, ~,~, ~, A] = knn_graph(coordinates, K);


%% Global sampling
srate = 0.1; 
num_samples = floor(srate*npoints);
self_loop_fn = @no_self_loops; %No self-loops for global sampling
bsize = 64;  p = ceil(1/(2*srate));

block_logical = index_bounds_to_logical_arr(npoints, [1 npoints]);
self_loop_mat = self_loop_fn(A, block_logical);

[Sampling_set,tEnd] = Recon_aware_global_sampling(A, self_loop_mat, num_samples, p);

%% Reconstruction (label propagation or inbuilt matlab function)
    
%Get graph signal (point color attributes)
Crgb = double(pc.Color);
Cyuv = RGBtoYUV(Crgb);
Y = Cyuv(:,1);
%ChromU = Cyuv(:,2); ChromV = Cyuv(:,3);

D = diag(sum(A,2));
%D_inv = diag(diag(D).^(-1)); Lrw = speye(npoints, npoints) - D_inv*A;
%Y_recon = label_propagation(Lrw, Sampling_set, Y(Sampling_set,:));
    
L = D - A;L = L + (10^(-10)*speye(npoints, npoints)); 
Lrr = L(~Sampling_set, ~Sampling_set); Lrs = L(~Sampling_set, Sampling_set);
Yr = -Lrr\(Lrs*Y(Sampling_set,:));%matlab inbilt solver
Y_recon = zeros(npoints,1); Y_recon(~Sampling_set) = Yr; Y_recon(Sampling_set) = Y(Sampling_set,:);


%% Calculate PSNR
psnr = -10*log10(norm(Y_recon(:,1) - Y)^2/(npoints*255^2));
show= sprintf('PSNR (dB): %f, runtime (mins): %f',psnr,tEnd/60);
disp(show);
