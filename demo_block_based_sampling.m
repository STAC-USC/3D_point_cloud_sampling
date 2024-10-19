
% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Script to demonstrate proposed Rreconstruction-aware block sampling (RABS, Algorithm 2 in the paper).
 
 clear
 clc
 close all

extra_package_paths = [genpath("./Utils")];
addpath(extra_package_paths);

point_cloud_path = "./longdress_vox10_1121.ply";
pc = pcread(point_cloud_path);


%% Block-based sampling with/without self_loops
srate = 0.1;
self_loop_fn = @edge_weight_self_loops;
bsize = 64;

[A, Sampling_set, tEnd] = block_based_PC_sampling(pc, srate, self_loop_fn, bsize);

%% Reconstruction (label propagation or inbuilt matlab function)
    
%Get graph signal (point color attributes)
Crgb = double(pc.Color);
Cyuv = RGBtoYUV(Crgb);
Y = Cyuv(:,1);
npoints = length(Y);
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
