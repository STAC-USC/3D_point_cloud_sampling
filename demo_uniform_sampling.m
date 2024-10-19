% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Script to demonstrate proposed Uniform sampling (fixed-geometry sampling)

 clear
 clc
 close all

extra_package_paths = [genpath("../Utils")];
addpath(extra_package_paths);

point_cloud_path = "./longdress_vox10_1121.ply";
pc = pcread(point_cloud_path);
coordinates = double(pc.Location);


%% Uniform sampling (fixed geometry sampling)
m = 10;  %1/m is the sampling rate
[~, Sampling_set, tEnd] = uniform_sampling(coordinates, m);


%% Reconstruction (label propagation or inbuilt matlab function)

% construct a k-NN graph on the entire point cloud
K = 5;
[~, ~,~, ~, A] = knn_graph(coordinates, K);
    
%Get graph signal (point color attributes)
Crgb = double(pc.Color);
Cyuv = RGBtoYUV(Crgb);
Y = Cyuv(:,1);
npoints = length(Y);
%ChromU = Cyuv(:,2); ChromV = Cyuv(:,3);

D = diag(sum(A,2));
%D_inv = diag(diag(D).^(-1)); Lrw = speye(npoints, npoints) - D_inv*A;
%Y_recon = label_propagation(Lrw, Sampling_set, Y(Sampling_set,:));
    
L = D - A;
Lrr = L(~Sampling_set, ~Sampling_set); Lrs = L(~Sampling_set, Sampling_set);
Yr = -Lrr\(Lrs*Y(Sampling_set,:));%matlab inbilt solver
Y_recon = zeros(npoints,1); Y_recon(~Sampling_set) = Yr; Y_recon(Sampling_set) = Y(Sampling_set,:);


%% Calculate PSNR
psnr = -10*log10(norm(Y_recon(:,1) - Y)^2/(npoints*255^2));
show= sprintf('PSNR (dB): %f, runtime (mins): %f',psnr,tEnd/60);
disp(show);
