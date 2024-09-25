
% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Script generates the Fig7 in the paper. 

% It demonstrates the effect of p on reconstruction PSNR and runtime


clear
clc
close all

extra_package_paths = [genpath("./Utils")];
addpath(extra_package_paths);

point_cloud_path = "/home/shashank/Desktop/Point_cloud_attribute_sampling_TIP202x/TestPCs/Experiments/longdress_vox10_1121.ply";
pc = pcread(point_cloud_path);

%Get graph signal (point color attributes)
Crgb = double(pc.Color);
Cyuv = RGBtoYUV(Crgb);
Y = Cyuv(:,1);
npoints = length(Y);
        

%% Block-based sampling with/without self_loops
srates = [0.1, 0.15, 0.2, 0.25, 0.33, 0.40];
p = [1, 2, 3, 4, 5];
self_loop_fn = @edge_weight_self_loops;
bsize = 64;
runtime_arr = zeros(length(srates), length(p));
psnr_arr = zeros(length(srates), length(p));
for i = 1:length(srates)
   for j = 1:length(p)
       [A, Sampling_set, tEnd] = block_based_PC_sampling(pc, srates(i), self_loop_fn, bsize ,p(j));

       % Reconstruction (inbuilt matlab function)
        D = diag(sum(A,2));
        L = D - A;L = L + (10^(-10)*speye(npoints, npoints)); 
        Lrr = L(~Sampling_set, ~Sampling_set); Lrs = L(~Sampling_set, Sampling_set);
        Yr = -Lrr\(Lrs*Y(Sampling_set,:));%matlab inbilt solver
        Y_recon = zeros(npoints,1); Y_recon(~Sampling_set) = Yr; Y_recon(Sampling_set) = Y(Sampling_set,:);
        % Calculate PSNR
        psnr = -10*log10(norm(Y_recon(:,1) - Y)^2/(npoints*255^2));

        psnr_arr(i, j) = psnr;
        runtime_arr(i,j) = tEnd/60;
        disp(psnr_arr)
   end
end

% Plot the results
figure;
scatter(srates, psnr_arr(:,1),100, "square",'filled', 'r'); hold on;
scatter(srates, psnr_arr(:,2), 100,"diamond",'filled', 'g'); hold on;
scatter(srates, psnr_arr(:,3),100 ,"^",'filled', 'b'); hold on;
scatter(srates, psnr_arr(:,4),100 ,"hexagram",'filled', 'c');hold on;
scatter(srates, psnr_arr(:,5),100 ,"*", 'k');
ax = gca;
ax.FontSize = 14;
xlabel("Sampling rates", "fontSize", 16);
xticks(srates);
xticklabels({'10%','15%', '20%','25%', '33%', '40%'});
ylabel("PSNR(dB)", "fontSize", 16);
legend({"$p=1$", "$p=2$", "$p=3$","$p=4$", "$p=5$"}, 'Interpreter', 'latex', 'fontSize', 16);
legend boxoff
title("Longdress: PSNR", 'fontSize', 16);

figure;
scatter(srates, runtime_arr(:,1),100, "square",'filled', 'r'); hold on;
scatter(srates, runtime_arr(:,2), 100,"diamond",'filled', 'g'); hold on;
scatter(srates, runtime_arr(:,3),100 ,"^",'filled', 'b'); hold on;
scatter(srates, runtime_arr(:,4),100 ,"hexagram",'filled', 'c'); hold on;
scatter(srates, runtime_arr(:,5),100 ,"*", 'k');
ax = gca;
ax.FontSize = 14;
xlabel("Sampling rates", "fontSize", 16);
xticks(srates);
xticklabels({'10%','15%', '20%','25%', '33%', '40%'});
ylabel("PSNR(dB)", "fontSize", 16);
legend({"$p=1$", "$p=2$", "$p=3$","$p=4$", "$p=5$"}, 'Interpreter', 'latex', 'fontSize', 16);
legend boxoff
title("Longdress: PSNR", 'fontSize', 16);

