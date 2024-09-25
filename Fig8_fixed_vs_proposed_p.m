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
srates = [0.12, 0.16, 0.23,0.30, 0.40, 0.45];
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

p_uni = ceil(1./(2*srates));
p_rand = ceil(-1./(2*log(1-srates)));


% Plot the results
idx_uni = sub2ind([6,5], [1,2,3,4,5,6], p_uni);
idx_rand = sub2ind([6,5], [1,2,3,4,5,6], p_rand);
figure;

scatter(srates, psnr_arr(idx_uni),100 ,"diamond",'filled', 'g'); hold on;
scatter(srates, psnr_arr(idx_rand),100 ,"^",'filled', 'b');
scatter(srates, psnr_arr(:,5), 100,"hexagram",'filled', 'k'); hold on;
ax = gca;
ax.FontSize = 14;
xlabel("Sampling rates", "fontSize", 20);
xticks(srates);
xticklabels({'12%','16%', '23%','30%', '40%', '45%'});
ylabel("PSNR(dB)", "fontSize", 20);
legend({"$p = \lceil \frac{1}{2 \alpha} \rceil$", "$p = \lceil \frac{-1}{2 log(1- \alpha)} \rceil$", '$p=5$'}, 'Interpreter', 'latex', 'fontSize', 20);
legend boxoff
title("Longdress", 'fontSize', 20);

figure;

scatter(srates, runtime_arr(idx_uni),100, "diamond",'filled', 'g');  hold on;
scatter(srates, runtime_arr(idx_rand),100 ,"^",'filled', 'b');
scatter(srates, runtime_arr(:,5), 100,"hexagram",'filled', 'k'); hold on;
ax = gca;
ax.FontSize = 14;
xlabel("Sampling rates", "fontSize", 20);
xticks(srates);
xticklabels({'12%','16%', '23%','30%', '40%', '45%'});
ylabel("runtime (mins)", "fontSize", 20);
legend({"$p = \lceil \frac{1}{2 \alpha} \rceil$", "$p = \lceil \frac{-1}{2 log(1- \alpha)} \rceil$", '$p=5$'}, 'Interpreter', 'latex', 'fontSize', 20);
legend boxoff
title("Longdress", 'fontSize', 20);




