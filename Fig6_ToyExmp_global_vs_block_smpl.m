
% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

% Script generates the Fig6 in the paper. 

% It demonstrates that block-sampling without self-loops result in oversampling around block
% boundaries

clear
clc
close all


%% Construct grid graph

cnt = 1;
cols = 10; rows = 5;
N = cols*rows;
V = zeros(N , 2);
for i = 1:cols
    for j = 1:rows
        V(cnt,:) = [i,j];
        cnt = cnt+1;
    end
end

[edges, idx,D, distanceVector, ~] = knn_graph(V, 5);
R = sqrt(2);
if(R < inf && R>0)
    keep = (distanceVector < R);
    edges = edges(keep,:);
    distanceVector = distanceVector(keep);
end
edges = edges(2:size(edges,1),:);
distanceVector = distanceVector(2: size(distanceVector,1),:);
%Construct Adjacency matrix
sigma = mean(distanceVector)/3;
A = sparse(edges(:,1), edges(:,2), ones(length(distanceVector),1), N,N);
A = (A + A')/2;
I = edges(:,1); J = edges(:,2);

%% Plot global sampling results (Fig 6a)
%set parameters to sample
p = 3; num_samples = 12;

% 2) perform global sampling and plot
S = Recon_aware_global_sampling(A, sparse(N, N), num_samples, p);
Vs = V(S,:); Vsc = V(~S,:);
figure
scatter(Vs(:,1), Vs(:,2), 500, 'r', 'o', 'filled'); hold on;
scatter(Vsc(:,1), Vsc(:,2), 200, 'b', 'o'); hold on;
X = [ V(I(:),1) V(J(:),1) repmat(NaN,size(I))]';
Y = [ V(I(:),2) V(J(:),2) repmat(NaN,size(I))]';
X = X(:);
Y = Y(:);
title("Global sampling", 'fontSize', 16);
axis equal
plot(X, Y, 'color', 'k', 'LineStyle', '-.', 'linewidth', 0.5);


%% Plot block sampling without self-loops
A_sub1 = A(1:25, 1:25);A_sub2 = A(26:50, 26:50);num_samples_sub = 6;
S1 = Recon_aware_global_sampling(A_sub1, sparse(25, 25), num_samples_sub, p);
S2 = Recon_aware_global_sampling(A_sub2, sparse(25, 25), num_samples_sub, p);

[I_sub1, J_sub1, ~] = find(A_sub1);
Vb1 = V(1:25,:);
Vs1 = Vb1(S1,:); Vsc1 = Vb1(~S1,:);

[I_sub2, J_sub2, ~] = find(A_sub2);
Vb2 = V(26:50,:);
Vs2 = Vb2(S2,:); Vsc2 = Vb2(~S2,:);


figure;
clf;
subplot(121);
scatter(Vs1(:,1), Vs1(:,2), 300, 'r', 'o', 'filled'); hold on;
scatter(Vsc1(:,1), Vsc1(:,2), 100, 'b', 'o'); hold on;
X = [ Vb1(I_sub1(:),1) Vb1(J_sub1(:),1) repmat(NaN,size(I_sub1))]';
Y = [ Vb1(I_sub1(:),2) Vb1(J_sub1(:),2) repmat(NaN,size(I_sub1))]';
X = X(:);
Y = Y(:);
axis equal
plot(X, Y, 'color', 'k', 'LineStyle', '-.', 'linewidth', 0.5);
title("S1", 'fontSize', 16);
hold off;

subplot(122);
scatter(Vs2(:,1), Vs2(:,2), 300, 'k', 'o', 'filled'); hold on;
scatter(Vsc2(:,1), Vsc2(:,2), 100, 'b', 'o'); hold on;
X = [ Vb2(I_sub2(:),1) Vb2(J_sub2(:),1) repmat(NaN,size(I_sub2))]';
Y = [ Vb2(I_sub2(:),2) Vb2(J_sub2(:),2) repmat(NaN,size(I_sub2))]';
X = X(:);
Y = Y(:);
axis equal
plot(X, Y, 'color', 'k', 'LineStyle', '-.', 'linewidth', 0.5);
title("S2", 'fontSize', 16);
sgtitle("Block sampling w/o self-loops", 'fontSize', 14)

%% Plot block sampling with self-loops

A_sub1 = A(1:25, 1:25);A_sub2 = A(26:50, 26:50);num_samples_sub = 6;
c1 = zeros(N,1); c1(26:50,:) = ones(25,1);
c2 = zeros(N,1); c2(1:25,:) = ones(25,1);
self1 = A(1:25,:)*c1; self1 = sparse(diag(self1));
self2 = A(26:50,:)*c2; self2 = sparse(diag(self2));

S1 = Recon_aware_global_sampling(A_sub1, self1, num_samples_sub, p);
S2 = Recon_aware_global_sampling(A_sub2, self2, num_samples_sub, p);

[I_sub1, J_sub1, ~] = find(A_sub1);
Vb1 = V(1:25,:);
Vs1 = Vb1(S1,:); Vsc1 = Vb1(~S1,:);

[I_sub2, J_sub2, ~] = find(A_sub2);
Vb2 = V(26:50,:);
Vs2 = Vb2(S2,:); Vsc2 = Vb2(~S2,:);


figure;
clf;
subplot(121);
scatter(Vs1(:,1), Vs1(:,2), 300, 'r', 'o', 'filled'); hold on;
scatter(Vsc1(:,1), Vsc1(:,2), 100, 'b', 'o'); hold on;
X = [ Vb1(I_sub1(:),1) Vb1(J_sub1(:),1) repmat(NaN,size(I_sub1))]';
Y = [ Vb1(I_sub1(:),2) Vb1(J_sub1(:),2) repmat(NaN,size(I_sub1))]';
X = X(:);
Y = Y(:);
axis equal
plot(X, Y, 'color', 'k', 'LineStyle', '-.', 'linewidth', 0.5);
title("S1", 'fontSize', 16);
hold off;

subplot(122);
scatter(Vs2(:,1), Vs2(:,2), 300, 'k', 'o', 'filled'); hold on;
scatter(Vsc2(:,1), Vsc2(:,2), 100, 'b', 'o'); hold on;
X = [ Vb2(I_sub2(:),1) Vb2(J_sub2(:),1) repmat(NaN,size(I_sub2))]';
Y = [ Vb2(I_sub2(:),2) Vb2(J_sub2(:),2) repmat(NaN,size(I_sub2))]';
X = X(:);
Y = Y(:);
axis equal
plot(X, Y, 'color', 'k', 'LineStyle', '-.', 'linewidth', 0.5);
title("S2", 'fontSize', 16);
sgtitle("Block sampling with self-loops", 'fontSize', 14)
