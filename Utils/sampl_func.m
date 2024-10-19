% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California

%Function to address corner cases for block-based sampling. 




function [S] = sampl_func(Adj, srate, self_loop_mat, series_approx)
    %This function returns sampling set given Adjacency matrix and
    %self-loop matrix.
    
    %If the block has multiple connected components, sampling Algorithm is
    %applied to each connected component separately.
       
    [p, ~, r, ~] = dmperm(Adj + speye(size(Adj)));
    numConnComp = size( r, 2 )-1;
    npoints = size(Adj, 1);
    number_samples = round(srate*npoints);
    if(npoints <= 2 || number_samples <=1 ) % We don't have to sample blocks with fewer points
        S = true(npoints, 1);
    else
        %Apply sampling algorithm for connected components 
        if(numConnComp == 1)
           S = Recon_aware_global_sampling(Adj, self_loop_mat, number_samples, series_approx);
        else
            S = false(npoints, 1);
            for comp = 1:numConnComp
                idx=p(r(comp):r(comp+1)-1);
                nsub = length(idx);
                num_samples_sub = round(srate*nsub);
                if(nsub <=2 || num_samples_sub <=1)
                    Ssub = true(nsub, 1);
                else
                    [Ssub] = Recon_aware_global_sampling(Adj(idx, idx), self_loop_mat(idx, idx), num_samples_sub, series_approx);
                end
                S(idx) = Ssub;
            end        
        end
    end   
end

%% To plot the toy examples
%S = vertex_domain_sampling(A, zeros(36, 36), 6, 1);
%Vs = V(S,:); Vsc = V(~S,:);
%scatter(Vs(:,1), Vs(:,2), 80, 'r', 'o', 'filled'); hold on;
%scatter(Vsc(:,1), Vsc(:,2), 20, 'b', 'o'); hold on;
%X = [ V(I(:),1) V(J(:),1) repmat(NaN,size(I))]';
%Y = [ V(I(:),2) V(J(:),2) repmat(NaN,size(I))]';
%X = X(:);
%Y = Y(:);
%axis equal
%plot(X, Y, 'color', 'g', 'LineStyle', '-.', 'linewidth', 0.5);
