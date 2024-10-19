% Graph-based scalable sampling of 3D point cloud attributes
% IEEE. Transactions on Image Processing
% Author: Shashank N. Sridhara, University of Southern California



function final_arr = index_bounds_to_logical_arr(N, index_bounds)
    % Given the size of the final array and the indices - index_bounds[1]
    % and index_bounds[2] in which the final array should be true(and false
    % otherwise), this function returns the final desired array.
    
    final_arr = false(N, 1);
    final_arr(index_bounds(1):index_bounds(2)) = true;
end
