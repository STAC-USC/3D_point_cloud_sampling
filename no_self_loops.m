function loop_wts_mat = no_self_loops(A, block_logical)
    n_block = nnz(block_logical);
    loop_wts_mat = sparse(n_block, n_block);
end
