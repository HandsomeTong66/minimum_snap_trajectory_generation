function M = getM(n_seg, n_order, ts)
    M = [];
    n = 0;
    for k = 1:n_seg
        M_k = [];
        %#####################################################
        % STEP 1.1: calculate M_k of the k-th segment 
        %n_all_poly = n_seg*(n_order+1);%n_all_poly：所有多项式的系数
        start_point_M = zeros(4 ,n_order+1);
        end_point_M   = zeros(4, n_order+1);
        
        %j:求导的阶数 0:pos 1:vel 2:acc 3:jerk (n_order+1)*n+i
        for j = 0:3
            for i = 1:n_order+1
                if i > j
                    start_point_M(j+1 ,i) = factorial(i-1)/factorial(i-1-j)*0^(i-1-j);
                    end_point_M(j+1 ,i) = factorial(i-1)/factorial(i-1-j)*ts(k)^(i-1-j);
                end
            end
        end
        
        n = n+1;       
        M_k = [start_point_M;end_point_M];  
        M = blkdiag(M, M_k);
    end
end