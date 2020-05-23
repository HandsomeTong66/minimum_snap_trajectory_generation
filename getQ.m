function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = [];
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment(¼ÆËãµÚk¶ÎµÄQ_k¾ØÕó) 
        %
        %
        %
        %
        %¼ÆËãÒ»¶ÎµÄQ¾ØÕó  *ts(k)^(i+l-7)
        for i = 1:n_order+1
            for l = 1:n_order+1 
                if i > 4 && l > 4
                %Q_k(i,l) = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)/(i+l-7);
                Q_k(i,l) = (i-1)*(i-2)*(i-3)*(i-4)*(l-1)*(l-2)*(l-3)*(l-4)/(i+l-7-2)*ts(k)^(i+l-7-2);
                end
            end
        end
        Q = blkdiag(Q, Q_k);
    end
end