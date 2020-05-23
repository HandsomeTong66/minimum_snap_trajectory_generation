function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);%n_all_poly：所有多项式的系数
    
    %#####################################################
    % STEP 2.1: write expression of Aeq_start and beq_start
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);%Aeq_start的行数为约束个数(p,v,a,j)，列数为所有多项式的系数
    beq_start = zeros(4, 1);
    
    %求Aeq_start
    %k对应求几阶导数 0:pos 1：vel 2：acc 3:jerk  *0^(i-1-k)
    for k = 0:3
        for i = 1:n_order+1
            if i > k
            Aeq_start(k+1,i) = factorial(i-1)/factorial(i-1-k)*0^(i-1-k);%起点时间都是0，matlab中规定0^0 = 1；
                                                               %factorial():求阶乘函数
                                                               %factorial(0) = 1;                                                            
            end
        end
    end
    %beq_start为start_cond的p,v,a,j
    for i = 1:4
        beq_start(i,1) = start_cond(i);
    end

    %#####################################################
    % STEP 2.2: write expression of Aeq_end and beq_end
    % p,v,a,j constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = zeros(4, 1);
    
    %求Aeq_end
    %k对应求几阶导数 0:pos 1：vel 2：acc 3:jerk  *ts(end)^(i-1-k)
    for k = 0:3
        for i = 1:n_order+1
            if i > k
           %终点段在Aeq的起始位置(列)：未知数个数*(段数-1)+1    
            Aeq_end(k+1 ,(n_order+1)*(n_seg-1)+i) = factorial(i-1)/factorial(i-1-k)*ts(end)^(i-1-k);%起点时间都是0，matlab中规定0^0 = 1；
                                                                                                            %factorial():求阶乘函数
                                                                                                            %factorial(0) = 1;                                                            
            end
        end
    end
    %beq_end为end_cond的p,v,a,j
    for i = 1:4
        beq_end(i,1) = end_cond(i);
    end 
   
    %#####################################################
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    % position constrain in all middle waypoints(位置约束)  *ts(wp)^(i-1)
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
   
    for wp = 1:n_seg-1
        for i = 1:n_order+1
            %中间点在Aeq的起始位置(列)：未知数个数*(中间点段数-1)+1;   ts(wp):表示第wp段的时间
            Aeq_wp(wp, (n_order+1)*wp+i) = factorial(i-1)/factorial(i-1)*0^(i-1);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%最后一改：将ts(wp)^(i-1)改成0^(i-1)
        end
    end
    
    for i = 1:n_seg-1
        beq_wp(i,1) = waypoints(i+1);
    end     
        
    %#####################################################
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    % position continuity constrain between each 2 segments *ts(wp)^(i-1)  *ts(wp)^(i-1)
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);%求中间点的连续性约束beq_con_p就是全是0
       
    %每段的p约束
    for wp = 1:n_seg-1
        for i = 1:n_order+1
            Aeq_con_p(wp, (n_order+1)*(wp-1)+i) = factorial(i-1)/factorial(i-1)*ts(wp)^(i-1);
            Aeq_con_p(wp, (n_order+1)*wp+i) = -(factorial(i-1)/factorial(i-1)*0^(i-1));
        end
    end
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v   *ts(wp)^(i-1-k)   *ts(wp)^(i-1-k)
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
   
     %每段的v约束
     %k = 1;  ：vel
     k = 1;
    for wp = 1:n_seg-1
        for i = 1:n_order+1
            if i > k
            Aeq_con_v(wp, (n_order+1)*(wp-1)+i) = factorial(i-1)/factorial(i-1-k)*ts(wp)^(i-1-k);
            Aeq_con_v(wp, (n_order+1)*wp+i) = -(factorial(i-1)/factorial(i-1-k)*0^(i-1-k));
            end
        end
    end
    
    %#####################################################
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    % acceleration continuity constrain between each 2 segments   *ts(wp)^(i-1-k)  *ts(wp)^(i-1-k)
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
   
    %每段的a约束
     %k = 2;  ：acc
     k = 2;
    for wp = 1:n_seg-1
        for i = 1:n_order+1
            if i > k
            Aeq_con_a(wp, (n_order+1)*(wp-1)+i) = factorial(i-1)/factorial(i-1-k)*ts(wp)^(i-1-k);
            Aeq_con_a(wp, (n_order+1)*wp+i) = -(factorial(i-1)/factorial(i-1-k)*0^(i-1-k));
            end
        end
    end
    
    
    
    
    %#####################################################
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    % jerk continuity constrain between each 2 segments  *ts(wp)^(i-1-k)  *ts(wp)^(i-1-k)
    Aeq_con_j = zeros(n_seg-1, n_all_poly);
    beq_con_j = zeros(n_seg-1, 1);
    
    %每段的j约束
     %k = 3;  ：jerk
     k = 3;
    for wp = 1:n_seg-1
        for i = 1:n_order+1
            if i > k
            Aeq_con_j(wp, (n_order+1)*(wp-1)+i) = factorial(i-1)/factorial(i-1-k)*ts(wp)^(i-1-k);
            Aeq_con_j(wp, (n_order+1)*wp+i) = -(factorial(i-1)/factorial(i-1-k)*0^(i-1-k));
            end
        end
    end
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];%Aeq_con:中间点的p,v,a,j连续性约束
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end