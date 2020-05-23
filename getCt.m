function Ct = getCt(n_seg, n_order)
    Ct = zeros(8*n_seg,(n_seg+1)*4);
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    C_temp = zeros(8,(n_seg+1)*4);
    
%     for l = 1: (n_seg-1)*2
%         for i = 1: (n_seg+1)*4
%      
%         end
%     end
    
    for i=1:4
        Ct(i,i) = 1;
    end
    
    for k = 1:n_seg-1
        C_temp(1,4+k) = 1;
        C_temp(2,4+4+(n_seg-1)+1+3*(k-1)) = 1;
        C_temp(3,4+4+(n_seg-1)+1+3*(k-1)+1) = 1;
        C_temp(4,4+4+(n_seg-1)+1+3*(k-1)+2) = 1;
        
        C_temp(5,4+k) = 1;
        C_temp(6,4+4+(n_seg-1)+1+3*(k-1)) = 1;
        C_temp(7,4+4+(n_seg-1)+1+3*(k-1)+1) = 1;
        C_temp(8,4+4+(n_seg-1)+1+3*(k-1)+2) = 1;  
        
        Ct(4+1+8*(k-1):4+8+8*(k-1),:) = C_temp;
        
        C_temp(:,:)=0;
    end
    
    for i=1:4
        Ct((n_seg-1)*8+4+i,4+(n_seg-1)+i) = 1;
    end
    
end