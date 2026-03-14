function [A,B,mask] = Generation_of_mask(images,N)

% 计算每种频率对应的相位主值
% 输出相位主值，用于相差计算
[height,width]=size(images{1,1});
A = cell(1,3);
B = cell(1,3);
mask=cell(1,3);
for i = 1:3
    A{1,i} = zeros(height,width);
    B{1,i} = zeros(height,width);
    mask{1,i}=zeros(height,width);
end

 for i = 1:3
     x=0;
     for n=1:N
       x=x+images{i,n}./N;
     end
     A{1,i}=x;
 end



    for i = 1:3 % 对于3组中的每一组图片，每一组相同频率的有四张图片
        x=0;y=0;
        for n=1:N
            step=-(2*(n-1)*pi/N);
            y=images{i,n}.*sin(step)+y;
            x=images{i,n}.*cos(step)+x;
        end
        B{1,i}=2.*sqrt(x.^2+y.^2)./N;
    end

 for i = 1:3 
    delta = B{1,i}./ A{1,i};

    mask{1,i} = (delta>0.1);



 end

end