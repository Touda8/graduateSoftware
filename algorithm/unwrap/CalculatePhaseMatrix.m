function phase = CalculatePhaseMatrix(images,N)

% 计算每种频率对应的相位主值
% 输出相位主值，用于相差计算
[height,width]=size(images{1,1});
phase = cell(1,3);
for i = 1:3
    phase{1,i} = zeros(height,width);
end
if N==4
    for i = 1:3 % 对于3组中的每一组图片，每一组相同频率的有四张图片
     I1 = images{i,1};
     I2 = images{i,2};
     I3 = images{i,3};     
     I4 = images{i,4};
     phase{1,i}=atan2((I4-I2),I1-I3);
     
    end
else
    for i = 1:3 % 对于3组中的每一组图片，每一组相同频率的有四张图片
        x=0;y=0;
        for n=1:N
            step=-(2*(n-1)*pi/N);
            y=images{i,n}.*sin(step)+y;
            x=images{i,n}.*cos(step)+x;
        end
        y=-y;
        phase{1,i}=atan2(y,x);
    end
end