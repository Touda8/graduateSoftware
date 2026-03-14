function [mask,datamodulationpic,datamodulation,meanpic]=PhaseQualityMask(Img,hDeformedImages,PHDeformed_1,PHDeformed_2,PHDeformed_3,N)
%计算相位四领域内的最大的相位差，使用原相位图计算。
%获取图像的长和宽
[h,w] = size(Img);
%% 定义一个全0的数组用于存储每个像素的最大绝对灰度差
maxDiff = zeros(h,w);
threshold_phase=2*pi;
threshold_datamod=0.2;
threshold_ErrorEnergy=0.3;
threshold_minconncted=100;
%% 计算需要的能量误差图
mean_ErrorEnergy1=CalcPhaseErrorEnergy(hDeformedImages(1,:),PHDeformed_1,N);
% figure,imshow(mean_ErrorEnergy1);title('误差能量图1');
mean_ErrorEnergy2=CalcPhaseErrorEnergy(hDeformedImages(2,:),PHDeformed_2,N);
% figure,imshow(mean_ErrorEnergy2);title('误差能量图2');
mean_ErrorEnergy3=CalcPhaseErrorEnergy(hDeformedImages(3,:),PHDeformed_3,N);
% figure,imshow(mean_ErrorEnergy3);title('误差能量图3');
%% 调制度对比度图
[datamodulationpic,datamodulation,meanpic]=CalcModulationContrast(hDeformedImages(1,:),N);
% figure;imshow(datamodulationpic);title('调制度对比度');
%% 计算相位四领域梯度图，循环遍历每个像素进行计算
for i = 2:h-1
    for j = 2:w-1
        %四领域中灰度值最大的像素值
        maxVal = max([Img(i-1,j) Img(i+1,j) Img(i,j-1) Img(i,j+1)]);
        %四领域中灰度值最小的像素值
        minVal = min([Img(i-1,j) Img(i+1,j) Img(i,j-1) Img(i,j+1)]);
        %当前像素的灰度值
        curVal = Img(i,j);
        %计算当前像素四领域中最大的绝对灰度值差
        maxDiff(i,j) = max([abs(curVal-maxVal),abs(curVal-minVal)]);
    end
end

%处理边界像素
%左边界
j= 1;
for i = 2:h-1
    maxDiff(i,j) = max(abs(Img(i,j)-[Img(i+1,j) Img(i-1,j) Img(i,j+1)]));
end
%右边界
j= w;
for i = 2:h-1
    maxDiff(i,j) = max(abs(Img(i,j)-[Img(i+1,j) Img(i-1,j) Img(i,j-1)]));
end
%上边界
i = 1;
for j = 2:w-1
    maxDiff(i,j) = max(abs(Img(i,j)-[Img(i+1,j) Img(i,j-1) Img(i,j+1)]));
end
%下边界
i = h;
for j = 2:w-1
    maxDiff(i,j) = max(abs(Img(i,j)-[Img(i-1,j) Img(i,j-1) Img(i,j+1)]));
end

%处理四个角点像素
maxDiff(1,1) = max(abs(Img(1,1)-[ Img(2,1) Img(1,2)]));
maxDiff(1,w) = max(abs(Img(1,w)-[ Img(2,w) Img(1,w-1)]));
maxDiff(h,1) = max(abs(Img(h,1)-[ Img(h-1,1) Img(h,2)]));
maxDiff(h,w) = max(abs(Img(h,w)-[ Img(h-1,w) Img(h,w-1)]));
%% 计算调制度对比度阈值
% grayImage = datamodulationpic*255;
% % 将灰度图像向量化
% vectorizedImage = double(grayImage(:));
% % 执行聚类算法（例如 K-Means）
% numClusters =5; % 聚类簇数
% [idx, centroids] = kmeans(vectorizedImage, numClusters);
% % 将每个像素的聚类结果映射回原始图像
% colored = ind2rgb(reshape(idx, size(grayImage)), jet(numClusters));
% % 显示聚类结果
% % figure;
% % imshow(colored);
% % 可选：显示聚类中心的灰度值
% % figure;
% % plot(1:numClusters, centroids, 'ko-');
% % xlabel('Cluster');
% % ylabel('Gray Level');
% %获得阴影阈值，第二个最小的便是阈值；
% sortedVector = sort(centroids);
% secondSmallest = sortedVector(1);
% secondSmallestIndex = find(centroids == secondSmallest);
% % 找到等于3的元素的位置
% indices = find(idx == secondSmallestIndex);
% maxValue = max(grayImage(indices));
% threshold_dark=maxValue/255;%阴影的调制度对比度
% disp(sprintf("调制度对比度阈值：%f",threshold_dark));
% %更改调制度阈值
% if threshold_dark<0.5%防呆操作
%     threshold_datamod=threshold_dark;
% end
%%
% 将图像转换为灰度图像
grayImg = mean_ErrorEnergy1+mean_ErrorEnergy2+mean_ErrorEnergy3;
% 找到非零像素的索引
[indX, indY] = find(grayImg ~= 0);
% 获取非零像素的灰度值
nonZeroPixels = grayImg(sub2ind(size(grayImg), indX, indY));
% 按升序排序非零像素的灰度值
sortedPixels = sort(nonZeroPixels);
% 计算百分之八十低灰度值的均值
numPixels = numel(sortedPixels);
numEigthyPercent = round(0.9 * numPixels);
meanLowEightyPercent = max(sortedPixels(1:numEigthyPercent));
% 显示百分之八十低灰度值的均值
disp(['百分之八十低的非零像素的灰度均值：', num2str(meanLowEightyPercent)]);
% % figure;
% imshow(maxDiff);
% title("误差能量直方图");
%%  相位点分类，第一类是四领域最大相位差值的绝对值小于阈值的，第二类则是大于阈值的
mask=maxDiff;
mask(maxDiff >=threshold_phase) = 0;
mask(maxDiff <threshold_phase) = 1;
%根据3个误差能量图和数据调制图获得mask
errormask1=zeros(size(mean_ErrorEnergy1));
errormask1(mean_ErrorEnergy1>threshold_ErrorEnergy)=1;
errormask1=~errormask1;
% figure;imshow(errormask1);title('误差能量mask1');

errormask2=zeros(size(mean_ErrorEnergy2));
errormask2(mean_ErrorEnergy2>threshold_ErrorEnergy)=1;
errormask2=~errormask2;
% figure;imshow(errormask2);title('误差能量mask2');

errormask3=zeros(size(mean_ErrorEnergy3));
errormask3(mean_ErrorEnergy3>threshold_ErrorEnergy)=1;
errormask3=~errormask3;
% figure;imshow(errormask3);title('误差能量mask3');

datamask=zeros(size(datamodulationpic));
datamask(datamodulationpic>threshold_datamod)=1;
% figure;imshow(datamask);title('数据调制mask');
errormask=errormask1.*errormask2.*errormask3;
%% 画出errormask和datamask和mask
% 设置掩膜
mask1 =errormask;
mask2 = datamask;
mask3 = mask;
% 创建彩色图像
colored_image = zeros(size(mask, 1), size(mask, 2), 3, 'uint8');
% 设置颜色映射
color_map = [255 255 255; 255 255 0; 255 0 0; 0 255 0; 0 0 255];
% 根据掩膜生成彩色图像
for i = 1:size(mask, 1)
    for j = 1:size(mask, 2)
        if mask1(i, j) == 0 && mask2(i, j) == 0 && mask3(i, j) == 0
            % 三个掩膜都为0的位置显示白色
            colored_image(i, j, :) = color_map(1, :);
        elseif ((mask1(i, j) == 0 && mask2(i, j) == 0&&mask3(i, j) == 1)|(mask1(i, j) == 1 && mask2(i, j) == 0&&mask3(i, j) == 0)|(mask1(i, j) == 0 && mask2(i, j) == 1&&mask3(i, j) == 0))
            % 有其中两个掩膜都为0的位置显示黄色
            colored_image(i, j, :) = color_map(2, :);
        elseif mask1(i, j) == 0 && mask2(i, j) == 1 && mask3(i, j) == 1
            % 只有一个掩膜为0的位置按照其属于哪个掩膜显示红色
            colored_image(i, j, :) = color_map(3, :);
        elseif mask1(i, j) == 1 && mask2(i, j) == 0 && mask3(i, j) == 1
            % 只有一个掩膜为0的位置按照其属于哪个掩膜显示绿色
            colored_image(i, j, :) = color_map(4, :);
        elseif mask1(i, j) == 1 && mask2(i, j) == 1 && mask3(i, j) == 0
            % 只有一个掩膜为0的位置按照其属于哪个掩膜显示蓝色
            colored_image(i, j, :) = color_map(5, :);
        end
    end
end
% 显示彩色图像
% imshow(colored_image);
title('彩色图像');
%% 获得总mask
finalmask=datamask.*errormask;
% figure;imshow(errormask);title('误差能量总mask');
% figure;imshow(mask);title('梯度mask');
mask=mask.*finalmask;
mask=bwareaopen(mask,threshold_minconncted);
% figure;imshow(mask);title('倒数第二mask');
%% 边缘点修正
maskbf=mask;
flag_over=0;%修补完成的标志
while flag_over==0
    flag_over=1;
    %循环遍历每个非0像素进行计算
    [rowmsak,colmask]=find(maskbf==0);
    for k = 1:size(rowmsak,1)
        i=rowmsak(k);
        j=colmask(k);
        if(rowmsak(k)==1|rowmsak(k)==h|colmask(k)==1|colmask(k)==w)
            continue;
        end

        if(errormask(i,j)==0)%调制度对比度或者误差能量不满足的就去除
            continue;
        end
        %
        if(datamodulationpic(i,j)<0.9*threshold_datamod)%调制度对比度或者误差能量不满足的就去除
            continue;
        end

        %四领域判断是否为正常点
        if( maskbf(i-1,j)==1)
            if(abs(Img(i-1,j)-Img(i,j))<threshold_phase)
                maskbf(i,j)=1;
                flag_over=0;
            end
        end

        if( maskbf(i+1,j)==1)
            if(abs(Img(i+1,j)-Img(i,j))<threshold_phase)
                maskbf(i,j)=1;
                flag_over=0;
            end
        end

        if( maskbf(i,j-1)==1)
            if(abs(Img(i,j-1)-Img(i,j))<threshold_phase)
                maskbf(i,j)=1;
                flag_over=0;
            end
        end

        if( maskbf(i,j+1)==1)
            if(abs(Img(i,j+1)-Img(i,j))<threshold_phase)
                maskbf(i,j)=1;
                flag_over=0;
            end
        end
    end
end
mask=maskbf;
% figure,imshow(maskbf);title('修补后mask-多特征');
%% 相位修正
%将结果输出成一张图片
%figure,imshow(uint8(maxDiff));
