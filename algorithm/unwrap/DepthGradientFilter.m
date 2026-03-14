function mask=DepthGradientFilter(aftermask,Img)
%计算相位四领域内的最大的相位差，使用原相位图计算。
%获取图像的长和宽
[h,w] = size(Img);
%定义一个全0的数组用于存储每个像素的最大绝对灰度差
maxDiff = zeros(h,w);
threshold_phase=0.6; %0.4
threshold_datamod=0.1;%0.5
threshold_ErrorEnergy=0.9;%0.5
threshold_minconncted=500;
%循环遍历每个像素进行计算
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
%%  相位点分类，第一类是四领域最大相位差值的绝对值小于阈值的，第二类则是大于阈值的

% 将图像转换为灰度图像
grayImg = maxDiff.*aftermask;
% 找到非零像素的索引
[indX, indY] = find(grayImg ~= 0);
% 获取非零像素的灰度值
nonZeroPixels = grayImg(sub2ind(size(grayImg), indX, indY));
% 按升序排序非零像素的灰度值
sortedPixels = sort(nonZeroPixels);
% 计算百分之八十低灰度值的均值
numPixels = numel(sortedPixels);
numEigthyPercent = round(0.5 * numPixels);
meanLowEightyPercent = max(sortedPixels(1:numEigthyPercent));
% 显示百分之八十低灰度值的均值
disp(['百分之八十低的非零像素的灰度均值：', num2str(meanLowEightyPercent)]);

% figure;
% imhist(maxDiff);
% title("直方图");
threshold_phase = 10*meanLowEightyPercent
mask=maxDiff;
mask(maxDiff >=threshold_phase) = 0;
mask(maxDiff <threshold_phase) = 1;


% figure;imshow(mask);title('梯度mask');
mask=mask.*aftermask;
% figure;imshow(mask);title('梯度mask');
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
  
% if(errormask1(i,j)==0)%调制度对比度或者误差能量不满足的就去除
%     continue;
% end
if(aftermask(i,j)==0)%调制度对比度或者误差能量不满足的就去除
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
% figure,imshow(maskbf);title('修补后mask-最终');
%% 相位修正
%将结果输出成一张图片
%figure,imshow(uint8(maxDiff));