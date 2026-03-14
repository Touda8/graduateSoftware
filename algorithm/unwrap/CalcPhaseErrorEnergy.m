%输入：归一化后的条纹图容器：SourceImg；相位主值图：phaseprincipal  相移步数：N，默认4步     输出：mask
function mean_ErrorEnergy=CalcPhaseErrorEnergy(SourceImg,phaseprincipal,N)
[~,num]=size(SourceImg);
if num <4  
error('输入图像必须大于等于4张');
end
%默认是四步相移
if (nargin<3)
       N=4;
end
%获得调制度图
modulationpic=CalcFringeModulation(SourceImg,N);
%figure;imshow(modulationpic);title('PHH_1');
%modulationpic(modulationpic<1)=0;
%figure;imshow(modulationpic);title('PHH_1');
[height,width]=size(SourceImg{1,1});
%获得平均亮度图
meanpic=zeros(height, width);
for i=1:N
meanpic=SourceImg{1,i}*255+meanpic;
end
meanpic=meanpic/N;
%figure;imshow(meanpic);title('PHH_1');
%计算平均误差能量
mean_ErrorEnergy=zeros(height, width);
for i=1:N
value1=(SourceImg{1,i}*255-meanpic)./modulationpic;
value2=cos(phaseprincipal-2*pi*(i-1)/N);
mean_ErrorEnergy=(value1-value2).^2+mean_ErrorEnergy;
end
mean_ErrorEnergy=sqrt(mean_ErrorEnergy/N);

%将结果输出成一张图片
%figure,imshow(mean_ErrorEnergy);
%figure,imshow(uint8(mean_ErrorEnergy));