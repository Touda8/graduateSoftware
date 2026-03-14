%输入：归一化后的条纹图容器：SourceImg；相移步数：N，默认4步     输出：调制度图modulationpic 
function [datamodulationpic,datamodulation,meanpic]=CalcModulationContrast(SourceImg,N)
[~,num]=size(SourceImg);
if num <4  
error('输入图像必须大于等于4张');
end
%默认是四步相移
if (nargin<2)
       N=4;
end
%%
%计算调制度图
[height,width]=size(SourceImg{1,1});
datamodulationpic=zeros(height, width);
part_sin=zeros(height, width);
part_cos=zeros(height, width);
for i=1:N
part_sin=sin(2 * pi * (i-1) / N)*SourceImg{1,i}*255+part_sin;
part_cos=cos(2 * pi * (i-1) / N)*SourceImg{1,i}*255+part_cos;
end
%调制度赋值
%获得平均亮度图
meanpic=zeros(height, width);
for i=1:N
meanpic=SourceImg{1,i}*255+meanpic;
end
meanpic=meanpic/N;
datamodulation= (2.0 / N)*sqrt(part_sin.*part_sin + part_cos.*part_cos);
datamodulationpic = datamodulation./meanpic;
% datamodulationpic = meanpic./datamodulation;

%figure;imshow(modulationpic);title("调制度图");
