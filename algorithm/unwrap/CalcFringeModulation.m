%输入：归一化后的条纹图容器：SourceImg；相移步数：N，默认4步     输出：调制度图modulationpic 
function modulationpic=CalcFringeModulation(SourceImg,N)
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
modulationpic=zeros(height, width);
part_sin=zeros(height, width);
part_cos=zeros(height, width);
for i=1:N
part_sin=sin(2 * pi * i / N)*SourceImg{1,i}*255+part_sin;
part_cos=cos(2 * pi * i / N)*SourceImg{1,i}*255+part_cos;
end
%调制度赋值
modulationpic= (2.0 / N)*sqrt(part_sin.*part_sin + part_cos.*part_cos);
%figure;imshow(modulationpic);title("调制度图");






