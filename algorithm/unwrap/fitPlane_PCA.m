function planes = fitPlane_PCA(data)
% 功能：利用PCA拟合平面
% 输入：data   - 原始数据(m*3)    
% 输出：planes - 拟合所得平面参数 
points = data(:,1:3);
[m,~] = size(points);
% 计算协方差矩阵
M = points-ones(m,1)*(sum(points,1)/m);
C = M.'*M./(m-1);  
% 计算特征值与特征向量
[V, D] = eig(C);
% 最小特征值对应的特征向量为法向量
s1 = D(1,1);
s2 = D(2,2);
s3 = D(3,3);
if (s1 <= s2 && s1 <= s3)
    normal(1,:) = V(:,1)/norm(V(:,1));
elseif (s2 <= s1 && s2 <= s3)
    normal(1,:) = V(:,2)/norm(V(:,2));
elseif (s3 <= s1 && s3 <= s2)
    normal(1,:) = V(:,3)/norm(V(:,3));
end 
% 平面参数标准化
dtmp = mean(points*normal');
planes(1:3) = normal'*sign(dtmp);
planes(4) = -dtmp*sign(dtmp);
end


