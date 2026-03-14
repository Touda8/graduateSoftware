function  [a,b,c,d] = PCALeastSquaresFittingPlane(data)
% 
%     %--------------------------获取xyz坐标---------------------------
%     x=data(:,1);
%     y=data(:,2);
%     z=data(:,3);
%     planeData=[x,y,z];
% %     %--------------------------可视化点云----------------------------
% %     figure
% %     pcshow(pc);
%     %-------------------------SVD拟合平面----------------------------
%     % 1、计算点云质心
%     centroid=mean(planeData,1);
%     % 2、去质心化
%     deMean=bsxfun(@minus,planeData,centroid);
%     % 3、构建协方差矩阵
%     Cov=deMean'*deMean;
%     % 4、协方差矩阵进行奇异值分解，最小奇异值对应的奇异向量就是平面的法向量
%     [U,S,V]=svd(Cov);
%     % 5、获取最小特征值对应的特征向量 
%     a=V(1,3);
%     b=V(2,3);
%     c=V(3,3);
%     % 6、计算原点到拟合平面的距离
%     d = -dot([a b c],centroid');


%     %-----------------------可视化拟合结果--------------------------
    %  figure
    % % 图形绘制
    % scatter3(x,y,z,'r','filled')
    % hold on;
    % xfit = min(x):0.1:max(x);
    % yfit = min(y):0.1:max(y);
    % [XFit,YFit]= meshgrid (xfit,yfit);
    % ZFit = -(d + a * XFit + b * YFit)/c;
    % mesh(XFit,YFit,ZFit);
    % title('最小二乘拟合平面(PCA法)');
% 
% end

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
a=planes(1);
b=planes(2);
c=planes(3);
d=planes(4);
% %-----------------------可视化拟合结果--------------------------
% figure
% % 图形绘制
% x=data(:,1);
% y=data(:,2);
% z=data(:,3);
% scatter3(x,y,z,'r','filled')
% hold on;
% xfit = min(x):0.1:max(x);
% yfit = min(y):0.1:max(y);
% [XFit,YFit]= meshgrid (xfit,yfit);
% ZFit = -(d + a * XFit + b * YFit)/c;
% mesh(XFit,YFit,ZFit);
% title('最小二乘拟合平面(PCA法)');

end


