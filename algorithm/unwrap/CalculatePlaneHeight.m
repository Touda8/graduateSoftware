function [HeightValue ,rms1,rms2 ]=CalculatePlaneHeight(ROI_1XYZ,ROI_2XYZ)

    planes_1 = fitPlane_PCA(ROI_1XYZ);
    planes_2 = fitPlane_PCA(ROI_2XYZ);
    [m,n]=size(ROI_1XYZ);
    sum=0.;

    for i=1:m
      N=  planes_2(:,1:3);
      D=planes_2(:,4);
      P=ROI_1XYZ(i,:);
      dist = abs(dot(N,P)+D)/norm(N);
      sum=sum+dist;
    end

    sum2=0.;
    for j=1:m
      N=  planes_1(:,1:3);
      D=planes_1(:,4);
      P=ROI_2XYZ(j,:);
      dist = abs(dot(N,P)+D)/norm(N);
      sum2=sum2+dist;
    end




    rms1=0;
    for i=1:m
      N=planes_1(:,1:3);
      P=ROI_1XYZ(i,:);
      D=planes_1(:,4);
      dis1 = abs(dot(N,P)+D)/norm(N);
      rms1=rms1+dis1*dis1;
    end
    rms1=sqrt(rms1./m);




    rms2=0;
    for i=1:m
      N=planes_2(:,1:3);
      P=ROI_2XYZ(i,:);
      D=planes_2(:,4);
      dis2 = abs(dot(N,P)+D)/norm(N);
      rms2=rms2+dis2*dis2;
    end
    rms2=sqrt(rms2./m);



HeightValue=(sum./m+sum2./m)/2;




end