clear;clc;
close all;
dataCalibration='20260131';
data='20260131';
ResName = 'BGARec静态20000';
addpath("过去辅助函数\");
mkdir(strcat('..\..\experiment\',data,'\thing\',ResName,'\info\'))

%%
%获取相位信息
N=8;    % 相移步数
numImages=10; 
first=0;
hDeformedImages = cell(3,N);
vDeformedImages = cell(3,N);
%生成高斯滤波
GaussianFilter = fspecial('gaussian', [9,9], 3);
X=load(strcat("..\..\标定\Calibration_result\",dataCalibration,"\X.mat")).X;
Y=load(strcat("..\..\标定\Calibration_result\",dataCalibration,"\Y.mat")).Y;


% X=load(strcat("X.mat")).X;
% Y=load(strcat("Y.mat")).Y;
Roi_Hb={2,5};
P = 720/64;
Q = 1280/121;
%% 
for index = 1:numImages
    for ProNum =1:2
        if ProNum==1
            allZp_i=load("CalibrationResult\"+dataCalibration+"\1\allK_i.mat");
            disp("视图一开始测量");
        else
            allZp_i=load("CalibrationResult\"+dataCalibration+"\2\allK_i.mat");

            disp("视图二开始测量");
        end
        allZp_i=allZp_i.allK_i;
        %读入条纹图片
        for i = 1:3 %三频四项
            for  j = 1:N
                hDeformedImages{i,j} = imread(strcat('..\..\experiment\',data,'\thing\',ResName,'\',num2str(ProNum),'\',num2str(index),'\', num2str((i-1)*N+j),'.bmp'));
                hDeformedImages{i,j} = im2double((hDeformedImages{i,j}));%mat2gray:归一化
                hDeformedImages{i,j} = imfilter(hDeformedImages{i,j}, GaussianFilter, 'replicate');
                vDeformedImages{i,j} = imread(strcat('..\..\experiment\',data,'\thing\',ResName,'\',num2str(ProNum),'\',num2str(index),'\', num2str((i-1)*N+j++N*3),'.bmp'));
                vDeformedImages{i,j} = im2double((vDeformedImages{i,j}));%mat2gray:归一化
                vDeformedImages{i,j} = imfilter(vDeformedImages{i,j}, GaussianFilter, 'replicate');
            end
        end
        [height, width] = size(hDeformedImages{1,1});
        % 初始化三组处理后的图片灰度矩阵,存储相位主值图像
        % 横条纹
        hPhiDeformed = cell(3,1);
        for i = 1:3
            hPhiDeformed{i,1} = nan(height,width);
        end
        % 纵条纹
        vPhiDeformed = cell(3,1);
        for i = 1:3
            vPhiDeformed{i,1} = nan(height,width);
        end

        % 计算横纵相位主值
        hPhiDeformed = CalculatePhaseMatrix(hDeformedImages,N);
        vPhiDeformed = CalculatePhaseMatrix(vDeformedImages,N);

        [hA,hB,mask_delta] = Generation_of_mask(hDeformedImages,N);
        COLOR{ProNum,index} = hA{1,1};
        % Hb{ProNum,index}=hB{1,1};
        % 实例化各频相位主值
        PHDeformed_1 = hPhiDeformed{1,1};   %频率1
        PHDeformed_2 = hPhiDeformed{1,2};   %频率2
        PHDeformed_3 = hPhiDeformed{1,3};   %频率3
        PVDeformed_1 = vPhiDeformed{1,1};   %频率1
        PVDeformed_2 = vPhiDeformed{1,2};   %频率2
        PVDeformed_3 = vPhiDeformed{1,3};   %频率3
        PHDeformed_12=Multifrequency_phase(PHDeformed_1,PHDeformed_2);
        PHDeformed_23= Multifrequency_phase(PHDeformed_3,PHDeformed_2);
        PHDeformed_123 =Multifrequency_phase(PHDeformed_12,PHDeformed_23);
        PVDeformed_12=Multifrequency_phase(PVDeformed_1,PVDeformed_2);
        PVDeformed_23= Multifrequency_phase(PVDeformed_3,PVDeformed_2);
        PVDeformed_123 =Multifrequency_phase(PVDeformed_12,PVDeformed_23);

        %%%%%%%%%%%%%%%% 时间相位解包裹 %%%%%%%%%%%%%%%%
        % 解包裹#1：phi123->phi12
        PHDeformed_12Unwrap = UNwrapPhase(PHDeformed_123, 8, PHDeformed_12);
        PVDeformed_12Unwrap = UNwrapPhase(PVDeformed_123, 11, PVDeformed_12);

        % 解包裹#2：phi12->phi1
        PHDeformed_1Unwrap = UNwrapPhase(PHDeformed_12Unwrap, 8, PHDeformed_1);
        PVDeformed_1Unwrap = UNwrapPhase(PVDeformed_12Unwrap, 11, PVDeformed_1);

        [mask1,SNR,datamodulation]=PhaseQualityMask(PHDeformed_1Unwrap,hDeformedImages,PHDeformed_1,PHDeformed_2,PHDeformed_3,N);
        imwrite(mat2gray(datamodulation),strcat('..\..\experiment\',data,'\thing\',ResName,'\info\datamodulationpic_',num2str(index),'_',num2str(ProNum),'.bmp'))
        imwrite(mat2gray(SNR),strcat('..\..\experiment\',data,'\thing\',ResName,'\info\SNR_',num2str(index),'_',num2str(ProNum),'.bmp'))
        Hb{ProNum,index}=SNR;
        % [mask222,SNR222,datamodulation222]=Copy_of_Monntonicity_Connected_Domain_Seg1(PVDeformed_1Unwrap,vDeformedImages,PVDeformed_1,PVDeformed_2,PVDeformed_3,N);
        % imwrite(mat2gray(datamodulation222),strcat('..\..\experiment\',data,'\thing\',ResName,'\info\VVVdatamodulationpic_',num2str(index),'_',num2str(ProNum),'.bmp'))
        % imwrite(mat2gray(SNR222),strcat('..\..\experiment\',data,'\thing\',ResName,'\info\VVVSNR_',num2str(index),'_',num2str(ProNum),'.bmp'))
        % 
        % 
        % Vp = PHDeformed_1Unwrap * P / 2 / pi;
        % Up = PVDeformed_1Unwrap * Q / 2 / pi;

        
        ZZZ{ProNum,index} =(PHDeformed_1Unwrap.*allZp_i(:,:,1)+allZp_i(:,:,3))./(allZp_i(:,:,2).*PHDeformed_1Unwrap+1);
        MASK{ProNum,index}=DepthGradientFilter(mask1,ZZZ{ProNum,index});
        MASK{ProNum,index}=ones(size(mask1));

        close all

    end
% end
% %% 融合重建
% for index = 1:numImages
    ALL_mask = MASK{1,index}.*MASK{2,index};
    ALL_fan = ~ALL_mask;
    mix_Z = ALL_fan.*MASK{1,index}.*ZZZ{1,index}+ALL_fan.*MASK{2,index}.*ZZZ{2,index};  % 保留第一个或者保留第二个
    temp =(Hb{1,index}.*ZZZ{1,index}+Hb{2,index}.*ZZZ{2,index})./(Hb{1,index}+Hb{2,index}).*ALL_mask;  % 两个都不好的做一下调制度加权相加
    mix_Z=mix_Z+temp;
    Z = mix_Z;
    save(strcat('..\..\experiment\',data,'\thing\',ResName,'\info\Z_0',num2str(index),'.mat'),'Z')
    XX=X(:);
    YY=Y(:);
    ZZ=mix_Z(:);
    
    XX(mix_Z==0)=[];
    YY(mix_Z==0)=[];
    ZZ(mix_Z==0)=[];
    GrayImageOrigin = imread(strcat('..\..\experiment\',data,'\thing\',ResName,'\',num2str(ProNum),'\',num2str(index),'\', num2str(49),'.bmp'));
    GrayImageOriginCols = double(GrayImageOrigin(:));
    GrayImageOriginCols(mix_Z==0)=[];
    XYZ=[XX,YY,ZZ,GrayImageOriginCols];
    % save(strcat('..\..\experiment\',data,'\thing\',ResName,'\info\',num2str(index),'.txt'), '','XYZ', '-ascii')

end



