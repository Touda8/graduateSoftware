% convert_calib.m
% 将标定 .mat 转为 raw 二进制（C++ 可读）
% raw 格式：[int32 rows][int32 cols][float64 x rows x cols 行主序]
% 调用：matlab -nosplash -nodesktop -batch "convert_calib('data/calibrationResult')"
function convert_calib(baseDir)
    if nargin < 1; baseDir = 'data/calibrationResult'; end
    outDir = fullfile(baseDir, 'raw');
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    % --- X.mat [2048 2448] double ---
    disp('Converting X.mat ...');
    s = load(fullfile(baseDir, 'X.mat'));
    writeRaw(fullfile(outDir, 'X.raw'), s.X);

    % --- Y.mat [2048 2448] double ---
    disp('Converting Y.mat ...');
    s = load(fullfile(baseDir, 'Y.mat'));
    writeRaw(fullfile(outDir, 'Y.raw'), s.Y);

    % --- 1/allK.mat [5013504 3] double (已是 2D) ---
    disp('Converting 1/allK.mat ...');
    s = load(fullfile(baseDir, '1', 'allK.mat'));
    writeRaw(fullfile(outDir, 'allK1.raw'), s.allK);

    % --- 1/allK_i.mat [2048 2448 3] double -> reshape [5013504 3] ---
    disp('Converting 1/allK_i.mat ...');
    s = load(fullfile(baseDir, '1', 'allK_i.mat'));
    M = reshape(s.allK_i, [], 3);
    writeRaw(fullfile(outDir, 'allK_i1.raw'), M);

    % --- 2/allK.mat [5013504 3] double (已是 2D) ---
    disp('Converting 2/allK.mat ...');
    s = load(fullfile(baseDir, '2', 'allK.mat'));
    writeRaw(fullfile(outDir, 'allK2.raw'), s.allK);

    % --- 2/allK_i.mat [2048 2448 3] double -> reshape [5013504 3] ---
    disp('Converting 2/allK_i.mat ...');
    s = load(fullfile(baseDir, '2', 'allK_i.mat'));
    M = reshape(s.allK_i, [], 3);
    writeRaw(fullfile(outDir, 'allK_i2.raw'), M);
    disp('convert_calib: all done');
end

function writeRaw(path, M)
    % M 必须是 2D double 矩阵
    if ~isa(M, 'double'); M = double(M); end
    [rows, cols] = size(M);
    fid = fopen(path, 'wb');
    if fid == -1; error('writeRaw: cannot open %s', path); end
    fwrite(fid, int32(rows), 'int32');
    fwrite(fid, int32(cols), 'int32');
    % MATLAB 列主序 -> C++ 行主序：转置后按列写
    Mt = M.';
    fwrite(fid, Mt(:), 'float64');
    fclose(fid);
    fprintf('  wrote %s (%d x %d)\n', path, rows, cols);
end