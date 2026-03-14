# graduateSoftware 构建和运行指南

## 快速开始

### 方式 1: 一键构建和运行（推荐）
```bash
build_and_run.bat
```
或指定配置：
```bash
build_and_run.bat Release      # 发布版本
build_and_run.bat Debug        # 调试版本
```

### 方式 2: 分步构建和运行
```bash
# 构建
build.bat Release              # 或 build.bat Debug

# 运行
run.bat                        # 运行 Release 版本
run.bat Debug                  # 运行 Debug 版本
```

## 诊断与故障排除

### 检查环境
```bash
check_build.bat
```
此脚本会检查：
- 源文件完整性
- 生成的文件（ui_twoProjector.h, moc_*.cpp）
- 库文件依赖
- Visual Studio 2019 安装

### 常见问题

#### 1. "ui_twoProjector.h not found" 错误

**原因**: Qt uic 工具不可用，UI 头文件未生成

**解决方案**:
```bash
# 如果已安装 Python 3
python3 generate_ui.py

# 或 Python 2/3 兼容
python generate_ui.py
```

#### 2. 编译失败 - MSBuild 错误

**查看详细日志**:
```bash
# 查看最新的编译日志
logs\
```

日志文件格式: `build_YYYYMMDD_HHMM.log`

#### 3. "Visual Studio 2019 not found" 错误

**解决方案**: 安装 Visual Studio 2019（Community/Professional/Enterprise）

下载地址: https://visualstudio.microsoft.com/vs/older-downloads/

#### 4. 运行时出现 Qt 库错误

**解决方案**: 确保执行 `run.bat`，它会自动配置 Qt 库路径

## 项目结构

```
graduateSoftware/
├── src/                      # 源代码
│   ├── MainWindow.*         # 主窗口
│   ├── common/              # 公共库
│   ├── reconstruction/      # 重建算法
│   ├── measurement/         # 测量算法
│   ├── pointcloud/          # 点云处理
│   ├── visualization/       # 可视化
│   ├── moc_MainWindow.cpp   # Qt MOC 生成文件
│   └── ...
├── build/                    # 编译输出（自动生成）
│   ├── Release/
│   │   ├── twoProjector.exe
│   │   ├── *.dll
│   │   └── obj/
│   └── Debug/
├── qt_deploy/               # Qt 5 库
├── twoProjector.ui          # Qt UI 文件
├── ui_twoProjector.h        # 生成的 UI 头文件
├── build.bat                # 编译脚本
├── run.bat                  # 运行脚本
├── build_and_run.bat        # 一键构建和运行
├── check_build.bat          # 诊断脚本
└── generate_ui.py           # UI 生成脚本
```

## 配置

### 修改库路径

编辑 `twoProjector.vcxproj.props` 文件的 `<EnvLibPaths>` 部分：

```xml
<PropertyGroup Label="EnvLibPaths">
  <EnvLib>F:\project\Envlib</EnvLib>  <!-- 修改为你的库目录 -->
  <!-- ... 其他路径 -->
</PropertyGroup>
```

### 编译配置

- **VS 版本**: Visual Studio 2019
- **工具集**: v142
- **平台**: x64
- **标准**: C++17

## Qt 工具说明

本项目使用 Qt 5.15.2，但 uic 和 moc 工具不可用时：

1. **ui_twoProjector.h** 由 `generate_ui.py` 脚本生成
2. **moc_*.cpp** 文件已预生成，如需更新请安装 Qt 并运行：

```bash
# 安装 Qt 5.15.2 后
"C:\Qt\5.15.2\msvc2019_64\bin\uic.exe" twoProjector.ui -o ui_twoProjector.h
"C:\Qt\5.15.2\msvc2019_64\bin\moc.exe" src\MainWindow.h -o src\moc_MainWindow.cpp
"C:\Qt\5.15.2\msvc2019_64\bin\moc.exe" src\visualization\VtkWidget.h -o src\visualization\moc_VtkWidget.cpp
```

## 依赖库

| 库 | 版本 | 位置 |
|---|---|---|
| OpenCV | 4.5.5 | F:\project\Envlib\opencv455 |
| PCL | 1.12.1 | F:\project\Envlib\PCL1.12.1 |
| VTK | 9.1 | F:\project\Envlib\PCL1.12.1\3rdParty\VTK |
| Eigen | 3.4.0 | F:\project\Envlib\eigen-3.4.0 |
| Boost | 1.78 | F:\project\Envlib\PCL1.12.1\3rdParty\Boost |
| Ceres | - | F:\project\Envlib\ceres |
| Qt | 5.15.2 | ./qt_deploy |

## 日志和输出

- **编译日志**: `logs/build_*.log`
- **可执行文件**: `build/Release/twoProjector.exe` 或 `build/Debug/twoProjector.exe`
- **编译中间文件**: `build/Release/obj/` 或 `build/Debug/obj/`

## 开发工作流

```bash
# 1. 检查环境
check_build.bat

# 2. 修改代码
# ... 编辑源文件 ...

# 3. 构建和运行
build_and_run.bat Release

# 4. 调试（如需要）
build_and_run.bat Debug
```

## 获取帮助

如遇到问题：
1. 运行 `check_build.bat` 诊断环境
2. 查看 `logs/` 目录的编译日志
3. 确保所有依赖库都已安装
4. 运行 `generate_ui.py` 重新生成 UI 文件
