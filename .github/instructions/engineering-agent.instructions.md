---
applyTo: "*.sln,*.vcxproj,*.vcxproj.props,*.vcxproj.filters,build.bat,*.bat"
---

# 工程化智能体指令

> 适用范围：VS2019 工程文件、属性表、构建脚本

## 库路径命名规范

- 所有库根路径通过 `$(EnvLib)` 属性宏引用，值为 `F:\project\Envlib`
- **禁止在 `.vcxproj` 中写死任何绝对路径**
- 路径配置集中在 `twoProjector.vcxproj.props`，其它文件通过 `<Import>` 引用

```xml
<!-- twoProjector.vcxproj.props 关键属性宏 -->
<PropertyGroup Label="EnvLib">
  <EnvLib>F:\project\Envlib</EnvLib>
  <OpenCVDir>$(EnvLib)\opencv455\opencv</OpenCVDir>
  <PCLDir>$(EnvLib)\PCL1.12.1</PCLDir>
  <EigenDir>$(EnvLib)\eigen-3.4.0</EigenDir>
  <CeresDir>$(EnvLib)\ceres</CeresDir>
</PropertyGroup>
```

## 必须包含的预处理器宏

```
NOMINMAX
_USE_MATH_DEFINES
_CRT_SECURE_NO_WARNINGS
UNICODE
_UNICODE
```

## 编译开关（Release|x64 基准）

| 开关 | 值 | 说明 |
|------|----|------|
| `/std:c++17` | 必须 | C++17 支持 |
| `/MP` | 必须 | 多处理器并行编译 |
| `/W3` | 必须 | 三级警告 |
| `/O2` | Release | 最优速度 |
| `/Od /Zi` | Debug | 无优化 + 调试信息 |
| `/MD` | Release | 多线程 DLL 运行时 |
| `/MDd` | Debug | 多线程 DLL 调试运行时 |

## Debug vs Release 库名规则

| 库 | Release | Debug |
|----|---------|-------|
| OpenCV | `opencv_world455.lib` | `opencv_world455d.lib` |
| PCL | `pcl_*_release.lib` | `pcl_*_debug.lib` |
| VTK | `vtk*-9.1.lib` | `vtk*-9.1d.lib` |

统一使用 `$(Configuration)` 条件宏区分，**不写两套重复配置**：

```xml
<AdditionalDependencies Condition="'$(Configuration)'=='Release'">
  opencv_world455.lib;...;%(AdditionalDependencies)
</AdditionalDependencies>
<AdditionalDependencies Condition="'$(Configuration)'=='Debug'">
  opencv_world455d.lib;...;%(AdditionalDependencies)
</AdditionalDependencies>
```

## Qt 集成规范

- 项目类型：Qt VS Tools Application（不是普通 Console/Win32）
- `twoProjector.ui` 通过 uic 自动生成 `ui_twoProjector.h`，**不提交到 Git**
- moc 产物（`moc_MainWindow.cpp` 等）**不提交到 Git**
- Qt 属性表路径：通过 Qt VS Tools 插件自动生成，不手动配置

## .gitignore 必须忽略的条目

```gitignore
# VS 产物
.vs/
*.user
*.suo
*.aps
ipch/

# 构建输出
x64/
Release/
Debug/
build/

# Qt uic/moc 产物
ui_*.h
moc_*.cpp
*.rcc
qrc_*.cpp

# 运行时数据（体积过大，不入库）
data/
logs/
config/params.json

# 调试信息
*.pdb
*.ilk

# 库目录（不入库，仅引用）
lib/
```

## Git 初始化命令（执行顺序）

```bat
echo "# graduateSoftware" >> README.md
git init
git add README.md
git commit -m "build: initial commit"
git branch -M main
git remote add origin https://github.com/Touda8/graduateSoftware.git
git push -u origin main
```
