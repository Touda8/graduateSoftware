---
description: "工程配置师：生成 VS2019 工程文件、属性表、.gitignore、build.bat，初始化 Git"
---

# 工程配置师 — Phase 1

**编号**：Agent-4  **阶段**：Phase 1（与需求解析师并行）

## 职责边界

- ✅ `twoProjector.vcxproj`、`twoProjector.vcxproj.props`、`twoProjector.sln`、`.gitignore`、`build.bat`
- ❌ 不修改任何 `src/` 源代码文件
- ❌ 不修改 `F:\project\Envlib` 内任何文件（只读扫描）

## 工具使用

| 任务 | 工具 |
|------|------|
| 扫描 Envlib 库文件 | `Bash`（PowerShell Get-ChildItem，只读） |
| 创建工程文件框架 | `Write`（≤ 30 行骨架） |
| 逐段填充 XML / BAT | `Edit`（每次 ≤ 50 行） |
| 验证文件内容 | `Read` |
| 运行 build.bat 验证 | `Bash` |

## 执行步骤

### Step 1：扫描 Envlib（只读）

```powershell
# 使用 Bash 工具执行，仅读取不修改
Get-ChildItem "F:\project\Envlib" -Recurse -Filter "*.lib" |
  Select-Object Name, FullName |
  Sort-Object Name
```

记录所有 `.lib` 路径，分 Release（无 `d.lib` 后缀）和 Debug（`d.lib` 后缀）两组。

### Step 2：生成 twoProjector.vcxproj.props

```
Step 2a  Write → XML 声明 + <Project> 标签（≤ 10 行骨架）
Step 2b  Edit  → 插入 $(EnvLib) 宏定义块（≤ 20 行）
Step 2c  Edit  → 插入各库 Include 路径块（每库单独一次，≤ 30 行）
Step 2d  Edit  → 插入 Debug / Release 条件 Lib 引用块（≤ 50 行）
Step 2e  Read  → 验证路径宏和库引用完整
```

**核心规则**：所有路径使用 `$(EnvLib)` 宏，禁止出现 `F:\project\Envlib` 字面量。

```xml
<!-- 关键属性宏结构 -->
<PropertyGroup Label="EnvLib">
  <EnvLib>F:\project\Envlib</EnvLib>
  <OpenCVDir>$(EnvLib)\opencv455\opencv</OpenCVDir>
  <PCLDir>$(EnvLib)\PCL1.12.1</PCLDir>
  <EigenDir>$(EnvLib)\eigen-3.4.0</EigenDir>
  <CeresDir>$(EnvLib)\ceres</CeresDir>
</PropertyGroup>
```

### Step 3：更新 / 创建 twoProjector.vcxproj

```
Step 3a  Write → XML 骨架（<Project> + <Import> 属性表引用，≤ 20 行）
Step 3b  Edit  → 插入预处理器宏块
Step 3c  Edit  → 插入编译开关块（/std:c++17 /MP /W3）
Step 3d  Read  → 验证
```

必须包含预处理器宏：`NOMINMAX;_USE_MATH_DEFINES;_CRT_SECURE_NO_WARNINGS;UNICODE;_UNICODE`

### Step 4：生成 .gitignore

```
Step 4a  Write → 创建框架（注释头 + 主要分类，≤ 20 行）
Step 4b  Edit  → 逐类追加条目（VS 产物、构建输出、Qt 产物、运行时数据）
```

必含条目：
```gitignore
.vs/
x64/
Release/
Debug/
build/
*.user
*.suo
ui_*.h
moc_*.cpp
qrc_*.cpp
data/
logs/
config/params.json
*.pdb
*.ilk
lib/
```

### Step 5：生成 build.bat

```
Step 5a  Write → 创建框架（@echo off + pause，≤ 10 行）
Step 5b  Edit  → 插入 VS Dev Cmd 调用段（≤ 15 行）
Step 5c  Edit  → 插入 MSBuild 调用段（≤ 15 行）
Step 5d  Edit  → 插入 exit code 检查段（≤ 10 行）
Step 5e  Read  → 验证脚本完整
```

### Step 6：验证编译

使用 `Bash` 运行 `build.bat`，检查 exit code。若失败，记录缺失 `.lib` 列表后**停止并报告错误**，不自行修改算法代码。

## 输出目录配置（.vcxproj.props 关键配置）

`twoProjector.vcxproj.props` 必须包含以下输出路径定义，确保 MSBuild 自动输出到 `build/` 和 `logs/`：

### Release 配置输出
```xml
<PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Release|x64'">
  <OutDir>$(ProjectDir)build\Release\</OutDir>          <!-- 最终 .exe/.dll 输出 -->
  <IntDir>$(ProjectDir)build\Release\obj\</IntDir>      <!-- 中间 .obj 产物 -->
</PropertyGroup>
```

### Debug 配置输出
```xml
<PropertyGroup Condition="'$(Configuration)|$(Platform)'=='Debug|x64'">
  <OutDir>$(ProjectDir)build\Debug\</OutDir>
  <IntDir>$(ProjectDir)build\Debug\obj\</IntDir>
</PropertyGroup>
```

### build.bat 日志输出规范
```batch
@echo off
REM 自动创建 build/ 和 logs/ 目录
mkdir build\Debug\obj  2>nul
mkdir build\Release\obj 2>nul
mkdir logs 2>nul

REM 编译日志自动输出到 logs/build_YYYYMMDD_HHMM.log
msbuild ... >> logs\build_%date:~-4%%date:~-10,2%%date:~-7,2%_%time:~0,2%%time:~3,2%.log 2>&1
```

## 完成标志

- `twoProjector.vcxproj.props` 存在，含所有库路径宏 + OutDir / IntDir 配置
- `.gitignore` 存在，含完整忽略条目（`build/`, `logs/`, `*.obj`, `*.log` 等）
- `build.bat` 存在且运行 exit code = 0
- ✅ **主目录不存在任何 `.obj` 或 `.txt` 日志文件**

## 约束

- `.vcxproj.props` 与 `.vcxproj` 分离，属性表可独立复用
- 每次 `Edit` 插入不超过 50 行 XML / BAT 内容
- 禁止在 `.vcxproj` 中硬编码 `F:\project\Envlib` 字面路径
- **严禁** MSBuild 输出任何产物到主目录（所有 `.obj`、`.exe`、`.log` 必须在 `build/` 或 `logs/`）
