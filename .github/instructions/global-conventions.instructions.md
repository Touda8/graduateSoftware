---
applyTo: "**"
---

# 全局编码约定

> 适用于仓库内所有文件。所有 Agent 必须遵守。

## C++17 规范

- 统一使用 C++17 标准 (`/std:c++17`)
- 优先使用 `std::optional<T>` 代替"可能无效"的返回值
- 文件系统操作一律使用 `std::filesystem`（禁用 `_mkdir`、`access` 等 POSIX API）
- 只读字符串参数使用 `std::string_view`，避免不必要拷贝
- 禁止使用裸 `new` / `delete`；堆内存统一使用 `std::unique_ptr` / `std::shared_ptr`
- 禁止使用 C 风格数组（用 `std::array` 或 `std::vector`）

## 命名规范

| 元素 | 规则 | 示例 |
|------|------|------|
| 类名 | PascalCase | `PhaseDecoder`, `BGADetector` |
| 函数名 | camelCase | `calcPhaseMatrix()`, `runAsync()` |
| 成员变量 | 尾随下划线 | `modThresh_`, `calibData_` |
| 局部变量 | camelCase | `absPhase`, `heightMap` |
| 常量 / 枚举项 | UPPER_SNAKE_CASE | `MAX_FREQ_COUNT`, `DEFAULT_THRESH` |
| 宏 | `TP_` 前缀 + UPPER_SNAKE | `TP_CHECK_EMPTY(mat)` |
| 头文件守卫 | `#pragma once`（禁止 `#ifndef` 风格） | — |

## 头文件规则

- 所有类必须分离 `.h` / `.cpp`（纯模板类例外）
- 头文件只暴露必要公共接口，实现细节留在 `.cpp`
- 使用前向声明减少 `#include` 层数（宁可在 `.cpp` include，不在 `.h` include）
- 头文件中**禁止** `using namespace std`

## 资源管理（RAII）

- `cv::Mat`、`pcl::PointCloud::Ptr`、Qt 对象树均通过 RAII 管理
- 析构函数不抛出任何异常（加 `noexcept`）
- 跨线程传递数据用 `std::shared_ptr`，配合读写锁或 `std::atomic`

## 禁止事项

- 禁止全局非 `const` 变量
- 禁止将单个 `cv::Mat` 对象不加锁地共享给多个线程（传前 `.clone()`）
- 禁止魔法数字——阈值、频率、步数等均定义为 `constexpr` 常量或从配置读取
- 禁止注释掉大段代码提交；需要保留历史用 Git 分支

## 注释规范

- 公共接口函数写一行 `// 功能：xxx，输入：xxx，返回：xxx`
- 复杂算法逻辑写行内注释说明**意图**，不解释代码本身
- 待办格式：`// TODO(agentN): 描述`；缺陷格式：`// FIXME(agentN): 描述`

## Git 提交格式

```
<type>(<scope>): <subject>
```

| type | 含义 |
|------|------|
| `feat` | 新功能 |
| `fix` | 缺陷修复 |
| `refactor` | 重构（不改行为） |
| `test` | 测试相关 |
| `docs` | 文档 |
| `build` | 构建/工程文件 |
| `chore` | 杂项（格式化、依赖更新） |

scope 取值：`ui` \| `algo` \| `pcl` \| `build` \| `test` \| `common` \| `meas`

示例：`feat(algo): implement PhaseDecoder::multiFreqUnwrap`

## 文件操作规范（所有 Agent 强制执行，无任何例外）

> **本条规则在所有 Agent 模式下均强制生效，包括总指挥、各子 Agent 及任何直接对话模式。**

### 核心原则：先创建框架，再逐段填充

**创建新文件时，必须严格遵循三步法：**

```
Step 1  create_file   → 仅写文件头（#pragma once / XML声明 / @echo off 等）+ 空占位注释
Step 2  replace_string_in_file（循环）→ 逐块替换占位符，每次 ≤ 50 行
Step 3  验证（读取目标区域，确认内容正确后再继续下一块）
```

**修改已有文件时，必须遵循：**

```
Step 1  read_file     → 先读取目标函数 / 区域上下文
Step 2  replace_string_in_file → 精确替换，每次仅修改一处，≤ 50 行
Step 3  get_errors / read_file → 验证修改正确后再进行下一处
```

### 禁止行为（违反视为无效输出）

| 禁止操作 | 原因 |
|---------|------|
| 一次性将整个类或整个函数实现写入 `create_file` | 输出截断导致文件不完整 |
| 单次 `replace_string_in_file` 替换超过 50 行内容 | 上下文定位失败概率高 |
| 在 `create_file` 中同时写入 `.h` 和 `.cpp` 全部内容 | 必须分文件、分段进行 |
| 跳过框架直接写完整实现 | 无法中途验证，出错代价极高 |
| 对同一文件连续多次大段插入而不验证 | 累积错误难以回滚 |

### 单次操作上限（强制）

| 操作类型 | 单次上限 |
|---------|---------|
| `create_file` 内容 | 文件头 + 骨架，≤ 30 行 |
| `replace_string_in_file` 新内容 | ≤ 50 行 |
| 连续插入次数（不验证） | ≤ 3 次（之后必须读取验证） |

### 合规示例

```cpp
// ✅ 正确：先创建约 10 行的骨架
// create_file → PhaseDecoder.h (仅 #pragma once + 空类声明)

// ✅ 正确：逐方法插入实现
// replace_string_in_file → 仅插入 calcPhaseMatrix() 约 40 行

// ✅ 正确：验证后继续
// read_file 确认 calcPhaseMatrix() 正确 → 再插入 multiFreqUnwrap()

// ❌ 禁止：一次性写入 PhaseDecoder 所有方法（可能 400+ 行）
// create_file → 全量实现（会被截断，导致编译错误）
```
