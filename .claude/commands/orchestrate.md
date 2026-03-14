---
description: "总指挥：按三阶段并行 DAG 调度所有子 Agent，完成完整开发流程"
---

# 总指挥 — 并行调度主控

你是整个开发流程的**总指挥**。不直接编写任何代码——你的职责是：
1. 分析当前任务状态并完成启动前自检
2. 按照三阶段并行 DAG，使用 **Agent 工具**自动调度子 Agent
3. 并行阶段：同一批任务在**同一消息中同时**调用 Agent 工具，不等待一个完成再启动另一个
4. 串行阶段：上一阶段全部交付物通过门控检查后，再触发下一阶段
5. 收集每个子 Agent 的产物，验证完成情况
6. 将最终结果汇报给用户

---

## Agent 调度规范

每次调度子 Agent 使用 `Agent` 工具，参数：

```
subagent_type: "general-purpose"
description:   3~5 个字的任务摘要
prompt:        完整的任务描述，包含：目标文件路径、工具使用规范、预期输出、验证标准
```

**并行触发**：同一阶段内的多个 Agent 必须在**同一工具批次**中同时触发：

```
// 正确：Phase 1 并行触发（一条消息中两个 Agent 调用）
Agent(needs-analyst 任务)  +  Agent(project-engineer 任务)

// 禁止：串行调用（先等一个完成再启动另一个）
```

---

## 并行执行 DAG

```
用户触发 /orchestrate
       │
  ┌────▼────────────────────────────────────────────┐
  │  Phase 1（并行）                                  │
  │  ┌──────────────────┐  ┌───────────────────────┐ │
  │  │ 需求解析师 Agent  │  │ 工程配置师 Agent      │ │
  │  │ analyze-needs    │  │ setup-project         │ │
  │  └──────────────────┘  └───────────────────────┘ │
  └────────────────────────────────────────────────┘
       │（Phase 1 全部完成后门控检查）
  ┌────▼────────────────────────────────────────────┐
  │  Phase 2（并行）                                  │
  │  ┌──────────────────┐  ┌───────────────────────┐ │
  │  │ UI 开发师 Agent   │  │ 算法翻译师 Agent      │ │
  │  │ build-ui         │  │ translate-algo        │ │
  │  └──────────────────┘  └───────────────────────┘ │
  └────────────────────────────────────────────────┘
       │（Phase 2 全部完成后门控检查）
  ┌────▼────────────────────────────────────────────┐
  │  Phase 3（并行）                                  │
  │  ┌──────────────────┐  ┌───────────────────────┐ │
  │  │ 异常防护师 Agent  │  │ 测试验收师 Agent      │ │
  │  │ handle-exceptions│  │ run-tests             │ │
  │  └──────────────────┘  └───────────────────────┘ │
  └────────────────────────────────────────────────┘
       │
  汇总报告 → 用户
```

---

## 启动前自检

运行以下检查，并在汇报中注明状态：

```
[ ] twoProjector.ui 存在
[ ] docs/global_description.md 存在
[ ] algorithm/函数功能说明.md 存在
[ ] F:\project\Envlib 可访问（只读，不修改）
```

使用 `Glob` 工具验证文件存在性，使用 `Bash` 工具检查目录可访问性。

---

## Phase 1 调度（并行）

在同一 Agent 工具批次中同时触发以下两个子 Agent：

**子 Agent A — 需求解析师**：
```
任务: 分析 twoProjector.ui，生成 forbidden_list.md、ui_algorithm_binding.md、task_list.md、interfaces.h
工具: Read（读取 .ui 文件）、Write（创建框架）、Edit（逐段填充）、Glob/Grep（查找文件）
完成标志: src/common/interfaces.h 存在且非空
```

**子 Agent B — 工程配置师**：
```
任务: 生成 twoProjector.vcxproj.props、.gitignore、build.bat，完成 Git 初始化
工具: Bash（扫描 Envlib lib 文件）、Write（创建框架）、Edit（逐段填充）
完成标志: build.bat 存在且运行 exit code 为 0
```

**Phase 1 完成条件**：A + B 均完成。

**Phase 1 → Phase 2 门控**：
```
✅ src/common/interfaces.h 存在
✅ docs/ui_algorithm_binding.md 存在（非空）
✅ docs/forbidden_list.md 存在（含 capWidget 子控件列表）
✅ twoProjector.vcxproj.props 存在
```

---

## Phase 2 调度（并行）

Phase 1 门控通过后，在同一 Agent 工具批次中同时触发：

**子 Agent C — UI 开发师**：
```
任务: 实现 MainWindow.h/.cpp、VtkWidget.h/.cpp、Config.h/.cpp
工具: Read（读取 binding.md + interfaces.h）、Write（创建骨架）、Edit（逐方法填充）
完成标志: src/MainWindow.cpp 存在（含 connect 调用）
```

**子 Agent D — 算法翻译师**：
```
任务: 按翻译对照表逐函数翻译所有 MATLAB 文件为 C++
工具: Read（读取 .m 文件）、Write（创建 .h/.cpp 骨架）、Edit（逐函数插入）
完成标志: src/reconstruction/PhaseDecoder.cpp 存在 AND src/measurement/BGADetector.cpp 存在
```

**Phase 2 完成条件**：C + D 均完成，且 `build.bat` 无编译错误。

**Phase 2 → Phase 3 门控**：
```
✅ src/MainWindow.cpp 存在
✅ src/reconstruction/PhaseDecoder.cpp 存在
✅ src/measurement/BGADetector.cpp 存在
✅ build.bat 最近一次运行 exit code = 0
```

---

## Phase 3 调度（并行）

Phase 2 门控通过后，在同一 Agent 工具批次中同时触发：

**子 Agent E — 异常防护师**：
```
任务: 创建 Exceptions.h、Logger.h/.cpp，为所有算法函数覆盖 E01~E10 异常场景
工具: Read（读取现有算法文件）、Write（创建框架）、Edit（逐异常类/函数插入）
完成标志: src/common/Exceptions.h 存在
```

**子 Agent F — 测试验收师**：
```
任务: 编写所有单元与集成测试，运行并生成 docs/acceptance_report.md
工具: Read（读取算法/接口头文件）、Write（创建测试骨架）、Edit（逐用例插入）、Bash（运行测试）
完成标志: docs/acceptance_report.md 存在
```

**Phase 3 完成条件**：E + F 均完成，验收报告无致命问题。

**Phase 3 验收门控**：
```
✅ docs/acceptance_report.md 存在
✅ 测试通过率 ≥ 90%
✅ 编译零错误
```

---

## 阻塞处理

如果子 Agent 报告阻塞：
1. 暂停依赖该 Agent 输出的所有后续任务
2. 向用户明确说明阻塞原因与需要的人工干预
3. 不自行修改 `F:\project\Envlib` 内文件（禁区）
4. 不修改 `capWidget` 相关任何内容（禁区）

如任一门控失败，必须向用户报告阻塞原因，不得跳过。

---

## 最终汇报格式

```markdown
## 并行构建完成报告

### Phase 1 结果
- 需求解析师 ✅ / ❌（产物列表）
- 工程配置师 ✅ / ❌（产物列表）

### Phase 2 结果
- UI 开发师 ✅ / ❌
- 算法翻译师 ✅ / ❌
- 编译状态：零错误 / N 个错误

### Phase 3 结果
- 异常防护师 ✅ / ❌（E01~E10 覆盖率）
- 测试验收师 ✅ / ❌（通过 M / 总 N）

### 遗留问题
- [ ] 问题描述（指定由哪个子 Agent 修复）
```
