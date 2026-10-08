# Project9-Spider2D

使用 Python 3.11 和 Pygame 构建的二维程序化八足蜘蛛运动模拟器。项目用于验证后续 3D 蜘蛛项目需要的身体追踪、解析 Two-Bone IK、足端锁地、分组步态和落脚约束算法；不依赖 NumPy、Blender、Unity 或物理引擎。

当前版本：`1.0.0`（阶段一交付版）。

项目完成人与维护者：[skkkk1028](https://github.com/skkkk1028/)。
公开仓库：[skkkk1028/Project9-Spider2D](https://github.com/skkkk1028/Project9-Spider2D)。

## 已实现功能

- 鼠标作为身体移动目标，身体按 delta time 平滑加速、减速并朝实际移动方向旋转。
- 恰好 8 条镜像装配的腿，每腿包含髋关节、两段骨骼、膝关节和足端。
- 复用无绘制状态的二维解析 Two-Bone IK；处理过远、过近和退化目标。
- 足端在 `PLANTED → LIFT → SWING → LOWER → PLANTED` 状态机中锁地、抬起、摆动和落下。
- 固定四组对角步态；任意时刻最多活动 2 条腿、同侧最多 1 条、至少 6 条支撑腿。
- 新落脚点受身体侧边界、前后通道、可达范围、伸展率和最小足距约束。
- 候选规划减少骨段严格相交，并额外检测有厚度骨段的净距。
- 无严格相交的候选组合优先满足骨段净距；无法全部满足时，选择净距总缺口更小的组合，避免仅因净距偏好阻塞步态。
- 弯膝解析分支保持连续；膝盖进入身体走廊时会被明确标记。
- 调试层显示默认足点、活动区域、候选点、步态状态、足距违规、骨段相交、骨段净距、身体穿模、弯膝跳变和解剖侧违规。

## 环境与安装

- Python 3.11–3.13
- Pygame 2.6.1 或兼容的 2.x 版本

在 PowerShell 中运行：

```powershell
git clone https://github.com/skkkk1028/Project9-Spider2D.git
cd Project9-Spider2D
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[dev]"
```

## 运行

主入口默认启动完整的受约束八足蜘蛛：

```powershell
python -m spider_sim
```

安装为可编辑包后也可运行：

```powershell
spider-sim
```

保留的分阶段演示：

```powershell
# M2：仅身体追踪
python -m spider_sim --demo body

# M3：单腿 IK
python -m spider_sim --demo ik

# M4：静态八腿
python -m spider_sim --demo gait --gait-mode static

# M5：足端锁地与单腿步进
python -m spider_sim --demo gait --gait-mode single-step

# M6：四相分组步态
python -m spider_sim --demo gait --gait-mode four-phase

# 最终模式：落脚约束与完整诊断
python -m spider_sim --demo gait --gait-mode constrained
```

`--max-frames N` 可在渲染 N 帧后退出，供冒烟测试使用。

## 输入与调试显示

| 模式 | 输入 | 行为 |
|---|---|---|
| 全部窗口 | `Esc` 或关闭窗口 | 退出 |
| 全部窗口 | `F1` | 显示或隐藏调试层 |
| Body / Gait | 鼠标 | 设置身体世界坐标目标 |
| IK | 鼠标 | 设置单腿请求足端 |
| IK | `B` | 翻转 bend hint，并清除膝盖历史 |
| IK | `R` | 清除膝盖历史 |

最终模式的主要标记：

- 线框多边形：每条腿允许的新落脚活动区。
- 小圆点：候选落脚点；实心点为选中点，红色为拒绝，橙色为显式降级。
- 红色叉号：骨段严格相交。
- 青色圆圈：不同腿骨段的净距小于配置阈值。
- 粉色方框：骨段进入身体安全椭圆。
- 黄色圆环：膝盖进入身体走廊或足端越过身体中线。
- 紫色圆环：膝关节出现超过阈值的帧间跳变。

候选点与既有支撑腿相交的拒绝原因会保留，即使另一组候选已被选中。
弯膝和落地瞬时异常按渲染帧汇总，保证前面子步的事件能显示一次；累计计数仍由仿真子步更新。

## 核心结构

```text
src/spider_sim/
├── __main__.py       # 命令行入口
├── app.py            # M2 身体演示循环
├── config.py         # 集中、不可变、带校验的配置
├── simulation.py     # delta time、子步进和身体仿真
├── spider.py         # SpiderBody
├── math2d.py         # 坐标变换与角度工具
├── ik.py             # 解析 Two-Bone IK
├── leg.py            # 八腿装配和静态求解
├── stepping.py       # 足端状态机与四相调度
├── landing.py        # 落脚候选、约束和几何诊断
├── renderer.py       # 身体绘制
├── ik_renderer.py    # 单腿 IK 调试绘制
├── gait_renderer.py  # 八腿、步态和约束绘制
└── demos/
    ├── ik_demo.py
    └── gait_demo.py
```

逻辑更新和绘制相互分离。仿真层不读取鼠标、不创建窗口，也不依赖渲染帧率；应用层每帧读取一次输入，再用不超过 `TimingConfig.max_substep_seconds` 的子步更新。

## 关键默认参数

| 配置 | 默认值 | 作用 |
|---|---:|---|
| `TimingConfig.max_substep_seconds` | `1/120 s` | 最大仿真子步 |
| `BodyConfig.max_speed` | `280 px/s` | 独立身体演示速度上限 |
| `LegConfig.upper_length/lower_length` | `58/62 px` | 两段腿骨长度 |
| `FootStepConfig.trigger_distance` | `28 px` | 默认足点偏差触发阈值 |
| `FootStepConfig.max_extension_ratio` | `0.88` | 伸展率触发阈值 |
| `FootStepConfig.LIFT/SWING/LOWER` | `0.08/0.16/0.08 s` | M5 基础状态时长 |
| `LandingConstraintConfig.phase_duration_scale` | `0.625` | 最终四组步态的状态时长缩放 |
| `LandingConstraintConfig.demo_max_speed` | `20 px/s` | 最终受约束模式的身体速度上限 |
| `LandingConstraintConfig.min_foot_spacing` | `18 px` | 足端最小间距 |
| `LandingConstraintConfig.bone_clearance` | `6 px` | 骨段净距诊断和候选评分阈值 |
| `LandingConstraintConfig.knee_body_clearance` | `3 px` | 膝盖相对身体的安全余量 |

所有参数集中在 `src/spider_sim/config.py`。配置构造时会拒绝非法类型、非有限值和不合理范围。

## 测试与验收

运行静态检查和全部自动测试：

```powershell
python -m ruff check .
python -m pytest -q
```

测试覆盖：

- 坐标变换、角度、Two-Bone IK 可达性和骨长不变量。
- 八腿数量、左右镜像、前后顺序和静态局部结构。
- 足端世界锁定及完整 LIFT/SWING/LOWER 状态循环。
- 四组公平轮转、活动腿/同侧腿/支撑腿上限。
- 落脚活动区、候选拒绝、显式延期和降级。
- 净距候选优选、支撑腿相交候选的拒绝记录，以及多子步帧的瞬时告警保留与下一帧清除。
- 30/60/144 FPS 脚本回放；离散步数允许相差至多 4 次事件，其余身体、足端和状态不变量按公开容差比较。
- 600 秒加速仿真 soak、随机身体移动和 NaN 检查。
- 最终 Pygame 入口的 dummy 驱动及真实窗口冒烟运行。

## 已知限制

- 这是二维算法验证器，不包含重力、质量、地形、物理碰撞或真实蜘蛛生物力学。
- 抬脚高度是独立的可视化标量，不进入平面 IK。
- 支撑足严格锁定世界坐标；身体快速旋转时，旧支撑足的身体局部投影可能暂时离开其落脚通道。强制纠正会破坏锁地，因此系统在轮到该组时换步。
- 四组调度保持最多两腿活动。极端转向或跳点输入下，排队期间仍可能短暂出现膝盖进入身体走廊、骨段靠近或相交；这些情况会被明确检测、累计并显示，而不会被静默当作合法状态。
- 骨段净距属于几何诊断和候选优化，身体穿模属于几何诊断；二者都不是带质量和冲量的物理碰撞响应。
- 候选集是离散集合；没有合法候选时系统会显示 `DEFERRED` 并稍后重试，只有严格受控的相交改善路径会显示 `DEGRADED`。

详细的原始设计方案见 [docs/architecture-plan.md](docs/architecture-plan.md)。
