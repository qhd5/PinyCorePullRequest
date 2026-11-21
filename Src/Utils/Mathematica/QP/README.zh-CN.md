# QP / TinyMPC

## 说明

本模块是对上游 [TinyMPC](https://github.com/TinyMPC/TinyMPC) 的封装。
依赖固定为提交 `023f36bd5b27267e1a96ed47a774d120b5707f13`。
`tools/cmake/FindLib/TinyMpcPatch.cmake` 在构建目录生成修正后的源文件，主机测试与固件共用：

- Q、R 保留原始阶段权重，由预计算函数添加一次 ADMM 惩罚，修复上游重复添加 `rho·I` 的问题。
  上游至今未修复（`main` 仍是 `Q+2ρI`），维护者在
  [issue #79](https://github.com/TinyMPC/TinyMPC/issues/79) 中确认 `rho` 会改变被求解的问题，
  但将其视为既有设计，未列为缺陷。
- 默认关闭每次求解成功时的输出；需要调试时可设置 `-DPINY_TINYMPC_ENABLE_SOLVE_LOG=ON`。
- 构建统一使用框架自带的 `Lib/Eigen3`，上游捆绑的 `include/Eigen` 不再进入编译单元。

补丁不修改 `ThirdParty/TinyMPC` 原始检出；版本或补丁匹配失败时配置会报错。
上层代码只使用框架的 `Matrix`，不直接接触 Eigen，关系类似 HAL 库之于标准库/寄存器。

```text
业务代码 ──> TinyMPC<nx, nu, N>（Matrix 接口，TinyMPC.hpp）
              └─> TinyMpcBackend（float 数组接口，TinyMpcBackend.hpp/.cpp）
                    └─> 上游 TinyMPC（Eigen，double，ThirdParty/TinyMPC）
```

Eigen 和上游头文件只在 `TinyMpcBackend.cpp` 中包含。上游头文件在全局写了 `using namespace Eigen`，
如果暴露到业务代码中会与框架的 `Matrix` 重名冲突。

## 求解问题

```text
min  Σ 1/2 (x_k - xr)' Q (x_k - xr) + 1/2 (u_k - ur)' R (u_k - ur)
s.t. x_{k+1} = A x_k + B u_k + f
     xMin <= x_k <= xMax
     uMin <= u_k <= uMax
```

`N` 表示 N 个状态节点和 N−1 个控制量。阶段代价适用于非终端节点；
终端处理沿用上游稳态 Riccati 缓存，不是简单地把终端权重设为 Q。

## 启用

在 menuconfig 中打开 `utils config -> mathematica config -> QP (TinyMPC)`（`USE_MATHEMATICA_QP`），
会自动打开 `USE_MATHEMATICA_MATRIX`。打开后 `utils` 会私有链接上游的 `tinympcstatic`；
未打开时不会下载或编译 TinyMPC 与 Eigen。

## 使用示例

本模块只做数学运算，不创建任务、不涉及调度。与 `LQR`、`KF` 一样，直接作为 APP 层类
（如底盘 `Mecanum`）的成员构造，在该类已有的任务/控制周期中调用 `solve()` 即可。
每个实例拥有独立的求解器，多个模块可以各自持有自己的 `TinyMPC`。

```c++
#include "TinyMPC.hpp"

// Construct after HAL initialization: 2 states, 1 input, 10 state nodes.
class Planner final {
public:
    Planner()
    {
        ready_ = mpc_.ready() &&
                 mpc_.setBounds({ { -5 }, { -5 } }, { { 5 }, { 5 } }, { { -2 } }, { { 2 } }) &&
                 mpc_.setStateRef({ { 0 }, { 0 } });
    }

    // On failure, preserve _u and let the caller choose its fallback controller.
    bool step(const Matrix<2, 1> &_x, float &_u)
    {
        if (!ready_ || !mpc_.setInitialState(_x) || !mpc_.solve())
            return false;
        Matrix<1, 1> input;
        if (!mpc_.getInput(input))
            return false;
        _u = input(0, 0);
        return true;
    }

private:
    TinyMPC<2, 1, 10> mpc_{ { { 1, 0.1f }, { 0, 1 } }, // A
                            { { 0.005f }, { 0.1f } },  // B
                            { { 10, 0 }, { 0, 1 } },   // Q
                            { { 0.1f } },              // R
                            1.f };                     // rho
    bool ready_ = false;
};
```

## 独立求解任务

APP 的周期不能等待 `solve()` 时，启用 `USE_MATHEMATICA_QP` 后可使用 APP 层的 `MPCTask.hpp`。
`MPCTask<nx, nu, N, stackWords>` 由机构持有，需在 HAL 初始化后构造并存活到应用结束。
最后一个模板参数为 FreeRTOS 栈字数，不是字节；应按上板测得的栈余量配置。

1. 初始化时按需调用 `setBounds()`，检查返回值，再调用 `start()`。默认优先级为 `LOW1`，低于 `AppTask` 的 `HIGH1`。启动后不再从 APP 修改求解器配置。
2. 机构按所需周期调用 `submit()`，传入 `Request_s`：状态 `x`、可非零的 `xRef` 和 `uRef`，以及采样节拍 `sample.tick`、请求序号 `sample.sequence` 和控制模式代号 `sample.epoch`。同一个任务只接收一个机构的请求；每次覆盖尚未开始的旧请求，不打断正在进行的求解。
3. 控制回调调用 `tryRead()`，不等待新结果。它返回 `true` 表示读到一条消息，不代表求解成功。只有 `status == Status_e::SUCCESS` 时 `u` 才是可用的首步控制量。`elapsedTicks` 是从开始处理请求到完成求解的墙钟节拍数，含调度抢占、不含排队时间，不能用来判定数据新鲜度。
4. 机构只采用代号一致、未过期的成功结果。用 `static_cast<TickType_t>(now - result.sample.tick)` 计算样本年龄，可跨一次节拍计数回绕；不要因存在更新的待处理请求就丢弃尚有效的结果。没有新消息时可保持上一份仍有效的控制量；同一模式下求解失败、结果过期、启动后尚无有效结果或切换模式时，执行机构定义的回退策略。

两个队列和任务栈使用静态存储。空闲时求解任务阻塞等待请求；若提交快于求解，任务可以连续运行，但不会阻塞等待它的 APP，因为 APP 不等待结果。
这不保证严格的 1 ms 周期，也不消除 TinyMPC 的运行期内存分配。接入前应确认 newlib 堆的任务间并发保护；上板验证最坏求解时间、样本到控制输出的延迟、APP 周期抖动和栈余量。
提交周期、模型离散步长和输出有效期由具体机构设定，本适配层不代替电机保护。

## 接口

| 接口                                      | 说明                             |
| ----------------------------------------- | -------------------------------- |
| 构造函数 `(A, B, Q, R, rho, f, settings)` | 创建求解器并预计算 Riccati 缓存  |
| `ready()`                                 | 构造时求解器是否初始化成功       |
| `setSettings(settings)`                   | 校验并更新设置；成功返回 `true`，失败保留原配置 |
| `setBounds(xMin, xMax, uMin, uMax)`       | 设置上下限，未调用前视为无约束   |
| `setInitialState(x0)`                     | 设置当前状态                     |
| `setStateRef(xRef)` / `setInputRef(uRef)` | 设置参考值                       |
| `solve()`                                 | 求解，收敛返回 `true`            |
| `getInput(u, k=0)` / `getState(x, k)`      | 检查索引并读取；失败返回 `false`，保持输出不变 |
| `input(k=0)` / `state(k)`                 | 兼容便捷读取；未初始化或索引非法时返回零 |
| `iterations()`                            | 上一次求解的迭代次数             |

`input/getInput` 的有效索引是 `0…N−2`，`state/getState` 是 `0…N−1`。
读取接口只检查初始化和索引，不表示最近一次求解已收敛；下发控制量前必须检查 `solve()` 返回值。
达到迭代上限时 `solve()` 返回 `false`，候选序列仅供诊断，由业务层决定备用控制策略；
收敛但结果非有限时同样返回 `false`。

设置要求：`maxIter`、`checkTermination` 大于零，两个容差为正且有限，`rho` 为正且有限。
`Q`、`R` 只使用对角线，且 `Q_ii + rho`、`R_ii + rho` 必须为正（Riccati 预计算需要可逆）。
模型、状态、参考值、边界必须有限，边界逐元素满足下限不大于上限（允许相等）。
非法构造参数使 `ready()` 为 `false`；非法设置、边界和参考值不会覆盖已有有效数据。

## 注意事项

1. `Q`、`R` 只有对角线生效，这是上游的行为。
2. 上下限和参考值在整个预测窗口内共用同一组；需要逐步变化的轨迹时要扩展 `TinyMpcBackend`。
3. 未封装上游的锥约束（SOC）、一般线性约束、时变约束、自适应 rho 和代码生成。
4. 构造时会在堆上（`new` -> `pvPortMalloc`）分配上游求解器，上游没有释放接口，
   实例应与所属 APP 对象同生命周期；对象禁止拷贝。
5. 上游的 `tinytype` 是 `double`。H723 的 FPU 支持双精度，F407 只支持单精度，F407 上会明显变慢；
   上板后建议用 `Dwt` 测一次 `solve()` 的耗时，再确定预测步数和 `maxIter`。
6. Eigen 动态矩阵通过 newlib 的 `malloc` 分配，不走 FreeRTOS 堆。`setInitialState()` 类型转换和
   `solve()` 内部临时矩阵也会在控制周期中分配内存；本封装不保证运行期零分配。
   需确认 newlib 堆容量、并发分配保护和任务栈，测量长期运行内存峰值与最坏求解时间。
7. 收敛日志默认关闭，但上游错误信息仍使用 `std::cout`，可能引入 iostream 占用。
8. 本次数值修正会改变控制量和收敛速度，升级后应重新检查约束、调参和周期预算。

## 测试

`test/utils/test_tinympc.cpp` 在主机上将本封装与直接调用上游 API 的结果对比
（第一步控制量误差 < 1e-4，迭代次数一致）。测试还覆盖闭环逐次收敛、非法设置与边界、
非有限值、索引边界、未初始化读取、迭代耗尽、日志开关，以及不依赖上游参考的标量解析解。
解析用例使用 `A=0, B=1, Q=1, R=3, N=3, uRef=1`，验证不同 rho 下首步输入均为 `3/4`。

```bash
cmake -S test -B build/test -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++
ninja -C build/test
./build/test/pinyTest "[tinympc]"
ctest --test-dir build/test --output-on-failure
```

## 验证范围

STM32F407IGHx 配置已通过 ARM 编译（Debug），并通过强制保留封装入口的独立链接检查，
确认 `tiny_solve` 和初始化路径实际进入 ELF。Debug 静态占用为 RAM 84,184 B、Flash 961,500 B
（1 MiB Flash 的 91.70%）。

QP 相关四个编译单元的 Flash 占用（`arm-none-eabi-size`，未链接裁剪）：

| 配置                | .text 合计 | 说明 |
| ------------------- | ---------- | ---- |
| Debug（-O0）        | 802,071 B  | 占 Debug 全量固件的约 83% |
| Release（-O3）      | 117,158 B  | 缩小约 85% |

按该差值推算，Release 全量固件的强制链接约 270 KB；准确数值需在 Release 配置下重跑一次
强制链接检查（`cmake -B build -DCMAKE_BUILD_TYPE=Release`）。以上均不含求解器运行期堆峰值，
也不代表所有尺寸的控制器占用。实际应用尚未接入该求解器；需上板验证最坏耗时、堆峰值、
长时间运行和求解失败时的控制切换。

## 未来优化

    TinyMPC,本质上是引入ADMM算法将MPC的凸QP问题转化为一个LQR问题。
    所以未来可以将数学库中的算法拆分为LQP,Riccati,ADMM,QP，最后单开一个文件夹来拼装MPC。实现不依赖第三方库的本土化MPC求解器
