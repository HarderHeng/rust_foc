# 后续实施规划

> 状态：阶段性收口
>
> 本文记录当前软件阶段的完成情况、硬件阻塞项和下一步实施顺序。除非重新确认，否则不在本阶段继续修改控制逻辑或自动烧录固件。

## 1. 当前收口状态

本阶段已完成以下软件工作：

- 增加 VBUS/NTC 运行时非阻塞采样和采样过期保护。
- 增加启动前母线、温度、编码器互锁。
- 统一启动时的安全占空比准备、PWM 使能和模式提交。
- 故障锁存首个故障原因，停机/故障清除电流、转速和开环目标。
- 禁止运行中修改极对数和电角度偏置。
- 编码器角度改为单圈控制角度，多圈位置使用整数计数，避免长时间运行浮点精度退化。
- 重复 `foc rpm` 命令改为更新目标，不重置速度 PI 和测速状态。
- Id/Iq/RPM 参考斜坡增加固定点小数累积，避免每毫秒取整导致速率失真；正负半单位均采用对称的远离零四舍五入。
- Park 与逆 Park 在一个控制步内共享同一组 `sin/cos`。
- `foc isr` 增加 JEOS 处理耗时、最大值、调用次数和超预算计数。
- 增加 FOC 算法测试、控制状态机安全回归测试、计时统计测试和验收说明。
- 固件、算法测试、Clippy、格式检查和 release 构建已经通过。

当前测试基线：

```text
cargo htest --locked --offline                         49 个算法测试 + 1 个控制安全测试通过
cargo check --release --locked --offline               通过
cargo clippy --release --locked --offline -- -D warnings 通过
cargo clippy -p foc --target x86_64-unknown-linux-gnu \
  --all-targets --locked --offline -- -D warnings      通过
cargo fmt --all -- --check                             通过
cargo build --release --locked --offline               通过
```

## 2. 当前硬件阻塞项

本次尝试进行上板验收时，当前环境未发现硬件：

- `probe-rs list` 未发现调试器。
- `/sys/bus/usb/devices` 不可用。
- `/dev/bus/usb` 不可用。
- 没有 `/dev/ttyACM*` 或 `/dev/ttyUSB*` 串口。
- 未执行烧录、复位、串口控制或电机启动。

因此当前不能确认以下内容：

- ADC1/ADC2 注入采样与普通 VBUS/NTC 采样的实际时序。
- 20 kHz JEOS 中断的真实最坏延迟。
- TIM1 CH4 触发窗口和采样窗口是否满足功率级要求。
- 启动、故障、编码器掉线和母线异常在真实硬件上的 MOE 关断时间。
- 电机空载运行、电流环稳定性、速度环方向策略和对齐质量。

**重要：主机测试通过不等于功率级安全。恢复实施时，必须先完成硬件连接和空载验收。**

## 3. 恢复工作前的准备

恢复前需要提供或确认：

1. 板卡已通过 USB 连接到可执行 `probe-rs` 的环境。
2. 调试器型号、目标芯片确认：`STM32G431CB`。
3. 串口设备路径及访问权限；当前设计为 USART2，921600 8N1。
4. 电源使用限流电源，初次测试不接电机或断开电机功率连接。
5. 已准备示波器或逻辑分析仪，用于观察 PWM、ADC 触发和故障关断。
6. 明确是否允许烧录。没有明确许可时，只做设备枚举和只读检查，不执行 `probe-rs run`。

建议先在目标主机执行：

```sh
probe-rs list
ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
id
```

## 4. 下一阶段 P0：硬件连通性和只读验证

### 4.1 工具链与固件确认

- 确认 `flip-link`、`probe-rs`、目标架构和 linker script 可用。
- 使用提交后的 `Cargo.lock` 构建，避免依赖漂移。
- 记录固件 ELF 的构建时间、Git revision 和 `.text/.data/.bss` 大小。
- 不在电机连接状态下设置 SWD 断点；PWM 开启时禁止暂停核心。

### 4.2 上电前检查

- 核对电源极性、母线电压、限流值和板卡跳线。
- 核对 TIM1 六路 PWM 引脚、ADC shunt 输入、VBUS、NTC、AS5600 I2C 引脚。
- 确认功率级硬件刹车输入未悬空。
- 确认电机断开或机械负载完全安全。

### 4.3 最小启动验证

只允许执行到 Idle：

```text
foc stop
system info
adc
enc
foc status
```

确认：

- VBUS、NTC 数值合理。
- AS5600 状态和角度合理。
- `fault=none` 或故障原因可解释。
- 未执行 `foc start`、`foc rpm`、`foc openloop` 或 `foc pwm` 前，MOE 为关闭状态。

## 5. 下一阶段 P0：ADC 和保护链路验收

### 5.1 普通 ADC 与注入 ADC 共存

在电机断开、PWM 尽可能安全的条件下验证：

- VBUS/NTC 数值在输出关闭时持续刷新。
- 允许安全 PWM 后，VBUS/NTC 仍持续刷新。
- 普通 ADC 采样没有重写注入 ADC 的 JSQR、采样时间或触发源。
- ADC1/ADC2 JEOS 仍以 20 kHz 触发。
- 普通 ADC 轮询保持非阻塞：首轮启动 VBUS，随后读取 VBUS 后启动 NTC，读取 NTC 后发布完整数据并启动下一轮 VBUS。
- shunt offset 校准后，注入采样恢复正常。
- `cal current` 期间 PWM 保持关闭，校准失败不会重新开启功率输出。

### 5.2 保护触发

需要通过专门的测试构建、可控输入或断开传感器来验证：

- VBUS 低于 `VBUS_UV_MV` 时进入 `fault=vbus`。
- VBUS 高于 `VBUS_OV_MV` 时进入 `fault=vbus`。
- NTC 超温进入 `fault=ntc`。
- VBUS/NTC 数据超过 `BUS_FAULT_MS` 未更新时进入 `fault=adc`。
- AS5600 I2C 错误、磁场无效和角度数据过期进入 `fault=enc`。
- 硬件 Break 触发进入 `fault=brk`，并且启动请求不能覆盖该故障。
- 软件过流进入 `fault=ocp`。
- 所有故障都关闭 MOE，并清除参考目标。

记录每种故障从输入异常到 MOE 关闭的时间；不能只依赖日志输出判断保护速度。

## 6. 下一阶段 P0：JEOS 中断性能测量

`foc isr` 当前提供以下信息：

```text
isr handler last=<us> us max=<us> us cyc=<last>/<max> \
  budget_cyc=8500 calls=<n> over=<n>
```

含义：

- `last/max`：JEOS handler body 的最近/最大周期数。
- `calls`：统计窗口内的 handler 次数。
- `over`：耗时大于等于 8500 cycles 的次数。
- 8500 cycles 对应 170 MHz、20 kHz、50 us PWM 周期。
- 计时包含 ADC 读取、当前重构、保护检查、FOC 计算和占空比写入。
- 计时不包含 Cortex-M 异常入栈/出栈、等待中断响应时间和统计发布本身。

测量步骤：

1. 进入安全 Idle，执行 `foc isr reset`。
2. 启动 RTT/串口诊断，但不要阻塞 ISR。
3. 在不接负载的条件下运行短时间，执行 `foc isr`。
4. 分别测量：无 PWM、Bench、Align、Run、Speed 及普通 ADC 活跃状态。
5. 用示波器同时观察 TIM1 CH4/ADC 触发和 GPIO 或 PWM 边沿，估计完整 IRQ 延迟。
6. 保存 `last/max/calls/over`、电源电压、编译版本和测试模式。

判定原则：

- 不能只看平均值，必须看最大值和外部波形。
- `over=0` 不是硬件 deadline 的充分证明，因为当前统计不含异常入栈/出栈及 IRQ 等待时间。
- 如果出现超预算，优先减少 ISR 内的共享状态访问、遥测发布和临界区，而不是直接降低 PWM 频率。

## 7. 下一阶段 P1：控制行为和空载测试

硬件保护验证通过后，才允许接入电机，并从低风险模式开始。

### 7.1 对齐

```text
foc stop
cal current
foc align 500
foc status
foc save
```

确认：

- Id 能稳定到对齐目标。
- 对齐期间 Iq 为零。
- 编码器有效且无跳变。
- 对齐完成后 PWM 关闭并进入 Idle。
- 重新上电后 NVM 偏置和极对数加载正确。

### 7.2 当前环

- 先使用很小的 Id/Iq 参考。
- 验证 `id/iq` 实际值与目标值方向正确。
- 观察电流过渡是否符合配置的斜坡。
- 观察电压限幅、Vd 优先策略和积分器是否出现明显累积。
- 检查电流采样重构的三相和是否接近零。

### 7.3 速度环

- 先给低正转速目标，再给零速。
- 再测试低负转速目标，确认无错误方向启动扭矩。
- 重复发送 `foc rpm` 时确认 PI/测速状态不会重置。
- 验证 RPM 滤波未准备好时不会注入启动电流。
- 验证参考斜坡在不同任务周期下仍由实际 elapsed time 决定。
- 测试超速、编码器掉线、母线异常和停止后的目标清除。

## 8. 下一阶段 P1：性能优化候选

只有完成真实耗时采集后才选择以下项目：

1. 缩短 `cortex_m::interrupt::free` 范围，避免把完整 FOC 计算包在全局关中断内。
2. 减少 ISR 中的遥测原子写入；显示数据可以降采样，但保护数据不得降采样。
3. 评估普通 ADC 轮询与 JEOS 的竞争，必要时迁移到 DMA 或定时触发采集。
4. 根据实测结果比较 release `opt-level = "z"`、`"2"` 和 `"3"` 的代码尺寸及最坏耗时。
5. 评估 `sin_cos` 优化的实际收益，不以主机 benchmark 代替 Cortex-M 测量。
6. 增加完整 IRQ 入口/出口 GPIO 测量，区分 handler body 与端到端延迟。
7. 评估是否需要硬件/软件 watchdog，以及 watchdog 故障记录策略。

**禁止在没有基线测量时凭直觉改 PI 参数、PWM 频率、采样窗口或死区参数。**

## 9. 下一阶段 P2：工程收尾

- 将固件版本、板卡版本、功率级参数和电机参数分离记录。
- 在 CI 中固定执行 `cargo htest`、`cargo check`、`cargo clippy` 和 `cargo fmt`。
- 为 ADC 采样时序、启动状态机和故障响应增加可重复的测试构建。
- 为每次上板测试保存：Git revision、固件 SHA、工具链版本、命令记录、日志和示波器截图。
- 增加 NVM 写入次数/磨损策略评估；当前保存仍是整页擦除后写入。
- 增加对齐有效性标志或参数版本；修改极对数后强制重新对齐并持久化。
- 评估硬件 watchdog、故障复位策略和上电默认输出状态。
- 测试长期运行下的角度、速度、NVM 和遥测计数器是否稳定。

## 10. 恢复开发时的推荐顺序

```text
1. 接通 probe-rs / UART，完成只读设备确认
2. 编译并记录固件 artifact
3. 空载上电，确认 Idle 和传感器数据
4. 验证普通 ADC + 注入 ADC 共存
5. 验证软件/硬件保护及 MOE 关断
6. 采集 JEOS 端到端耗时和最大值
7. 低风险对齐
8. 低电流当前环
9. 低速速度环
10. 根据测量结果决定性能优化
11. 更新本文和 docs/safety-bringup.md 的实际测量结果
```

## 11. 当前明确不做的事项

- 不在无硬件连接时烧录或复位目标板。
- 不在没有波形和电流限流条件时接入负载。
- 不凭主机测试结果宣称 ADC/PWM 时序安全。
- 不在没有 ISR 基线数据时缩短临界区或调整 PWM 频率。
- 不在运行模式中在线修改极对数、电角度偏置或功率级参数。
- 不把 RTT/串口日志放入 20 kHz ISR。
