# Talos 自瞄联调备份包

这个目录保存了 Talos 相关的关键改动，方便在新环境中恢复、核对和复用模拟器联调配置。重点包括：

- 图像与 pose 的共享内存同步消费修正
- Talos 模式下的 gimbal 输出接入
- YAML 中相机/云台后端命名调整
- CMake 适配与工程替换说明

---

## 1. 目录和文件说明

本包中的核心文件：

- `Sc_vision/io/talos.hpp`
- `Sc_vision/io/talos.cpp`
- `Sc_vision/io/cboard.cpp`
- `Sc_vision/io/CMakeLists.txt`

它们对应到主工程中的位置：

- 自瞄项目根目录：`$HOME/Sc_vision`
- Talos 底层实现：`$HOME/Sc_vision/io/`
- CMake 文件原位置：`$HOME/Sc_vision/io/CMakeLists.txt`
- 模拟器目录：`$HOME/bevy_robomaster_simulator-master`

如果你的目录不是这个位置，可以直接替换下面的环境变量：

```bash
export SC_VISION_ROOT="$HOME/Sc_vision"
export SIMULATOR_ROOT="$HOME/bevy_robomaster_simulator-master"
```

---

## 2. 如何覆盖回主工程

如果需要把备份中的文件写回自瞄项目，执行：

```bash
cp "$PWD/Sc_vision/io/talos.hpp" "$SC_VISION_ROOT/io/"
cp "$PWD/Sc_vision/io/talos.cpp" "$SC_VISION_ROOT/io/"
cp "$PWD/Sc_vision/io/cboard.cpp" "$SC_VISION_ROOT/io/"
cp "$PWD/Sc_vision/io/CMakeLists.txt" "$SC_VISION_ROOT/io/"
```

如果你没有在当前目录执行，而是在其他目录下，直接把 `$PWD` 换成实际路径即可。

---


## 3. 如何修改配置中的相机 / 云台命名

在你自己的工程配置文件里，最关键的是要保证后端名字和代码中使用的名字一致。

你只需要在原工程的配置中检查这几项：

```yaml
camera_name: "talos"
gimbal_backend: "talos"
# 或者
output_backend: "talos"
```

这些字段的含义是：

- `camera_name`：控制相机后端实例选择
- `gimbal_backend` / `output_backend`：控制云台输出走 Talos 还是串口

如果配置里写成别的名字，比如：

```yaml
camera_name: "simulator"
```

但代码里只识别 `talos`，就会导致相机实例被错误创建，或者云台输出口不匹配。

你只需要在原工程配置文件中把对应字段改成和代码一致的名字即可，不需要把配置文件放进这个备份目录。

---

## 4. 编译命令

先进入自瞄项目目录：

```bash
cd "$SC_VISION_ROOT"
```

然后执行：

```bash
cmake -S . -B build
cmake --build build -j$(nproc) --target standard_mpc_se
```

如果已经生成了 build，可以直接：

```bash
cmake --build build -j$(nproc) --target standard_mpc_se
```

编译成功后，主程序路径大致为：

```bash
$SC_VISION_ROOT/build/standard_mpc_se
```

直接运行：

```bash
./build/standard_mpc_se configs/standard.yaml
```

---

## 5. 模拟器运行步骤

### 5.1 启动模拟器

```bash
cd "$SIMULATOR_ROOT"
cargo run
```

如果看到模拟器界面后，按：

- `F5`：开启/关闭 auto-aim 订阅状态

这是关键步骤。只有在订阅状态打开时，模拟器才会接受并响应云台控制命令。

### 5.2 启动自瞄程序

另开一个终端：

```bash
cd "$SC_VISION_ROOT"
./build/standard_mpc_se configs/standard.yaml
```

如果你的配置文件路径不在 `configs/standard.yaml`，就直接把命令中的路径改成你实际配置文件所在的位置即可。

如果一切正常，程序会：

- 从 Talos 共享内存读取图像
- 读取 pose / gimbal 状态
- 计算目标并输出云台控制
- 将命令写入模拟器可消费的共享内存通道

---

## 6. 关键注意事项

这里最关键的一点：

- 图像和 pose 是同步帧，不能只消费图像不消费 pose。
- 如果只读图像而不消费 pose，下一帧同步图像会被阻塞，最终表现为：第一帧正常，第二帧卡住。

对应代码核心在：

- `Sc_vision/io/talos.cpp`
- `Sc_vision/io/cboard.cpp`

这里的结构需要保持：

- 相机层：负责读取图像与 pose
- CBoard 层：负责输出云台命令
- 两者保持清晰分离，避免混在同一个函数里

---

## 7. 常见问题排查

### 问题 1：云台没有转

优先检查：

- 是否按了 `F5`
- 模拟器是否已启动
- 你工程配置中 `camera_name` / `gimbal_backend` 是否为 `talos`
- 自瞄程序是否真正选择了 Talos 后端

### 问题 2：程序只跑了一帧就卡住

重点检查：

- 是否只有图像被消费，没有 pose 被消费
- 共享内存的状态位和索引是否按照协议处理
- 是否有时序错位，导致下一帧被阻塞

### 问题 3：编译失败

重新执行：

```bash
cd "$SC_VISION_ROOT"
cmake -S . -B build
cmake --build build -j$(nproc) --target standard_mpc_se
```

---

## 8. 后续

后面如果继续推进 Talos / 模拟器联调，有两块非常值得专项研究：

### 8.1 图像色彩传输问题

当前输入图像的色彩通道顺序存在偏差，例如：

- `BGR` vs `RGB`
- OpenCV 容器中的通道布局和模拟器输出不一致

这会导致：

- 检测算法看到偏色
- 图像显示和真实场景不一致
- 训练 / 推理的图像语义不一致

后续：

- 统一检查 `cv::Mat` 的通道顺序
- 确认 simulator output 与 OpenCV 读取是否一致
- 增加一个色彩格式日志或转换函数

### 8.2 模型添加问题

后续如果增加新的检测/跟踪模型，需要重点处理：

- 模型路径与配置文件路径如何统一管理
- `yaml` 中新增模型参数项
- 模型输入尺寸与图像大小是否匹配
- 模型运行时是否需要固定 `BGR` 或 `RGB` 输入格式

建议以后统一按照：

- 配置文件控制模型路径
- 代码里统一读取参数
- 模型输入输出做一层适配层

这样能避免后面继续硬编码，便于扩展更多模型。

---

