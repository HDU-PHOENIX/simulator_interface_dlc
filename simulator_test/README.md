# Talos IPC 测试程序说明

这个目录中放置了两个用于验证 Talos IPC 共享内存协议的最小测试程序：

- `talos_ipc_image_pose_receiver/`
- `talos_ipc_gimbal_cmd_sender/`

它们的目标都是验证 simulator 侧是否正常发布图像、pose 和云台命令，并帮助确认外部程序是否按协议正确消费/发送数据。

---

## 1. 程序总体说明

这两个程序都直接访问共享内存：

- `/tmp/talos_ipc_meta`
- `/tmp/talos_ipc_image_pool`

它们依赖 simulator 端的 Talos IPC 协议，并且要求与 Rust 侧定义的 `ShmMetaRegion`、`ImageTripleBuffer`、`PoseTripleBuffer`、`GimbalTripleBuffer` 完全一致。

其中最关键的协议规则是：

- `state` 的最高位 `0x80` 表示“有新数据”
- `state & 0x03` 表示当前 ready 的 slot index
- 生产者写完数据后需要更新 `state`，并轮换 `write_idx`
- 消费者必须在读取后更新 `read_idx`，否则下一帧会卡住

---

## 2. 图像 + Pose 接收器

目录：`talos_ipc_image_pose_receiver/`

### 功能

这个程序用来验证 simulator 是否正常输出：

- 图像数据
- pose 数据（gimbal / odom / muzzle / camera 等通道）

程序会：

1. 打开共享内存 meta 区
2. 映射 image pool
3. 读取 `ImageTripleBuffer`
4. 先消费图像，再消费所有 pose 通道
5. 使用 OpenCV 显示图像
6. 打印 `frame_seq`、时间戳和 pose 状态信息

### 关键点

这个程序最值得注意的是：

- 图像和 pose 是同一套同步协议
- 如果只消费了图像，却没有消费 pose，那么 simulator 可能会卡住，表现为“只显示第一帧”
- 正确顺序应该是：
  - 先 `consume_image_channel()`
  - 再循环调用 `consume_pose_channel()` 对 pose 0..4 全部消费

### 编译

在该目录中执行：

```bash
cmake -S . -B build
cmake --build build
```

### 运行

先确保 simulator 已经启动并在正常输出 Talos IPC 数据，然后运行：

```bash
./build/talos_ipc_image_pose_receiver
```

### 预期现象

- 窗口中出现图像
- 控制台输出 frame 序号、时间戳、FPS
- 如果 pose 同步正确，程序不会长期卡在第一帧

### 常见问题

#### 1）只显示第一帧

通常说明：

- 没有消费 pose
- 或者状态位更新不完整

解决方法：

- 确认 `consume_pose_channel()` 对所有 pose 0..4 都执行了
- 确认 `state` 里 `FLAG_NEW` 被正确清除和恢复

#### 2）窗口为空

检查：

- simulator 是否真的在发图
- `/tmp/talos_ipc_meta` 是否存在
- `/tmp/talos_ipc_image_pool` 是否存在
- `magic` 和 `version` 是否匹配

---

## 3. 云台命令发送器

目录：`talos_ipc_gimbal_cmd_sender/`

### 功能

这个程序主要用于测试云台命令接口是否有效。它会持续写入 `gimbal_cmd` 共享内存，并让 `yaw_deg` 不断增长，模拟云台不断按顺时针方向旋转。

它的目标是：

- 验证 simulator 是否能从共享内存中读取外部云台命令
- 验证 `gimbal_cmd` triple buffer 是否按协议正确工作
- 验证是否需要开启 auto-aim / 订阅功能

### 协议要求

生产者必须：

1. 选择当前 `write_idx`
2. 写入 `GimbalCmd` 到 `slots[write_idx]`
3. 更新 `timestamp_ns`
4. 设置 `yaw_deg` / `pitch_deg` / `distance_m`
5. 执行 `state.swap(write_idx | FLAG_NEW)`
6. 更新 `write_idx = old & INDEX_MASK`

### 编译

在该目录执行：

```bash
cmake -S . -B build
cmake --build build
```

### 运行

1. 启动 simulator
2. 按 `F5` 打开 auto-aim / subscription
3. 然后运行：

```bash
./build/talos_ipc_gimbal_cmd_sender
```

### 预期现象

- 程序控制台持续输出当前 yaw、pitch、distance
- `state` 会像 `0x80 | idx` 这样的形式更新
- 如果 simulator 正确订阅了命令，云台应该持续转动

### 常见问题

#### 1）模拟器云台不转

最常见原因有两个：

- 没按 `F5` 打开 auto-aim 订阅
- `gimbal_cmd.state` 没有按协议更新

在 simulator 侧，订阅开关定义在：

- `SubscribeAutoAim`
- 受 `KeyCode::F5` 控制

如果这个开关关闭，`process_subscription()` 不会执行，因此云台命令完全无效。

#### 2）命令写入了，但没有效果

检查：

- `state` 是否真的写成了 `slot | 0x80`
- `write_idx` 是否在轮换
- `yaw_deg` / `pitch_deg` 是否发生了变化
- `distance_m` 是否不是 `-1.0`

---

## 4. 一句话总结

这两个测试程序分别覆盖了两条最关键的 Talos IPC 路径：

- `talos_ipc_image_pose_receiver`：验证图像和 pose 的生产/消费是否正常
- `talos_ipc_gimbal_cmd_sender`：验证外部云台命令是否能正确进入 simulator

如果两者都工作正常，说明整个 Talos IPC 测试链路基本就通了。

---

## 5. 推荐测试顺序

建议按下面顺序使用：

1. 启动 simulator
2. 运行图像接收器，确认图像和 pose 正常输出
3. 按 `F5` 打开 auto-aim
4. 运行云台命令发送器
5. 观察云台是否跟随 `yaw_deg` 旋转

---

## 6. 注意

这些测试程序依赖于共享内存的确切布局，因此必须保持与 simulator 端完全一致，否则会出现：

- 读到错误数据
- 只显示第一帧
- 云台命令没有响应
- 共享内存状态异常

如果你在调试过程中发现数据不一致，优先检查：

- `struct` 对齐
- `state` 更新顺序
- `read_idx` / `write_idx` 轮换
- 是否按 `F5` 开启订阅

