# RoboMaster 制导镖 OpenMV 视觉 V1

基于深圳大学 RobotPilots 2026 OpenMV 视觉程序的 **H750 视觉端 V1 实现候选**。只负责 LAB 色块检测、单次限时任务、目标中心像素偏差及 H750 → F411 UART 状态上报；**不包含 F411 飞控、IMU、制导律或镖架制导**。

> 状态：开发候选，尚无 H750 实板、F411 台架或实飞通过证据。代码和初始参数不得视为已验证的比赛版本。

## 源码结构

    .
    ├── main.py                      # OpenMV 运行入口：相机/UART/状态循环
    ├── vision_v1.py                 # 纯逻辑：配置校验/状态机/ROI/协议编码
    ├── recorder.py                  # 仅开发用的可选 MJPEG 录像
    ├── config.json                  # 唯一可调运行参数文件
    ├── tests/test_vision_v1.py      # CPython 桌面单元测试
    └── .github/workflows/host-tests.yml

原始代码来源、许可和已知差异见下方“来源与许可”。

## V1 时序（一次上电仅执行一次）

    H750 启动 → 初始化相机和 UART
              → 发一次 FF 52 50 44 41 52 54 (FF + RPDART)
              → IDLE（state=0, valid=0，约 20Hz）
       收到 AB → WAIT（state=1, valid=0）
           T_delay 到达 → SEARCH（state=2，每张新图像发一帧）
       SEARCH 经过 2000ms → DONE（state=3, valid=0，约 20Hz）

- AB 是唯一的 V1 启动字节；深大原来的 11/22 模式选择与 55/FF 第二触发不再使用。
- WAIT 的计时从程序处理 AB 开始；SEARCH 的 2000ms **不包含** WAIT。
- SEARCH 每次成功处理新图像，只发送一份观测；没有新图像就不制造重复的有效数据。
- 丢失目标时立即上报 valid=0、dx=dy=0，恢复全图软件搜索 ROI。
- DONE 后仍周期发送状态，但忽略后续 AB；如需再执行一轮，请复位/重新上电。
- 初始化、sensor.skip_frames 和 sensor.snapshot 可能阻塞，20Hz 是非 SEARCH 状态的**目标频率**，需要在实际固件上测量。尤其是相机初始化期间不保证有状态上报。
- 录像文件的打开安排在 SEARCH 计时开始前；录像关闭完全不参与图像处理决策。

## UART V1 固定接口（无 CRC）

- H750 原始接口基线：UART3、115200、8N1；F411 端 UART 管脚、电平和实际线路需电控确认。
- F411 → H750：单字节 AB。
- H750 → F411：初始化就绪字节 FF 52 50 44 41 52 54，**不是**视觉状态帧。
- H750 → F411 视觉帧：固定 **8 字节**，双字节帧头 FF 01。

    Byte:   0   1    2      3      4     5     6     7
            FF  01  state  valid   dx_H  dx_L  dy_H  dy_L

state：0 IDLE、1 WAIT、2 SEARCH、3 DONE；valid：0/1，只有 SEARCH 允许 1。

dx、dy 是**大端有符号 int16**，有效时范围分别为 [-160,159] 和 [-120,119]。
采用 QVGA 320×240 的图像坐标系，目标框中心减去图像中心 (160,120)，右方 dx 正、下方 dy 正。

- 有效示例：FF 01 02 01 00 0F FF F6 → SEARCH，dx=15、dy=-10。
- 无效示例：FF 01 02 00 00 00 00 00 → SEARCH 无目标。
- 完成示例：FF 01 03 00 00 00 00 00 → DONE。
- F411 负责连续两张**不同新图像**的有效观测确认、状态约束及通信超时。无 CRC/帧序号无法检测所有合法范围内的误码，也无法单靠 UART 证明两帧来自不同图像。

协议 PC 侧验证器见 [rm_comm_validator](https://github.com/darkttsave/rm_comm_validator) 的 protocols/dart_openmv_v1.yaml。该工具现有 Live 接收入口**不会自动发送 AB**。

## 配置与开发录像

运行参数读取自同目录的 **config.json**；是开机/启动时读取一次的配置，**修改 JSON 后需重启脚本**，不是热切换。不需要 YAML 解析库。JSON 格式错误或核心字段不合法时，脚本直接报错，避免悄悄使用错误参数。

示例：将录像开关从默认关闭改为开启：

    "recording": {
      "enabled": true,
      "every_n_frames": 5,
      "basename": "record"
    }

- 录像仅为开发调试工具，默认关闭；只录制 SEARCH 段，结束即关闭文件。使用原程序的 mjpeg.Mjpeg，并按原命名习惯保存为 record01.avi、record02.avi 等（文件容器/播放兼容性需实测）。
- 每 5 张处理过的 SEARCH 图像保存一张，**不是固定 fps 的录像**；视频写入耗时可能明显降低识别帧率。
- 不再依赖原程序额外的串口时间戳重命名或 CSV 数据接收，避免阻塞新的 V1 状态机。
- 开启录像应确保存储介质可写、容量足够。录像打开/写入/关闭异常会打印日志并停用本轮录像，不主动停止视觉任务；但实际存储耗时/异常仍需在硬件上验证。
- 实验时建议先关闭录像测基准，再开启录像对照串口观测频率及丢帧情况。

其他主要配置：

| 配置项 | 当前默认值 | 说明 |
|---|---|---|
| timing.delay_ms | 1000 | **仅为台架临时值**，需根据相机唤醒和曝光稳定时间验证 |
| timing.search_ms | 2000 | 进入 SEARCH 后的限时窗口 |
| timing.status_interval_ms | 50 | 非 SEARCH 状态的上报间隔目标 |
| camera.readout_window | [0,700,640,480] | 原深大经验 READOUT，首版固定；视场/兼容性待实测 |
| camera.warmup_ms | 1500 | 原脚本 skip_frames 参数 |
| camera.sleep_until_trigger | true | 保留原脚本唤醒前休眠逻辑 |
| camera.exposure_us | 800 | 原脚本联调值，需重测 |
| camera.gain_db | 20 | 原脚本值，需重测 |
| vision.lab_threshold | [9,100,-102,-25,0,127] | 深大原始 LAB 阈值，需重测 |
| vision.roi_margin_x/y | 40/40 | 动态软件 ROI 扩展量；已补边界裁剪 |
| vision.draw_overlay | false | 调试绘图，会增加处理负载 |
| recording.enabled | false | 开发录像默认关闭 |
| recording.every_n_frames | 5 | 每 N 张处理图像录一张 |

注意：固定的传感器 READOUT 窗口与每张图像的**软件识别 ROI**不是一回事。V1 不继承深大的第二阶段 READOUT 切换。

## 部署（仍需 OpenMV 实板验证）

1. 确认目标板实际运行兼容的 OpenMV MicroPython 固件，并确认 OV5640、mjpeg、传感器 READOUT API 支持情况。
2. 把 main.py、vision_v1.py、recorder.py、config.json 放到 OpenMV 可读取的同一工作目录；文件名 main.py 用作入口。
3. **先不连接控制执行机构**，用 OpenMV IDE/串口台架检查相机初始化和 ready banner；确认 UART3 TX/RX、共地和电气电平。接线必须经电控确认。
4. PC 侧使用串口工具向 H750 发送单字节 0xAB；同一串口不允许两个进程并发抢占。配合 rm_comm_validator 解析来自 H750 的状态帧。
5. 记录 raw HEX、各状态时间戳、SEARCH 新图像帧率、录像开关前后性能、误检/丢目标行为；通过后再与 F411 联调。

### 桌面静态/逻辑测试

要求 Python 3（不需要安装 OpenMV 模块）：

    python -m json.tool config.json
    python -m compileall -q main.py vision_v1.py recorder.py tests
    python -m unittest discover -s tests -v

测试涉及配置、坐标/字节序、协议约束、状态机及 ROI。**不能证明 OpenMV 的 sensor、UART 或 mjpeg API 与实物硬件兼容**，也不能取代时序与 F411 联调。

## 来源与许可

- 识别、相机初始化及录像思路改编自 **深圳大学 RobotPilots 战队 2026 制导飞镖 OpenMV 原始 main_final.py**：[RoboMaster 社区发布帖](https://bbs.robomaster.com/article/1938958)。
- 上述原帖标注 **CC BY-NC-SA 4.0**（[许可条款](https://creativecommons.org/licenses/by-nc-sa/4.0/)）；本仓库中源自该脚本的改编部分按照该署名-非商业-相同方式共享条件使用。原始文件没有单独许可文件，原始 ZIP 与用户本地脚本的逐字节一致性尚未核验；请勿误认为深圳大学对本队代码进行了背书。
- 相对于深大原脚本的主要变更：去除双重触发和运行时 READOUT 切换；新增四状态机、独立 valid 标志、8 字节偏差帧、合法 ROI 裁剪以及 JSON 配置；录像保留为可选开发工具，不再使用原始录像文件时间戳/CSV 串口后处理。
- 工程决策与后续实测记录见 [rm_TuningManual 飞镖 V1](https://github.com/darkttsave/rm_TuningManual/blob/main/05_%E9%A1%B9%E7%9B%AE%E5%BC%80%E5%8F%91/04_%E9%A3%9E%E9%95%96/01_%E5%88%B6%E5%AF%BC%E9%95%96/2026-11-02_OpenMV%E8%A7%86%E8%A7%89V1%E6%96%B9%E6%A1%88%E4%B8%8E%E6%8E%A5%E5%8F%A3.md)。

本仓库当前仅含视觉 V1 候选及桌面检查，**不包含任何已完成的真实制导或飞行验证结果**。
