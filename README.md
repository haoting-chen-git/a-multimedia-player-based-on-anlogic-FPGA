# TF 卡 BMP 图片播放器（FPGA / 安路 EG4S20）

基于安路（Anlogic）EG4S20BG256 的 HDMI 图片播放器：上电自动扫描 TF 卡、把最多 4 张 24 位 BMP 装进 SDRAM、按可调周期轮播，输出 640×480 HDMI，图片按比例自适应缩放（不足部分补黑边），并叠加三行 OSD 字幕。

---

## 一、功能清单

| 编号 | 功能 | 实现情况 |
|---|---|---|
| 1 | 从 TF 卡读取并显示 BMP 图片 | ✅ FAT32 目录扫描 + 逐簇跳扫，最多 4 张 |
| 2 | 图片自适应缩放 | ✅ 最近邻 DDA 缩放 + letterbox 黑边，任意长宽比不变形 |
| 3 | 自动轮播 + 手动切图 | ✅ 4 张图预载进 SDRAM 4 个槽，切图瞬时完成 |
| 4 | 轮播周期可调 | ✅ 1 / 2 / 4 / 8 秒，OSD 实时显示 |
| 5 | OSD 字幕叠加 | ✅ 3 行中文/数字混合字幕，字库 ROM 生成 |
| 6 | 按键控制 | ✅ K1 复位、K2 自动/手动、K3 下一张、K4 周期（见 `按键功能说明.md`） |
| 7 | 数码管自检 | ✅ 8 位十六进制诊断字，反映卡状态/装载情况 |
| 8 | 音频（I2S） | ✅ 接口与代码框架保留，当前版本 `AUDIO_ENABLE=1` |

---

## 二、目录结构

```
40_player_release/
├─ README.md                 ← 本文件
├─ 按键功能说明.md            ← K1~K4 功能、OSD、诊断字、图片要求
├─ build.bat                 ← 命令行一键编译（syn + P&R + bitgen）
├─ bit/
│   └─ player_v34_release.bit ← 可直接烧录的比特流
└─ src/
    ├─ td_project/
    │   ├─ lab_ex5_i2s_v1.0.al      ← TD 工程文件（用 TD 打开这个）
    │   ├─ al_ip/PLL.ipc            ← IP 配置
    │   └─ lab_ex5_i2s_v1.0_Runs/   ← 编译输出目录（bit 在这里生成）
    └─ user_source/
        ├─ hdl_source/              ← 全部 Verilog 源码
        │   ├─ top_tf_hdmi_audio.v  ← 顶层
        │   ├─ SD/                  ← TF 卡（SPI）、FAT 解析、BMP 读取、帧缓存
        │   ├─ bmp_scaler.v         ← 自适应缩放核心
        │   ├─ diag_overlay.v       ← 数码管诊断字
        │   ├─ osd_overlay.v 等     ← OSD 字幕
        │   ├─ hdmi1.4b_transmitter_core/  ← HDMI 发送器（官方 IP + 封装）
        │   └─ ip/                  ← 异步 FIFO 等 IP
        └─ constraints_source/
            ├─ pin.adc              ← 引脚约束
            └─ timing.sdc           ← 时序约束
```

---

## 三、硬件平台

| 项目 | 参数 |
|---|---|
| FPGA | 安路 EG4S20BG256（HX4S20C 核心板） |
| 系统时钟 | 50 MHz（R7） |
| 内部时钟 | 视频 25 MHz、SD 控制 100 MHz、HDMI 5× 时钟（PLL 生成） |
| 显示输出 | HDMI（TMDS，640×480@60Hz） |
| 存储 | 片外 SDRAM，16 M×32 bit，用作 4 槽帧缓存（每槽 307200 字） |
| 存储卡 | TF 卡，SPI 模式（初始化后 25 MHz） |
| 按键 | K1 复位（A2）、K2（B2）、K3（C1）、K4（B1） |
| 数码管 | 8 位动态扫描（段 C15/C16/B6/A5/B5/A8/A7/A6，位选 E12/E10/C13/F10） |

---

## 四、如何编译

### 方式 A：TD 图形界面（推荐评委用）

1. 打开 TD：`D:\TD\bin\td.exe`
2. `File → Open Project`，选择 `src\td_project\lab_ex5_i2s_v1.0.al`
3. 工具栏 **Syn Opt**（综合优化）→ 完成后点 **Run**（布局布线 + 生成比特流）
4. 成功后比特流位于：
   `src\td_project\lab_ex5_i2s_v1.0_Runs\phy_1\lab_ex5_i2s_v1.0.bit`
   （TD 还会在 `Runs\best_result\lab_ex5_i2s_v1.0.bit` 放一份同样的副本。

   **注意**：TD 不会把 bit 放到工程的 `bit\` 目录里 —— 那个目录里的是手工归档的 `player_v34_release.bit`。要看有没有真的编译出来，去上面 `phy_1\` 看时间戳。）

### 方式 B：命令行一键编译

**先关闭 TD 图形界面**，然后双击 `build.bat`。
脚本会先综合、再布局布线，并用「`gate.db` 时间戳是否变化 + 综合日志是否含 ERROR」双重校验，避免该工具链"报错也返回成功"的陷阱。

编译全程约 4 分钟，输出：

```
src\td_project\lab_ex5_i2s_v1.0_Runs\phy_1\lab_ex5_i2s_v1.0.bit
```

脚本开头的三步自愈（重要）：TD 图形界面关闭时会用它内存里的旧文件列表覆盖工程文件，可能把 `bmp_scaler.v` 等 6 个模块丢掉、甚至删除 `Runs\phy_1\settings.cfg`。因此 `build.bat` 每次会先把下列备份拷回去再编译：

| 备份文件 | 用途 |
|---|---|
| `src\td_project\lab_ex5_i2s_v1.0.al.good` | 工程文件列表（54 项） |
| `Runs\syn_1\lab_ex5_i2s_v1.0.prj.good` | 综合文件列表 |
| `Runs\syn_1\settings.cfg.good` | 综合配置 |
| `Runs\phy_1\lab_ex5_i2s_v1.0.prj.good` | 布局布线文件列表 |
| `Runs\phy_1\settings.cfg.good` | 布局布线配置 |

---

## 五、如何烧录

见 `按键功能说明.md` 与本目录 `bit/`。要点：

1. **驱动签名**：板载 USB-JTAG 是 Cypress 芯片，其驱动无微软交叉签名，Win10/11 需先"禁用驱动程序强制签名"（开机启动设置里按 7），否则 TD 识别不到下载器（设备错误码 52）。
2. TD 中 **Tools → BitWriter** → Add 选 `bit/player_v34_release.bit`
3. Mode = **PROGRAM_SRAM**（掉电丢失）/ **PROGRAM_FLASH**（固化），JTAG，5 MHz，Server = AL-LINK
4. Run，进度 100% 即成功。

---

## 六、设计说明（技术要点）

### 1. 数据通路

```
TF 卡 ──SPI──> sd_card_top ──扇区──> bmp_read ──像素流──> bmp_scaler ──> 异步FIFO ──> SDRAM(4槽)
                  (100MHz)          (FAT/BMP解析)      (DDA缩放+黑边)                 │
                                                                                      │
   HDMI TMDS <── hdmi_phy <── video_timing_data <── OSD 叠加 <── video_fade <── SDRAM 读
```

### 2. 自适应缩放（`bmp_scaler.v`）

- **最近邻插值 + DDA 步进**：横向 `hstep = floor(W×65536/DW)`、余数用阈值比较 + 并行加法实现进位，避开串行 `add→cmp→sub` 的长组合链（否则 `sd_card_clk` 会违例）。
- **letterbox**：先算缩放后尺寸 `sw = min(SW, DW×SH/DH)`、`sh = min(SH, DH×SW/DW)`，居中放置，其余行列输出黑像素。
- **行缓存**：两片 BSRAM 乒乓，写侧（卡）与读侧（显示像素输出）跨时钟域，靠 `full` 标志与"跟住写侧"的兜底逻辑避免死锁（这是 v31 解决的关键 bug）。
- **输出节流**：每 4 个时钟输出 1 个像素，避免 SDRAM 忙时把前置 512 深 FIFO 冲爆导致整行像素被丢弃（v27 解决"满屏细白线"的根因）。

### 3. 四槽帧缓存与瞬时切图

- SDRAM 划分为 4 个槽，每槽 307200 字（640×480×32bit 中的 24bit 有效）。
- 上电扫描完成后，后台**逐张预载**；装好第一张就立即上屏，其余继续装。
- 切图时只切换读地址索引（`disp_buf_idx`），**不再访问 TF 卡**，因此是瞬时的。
- 每个槽有独立完成标志 `loaded_mask`；装载失败只累计失败次数并重试，**不会**把失败槽误标为"已装好"（v34 修复的致命死锁）。

### 4. TF 卡读取

- SPI 模式：CMD0 → CMD8 → CMD55/CMD41 → CMD16 → CMD17 读单块。
- 解析 FAT32 引导扇区得到 `bytes_per_sector / sectors_per_cluster / reserved / FATSz32`，算出数据区首扇区 `data0`，按**簇步长**跳扫（避免逐扇区全盘扫描）。
- 每个扇区带 2 ms 超时，超时自动跳过；连续 64 次超时则结束扫描。
- 头部校验：`'B','M'`、宽度 4 的倍数、24 位、无压缩、尺寸不超过上限。采样点选在 `rd_cnt == 34`（`compression` 字段最高字节晚一拍到达的 off-by-one 修复）。

### 5. OSD 字幕

- 字库由 `20_osd/make_osd_assets.py` 离线生成（Python + PIL，CJK 字体），输出 `osd_font_rom.v` 位图与 `osd_text_rom.v` 文本索引。
- 叠加器两级流水，零延迟；字幕条对底图做 50% 压暗，保证可读性。

### 6. 跨时钟域

- 卡域（100 MHz）、视频域（25 MHz）、系统域（50 MHz）、SDRAM 域互为异步。
- 所有跨域标量用两级同步器；帧完成信号用 toggle + 边沿检测；像素数据走异步 FIFO。

---

## 七、已知限制

1. **上电扫卡偏慢（约数秒）**：个别卡在首次读引导扇区时 FAT 参数解析失败，会退化成逐扇区扫描。只影响上电速度，不影响播放与切图。
2. 只支持 **24 位不压缩 BMP**；PNG/JPEG/8 位调色板 BMP 不支持。
3. 最多 **4 张**图参与轮播；扫描窗口为卡前 16 MB。
4. 音频通路（I2S）代码保留但默认关闭。
5. 跨时钟域异步路径仍有负 slack（官方 SDRAM IP 的移相时钟等），实测运行正常。

### 常见问题：TD 打开工程报 *black box*

如果 TD 的 Console 里出现

```
HDL-8007 ERROR: bmp_scaler(...) is a black box
HDL-7144 CRITICAL-WARNING: instantiate unknown module bmp_scaler
```

（`vsrc_sd` / `period_sel_key` / `demo_pattern_ctrl` / `diag_overlay` / `audio_vu_meter` 同理）

说明 **工程文件列表里漏了这 6 个 `.v`**，综合时它们被当成空模块，生成的 `.bit` 是坏的（但流程可能仍然报 DONE，必须看日志里的 `HDL-8007`）。
本工程已经补齐（`lab_ex5_i2s_v1.0.al` / `Runs/syn_1/*.prj` 均为 54 项、综合 46 个源文件）。若再次出现，按下表把这 6 个文件加入工程：

| 文件 | 路径（相对 `src/td_project`） |
|---|---|
| `bmp_scaler.v` | `../user_source/hdl_source/bmp_scaler.v` |
| `demo_pattern_ctrl.v` | `../user_source/hdl_source/demo_pattern_ctrl.v` |
| `period_sel_key.v` | `../user_source/hdl_source/period_sel_key.v` |
| `diag_overlay.v` | `../user_source/hdl_source/diag_overlay.v` |
| `audio_vu_meter.v` | `../user_source/hdl_source/audio_vu_meter.v` |
| `vsrc_sd.v` | `../user_source/hdl_source/SD/vsrc_sd.v` |

判断编译是否真的成功：综合日志里应出现 `PRJ-1401 : Successfully analyzed 46 source files.`，且 **没有** `HDL-8007`。

---

## 八、版本记录（关键节点）

| 版本 | 解决的问题 |
|---|---|
| v27 | 满屏细白线：SDRAM 写 FIFO 溢出丢像素 → 输出节流 |
| v29 | 只认 1 张图：加载超时误清扫描结果 → 逐簇跳扫 + 断点续扫 |
| v30 | 一张都不认：`compression` 字段采样早一拍 → `rd_cnt==34` |
| v31 | 只有第一张能装：行缓存乒乓被写侧冲掉 → 兜底逻辑 + `i_arm` |
| v32 | 轮播周期被加载耗时污染 → 计时器跨换图持续计数 |
| v33 | 4 槽 SDRAM 帧缓存 + 后台预载 + 瞬时切图 |
| v34 | 修掉 v33 的 abort 误标导致的上板死锁；补回自动/手动；按键重排为 K1~K4 |
| v34-release | 按键按 K1 复位 / K2 自动手动 / K3 下一张 / K4 周期 定稿；补齐工程文件列表（6 个模块曾是 black box，现综合 46 个源文件、0 个 HDL-8007）；`bit/player_v34_release.bit` 为 2026-11-01 16:33 编译输出，629,624 B，MD5 `bf971ee2e5bd477ccccaa0d8d88e3515` |
