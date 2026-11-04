`timescale 1ns / 1ps
//======================================================================
//  audio_melody_player.v —— 图片联动旋律播放器（直接产生 48kHz PCM）
//----------------------------------------------------------------------
//  【为什么这么做】《2026选题指南》硬性规定：
//      "提交作品中的整体实现架构、音频和图片处理方法不能和参考实例相同"
//  官方 lab_ex5_i2s 的做法：
//      内部方波音阶发生器 --> I2S(BCLK/LRCK/DOUT) --> I2S_receiver 解回 PCM
//      --> audio_arc_calculate --> HDMI TX
//  本模块的做法（刻意不同）：
//      每张图片对应一首旋律，正弦查表 + 起落包络，**直接**产生 PCM，
//      不经过 I2S 回环，也不需要 12.288MHz 音频主时钟。
//      → 音频处理方法与参考实例不同，且实现"音视频联动"（扩展要求 1.4(2)）
//
//  【接口说明】输出直接对接 hdmi_1_4b_transmitter_core_wrapper：
//      .I_audio_valid / .I_audio_left_data / .I_audio_right_data
//      O_audio_valid 为一个 video_clk 周期宽的采样有效脉冲（48kHz）
//      O_audio_*_data 在该脉冲当拍有效（24 位有符号 PCM，满幅约 ±2.09e6）
//
//  【资源】1 个 16x9 乘法器（TD 会推断到 DSP）+ 约 130 个寄存器 + 查表逻辑
//======================================================================
module audio_melody_player
#(
    parameter integer CLK_FREQ_HZ    = 25_000_000, // video_clk 实际频率
    parameter integer SAMPLE_RATE_HZ = 48_000,     // 目标采样率
    parameter integer DUR_SHIFT      = 11,         // 时值单位 = 2^11 = 2048 采样
    parameter integer RAMP_SAMPLES   = 32          // 每个音的起/落包络斜坡长度
)
(
    input  wire        I_clk,          // video_clk (25MHz)
    input  wire        I_rst,          // 高有效异步复位
    input  wire [1:0]  I_track,        // 曲目选择：接 disp_buf_idx 即为"图变曲变"
    output reg         O_audio_valid,  // 48kHz 采样有效（1 拍宽）
    output reg  [23:0] O_audio_left_data,
    output reg  [23:0] O_audio_right_data,

    // ---- 音频可视化接口（给 audio_vu_meter 用）----------------------
    //  这两个信号让"声音"变成画面上看得见的东西（扩展要求 1.4(2) 音视频联动）。
    //  即使 AUDIO_ENABLE=0 把声音静音，旋律仍在内部跑，所以静音时也能看到跳动。
    output reg  [4:0]  O_note,         // 当前音高编号 0..23（C4..B5）
    output reg  [5:0]  O_level         // 峰值保持后的电平 0..63，带约 170ms 衰减
);

    //------------------------------------------------------------------
    // 1. 48kHz 采样使能：分数分频（25MHz / 48kHz = 520.833…，非整数）
    //    累加器每拍加 SAMPLE_RATE_HZ，溢出即产生一个采样点，
    //    平均频率精确等于 48kHz，抖动 <= 1 个像素时钟（40ns），
    //    下游由 HDMI 接收端的 ACR 机制重建音频时钟，完全可接受。
    //------------------------------------------------------------------
    reg  [31:0] S_acc;
    wire [31:0] W_acc_next = S_acc + SAMPLE_RATE_HZ;
    wire        W_tick     = (W_acc_next >= CLK_FREQ_HZ);

    //------------------------------------------------------------------
    // 2. 曲目状态
    //------------------------------------------------------------------
    reg  [1:0]  S_track_d;    // 上一拍的曲目号，用于检测换曲
    reg  [2:0]  S_step;       // 当前正在播放第几个音（0..7）
    reg  [31:0] S_left;       // 当前音剩余采样点数
    reg  [31:0] S_len;        // 当前音总采样点数
    reg  [31:0] S_phase;      // 相位累加器
    reg  [7:0]  S_env;        // 备用（本版包络为组合计算）

    // 当 S_left 归零时自动前进到下一个音；此处用组合方式算出"待播放步号"
    wire [2:0]  W_step_next = (S_left == 32'd0)
                            ? ((S_step == 3'd7) ? 3'd0 : (S_step + 3'd1))
                            : S_step;

    //------------------------------------------------------------------
    // 3. 查表：旋律 -> 音高 -> 相位增量 -> 正弦值
    //------------------------------------------------------------------
    wire [4:0]  W_note;
    wire [4:0]  W_dur;
    wire [31:0] W_inc;
    wire signed [15:0] W_sine;

    audio_melody_rom u_melody_rom
    (
        .I_track (I_track),
        .I_step  (W_step_next),
        .O_note  (W_note),
        .O_dur   (W_dur)
    );

    audio_note_inc u_note_inc
    (
        .I_note (W_note),
        .O_inc  (W_inc)
    );

    audio_sine_lut u_sine_lut
    (
        .I_phase (S_phase[31:24]),   // 取高 8 位作查表地址 = 256 点/周期
        .O_sine  (W_sine)
    );

    //------------------------------------------------------------------
    // 4. 起落包络：每个音的前 32 / 后 32 个采样做线性斜坡，
    //    避免换音时的波形突变造成"咔哒"爆音。
    //    包络值 0..255，用移位实现，不做除法。
    //------------------------------------------------------------------
    wire [31:0] W_elapsed = S_len - S_left;
    wire [7:0]  W_env_up  = (W_elapsed > (RAMP_SAMPLES-1)) ? 8'd255 : {W_elapsed[4:0], 3'b000};
    wire [7:0]  W_env_dn  = (S_left    > (RAMP_SAMPLES-1)) ? 8'd255 : {S_left[4:0],    3'b000};
    wire [7:0]  W_env     = (W_env_up < W_env_dn) ? W_env_up : W_env_dn;

    //------------------------------------------------------------------
    // 5. 幅度缩放：sine(±32767) * env(0..255) >> 2  => 满幅约 ±2.09e6
    //    24 位有符号 PCM，与 HDMI 核期望的格式一致
    //------------------------------------------------------------------
    wire signed [24:0] W_mul = W_sine * $signed({1'b0, W_env});
    wire signed [23:0] W_pcm = W_mul >>> 2;

    //------------------------------------------------------------------
    // 5b. 音频可视化用的电平：取 |PCM| 的高位（满幅 ≈ +-2.09e6，>>15 得到 0..63）
    //     再做峰值保持 + 慢衰减，让柱状图看起来像真的电平表而不是硬跳变。
    //------------------------------------------------------------------
    wire [23:0] W_abs = W_pcm[23] ? (~W_pcm + 24'd1) : W_pcm;
    wire [5:0]  W_lvl = W_abs[20:15];

    reg  [6:0]  S_dec_cnt;              // 0..127，控制衰减节奏
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_dec_cnt <= 7'd0;
        end
        else if (W_tick) begin
            S_dec_cnt <= (S_dec_cnt == 7'd127) ? 7'd0 : (S_dec_cnt + 7'd1);
        end
    end

    //------------------------------------------------------------------
    // 6. 主时序
    //------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_acc              <= 32'd0;
            S_track_d          <= 2'd0;
            S_step             <= 3'd7;     // 初值 7：第一次装载自动进到第 0 音
            S_left             <= 32'd0;    // 0 = 立即装载第一个音
            S_len              <= 32'd0;
            S_phase            <= 32'd0;
            S_env              <= 8'd0;
            O_audio_valid      <= 1'b0;
            O_audio_left_data  <= 24'sd0;
            O_audio_right_data <= 24'sd0;
            O_note             <= 5'd0;
            O_level            <= 6'd0;
        end
        else begin
            // 采样有效脉冲默认拉低，只在 W_tick 当拍拉高一拍
            O_audio_valid      <= 1'b0;

            // ★ 分数分频累加器：每拍都必须累加，只有溢出那一拍才回卷。
            //   （曾经的 bug：这行只写在 if(W_tick) 里 -> S_acc 永远为 0 -> 永不产生采样）
            S_acc <= W_tick ? (W_acc_next - CLK_FREQ_HZ) : W_acc_next;

            if (W_tick) begin
                O_audio_valid      <= 1'b1;
                O_audio_left_data  <= W_pcm;
                O_audio_right_data <= W_pcm;     // 左右声道相同（单声道素材）

                // 音频可视化：当前音高 + 峰值保持电平
                O_note <= W_note;
                if (W_lvl >= O_level)
                    O_level <= W_lvl;                       // 立刻跟上峰值
                else if (S_dec_cnt == 7'd127)
                    O_level <= (O_level == 6'd0) ? 6'd0 : (O_level - 6'd1);

                // 换曲目：立刻从头播放，实现"切图 = 换曲"
                if (I_track != S_track_d) begin
                    S_track_d <= I_track;
                    S_step    <= 3'd7;
                    S_left    <= 32'd0;
                    S_phase   <= 32'd0;
                end
                else if (S_left == 32'd0) begin
                    // ---- 装载下一个音 ----
                    S_step  <= W_step_next;
                    S_len   <= {{27{1'b0}}, W_dur} << DUR_SHIFT;
                    S_left  <= {{27{1'b0}}, W_dur} << DUR_SHIFT;
                    S_phase <= 32'd0;            // 相位归零：正弦从 0 起，换音无爆音
                end
                else begin
                    // ---- 正常推进 ----
                    S_left  <= S_left  - 32'd1;
                    S_phase <= S_phase + W_inc;
                end
            end
        end
    end

endmodule
