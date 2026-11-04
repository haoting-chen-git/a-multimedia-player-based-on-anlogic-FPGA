`timescale 1ns / 1ps
//======================================================================
//  video_fade.v —— 图片切换「淡入」转场特效
//----------------------------------------------------------------------
//  作品简介承诺的加分项之一：图片轮播与转场特效
//
//  插入位置（与 osd_overlay.v 串联在同一个点，见 20_osd\README.md）：
//
//      vout_data_raw --> video_fade --> osd_overlay --> video_rgb_to_axis
//                                                                  ^
//                                                       .I_rgb(vout_data)
//
//  原理：
//    官方参考设计里换图 = sd_card_bmp 输出 disp_buf_idx 变化。
//    本模块【只观察不干预】——检测到 I_buf_idx 变化就令 alpha=0，
//    然后每帧把 alpha 加上 ALPHA_STEP，直到 255。
//    alpha 参与像素混合：alpha=0 全黑，alpha=255 原图。
//    效果就是"新图片从黑场淡入"。
//
//  优点：不动官方任何乒乓/地址逻辑，零额外 BRAM / FIFO，
//        像素通路纯组合，接在 25MHz 视频通路上时序极其宽松。
//
//  资源：3 个 8x8 乘法器（TD 会推断到 DSP），状态寄存器 ~20 个。
//======================================================================
module video_fade
#(
    parameter STYLE        = 0,        // 0 = 从黑场淡入；1 = 从白场闪入
    parameter ALPHA_STEP   = 8'd16,    // 每帧 alpha 增量：255/16 ≈ 16 帧 ≈ 267ms @60Hz
    parameter BLANK_FRAMES = 8'd2      // 换图后全黑保持的帧数
)
(
    input  wire        I_clk,          // video_clk (25MHz)
    input  wire        I_rst,          // 高有效异步复位（rst_all）
    input  wire        I_vs,           // 场同步，用于产生帧脉冲
    input  wire        I_en,           // 转场总开关：0 = 完全透传（自检图案模式用）
    input  wire [1:0]  I_buf_idx,      // 当前显示的 SDRAM 缓冲区编号（变化 = 换图）
    input  wire [23:0] I_rgb,          // 原始像素 {R,G,B}
    output wire [23:0] O_rgb           // 混合后像素
);

    //------------------------------------------------------------------
    // 1. 场同步上升沿 -> 帧脉冲
    //------------------------------------------------------------------
    reg s_vs_d;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) s_vs_d <= 1'b0;
        else       s_vs_d <= I_vs;
    end
    wire frame_tick = I_vs & (~s_vs_d);

    //------------------------------------------------------------------
    // 1b. I_buf_idx 跨时钟域同步（★ 必须）
    //     disp_buf_idx 是 sd_card_bmp 在 sd_card_clk(100MHz) 域产生的，
    //     本模块在 video_clk(25MHz) 域。直接采样 = 跨时钟域违例
    //     （TD 实测会报 80 条路径 WNS -6.565ns）。
    //     这里加两级同步器消除亚稳态。
    //------------------------------------------------------------------
    reg [1:0] s_buf_sync0;
    reg [1:0] s_buf_sync1;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            s_buf_sync0 <= 2'd0;
            s_buf_sync1 <= 2'd0;
        end
        else begin
            s_buf_sync0 <= I_buf_idx;
            s_buf_sync1 <= s_buf_sync0;
        end
    end
    wire [1:0] buf_idx_v = s_buf_sync1;    // 同步后的缓冲区编号

    //------------------------------------------------------------------
    // 2. 逐帧采样缓冲区编号，检测换图
    //    只在 frame_tick 更新，所以比较结果在 frame_tick 那一拍有效
    //------------------------------------------------------------------
    reg [1:0] s_buf_frame;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)                        s_buf_frame <= 2'd0;
        else if (frame_tick && I_en)      s_buf_frame <= buf_idx_v;
    end

    //------------------------------------------------------------------
    // 3. alpha 渐变状态机（帧域）
    //------------------------------------------------------------------
    reg [7:0] s_alpha;      // 0 = 全黑/全白，255 = 原图
    reg [7:0] s_blank;      // 全黑保持计数

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            s_alpha <= 8'd255;
            s_blank <= 8'd0;
        end
        else if (frame_tick) begin
            if (!I_en) begin
                // ---- 自检图案模式（display_valid=0）：不转场，alpha 拉满 = 完全透传 ----
                s_alpha <= 8'd255;
                s_blank <= 8'd0;
            end
            else if (buf_idx_v != s_buf_frame) begin
                // ---- 检测到换图：立刻归零，重新淡入 ----
                s_alpha <= 8'd0;
                s_blank <= BLANK_FRAMES;
            end
            else if (s_blank != 8'd0) begin
                // ---- 全黑保持阶段 ----
                s_blank <= s_blank - 8'd1;
                s_alpha <= 8'd0;
            end
            else if (s_alpha != 8'd255) begin
                // ---- 淡入阶段（饱和到 255，不会回绕）----
                if (s_alpha > (8'd255 - ALPHA_STEP))
                    s_alpha <= 8'd255;
                else
                    s_alpha <= s_alpha + ALPHA_STEP;
            end
        end
    end

    //------------------------------------------------------------------
    // 4. 像素混合通路（纯组合，零延迟）
    //    黑场版：O = (rgb * alpha + 255) >> 8
    //        alpha=255 -> (rgb*255+255)>>8 = rgb      (精确复原)
    //        alpha=0   -> 255>>8            = 0        (全黑)
    //    白场版：O = (rgb*alpha + 255*(255-alpha) + 255) >> 8
    //        alpha=0   -> (65025+255)>>8    = 255      (全白)
    //------------------------------------------------------------------
    generate
    if (STYLE == 0) begin : g_fade_black
        wire [15:0] r_mul = I_rgb[23:16] * s_alpha;
        wire [15:0] g_mul = I_rgb[15: 8] * s_alpha;
        wire [15:0] b_mul = I_rgb[ 7: 0] * s_alpha;

        wire [16:0] r_sum = {1'b0, r_mul} + 17'd255;
        wire [16:0] g_sum = {1'b0, g_mul} + 17'd255;
        wire [16:0] b_sum = {1'b0, b_mul} + 17'd255;

        assign O_rgb = { r_sum[15:8], g_sum[15:8], b_sum[15:8] };
    end
    else begin : g_fade_white
        wire [ 7:0] inv_a = 8'd255 - s_alpha;

        wire [15:0] r_mul = I_rgb[23:16] * s_alpha;
        wire [15:0] g_mul = I_rgb[15: 8] * s_alpha;
        wire [15:0] b_mul = I_rgb[ 7: 0] * s_alpha;

        wire [15:0] w_mul = 8'd255 * inv_a;      // 255*255 = 65025，16 位够

        wire [16:0] r_sum = {1'b0, r_mul} + {1'b0, w_mul} + 17'd255;
        wire [16:0] g_sum = {1'b0, g_mul} + {1'b0, w_mul} + 17'd255;
        wire [16:0] b_sum = {1'b0, b_mul} + {1'b0, w_mul} + 17'd255;

        assign O_rgb = { r_sum[15:8], g_sum[15:8], b_sum[15:8] };
    end
    endgenerate

endmodule
