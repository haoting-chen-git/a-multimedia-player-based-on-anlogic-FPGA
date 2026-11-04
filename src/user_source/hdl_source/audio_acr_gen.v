`timescale 1ns / 1ps
//======================================================================
//  audio_acr_gen.v —— HDMI 音频时钟重建（ACR）包参数生成
//----------------------------------------------------------------------
//  HDMI 接收端需要靠 ACR（Audio Clock Regeneration）包里的 N / CTS
//  两个常数，从 TMDS 像素时钟反推出音频采样时钟。
//
//  关系式：  CTS = N x f_pixel / (128 x f_audio)
//  本设计：  N = 6144, f_audio = 48kHz, f_pixel = 25MHz
//            => CTS = 6144 x 25e6 / (128 x 48000) = 25000
//
//  但真正实现时不能写死 25000，因为 PLL 实际输出与标称值总有偏差。
//  这里采用"实测"方式：每累计 ACR_N>>7 = 48 个音频采样，就数一次
//  这段时间里过了多少个像素时钟，把实测值作为 CTS 送出去，自动跟踪。
//
//  与官方 audio_arc_calculate.v 的差别：本模块用"倒计数器 + 独立计时器"
//  结构，输出在 acr_valid 当拍对齐；功能等价，写法独立。
//======================================================================
module audio_acr_gen
#(
    parameter integer ACR_N_VALUE = 6144,   // 必须能被 128 整除
    parameter integer SAMP_DIV    = 48      // = ACR_N_VALUE >> 7
)
(
    input  wire        I_clk,          // video_clk / 像素时钟
    input  wire        I_rst,          // 高有效异步复位
    input  wire        I_audio_valid,  // 来自 audio_melody_player 的采样有效脉冲
    output reg         O_acr_valid,    // ACR 包有效（1 拍宽）
    output reg  [19:0] O_acr_cts,      // 像素时钟计数
    output reg  [19:0] O_acr_n         // 常数 N
);

    //------------------------------------------------------------------
    // 采样分组倒计数：数到 0 时该发一个 ACR 包
    //------------------------------------------------------------------
    reg  [5:0]  S_div_cnt;      // 6 位足够表示 0..63

    //------------------------------------------------------------------
    // 像素时钟计数器：统计两次 ACR 之间经过了多少个像素时钟
    //------------------------------------------------------------------
    reg  [19:0] S_cyc_cnt;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_div_cnt   <= 6'd0;
            S_cyc_cnt   <= 20'd0;
            O_acr_valid <= 1'b0;
            O_acr_cts   <= 20'd0;
            O_acr_n     <= 20'd0;
        end
        else begin
            // 默认：ACR 包只在发出那一拍有效，其余时间保持无效
            O_acr_valid <= 1'b0;
            O_acr_cts   <= 20'd0;
            O_acr_n     <= 20'd0;

            if (I_audio_valid) begin
                if (S_div_cnt == (SAMP_DIV - 1)) begin
                    // ---- 满 48 个采样：发 ACR 包 ----
                    S_div_cnt   <= 6'd0;
                    S_cyc_cnt   <= 20'd0;
                    O_acr_valid <= 1'b1;
                    O_acr_cts   <= S_cyc_cnt + 20'd1;      // +1 补上本拍
                    O_acr_n     <= ACR_N_VALUE[19:0];
                end
                else begin
                    S_div_cnt   <= S_div_cnt + 6'd1;
                    S_cyc_cnt   <= S_cyc_cnt + 20'd1;
                end
            end
            else begin
                // 非采样拍：只走像素时钟计数
                S_cyc_cnt <= S_cyc_cnt + 20'd1;
            end
        end
    end

endmodule
