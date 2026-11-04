//=============================================================================
// period_sel_key.v
//   HDMI 多媒体播放系统 —— 轮播周期选择
//
//   对应《2026 选题指南》基础要求 1.3(3)：
//     "…以及自动轮播周期可配置"
//
//   官方参考实例把周期写死为 1 秒（sd_card_bmp.v 里 auto_cnt == CLK_FREQ_HZ-1），
//   本模块让用户用板载 KEY3 在 1s / 2s / 4s / 8s 之间循环切换，
//   当前值同步送到数码管显示（见 top 里的 seg_decoder_m1）。
//
//   按键极性：本板按键松开=1，按下=0
//   复位极性：高有效（与本工程 rst_all 一致）
//
//   自包含消抖（20ms），不依赖工程里其它 debounce 模块
//=============================================================================
module period_sel_key #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire       clk,
    input  wire       rst,           // 高有效
    input  wire       button_in,     // 松开=1，按下=0
    output reg  [2:0] period_sel     // 0=1s 1=2s 2=4s 3=8s
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg        s_key0;
reg        s_key1;
reg        s_stable;
reg [31:0] s_cnt;
reg        s_press;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s_key0     <= 1'b1;
        s_key1     <= 1'b1;
        s_stable   <= 1'b1;
        s_cnt      <= 32'd0;
        s_press    <= 1'b0;
        period_sel <= 3'd0;
    end else begin
        // 两级同步，消除按键异步抖动
        s_key0  <= button_in;
        s_key1  <= s_key0;
        s_press <= 1'b0;

        // 输入与稳定态不一致时开始计数，连续稳定 DEBOUNCE_MS 才认账
        if (s_key1 == s_stable) begin
            s_cnt <= 32'd0;
        end else if (s_cnt >= DEBOUNCE_CYCLES - 1) begin
            if (s_stable && !s_key1)
                s_press <= 1'b1;     // 确认"按下" -> 单周期脉冲
            s_stable <= s_key1;
            s_cnt    <= 32'd0;
        end else begin
            s_cnt <= s_cnt + 32'd1;
        end

        // 每次按下推进一档，3 -> 0 循环
        if (s_press)
            period_sel <= (period_sel == 3'd3) ? 3'd0 : (period_sel + 3'd1);
    end
end

endmodule
