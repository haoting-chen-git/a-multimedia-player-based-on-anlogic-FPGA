`timescale 1ns / 1ps
//=============================================================================
//  demo_pattern_ctrl.v  ——  无卡演示模式的"播放控制"（手动切图 / 启停 / 轮播）
//
//  【为什么需要这个模块】
//    官方 sd_card_bmp.v 里，切图和启停按键都挂在 S_PLAY 状态里，并且要求
//    scan_done && img_found_count > 0 —— 也就是【必须先扫到 TF 卡里的图】，
//    按键才会有反应。没插卡时按 KEY2/KEY4 完全没有反馈，无法演示、
//    也无法验证「手动切下一张」「启停自动轮播」「轮播周期可配置」这三条（基础 1.3(3)）。
//
//    本模块在"没有图片"(display_valid = 0) 时接管控制：
//      - KEY_NEXT 按下：图案序号 +1，并触发淡入转场
//      - KEY_AUTO 按下：启停自动轮播
//      - 自动轮播周期由 period_sel 决定（1s / 2s / 4s / 8s，与真图片用同一套档位）
//    display_valid = 1（有真图）时，本模块的输出没人看，自然让位。
//
//  【时钟域】全部工作在 clk = 50MHz。pattern_idx 跨到 video_clk 域由 top
//   里头两级寄存器完成（与 audio_track_sync / period_sel_v0/v1 同一套写法）。
//
//  【为什么用 >= 而不是 == 比较计数】
//    运行中把周期从 4s 调到 1s 时，计数器早已越过 1s 对应的值，
//    用 == 会永远不成立、轮播直接卡死。用 >= 则下一拍立刻触发。
//=============================================================================
module demo_pattern_ctrl #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire       clk,             // 50MHz
    input  wire       rst,             // 高有效异步复位（接 rst_all）
    input  wire       key_next,        // 手动下一张（低有效，引脚带 PULLUP）
    input  wire       key_auto,        // 启停自动轮播（低有效）
    input  wire [2:0] period_sel,      // 轮播周期档位 0/1/2/3 -> 1/2/4/8 秒
    output reg  [1:0] pattern_idx,     // 当前图案编号
    output reg        auto_en          // 1 = 自动轮播
);

    localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

    // ----------------------------------------------------------------------
    // 1. 两个按键各自 20ms 消抖
    //    st_* = 消抖后的稳定电平（1 = 未按）；st_*_d 打一拍，下降沿即为"真的按下"
    // ----------------------------------------------------------------------
    reg [19:0] db_cnt_n;
    reg [19:0] db_cnt_a;
    reg        st_n, st_n_d;
    reg        st_a, st_a_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            db_cnt_n <= 20'd0;
            st_n     <= 1'b1;
            st_n_d   <= 1'b1;
        end
        else begin
            st_n_d <= st_n;
            if (key_next == st_n)
                db_cnt_n <= 20'd0;                 // 电平一致，计数清零
            else if (db_cnt_n >= DEBOUNCE_CYCLES[19:0]) begin
                db_cnt_n <= 20'd0;
                st_n     <= key_next;              // 稳定 20ms，接受新电平
            end
            else
                db_cnt_n <= db_cnt_n + 20'd1;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            db_cnt_a <= 20'd0;
            st_a     <= 1'b1;
            st_a_d   <= 1'b1;
        end
        else begin
            st_a_d <= st_a;
            if (key_auto == st_a)
                db_cnt_a <= 20'd0;
            else if (db_cnt_a >= DEBOUNCE_CYCLES[19:0]) begin
                db_cnt_a <= 20'd0;
                st_a     <= key_auto;
            end
            else
                db_cnt_a <= db_cnt_a + 20'd1;
        end
    end

    wire press_next = st_n_d & ~st_n;   // 下降沿 = 按下
    wire press_auto = st_a_d & ~st_a;

    // ----------------------------------------------------------------------
    // 2. 自动轮播定时器（周期由 period_sel 决定）
    // ----------------------------------------------------------------------
    wire [31:0] reload = (period_sel == 3'd0) ? 32'd50_000_000  :   // 1s
                         (period_sel == 3'd1) ? 32'd100_000_000 :   // 2s
                         (period_sel == 3'd2) ? 32'd200_000_000 :   // 4s
                                                32'd400_000_000;    // 8s

    reg  [31:0] auto_cnt;
    wire        auto_tick = (auto_cnt >= (reload - 32'd1));

    always @(posedge clk or posedge rst) begin
        if (rst)           auto_cnt <= 32'd0;
        else if (auto_tick) auto_cnt <= 32'd0;
        else               auto_cnt <= auto_cnt + 32'd1;
    end

    // ----------------------------------------------------------------------
    // 3. 图案序号 + 自动轮播开关
    // ----------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pattern_idx <= 2'd0;
            auto_en     <= 1'b1;        // 上电默认自动轮播，演示时不用按键
        end
        else begin
            if (press_auto)
                auto_en <= ~auto_en;

            if (press_next)
                pattern_idx <= pattern_idx + 2'd1;      // 手动优先
            else if (auto_en && auto_tick)
                pattern_idx <= pattern_idx + 2'd1;
        end
    end

endmodule
