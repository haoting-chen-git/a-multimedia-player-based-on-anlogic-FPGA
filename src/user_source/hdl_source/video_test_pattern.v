`timescale 1ns / 1ps
//=============================================================================
//  video_test_pattern.v  ——  无卡自检图案（4 种，可按键切换）
//
//  【为什么需要这个模块】
//    官方参考设计（lab_ex4_tf）在 TF 卡没插 / 卡里没有合法 BMP 时，屏幕是纯黑的
//    （display_valid = 0 -> 送 24'd0）。这样在 TF 卡还没到手时就没法验证 HDMI 链路，
//    也看不到自己写的 OSD 字幕。
//
//    本模块把「没有图片」时的黑屏换成标准自检图案，于是：
//      ① 不插卡也能验证 HDMI 时序 / TMDS 输出是否正常
//      ② 不插卡也能看到我们自研的 OSD 字幕图层 + 实时时钟叠加效果
//      ③ 插卡并成功加载图片后，本模块输出自动让位给真实图片（display_valid=1）
//      ④ 【2026-10-17 新增】4 种图案 + 手动/自动轮播 + 淡入转场，
//         让「切图 / 启停 / 轮播周期」这几条要求在没有 TF 卡时也能演示和验证
//
//  【I_idx 的含义】由 demo_pattern_ctrl.v 产生（无卡演示模式的"第几张图"）
//      0 = 8 色竖彩条      —— 验颜色/通道顺序
//      1 = 8 级灰度横条    —— 验灰阶是否连续、有没有色偏
//      2 = 青红棋盘格      —— 验像素时钟抖动（抖动会看到竖纹）
//      3 = 居中准星 + 三分线 —— 验画面是否被裁切/居中（显示器缩放最明显的错）
//
//  【设计要点 —— 与 osd_overlay.v 完全一致的坐标处理】
//    寄存器 S_x/S_y 存的是【上一拍】的坐标，直接拿来用会让整幅图错位一个像素。
//    所以这里用 line_start / S_de_1d 组合推导出【本拍】像素的坐标，
//    保证与 I_de / 像素数据严格同拍（纯组合输出，零延迟）。
//
//  【可综合性】
//    只有比较器 + 移位 + 异或，没有除法、没有乘法、没有 BRAM。
//    竖条宽度 640/8 = 80 不是 2 的幂，所以彩条刻意用比较器而不是 “px / 80”；
//    灰度横条同理（480/8 = 60）。
//=============================================================================
module video_test_pattern #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480,
    parameter BAR_W    = 80            // 每条竖彩条宽度 = 640 / 8
)(
    input  wire        I_clk,
    input  wire        I_rst,          // 高有效异步复位（接 rst_all）
    input  wire        I_vs,           // 场同步
    input  wire        I_de,           // 数据有效
    input  wire [1:0]  I_idx,          // 图案编号（已在 video_clk 域同步好）
    output reg  [23:0] O_rgb           // 组合输出，与 I_de 同拍
);

    // ----------------------------------------------------------------------
    // 1. 自己数像素坐标
    // ----------------------------------------------------------------------
    reg        S_vs_1d;
    reg        S_de_1d;
    reg        S_frame_arm;
    reg [11:0] S_x;                    // 上一拍的 x
    reg [11:0] S_y;                    // 当前行的 y

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_vs_1d     <= 1'b0;
            S_de_1d     <= 1'b0;
            S_frame_arm <= 1'b1;
            S_x         <= 12'd0;
            S_y         <= 12'd0;
        end
        else begin
            S_vs_1d <= I_vs;
            S_de_1d <= I_de;

            if (S_vs_1d != I_vs)
                S_frame_arm <= 1'b1;           // 场同步跳变 -> 重新找帧头

            if (I_de) begin
                if (!S_de_1d) begin            // 本拍是这一行的第一个像素
                    S_x <= 12'd0;
                    if (S_frame_arm) begin
                        S_y         <= 12'd0;  // 本行是帧内第一行
                        S_frame_arm <= 1'b0;
                    end
                    else
                        S_y <= S_y + 12'd1;
                end
                else
                    S_x <= S_x + 12'd1;
            end
        end
    end

    // 本拍像素的坐标（行首那一拍要把 S_x/S_y 加回来）
    wire        line_start = I_de & ~S_de_1d;
    wire [11:0] px = line_start ? 12'd0 : (S_x + 12'd1);
    wire [11:0] py = S_y;

    // ----------------------------------------------------------------------
    // 2. 图案 0 —— 8 段竖彩条（标准 100% 彩条：白 黄 青 绿 品红 红 蓝 黑）
    // ----------------------------------------------------------------------
    localparam [11:0] B1 = BAR_W * 1;
    localparam [11:0] B2 = BAR_W * 2;
    localparam [11:0] B3 = BAR_W * 3;
    localparam [11:0] B4 = BAR_W * 4;
    localparam [11:0] B5 = BAR_W * 5;
    localparam [11:0] B6 = BAR_W * 6;
    localparam [11:0] B7 = BAR_W * 7;

    reg [23:0] bars_rgb;
    always @(*) begin
        if      (px < B1) bars_rgb = 24'hFFFFFF;  // 白
        else if (px < B2) bars_rgb = 24'hFFFF00;  // 黄
        else if (px < B3) bars_rgb = 24'h00FFFF;  // 青
        else if (px < B4) bars_rgb = 24'h00FF00;  // 绿
        else if (px < B5) bars_rgb = 24'hFF00FF;  // 品红
        else if (px < B6) bars_rgb = 24'hFF0000;  // 红
        else if (px < B7) bars_rgb = 24'h0000FF;  // 蓝
        else              bars_rgb = 24'h000000;  // 黑
    end

    // ----------------------------------------------------------------------
    // 3. 图案 1 —— 8 级灰度横条（480/8 = 60 行一级）
    // ----------------------------------------------------------------------
    localparam [11:0] R1 = 12'd60;
    localparam [11:0] R2 = 12'd120;
    localparam [11:0] R3 = 12'd180;
    localparam [11:0] R4 = 12'd240;
    localparam [11:0] R5 = 12'd300;
    localparam [11:0] R6 = 12'd360;
    localparam [11:0] R7 = 12'd420;

    reg  [7:0]  gray;
    always @(*) begin
        if      (py < R1) gray = 8'h00;
        else if (py < R2) gray = 8'h24;
        else if (py < R3) gray = 8'h48;
        else if (py < R4) gray = 8'h6C;
        else if (py < R5) gray = 8'h90;
        else if (py < R6) gray = 8'hB4;
        else if (py < R7) gray = 8'hD8;
        else              gray = 8'hFF;
    end
    wire [23:0] gray_rgb = {gray, gray, gray};

    // ----------------------------------------------------------------------
    // 4. 图案 2 —— 64x64 青/红棋盘格（像素时钟不稳时这里最先出竖纹）
    // ----------------------------------------------------------------------
    wire chk = px[6] ^ py[6];
    wire [23:0] checker_rgb = chk ? 24'hFF0000 : 24'h00FFFF;

    // ----------------------------------------------------------------------
    // 5. 图案 3 —— 黑底 + 白边框 + 中心准星 + 三分线（验居中/裁切）
    // ----------------------------------------------------------------------
    wire border2  = (px < 12'd3) || (px >= 12'd637) || (py < 12'd3) || (py >= 12'd477);
    wire cross_v  = (px >= 12'd318) && (px < 12'd322);
    wire cross_h  = (py >= 12'd238) && (py < 12'd242);
    wire third    = (px == 12'd213) || (px == 12'd426) || (py == 12'd160) || (py == 12'd320);

    reg [23:0] align_rgb;
    always @(*) begin
        if (border2 || cross_v || cross_h) align_rgb = 24'hFFFFFF;
        else if (third)                    align_rgb = 24'h505050;
        else                               align_rgb = 24'h000000;
    end

    // ----------------------------------------------------------------------
    // 6. 图案选择
    // ----------------------------------------------------------------------
    always @(*) begin
        case (I_idx)
            2'd1:    O_rgb = gray_rgb;
            2'd2:    O_rgb = checker_rgb;
            2'd3:    O_rgb = align_rgb;
            default: O_rgb = bars_rgb;
        endcase
    end

endmodule
