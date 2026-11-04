`timescale 1ns / 1ps
//=============================================================================
//  audio_vu_meter.v  ——  音频可视化：8 段柱状电平表（屏幕右下角）
//
//  【为什么做这个】
//    《2026 选题指南》1.4(2) 扩展要求「音视频联动」，报名表的作品简介里也明确写了
//    「音频可视化展示」。参考实例完全没有这个功能，所以它既是加分项，
//    也是"音频处理方法不能与参考实例相同"的又一个直接证据。
//
//  【数据来源】
//    audio_melody_player.v 输出：
//      I_note  —— 当前正在播放的音高编号 0..23（C4..B5）
//      I_level —— 峰值保持后的电平 0..63（带约 170ms 衰减）
//
//    旋律是单音旋律，同一时刻只有一个音在响，所以"哪个柱子在亮"直接等于音高位置，
//    这就是一个真实（而非装饰性）的频谱示意：低音在左、高音在右。
//    音高 -> 柱子：每 3 个半音一根柱，24 个半音正好铺满 8 根（两个八度）。
//
//  【衰减与拖尾】
//    每根柱子有自己的峰值保持寄存器（6 位），每帧衰减 2。
//    所以一个音结束后，它的柱子会慢慢落下来，音阶上行时能看到一串"拖尾"，
//    视觉上很像真的电平表在扫。
//
//  【坐标处理】与 video_test_pattern.v / osd_overlay.v 完全一致：
//    寄存器存的是【上一拍】坐标，用 line_start 推导出本拍坐标，保证与像素数据同拍。
//
//  【位置】x 384..639，y 416..479（屏幕右下角，正好避开第 3 行「轮播周期 N 秒」文字）。
//    每根柱子占 32px 槽位、实宽 26px、最高 64px。
//=============================================================================
module audio_vu_meter #(
    parameter X0       = 384,      // 左边界
    parameter Y_BOTTOM = 480,      // 底边（不含）
    parameter SLOT_W   = 32,       // 每个槽位宽
    parameter BAR_W    = 26,       // 柱子实宽（槽位内留 6px 间隙）
    parameter MAX_H    = 64,       // 柱子最大高度
    parameter DECAY    = 2         // 每帧衰减量
)(
    input  wire        I_clk,
    input  wire        I_rst,      // 高有效异步复位（接 rst_all）
    input  wire        I_vs,
    input  wire        I_de,
    input  wire [4:0]  I_note,     // 当前音高 0..23
    input  wire [5:0]  I_level,    // 电平 0..63
    input  wire [23:0] I_rgb,
    output reg  [23:0] O_rgb,
    // 输出整体比输入晚 1 拍（本模块内部打了拍），所以 de/vs 必须同步延后 1 拍
    // 给下游，否则下游会把"上一像素"当成"当前像素"，画面整体左移 1 像素。
    output reg         O_de,
    output reg         O_vs
);

    // 参数统一转成有宽度的 localparam，避免对 parameter 做位选（部分综合器不接受）
    localparam [11:0] X_BEG  = X0;                    // 384
    localparam [11:0] Y_END  = Y_BOTTOM;              // 480
    localparam [11:0] X_END  = X0 + 8 * SLOT_W;       // 640
    localparam [11:0] Y_TOP  = Y_BOTTOM - MAX_H;      // 416
    localparam [4:0]  BAR_W5 = BAR_W;                 // 26

    // ----------------------------------------------------------------------
    // 1. 自己数像素坐标（与 video_test_pattern.v 逐行对应）
    // ----------------------------------------------------------------------
    reg        S_vs_1d;
    reg        S_de_1d;
    reg        S_frame_arm;
    reg [11:0] S_x;
    reg [11:0] S_y;

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
                S_frame_arm <= 1'b1;

            if (I_de) begin
                if (!S_de_1d) begin
                    S_x <= 12'd0;
                    if (S_frame_arm) begin
                        S_y         <= 12'd0;
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

    wire        line_start = I_de & ~S_de_1d;
    wire [11:0] px = line_start ? 12'd0 : (S_x + 12'd1);
    wire [11:0] py = S_y;

    // ----------------------------------------------------------------------
    // 2. 音高 -> 柱子编号（每 2 个半音一根）
    //
    //    注意：本人写的 4 首旋律实际用到的音高范围是 0..14（C4 起的半音序号），
    //    如果按"每 3 个半音一根"划分，只会点亮左边 5 根，右边 3 根永远黑着。
    //    所以改成每 2 个半音一根，0..15 正好铺满 8 根，视觉上整排都会动。
    // ----------------------------------------------------------------------
    reg [2:0] R_bar;
    always @(*) begin
        if      (I_note < 5'd2)  R_bar = 3'd0;
        else if (I_note < 5'd4)  R_bar = 3'd1;
        else if (I_note < 5'd6)  R_bar = 3'd2;
        else if (I_note < 5'd8)  R_bar = 3'd3;
        else if (I_note < 5'd10) R_bar = 3'd4;
        else if (I_note < 5'd12) R_bar = 3'd5;
        else if (I_note < 5'd14) R_bar = 3'd6;
        else                     R_bar = 3'd7;
    end

    // ----------------------------------------------------------------------
    // 3. 8 根柱子各自的峰值保持寄存器（每帧衰减 DECAY）
    // ----------------------------------------------------------------------
    reg [5:0] S_pk0, S_pk1, S_pk2, S_pk3, S_pk4, S_pk5, S_pk6, S_pk7;

    reg  S_vs_d;
    wire W_frame_tick = I_vs & ~S_vs_d;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) S_vs_d <= 1'b0;
        else       S_vs_d <= I_vs;
    end

    localparam [5:0] D = DECAY;

    `define VU_PEAK(NAME, IDX)                                                   \
        always @(posedge I_clk or posedge I_rst) begin                            \
            if (I_rst)                                                           \
                NAME <= 6'd0;                                                    \
            else if (W_frame_tick) begin                                         \
                if ((R_bar == IDX) && (I_level > ((NAME > D) ? (NAME - D) : 6'd0))) \
                    NAME <= I_level;                                             \
                else                                                             \
                    NAME <= (NAME > D) ? (NAME - D) : 6'd0;                      \
            end                                                                  \
        end

    `VU_PEAK(S_pk0, 3'd0)
    `VU_PEAK(S_pk1, 3'd1)
    `VU_PEAK(S_pk2, 3'd2)
    `VU_PEAK(S_pk3, 3'd3)
    `VU_PEAK(S_pk4, 3'd4)
    `VU_PEAK(S_pk5, 3'd5)
    `VU_PEAK(S_pk6, 3'd6)
    `VU_PEAK(S_pk7, 3'd7)

    `undef VU_PEAK

    // ----------------------------------------------------------------------
    // 4. 当前像素落在哪个柱子的槽位、该柱子的高度
    // ----------------------------------------------------------------------
    wire        in_x = (px >= X_BEG) && (px < X_END);
    wire        in_y = (py >= Y_TOP)    && (py < Y_END);
    wire        in_region = I_de && in_x && in_y;

    wire [11:0] rel_x = px - X_BEG;
    wire [2:0]  slot  = rel_x[7:5];              // 32px 一个槽位
    wire [4:0]  xin   = rel_x[4:0];
    wire        x_lit = (xin < BAR_W5);

    reg [5:0] R_h;
    always @(*) begin
        case (slot)
            3'd0: R_h = S_pk0;
            3'd1: R_h = S_pk1;
            3'd2: R_h = S_pk2;
            3'd3: R_h = S_pk3;
            3'd4: R_h = S_pk4;
            3'd5: R_h = S_pk5;
            3'd6: R_h = S_pk6;
            default: R_h = S_pk7;
        endcase
    end

    wire [11:0] y_from_bottom = (Y_END - 12'd1) - py;   // 区域内在 0..63
    wire        y_lit = (y_from_bottom < {6'd0, R_h});

    // ----------------------------------------------------------------------
    // 5. 上色：下 32px 绿、中间 16px 黄、顶部 16px 红（真电平表的配色）
    //
    //    ★ 这里必须是"两级流水线"，不能纯组合直通，也不能只打一拍。
    //
    //    原因（实测踩过两次坑）：
    //      本模块串在 osd_overlay 之后。osd_overlay 输出的 I_rgb 是一大团组合逻辑
    //      （字体 ROM 查表 + cell 判定 + alpha 混合）。如果本模块的"选柱子/比高度"
    //      判定逻辑和它在同一个 LUT 网络里汇合，综合器会把两团逻辑缝成一条
    //      Logic Level = 31 的长路径，25MHz(40ns) 下实测 SWNS = -0.511ns / -0.142ns。
    //
    //    两级流水的分工：
    //      第一级：把 I_rgb（OSD 那一大团）和"本像素要不要画柱子、画什么颜色"
    //              分别打进寄存器 -> 两团组合逻辑被彻底切断，各自独立满足 40ns。
    //      第二级：纯 4 选 1，路径极短。
    //    代价是整体晚 2 拍，所以 O_de / O_vs 也必须同步延后 2 拍交给下游。
    // ----------------------------------------------------------------------
    reg [23:0] S_rgb_s1;
    reg        S_lit_s1;
    reg [1:0]  S_col_s1;      // 0=绿 1=黄 2=红
    reg        S_de_s1;
    reg        S_vs_s1;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_rgb_s1 <= 24'd0;
            S_lit_s1 <= 1'b0;
            S_col_s1 <= 2'd0;
            S_de_s1  <= 1'b0;
            S_vs_s1  <= 1'b0;
        end
        else begin
            S_rgb_s1 <= I_rgb;
            S_lit_s1 <= in_region && x_lit && y_lit;
            S_col_s1 <= (y_from_bottom < 12'd32) ? 2'd0 :
                        (y_from_bottom < 12'd48) ? 2'd1 : 2'd2;
            S_de_s1  <= I_de;
            S_vs_s1  <= I_vs;
        end
    end

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            O_rgb <= 24'd0;
            O_de  <= 1'b0;
            O_vs  <= 1'b0;
        end
        else begin
            O_de <= S_de_s1;
            O_vs <= S_vs_s1;

            if (!S_lit_s1)
                O_rgb <= S_rgb_s1;
            else if (S_col_s1 == 2'd0)
                O_rgb <= 24'h00FF00;
            else if (S_col_s1 == 2'd1)
                O_rgb <= 24'hFFFF00;
            else
                O_rgb <= 24'hFF0000;
        end
    end

endmodule
