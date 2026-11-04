// ==========================================================================
// osd_overlay.v —— OSD 字幕 / 图层叠加
//   安路 EG4S20 (HX4S20C) HDMI 多媒体播放器
//
// 接在哪里：
//   top_tf_hdmi_audio.v 里
//       osd_overlay u_osd_overlay(.I_rgb(vout_data_fade), .O_rgb(vout_data_osd) ...)
//   O_cell 送 osd_text_rom，字符/字色再送回来。
//
// --------------------------------------------------------------------------
// ★ 时序改造（2026-10-17，第二轮：真正的两级流水）
//
//   原始版本是一条 40ns 的组合长链，实测 video_clk(25MHz) SWNS = -0.042ns：
//        S_x(扇出99) -> 比较器 -> 格子号 -> osd_text_rom(80 项 case)
//        -> 字符 -> osd_font_rom(887 项 case) -> 32bit 字模 -> 32:1 mux -> 混色
//   TD 报告 Logic Level 30，cell 11.3ns / net 28.3ns，字模 ROM 那段吃掉 ~26ns。
//
//   现在拆成两级，仍然是【零延迟】（O_rgb 与本拍 I_rgb 同拍）：
//
//     第 1 级（组合）：px/py -> 下一个像素的 nx/ny -> in_area/格号/格内偏移
//                      -> O_cell -> osd_text_rom -> 字符/是否写字/字色
//     打一拍（寄存器）：in_area, gx, gy, char, char_en, char_rgb 全部寄存
//     第 2 级（组合）：char+gy -> osd_font_rom -> 32bit 字模 -> [gx] -> 混色
//
//   为什么还是零延迟：寄存器里存的是"下一个像素"的全部决策结果，下一拍正好
//   用上，所以 O_rgb 依旧与本拍 I_rgb 同拍，de/hs/vs 一个都不用改。
//
//   行尾回绕的处理：px=639 时 nx 回到 0、ny 加 1；消隐期间 de=0、S_x 停在 639，
//   于是 nx 恒为 0、ny 恒为 S_y+1，恰好就是下一行行首需要的那组值 ——
//   所以行首像素不会错位。
// --------------------------------------------------------------------------
//
// 设计要求 / 保证：
//   1) 【零延迟】O_rgb 与本拍 I_rgb 的像素严格同拍。
//   2) 【自带坐标】原设计没有把 x/y 计数器引出来，本模块自己数 I_de 得到坐标。
//   3) 【无除法器】所有几何计算只有移位和掩码，前提是 CELL_W/CELL_H 是 2 的幂。
//   4) 【字色/字符外置】字符、字色由 osd_text_rom.v 组合给出，本模块只管几何与混色。
//
// 资源：约 250 个 LUT（不含字库 ROM；字库见 osd_font_rom.v）
// ==========================================================================

module osd_overlay #(
    parameter H_ACTIVE    = 640,
    parameter V_ACTIVE    = 480,

    // ---- 字模与放大 ----
    parameter SCALE       = 1,        // 1 或 2（只能是 2 的幂）
    parameter SCALE_SH    = 0,        // log2(SCALE)
    parameter LOG2_CELL_W = 5,        // log2(32*SCALE)  SCALE=1 -> 5, SCALE=2 -> 6
    parameter LOG2_CELL_H = 5,        // 同上（行方向）

    // ---- 文本层几何 ----
    parameter COL_BITS    = 5,        // ceil(log2(COLS))   COLS=20 -> 5
    parameter ROW_BITS    = 2,        // ceil(log2(ROWS))   ROWS=4  -> 2
    parameter [11:0] AREA_X = 12'd0,  // OSD 区域左上角（像素）
    parameter [11:0] AREA_Y = 12'd352,// OSD 区域左上角（像素）
    parameter COLS        = 20,       // 一行几格
    parameter ROWS        = 4,        // 几行

    // ---- 字幕条（把背景压暗，字幕更清楚）----
    parameter EN_BAR      = 1,        // 1 = 开
    parameter BAR_SHIFT   = 1         // 每个颜色通道右移几位：1 = 亮度减半
)(
    input  wire        I_clk,         // video_clk (25MHz)
    input  wire        I_rst,         // rst_all，高有效
    input  wire        I_vs,          // 场同步（脉冲或电平都可以，只用到跳变）
    input  wire        I_de,          // 数据有效
    input  wire [23:0] I_rgb,         // 原始像素 {R,G,B}

    // ---- 文本层接口（接 osd_text_rom）----
    output wire [COL_BITS+ROW_BITS-1:0] O_cell,     // 当前像素所在格 {row, col}
    input  wire [7:0]  I_char,        // 该格字符编码
    input  wire        I_char_en,     // 该格是否写字
    input  wire [23:0] I_char_rgb,    // 该格字色

    output wire [23:0] O_rgb
);

    localparam CELL_W = 32 * SCALE;
    localparam CELL_H = 32 * SCALE;
    localparam AREA_W = COLS * CELL_W;
    localparam AREA_H = ROWS * CELL_H;

    // ----------------------------------------------------------------------
    // 1. 自己数像素坐标
    //    行首判定用 I_de & ~S_de_1d；帧头靠 I_vs 的任意跳变重新对齐。
    //    这套判定和 video_rgb_to_axis_640x480.v 内部完全一致，已验证可用。
    // ----------------------------------------------------------------------
    reg        S_vs_1d;
    reg        S_de_1d;
    reg        S_frame_arm;
    reg [11:0] S_x;                   // 上一拍的 x
    reg [11:0] S_y;                   // 当前行的 y

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
                S_frame_arm <= 1'b1;          // 帧同步跳变 -> 重新找帧头

            if (I_de) begin
                if (!S_de_1d) begin           // 本拍是这一行的第一个像素
                    S_x <= 12'd0;
                    if (S_frame_arm) begin
                        S_y         <= 12'd0; // 本行是帧内第一行
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

    // 算出【本拍像素】的坐标（行首那一拍 S_x/S_y 还是旧值，要加回来）。
    wire        line_start = I_de & ~S_de_1d;
    wire [11:0] px = line_start ? 12'd0 : (S_x + 12'd1);
    wire [11:0] py = line_start ? (S_frame_arm ? 12'd0 : (S_y + 12'd1)) : S_y;

    // ----------------------------------------------------------------------
    // 2. 第 1 级：算【下一个像素】的几何量
    //    下一个像素的坐标：行尾回绕到 0 并把 y 加 1。
    //    消隐期间 de=0、S_x 停住，于是 nx 恒 0、ny 恒 S_y+1 —— 正好是下一行行首。
    // ----------------------------------------------------------------------
    wire [11:0] nx_raw  = px + 12'd1;
    wire        nx_wrap = (nx_raw >= H_ACTIVE);
    wire [11:0] nx      = nx_wrap ? 12'd0        : nx_raw;
    wire [11:0] ny      = nx_wrap ? (py + 12'd1) : py;

    wire        n_in_area_x = (nx >= AREA_X) && (nx < (AREA_X + AREA_W));
    wire        n_in_area_y = (ny >= AREA_Y) && (ny < (AREA_Y + AREA_H));
    // 注意：这里【不能】再与 I_de 相与。因为本拍的 n_in_area 是给"下一拍"用的，
    // 而下一拍的 de 与本拍无关。行首那个像素的值是在消隐期算出来的（那时 de=0），
    // 一旦与上 I_de，行首像素就会丢掉字幕条 —— 每行第 0 列会亮一条原视频。
    // 消隐期的误命中无所谓：消隐像素根本不进 AXIS 流。
    wire        n_in_area   = n_in_area_x & n_in_area_y;

    wire [11:0] ndx = nx - AREA_X;
    wire [11:0] ndy = ny - AREA_Y;

    wire [COL_BITS-1:0]    n_col_idx = ndx[LOG2_CELL_W +: COL_BITS];
    wire [ROW_BITS-1:0]    n_row_idx = ndy[LOG2_CELL_H +: ROW_BITS];
    wire [LOG2_CELL_W-1:0] n_gx      = ndx[LOG2_CELL_W-1:0] >> SCALE_SH;
    wire [LOG2_CELL_H-1:0] n_gy      = ndy[LOG2_CELL_H-1:0] >> SCALE_SH;

    // O_cell 直接驱动外部 osd_text_rom（组合），其输出在本时钟沿被寄存。
    assign O_cell = {n_row_idx, n_col_idx};

    // ----------------------------------------------------------------------
    // 3. 打一拍：把"下一个像素"需要的全部决策结果锁存下来
    //    注意：char_rgb 也一起寄存，这样混色级不再依赖上游组合。
    // ----------------------------------------------------------------------
    reg                    S_in_area;
    reg [LOG2_CELL_W-1:0]  S_gx;
    reg [LOG2_CELL_H-1:0]  S_gy;
    reg [7:0]              S_char;
    reg                    S_en;
    reg [23:0]             S_rgb;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            S_in_area <= 1'b0;
            S_gx      <= {LOG2_CELL_W{1'b0}};
            S_gy      <= {LOG2_CELL_H{1'b0}};
            S_char    <= 8'd0;
            S_en      <= 1'b0;
            S_rgb     <= 24'd0;
        end
        else begin
            S_in_area <= n_in_area;
            S_gx      <= n_gx;
            S_gy      <= n_gy;
            S_char    <= I_char;
            S_en      <= I_char_en;
            S_rgb     <= I_char_rgb;
        end
    end

    // ----------------------------------------------------------------------
    // 4. 第 2 级：查字模（驱动源全是寄存器，扇出网络被彻底切断）
    // ----------------------------------------------------------------------
    wire [31:0] font_bits;

    osd_font_rom u_osd_font_rom (
        .I_code (S_char),
        .I_row  (S_gy[4:0]),          // 字模固定 32 行
        .O_bits (font_bits)
    );

    wire glyph_on = S_en & font_bits[S_gx[4:0]];   // S_gx 是变量索引 -> 32:1 mux

    // ----------------------------------------------------------------------
    // 5. 混色：字 -> 字幕条 -> 原图
    // ----------------------------------------------------------------------
    reg [23:0] bar_rgb;

    always @(*) begin
        if (EN_BAR) begin
            bar_rgb[23:16] = I_rgb[23:16] >> BAR_SHIFT;
            bar_rgb[15:8]  = I_rgb[15:8]  >> BAR_SHIFT;
            bar_rgb[7:0]   = I_rgb[7:0]   >> BAR_SHIFT;
        end
        else
            bar_rgb = I_rgb;
    end

    assign O_rgb = !S_in_area ? I_rgb      :
                   (glyph_on   ? S_rgb      :
                    (EN_BAR   ? bar_rgb    : I_rgb));

endmodule
