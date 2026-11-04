
module top(
    input                       clk,
    input                       rst_n,          // K1 = 系统复位（低有效�?
    input                       key1,           // K2 = 自动/手动 切换
    input                       key2,           // K3 = 下一张（手动切图�?
    input                       key3,           // K4 = 轮播周期 1s/2s/4s/8s

	output [5:0]                seg_sel,
	output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
// v33：SDRAM 里开 4 �?640x480 槽，每张图只�?TF 卡读一次，之后切图只换基址
parameter BUF2_ADDR     = FRAME_PIXELS * 2;   // 614400
parameter BUF3_ADDR     = FRAME_PIXELS * 3;   // 921600

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;

// ===== 轮播周期可配置（基础要求 1.3(3)�?====
wire [6:0]  seg_speed;          // 周期数值的 7 段码
wire [3:0]  speed_digit;        // 周期对应的十进制数字�?/2/4/8
wire [2:0]  period_sel_50m;     // clk(50MHz) �?
reg  [2:0]  period_sel_sd0;     // 同步�?sd_card_clk(100MHz) �?
reg  [2:0]  period_sel_sd1;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 相关
wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 链路
wire sys_pll_lock;
wire video_pll_lock;
wire rst_all;

reg [19:0] rst_cnt = 20'd0;
reg rst_all_reg = 1'b1;

always @(posedge clk) begin
    if (!rst_n) begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end else if (!rst_all_reg) begin
        // 一旦复位释放，忽略后续微小�?PLL 抖动，防止系统在使用时随机复�?
        rst_all_reg <= 1'b0;
    end else if (sys_pll_lock && video_pll_lock) begin
        // 等待 PLL 连续稳定 20ms @ 50MHz
        if (rst_cnt < 20'd1_000_000) begin
            rst_cnt <= rst_cnt + 1'b1;
            rst_all_reg <= 1'b1;
        end else begin
            rst_all_reg <= 1'b0;
        end
    end else begin
        rst_cnt <= 20'd0;
        rst_all_reg <= 1'b1;
    end
end

assign rst_all = rst_all_reg;

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (sys_pll_lock),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (video_pll_lock),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同�?
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
// ---- 虚拟 SD 卡：1 = 用内部假 BMP 图源（没有读卡器 / 卡里�?BMP 也能验证自适应缩放�?
//      此时屏幕左上�?8 位十六进制显示的就是"当前虚拟图宽、高"，例�?0320 0258 = 800x600
localparam VSRC_EN = 1'b0;
wire        vsrc_en = VSRC_EN;
wire [15:0] vsrc_w;
wire [15:0] vsrc_h;
wire [2:0]  vsrc_idx;
wire        sd_auto_en;         // 自动轮播开关（sd_card_clk 域），�?OSD 显示"自动/手动"
wire        sd_preload_en;      // 仍在往 SDRAM 预载图片（sd_card_clk 域），�?OSD 显示"预载"

sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd32767),
    .SCAN_TARGET_COUNT (3'd4)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key2),        // K3 = 下一�?
    .key_auto          (key1),        // K2 = 自动/手动
    .period_sel        (period_sel_sd1),
    .state_code        (state_code),
    .bmp_width         (16'd2048),   // 尺寸上限（bmp_read 里是"<=", 不再要求正好 640�?
    .bmp_height        (16'd1200),   // 缩放�?bmp_scaler 完成，输出恒�?640x480
    .display_valid     (display_valid),
    .o_auto_en         (sd_auto_en),
    .o_preload_en      (sd_preload_en),
    .o_diag            (sd_diag),
    .o_diag2           (sd_diag2),
    .vsrc_en           (vsrc_en),
    .o_vsrc_w          (vsrc_w),
    .o_vsrc_h          (vsrc_h),
    .o_vsrc_idx        (vsrc_idx),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

// ===== 轮播周期：KEY3 选择 + 数码管显示（基础要求 1.3(3)�?====
period_sel_key #(
    .CLK_FREQ_HZ       (50_000_000),
    .DEBOUNCE_MS       (20)
) period_sel_key_m0 (
    .clk               (clk),
    .rst               (rst_all),
    .button_in         (key3),        // K4 = 轮播周期
    .period_sel        (period_sel_50m)
);

// 周期档位是准静态量（按一次键才变），打两拍即可安全跨�?sd_card_clk �?
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        period_sel_sd0 <= 3'd0;
        period_sel_sd1 <= 3'd0;
    end else begin
        period_sel_sd0 <= period_sel_50m;
        period_sel_sd1 <= period_sel_sd0;
    end
end

// 0/1/2/3 -> 1/2/4/8 秒，显示在数码管�?5 位，评审一眼就能看到当前周�?
// 轮播周期数字 -> OSD 字符�?1' '2' '4' '8'，都在字�?0-9 范围内）
//
// �?跨时钟域处理�?026-10-17 实测踩坑）：
//   最初直接写 wire osd_per_char = f(period_sel_50m)，结�?period_sel_50m 属于
//   clk(50MHz) 域，�?osd_text_rom �?video_clk �?-> TD �?
//   "From Clock clk -> To Clock video_clk  SWNS -5.454ns / 24 个违例端�?�?
//   现在先把 3 �?period_sel 用两级寄存器同步�?video_clk，再�?video_clk 域里
//   组合�?ASCII：跨域部分只�?3 条单比特寄存器路径，是标�?CDC 写法�?
//   （按�?20ms 消抖 + 人手指按下，这个 3 位值对 video_clk 来说是准静态的。）
reg  [2:0] period_sel_v0;
reg  [2:0] period_sel_v1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        period_sel_v0 <= 3'd0;
        period_sel_v1 <= 3'd0;
    end
    else begin
        period_sel_v0 <= period_sel_50m;
        period_sel_v1 <= period_sel_v0;
    end
end

wire [7:0] osd_per_char = (period_sel_v1 == 3'd0) ? 8'h31 :
                          (period_sel_v1 == 3'd1) ? 8'h32 :
                          (period_sel_v1 == 3'd2) ? 8'h34 : 8'h38;

// ---- OSD 上的状态词（KEY2 按下去到底有没有生效，看这两个字�?---
//   auto_play_en / o_preload_en 都在 sd_card_clk 域，这里打两拍进 video_clk�?
//   字库编码�?osd_font_rom.v 注释表：�?0x81 �?0x89 �?0x91 �?0x94 �?0x96
localparam [7:0] CH_ZI = 8'h91;     // �?
localparam [7:0] CH_DONG = 8'h81;   // �?
localparam [7:0] CH_SHOU = 8'h89;   // �?
localparam [7:0] CH_ZAI  = 8'h94;   // �?
localparam [7:0] CH_YU   = 8'h96;   // �?

reg  auto_en_sd0;
reg  auto_en_sd1;
reg  preload_sd0;
reg  preload_sd1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        auto_en_sd0 <= 1'b0;
        auto_en_sd1 <= 1'b0;
        preload_sd0 <= 1'b1;
        preload_sd1 <= 1'b1;
    end
    else begin
        auto_en_sd0 <= sd_auto_en;
        auto_en_sd1 <= auto_en_sd0;
        preload_sd0 <= sd_preload_en;
        preload_sd1 <= preload_sd0;
    end
end

// 预载中显�?预载"，装完显�?自动 / 手动"
wire [7:0] osd_mode_char0 = preload_sd1 ? CH_YU   : (auto_en_sd1 ? CH_ZI : CH_SHOU);
wire [7:0] osd_mode_char1 = preload_sd1 ? CH_ZAI  : CH_DONG;

assign speed_digit = (period_sel_50m == 3'd0) ? 4'd1 :
                     (period_sel_50m == 3'd1) ? 4'd2 :
                     (period_sel_50m == 3'd2) ? 4'd4 : 4'd8;

seg_decoder seg_decoder_m1(
    .bin_data          (speed_digit),
    .seg_data          (seg_speed)
);

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,7'b1111_111}),
    .seg_data_1        ({1'b1,7'b1111_111}),
    .seg_data_2        ({1'b1,7'b1111_111}),
    .seg_data_3        ({1'b1,7'b1111_111}),
    .seg_data_4        ({1'b1,seg_speed}),
    .seg_data_5        ({1'b1,seg_data_0})
);

// ===================== 原图像时序与帧缓�?=====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前：显示自检 8 色竖彩条（TF 卡未�?/ 卡内无合法图时也能验�?HDMI + OSD�?
// 首图提交后：一直显示当前显示缓冲区内容
//
// �?注意：这里必须用同步后的 dvalid_v1，不能用原始�?display_valid�?
//   display_valid �?sd_card_clk(100MHz) 域的寄存器输出，直接拿它�?video_clk(25MHz)
//   域的像素 mux，会把一个纯跨域信号编进像素组合通路里（实测�?
//   "-0.511ns, Begin: sd_card_bmp_m0/display_valid_reg, End: u_audio_vu_meter/O_rgb"）�?
//   dvalid_v1 是它经过两级同步器后�?video_clk 域版本，语义相同、时序干净�?
wire [23:0] vout_data_pat;
assign vout_data = dvalid_v1 ? vout_data_raw : vout_data_pat;

// ===================== 无卡演示模式�?026-10-17 新增�?====================
//  官方 sd_card_bmp 的切�?启停按键要求先扫�?TF 卡里的图，没卡时完全没反应，
//  于是"手动切图 / 启停轮播 / 轮播周期可配�?这三条基础要求无法演示也无法验证�?
//  这里加一个独立的演示控制器：没卡时用 4 种自检图案�?图片"�?
//  按键、自动轮播、周期档位、淡入转场全部照常工作；插卡后自动让位给真图片�?
wire [1:0] demo_idx_50m;        // clk(50MHz) �?
wire       demo_auto_en;        // 1 = 自动轮播（演示模式）

demo_pattern_ctrl #(
    .CLK_FREQ_HZ (50_000_000),
    .DEBOUNCE_MS (20)
) demo_pattern_ctrl_m0 (
    .clk         (clk),
    .rst         (rst_all),
    .key_next    (key2),         // K3：手动切下一�?
    .key_auto    (key1),         // K2：启停自动轮�?
    .period_sel  (period_sel_50m),
    .pattern_idx (demo_idx_50m),
    .auto_en     (demo_auto_en)
);

// ---- 把演示模式的序号 / display_valid 同步�?video_clk 域（两级寄存器）----
reg [1:0] demo_idx_v0, demo_idx_v1;
reg       dvalid_v0,   dvalid_v1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        demo_idx_v0 <= 2'd0;
        demo_idx_v1 <= 2'd0;
        dvalid_v0   <= 1'b0;
        dvalid_v1   <= 1'b0;
    end
    else begin
        demo_idx_v0 <= demo_idx_50m;
        demo_idx_v1 <= demo_idx_v0;
        dvalid_v0   <= display_valid;
        dvalid_v1   <= dvalid_v0;
    end
end

// disp_buf_idx 属于 sd_card_clk 域。若直接把它�?video_clk 域的信号�?mux�?
// TD 会报 "sd_card_clk -> video_clk  SWNS -0.091ns"（mux 的组合路径跨域）�?
// 所以先老老实实同步进 video_clk，再�?mux —�?跨域只剩单比特寄存器路径�?
reg [1:0] disp_idx_v0, disp_idx_v1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        disp_idx_v0 <= 2'd0;
        disp_idx_v1 <= 2'd0;
    end
    else begin
        disp_idx_v0 <= disp_buf_idx;
        disp_idx_v1 <= disp_idx_v0;
    end
end

// 转场触发源：有真图时�?disp_buf_idx，没图时看演示序号（两者都已在 video_clk 域）
wire [1:0] fade_idx = dvalid_v1 ? disp_idx_v1 : demo_idx_v1;

// ---- 无卡自检图案�? 种，可用 KEY2 手动切换（纯比较�?移位实现，无除法/�?BRAM�?---
video_test_pattern #(
    .H_ACTIVE (640),
    .V_ACTIVE (480),
    .BAR_W    (80)
) u_video_test_pattern (
    .I_clk (video_clk),
    .I_rst (rst_all),
    .I_vs  (vs),
    .I_de  (de),
    .I_idx (demo_idx_v1),
    .O_rgb (vout_data_pat)
);

// ===================== 加分项：换图淡入转场 + OSD 字幕叠加 =====================
//  通路：vout_data -> [video_fade] -> [osd_overlay] -> vout_data_osd -> [video_rgb_to_axis]
//  两个模块都是纯组合像素通路 + 视频时钟域状态机，不�?SD/SDRAM/乒乓逻辑�?
wire [23:0] vout_data_fade;
wire [23:0] vout_data_osd;
wire [15:0] sd_diag;            // 调试：SD/图片通路�?16 位内部状�?
wire [23:0] vout_data_diag;     // 调试：叠了状态色块的画面
wire [7:0]  sd_diag2;           // 调试：扫描扇区号�?8 �?

// ===== 调试：屏幕左上角 16 位状态色块（定位 SD 图不出来的问题用�?====
// 定位完整后把这一段和 osd 实例�?.I_rgb 改回 vout_data_fade 即可�?
// 6 位十六进制：�?1~4 位是 sd_card_clk 域状态，�?5~6 位是 video_clk 域状态�?
//   digit5 = {dvalid_v1, disp_idx_v1[1:0], demo_idx_v1[1]}  (8 = 显示域认�?有图")
//   digit6 = {demo_idx_v1[0], 3'b000}
wire [7:0]  diag_vid  = { dvalid_v1, disp_idx_v1[1:0], 1'b0,   // digit5: 8=dvideo 域有�? �?�?缓冲�?
                          demo_idx_v1, 2'b00 };                  // digit6: 0/4/8 = 自检图案 0..3
// 虚拟卡模式下直接�?当前图的宽、高"打在屏幕上：�?4 �?= 宽，�?4 �?= �?
wire [31:0] diag_word = vsrc_en ? { vsrc_w, vsrc_h }
                               : { diag_vid, sd_diag[15:0], sd_diag2 };

// 只在"出故�?时才把这些十六进制字打到屏幕上：
// sd_diag[15]=sd_init_done、[14]=scan_done、[13]=display_valid�?
// 三个都成立说明图片通路已经完全正常 -> 关掉，答辩时画面干净�?
// 任意一个不成立 -> 自动出现，方便继续定位�?
wire diag_hide = sd_diag[15] & sd_diag[14] & sd_diag[13] & ~vsrc_en;

diag_overlay u_diag_overlay (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_vs       (vs),
    .I_de       (de),
    .I_rgb      (vout_data_fade),
    .I_code     (diag_word),
    .I_en       (~diag_hide),
    .O_rgb      (vout_data_diag)
);

// ---- 转场：检�?disp_buf_idx 变化，新图从黑场淡入（约 300ms�?---
video_fade #(
    .STYLE        (0),          // 0 = 从黑场淡入；1 = 从白场闪�?
    .ALPHA_STEP   (8'd16),
    .BLANK_FRAMES (8'd2)
) u_video_fade (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_vs       (vs),
    .I_en       (1'b1),            // 始终使能：自检图案模式下也要能看到淡入转场
    .I_buf_idx  (fade_idx),        // 有真图看 disp_buf_idx，没图看演示序号
    .I_rgb      (vout_data),
    .O_rgb      (vout_data_fade)
);

// ---- OSD：屏幕底�?3 行（0 行走时时�?+ 2 行中文字幕）----
wire [6:0]  osd_cell;
wire [7:0]  osd_char;
wire        osd_char_en;
wire [23:0] osd_char_rgb;
wire [63:0] osd_clk_str;

osd_clock_gen #(
    .CLK_FREQ_HZ (25_175_000)
) u_osd_clock_gen (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_restart  (1'b0),             // 开机即开始走字（自检图案下也能看到数字在动）
    .O_str      (osd_clk_str)
);

osd_text_rom u_osd_text_rom (
    .I_cell     (osd_cell),
    .I_clk_str  (osd_clk_str),
    .I_per_char (osd_per_char),
    .I_mode_char0(osd_mode_char0),
    .I_mode_char1(osd_mode_char1),
    .O_char     (osd_char),
    .O_char_en  (osd_char_en),
    .O_char_rgb (osd_char_rgb)
);

osd_overlay #(
    .SCALE       (1),
    .SCALE_SH    (0),
    .LOG2_CELL_W (5),
    .LOG2_CELL_H (5),
    .COL_BITS    (5),
    .ROW_BITS    (2),
    .AREA_X      (0),
    .AREA_Y      (352),
    .COLS        (20),
    .ROWS        (4),
    .EN_BAR      (1),
    .BAR_SHIFT   (1)
) u_osd_overlay (
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_vs       (vs),
    .I_de       (de),
    .I_rgb      (vout_data_diag),
    .O_cell     (osd_cell),
    .I_char     (osd_char),
    .I_char_en  (osd_char_en),
    .I_char_rgb (osd_char_rgb),
    .O_rgb      (vout_data_osd)
);
// ===================== 加分项结�?=====================

// ===================== 音频可视化：右下�?8 段电平柱 =====================
//  通路再往后接一级：[osd_overlay] -> [audio_vu_meter] -> vout_data_final -> HDMI
//  音高决定哪根柱子在亮（低音在左、高音在右），电平决定柱子高度�?
//  即使 AUDIO_ENABLE=0 静音，旋律仍在内部跑，所以静音时柱子照样在跳
//  —�?自习室演示时"看得见、听不见"�?
wire [23:0] vout_data_final;
wire        vout_de_final;
wire        vout_vs_final;

// OSD -> VU 之间插一级寄存器（时序收敛用�?
//   osd_overlay 的输出是一大团组合逻辑（行/列比�?+ 字模选�?+ 颜色 mux），
//   直接连到 VU 模块的首级寄存器时，25MHz(40ns) 下实�?SWNS -1.382ns�?
//   这里打一拍，�?长组合锥 + 长走�?拆成两段；de/vs 同步延后一拍，
//   像素流与使能的相对关系完全不变�?
reg [23:0] osd_rgb_r;
reg        osd_de_r;
reg        osd_vs_r;

always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        osd_rgb_r <= 24'd0;
        osd_de_r  <= 1'b0;
        osd_vs_r  <= 1'b0;
    end else begin
        osd_rgb_r <= vout_data_osd;
        osd_de_r  <= de;
        osd_vs_r  <= vs;
    end
end

audio_vu_meter #(
    .X0       (384),        // 右下角，避开�?3 行「轮播周�?N 秒」文�?
    .Y_BOTTOM (480),
    .SLOT_W   (32),
    .BAR_W    (26),
    .MAX_H    (64),
    .DECAY    (2)
) u_audio_vu_meter (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .I_vs    (osd_vs_r),
    .I_de    (osd_de_r),
    .I_note  (audio_note),
    .I_level (audio_level),
    .I_rgb   (osd_rgb_r),
    .O_rgb   (vout_data_final),
    .O_de    (vout_de_final),   // 本模块内部打了一拍，de/vs 必须跟着延后
    .O_vs    (vout_vs_final)
);

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (BUF2_ADDR),
    .read_addr_3       (BUF3_ADDR),
    .read_addr_index   (disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (BUF2_ADDR),
    .write_addr_3      (BUF3_ADDR),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== RGB/DE �?AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vout_vs_final),
    .I_de          (vout_de_final),
    .I_rgb         (vout_data_final),   // 改：原为 vout_data，接�?OSD+VU 叠加后的像素
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一�?EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 自研 HDMI 音频（差异化方案�?====================
// 【为什么自研】�?026选题指南》硬性规定："提交作品中的整体实现架构、音频和
//   图片处理方法不能和参考实例相�?。官�?lab_ex5_i2s 的做法是�?
//       内部方波音阶发生�?-> I2S(BCLK/LRCK/DOUT) -> I2S_receiver 解回 PCM -> ACR
//   本设计的做法是：
//       每张图片对应一首旋律，正弦查表 + 起落包络�?*直接**产生 PCM，不�?I2S
//   既满�?音频处理方法与参考实例不�?，又顺带实现了扩展要�?1.4(2)"音视频联�?
//   （换�?= 换曲）�?

// �?曲目选择统一�?fade_idx（已经在 video_clk 域里同步好了）：
//      有卡在放�?-> fade_idx = disp_idx_v1（当前显示缓冲区序号，换�?= 换曲�?
//      没卡（演示模式） -> fade_idx = demo_idx_v1（自检图案序号，KEY2 换图�?= 换曲�?
//   之前的写法是只同�?disp_buf_idx，导致没卡时 disp_buf_idx 恒为 0�?
//   永远只听得到�?0 首曲�?—�?换成 fade_idx 后，没卡也能演示"换画�?= 换旋�?�?
//   也就是能演示扩展要求 1.4(2)「音视频联动」�?
wire        audio_sample_valid;
wire [23:0] audio_left_pcm;
wire [23:0] audio_right_pcm;
wire [4:0]  audio_note;        // 当前音高 0..23（给音频可视化）
wire [5:0]  audio_level;       // 峰值保持电�?0..63（给音频可视化）
wire        acr_pkt_valid;
wire [19:0] acr_pkt_cts;
wire [19:0] acr_pkt_n;

// ===== 一键静音开关（2026-10-17 用户要求�?====
// AUDIO_ENABLE = 1 : 正常出声
// AUDIO_ENABLE = 0 : 静音（自习室/考场/演示需要安静时�?
// 改这一处、重新编译即可，不需要动其它任何代码�?
parameter AUDIO_ENABLE = 1;

wire        audio_out_valid     = AUDIO_ENABLE ? audio_sample_valid : 1'b0;
wire [23:0] audio_out_left      = AUDIO_ENABLE ? audio_left_pcm     : 24'd0;
wire [23:0] audio_out_right     = AUDIO_ENABLE ? audio_right_pcm    : 24'd0;
wire        audio_out_acr_valid = AUDIO_ENABLE ? acr_pkt_valid      : 1'b0;
wire [19:0] audio_out_acr_cts   = AUDIO_ENABLE ? acr_pkt_cts        : 20'd0;
wire [19:0] audio_out_acr_n     = AUDIO_ENABLE ? acr_pkt_n          : 20'd0;

// video_clk = 25.000 MHz（见 IP/video_pll.ipc �?C0 输出），48 kHz 由分数分频精确得�?
audio_melody_player #(
    .CLK_FREQ_HZ     (25_000_000),
    .SAMPLE_RATE_HZ  (48_000),
    .DUR_SHIFT       (11),
    .RAMP_SAMPLES    (32)
) u_audio_melody_player (
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_track            (fade_idx),
    .O_audio_valid      (audio_sample_valid),
    .O_audio_left_data  (audio_left_pcm),
    .O_audio_right_data (audio_right_pcm),
    .O_note             (audio_note),
    .O_level            (audio_level)
);

audio_acr_gen #(
    .ACR_N_VALUE (6144),
    .SAMP_DIV    (48)
) u_audio_acr_gen (
    .I_clk        (video_clk),
    .I_rst        (rst_all),
    .I_audio_valid(audio_sample_valid),
    .O_acr_valid  (acr_pkt_valid),
    .O_acr_cts    (acr_pkt_cts),
    .O_acr_n      (acr_pkt_n)
);

// ===================== HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_out_valid),
    .I_audio_left_data  (audio_out_left),
    .I_audio_right_data (audio_out_right),
    .I_acr_valid        (audio_out_acr_valid),
    .I_acr_cts          (audio_out_acr_cts),
    .I_acr_n            (audio_out_acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
