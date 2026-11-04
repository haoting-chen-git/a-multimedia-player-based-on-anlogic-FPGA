
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input  [2:0]                period_sel,     // 轮播周期选择：0=1s 1=2s 2=4s 3=8s
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output                      o_auto_en,      // 1 = 自动轮播开（送 OSD 显示"自动/手动"）
    output                      o_preload_en,   // 1 = 还在往 SDRAM 里预载图片（OSD 显示"预载"）
    output      [3:0]           state_code,
    output      [15:0]          o_diag,         // 调试用：16 位内部状态（屏幕十六进制）
    output      [7:0]           o_diag2,        // 调试用：扫描扇区号低 8 位（判断扫描是否前进）

    // ---- 虚拟 SD 卡（无卡演示：内置 BMP 图源，验证图片自适应缩放）----
    input                       vsrc_en,        // 1 = 用内部虚拟图源，0 = 透传真卡
    output      [15:0]          o_vsrc_w,       // 虚拟图当前宽（送屏幕显示）
    output      [15:0]          o_vsrc_h,       // 虚拟图当前高（送屏幕显示）
    output      [2:0]           o_vsrc_idx,     // 当前尺寸编号 0..7

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO
);

wire key_next_press;
wire key_auto_press;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;

// ---- 虚拟图源 vs 真实 SD 卡 的选择（vsrc_en=1 时用内部假卡）----
wire [7:0]       sd_sec_read_data_raw;
wire             sd_sec_read_data_valid_raw;
wire             sd_sec_read_end_raw;
wire             sd_init_done_raw;
wire [15:0]      vsrc_w;
wire [15:0]      vsrc_h;
wire [2:0]       vsrc_idx;
wire [3:0]       state_code_i;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire [15:0]      bmp_img_width;      // 原始图宽（缩放器输入）
wire [15:0]      bmp_img_height;     // 原始图高（缩放器输入）
wire             bmp_hdr_ready;      // 头部解析完成脉冲
wire             scaler_busy;        // 缩放器忙
wire [4:0]       sd_fsm_state;       // 调试：SD 初始化状态机（0空闲/1CMD0/2CMD8/3CMD55/4CMD41/18CMD16/17等待/5CMD17/6READ）
wire             scaler_wr_en;       // 缩放后写帧缓存
wire [23:0]      scaler_data;
wire             scaler_done;
wire             sd_init_done = vsrc_en ? 1'b1 : sd_init_done_raw;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg              buf_armed;      // 写帧缓存已受理本帧写请求（从 write_req_ack 起保持到下次装载）
reg [31:0]       load_sector;
reg              scan_kicked;
reg [31:0]       rescan_cnt;      // 扫不到图时的重扫计时
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号（= 目标槽号）
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              load_busy;
reg [31:0]       load_timeout_cnt;

// ---- 4 槽帧缓存（v33）----
reg  [3:0]       loaded_mask;      // 每个槽是否已经装好（1 = 装好，不再读卡）
reg  [1:0]       pend_slot;        // 用户点名要显示、但还没装好的槽
reg              pend_valid;
reg              load_seen_busy;   // 本次装载中是否已见到 bmp_read 离开空闲
reg  [31:0]      stall_cnt;        // 距上一次扇区读出的拍数（死锁看门狗）
reg  [7:0]       fail_cnt;         // 每个槽 2 位的失败计数（==3 表示本次放弃该槽）
reg  [3:0]       abort_cnt;        // 累计放弃次数（只做调试显示）
reg              wr_seen;          // 本次装载中是否真的往帧缓存写过像素

reg              load_abort;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
reg              write_done_seen;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
reg  [31:0] auto_reload;
wire [1:0] next_from_current;

//--------------------------------------------------------------------------
// 图片自适应缩放（《作品简介》承诺"具备自适应缩放能力"）
//   bmp_read 送出的原始像素流 -> bmp_scaler 重采样成固定 640x480 -> 写帧缓存。
//   这样任意分辨率的 BMP 都能正确满屏显示，且"图片处理方法"与参考实例
//   （原图直通、只认 640x480）产生实质差异。
//   scaler_rst 用 load_start_pulse：每次开始加载新图就把缩放器状态清零，
//   避免上一张图残留的行计数影响这一张。
//--------------------------------------------------------------------------
//  ★ 图片自适应缩放（bmp_scaler v2）：官方原版把像素直接写进 640x480 帧缓存，
//  所以只认 640x480 的图。现在先把像素过一遍缩放器：
//    原始宽高 img_width/img_height（bmp_read 从文件头解析）
//      -> 等比缩放（保持比例 + 黑边居中）
//      -> 640x480 整帧输出，一帧正好 307200 像素
//  缩放器 v2 的除法全部串行化（先算垂直 1 拍，再算水平），并把"决策时刻"和
//  "发射时刻"错开 1 拍，所以输出恒为 1 像素/输入像素、不丢像素、不需要深 FIFO。
//  start 只用 bmp_hdr_ready：宽高一定要在【文件头解析完之后】才有效。
//  早期版本写成 load_start_pulse | bmp_hdr_ready，load_start_pulse 那一拍
//  cfg 还是上一张（首张是 0/0），除法商为 0 会让装载阶段空转，虽然紧接着
//  hdr_ready 会重来一次，但中间把状态机搅乱了。现在只认 hdr_ready 一路。
bmp_scaler #(
    .DW               (640),
    .DH               (480),
    .SW_MAX           (2048)
) u_bmp_scaler (
    .clk              (clk),
    .rst              (rst),
    .start            (bmp_hdr_ready),
    .i_arm            (buf_armed),
    .cfg_w            (bmp_img_width),
    .cfg_h            (bmp_img_height),
    .s_valid          (bmp_data_wr_en),
    .s_data           (bmp_data),
    .m_valid          (scaler_wr_en),
    .m_data           (scaler_data),
    .m_done           (scaler_done),
    .o_busy           (scaler_busy),
    .o_ovf            ()
);

// ---------------------------------------------------------------------------
// 调试：屏幕上显示 4 个十六进制大字（高位在左），由 diag_overlay 绘制
//   第 1 位（最左）= 标志半字节 bit3..0：
//                     bit3 sd_init_done   卡初始化完成
//                     bit2 scan_done      目录扫描结束
//                     bit1 display_valid  有图可显示
//                     bit0 load_busy      正在加载一张图
//   第 2 位        = 扫到几张图（0..7）
//   第 3、4 位     = SD 状态机状态（00..12）
//                     00 空闲 / 01 CMD0 / 02 CMD8 / 03 CMD55 / 04 CMD41
//                     05 CMD17 / 06 READ / 12 CMD16 / 11 等待读写
// 例：8 0 11 -> 初始化完成、0 张图、状态 17(0x11) 表示已就绪在等读写命令
//    8 0 01 -> 初始化完成、0 张图、状态 1(CMD0) 表示…（不会出现，初始化完成前状态不会离开前 5 个）
//     0 0 01 -> 卡没插/没应答，停在 CMD0
// ---------------------------------------------------------------------------
// 先在本时钟域打一拍再出去：o_diag 的接收方（diag_overlay）跑在 video_clk 上，
// 组合输出会让 sd_card_clk -> video_clk 成为一条 22 端点的真实时序路径
// （曾报 -1.158ns）。寄存一拍后跨域只剩"寄存器到寄存器"，实测收敛。
reg [15:0] diag_q;
always @(posedge clk) begin
    // 位定义（第 3~8 位）：
    //   d3 = {初始化完成, 扫描结束, 有图可显示, 正在加载}
    //   d4 = {0, 找到几张图}
    //   d5 = bmp_read 扫描/加载状态机编号（1 空闲 2 读扇区 3 读头 4 等数据 5 灌像素）
    //   d6 = SD 底层状态机编号（0 空闲 1 CMD0 2 CMD8 3 CMD55 4 CMD41 5 CMD17 6 读 7 CMD24 8 写）
    // 第 7、8 位 = 当前扫描到的扇区号低 8 位（看扫描有没有在前进）
    diag_q <= { sd_init_done, scan_done, display_valid, load_busy,
                1'b0, img_found_count,
                state_code_i, sd_fsm_state[3:0] };
end
assign o_diag = diag_q;

wire [31:0] scan_sector_dbg;
wire [7:0]  sd_r1_s;                 // 调试：SD 最近一次命令应答字节（保留，未上屏）
// v34：第 7、8 位改为「哪些槽已装好 + 累计放弃次数」，比扫描扇区更能定位卡死
//   高 4 位 = loaded_mask（F = 4 个槽都装好），低 4 位 = abort_cnt（放弃次数）
assign o_diag2 = { loaded_mask, abort_cnt };

// v34 失败重试：某个槽连续 3 次装载失败就本轮跳过它（但绝不标成"已装好"）
// （need_mask 必须在使用之前声明，否则 ModelSim 会先建隐式线网，后面 assign 就报 2388）
wire [3:0] need_mask;
wire [3:0] fail_mask;
assign fail_mask = { (fail_cnt[7:6] == 2'd3), (fail_cnt[5:4] == 2'd3),
                     (fail_cnt[3:2] == 2'd3), (fail_cnt[1:0] == 2'd3) };
wire preload_go;
assign preload_go = scan_done && !load_busy && bmp_ready &&
                    (img_found_count != 3'd0) &&
                    ((loaded_mask & need_mask) != need_mask);
wire all_tried;
assign all_tried = (((loaded_mask | fail_mask) & need_mask) == need_mask);


assign write_en   = scaler_wr_en;
assign write_data = {scaler_data[23:16], scaler_data[15:8], scaler_data[7:0], 8'b0};

// ===== 轮播周期可配置（基础要求 1.3(3)）=====
// 官方原版是常数 1 秒：assign auto_tick = (auto_cnt == (CLK_FREQ_HZ - 1));
// 现改为由 period_sel 选择 1/2/4/8 秒
always @(*) begin
    case (period_sel)
        3'd0:    auto_reload = CLK_FREQ_HZ * 1;
        3'd1:    auto_reload = CLK_FREQ_HZ * 2;
        3'd2:    auto_reload = CLK_FREQ_HZ * 4;
        default: auto_reload = CLK_FREQ_HZ * 8;
    endcase
end

// 用 >= 而不是 ==：运行中切换周期时 auto_cnt 可能已经超过新的 reload 值，
// 用 == 会永远不匹配、轮播直接卡死
assign auto_tick  = (auto_cnt >= (auto_reload - 32'd1));
assign next_from_current = next_index_limited(img_idx, img_found_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign state_code = state_code_i;
assign o_auto_en  = auto_play_en;
// 还在预载（还有槽没装好）→ OSD 显示"预载"，让用户知道此时按键要等
assign o_preload_en = (need_mask != 4'b0) && ((loaded_mask & need_mask) != need_mask);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (50)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

// 本次扫描真正需要装载的槽（扫到几张就装几张，最多 4 张）
function [3:0] need_mask_f;
    input [2:0] cnt;
    begin
        case (cnt)
            3'd0:    need_mask_f = 4'b0000;
            3'd1:    need_mask_f = 4'b0001;
            3'd2:    need_mask_f = 4'b0011;
            3'd3:    need_mask_f = 4'b0111;
            default: need_mask_f = 4'b1111;
        endcase
    end
endfunction

// 按 0..3 顺序找下一个还没装好的槽
function [1:0] next_unloaded_f;
    input [3:0] mask;
    input [3:0] need;
    begin
        if (need[0] && !mask[0])      next_unloaded_f = 2'd0;
        else if (need[1] && !mask[1]) next_unloaded_f = 2'd1;
        else if (need[2] && !mask[2]) next_unloaded_f = 2'd2;
        else                          next_unloaded_f = 2'd3;
    end
endfunction

assign need_mask = need_mask_f(img_found_count);

// ---------------------------------------------------------------------------
//  ★ 4 槽帧缓存（v33）
//  以前只有 2 块 640x480 缓冲，每换一张图都要重新走一遍 TF 卡（1920x1080 那张
//  要读 6.2MB，硬件上 2 秒以上），所以"切图慢、周期对不上"。现在 SDRAM 里开 4 个
//  槽（每个 307200 字），每张图只从卡上读一次，装好之后切图只是把显示基址换成
//  另一个槽 —— 瞬间完成，跟卡速彻底解耦。
//  上电流程：扫描目录 → 依次把 4 张图装进槽 0..3（后台预载，第一张装好就先显示，
//            其余在后台慢慢装，屏幕上显示"预载"）→ 全部装好后开始自动轮播。
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= vsrc_en ? 1'b1 : 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        load_timeout_cnt      <= 32'd0;
        buf_armed             <= 1'b0;
        load_abort            <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
        loaded_mask           <= 4'd0;
        pend_slot             <= 2'd0;
        pend_valid            <= 1'b0;
        load_seen_busy        <= 1'b0;
        stall_cnt             <= 32'd0;
        fail_cnt              <= 8'd0;
        abort_cnt             <= 4'd0;
        wr_seen               <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        load_abort       <= 1'b0;
        // 写帧缓存受理写请求的窗口：每次装载开始时清零，收到 write_req_ack 置 1。
        if (load_start_pulse)       buf_armed <= 1'b0;
        else if (write_req_ack)     buf_armed <= 1'b1;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            load_abort            <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            display_valid         <= 1'b0;
            loaded_mask           <= 4'd0;
            pend_valid            <= 1'b0;
            load_seen_busy        <= 1'b0;
            stall_cnt             <= 32'd0;
            fail_cnt              <= 8'd0;
            abort_cnt             <= 4'd0;
            wr_seen               <= 1'b0;
        end else begin
            // ---------- 扫描结果入表 ----------
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // ---------- 装载进度监测 ----------
            // bmp_ready 在 load_start 之后还会保持 1 拍（bmp_read 要到再下一拍才离开
            // 空闲），旧代码在这时就置 source_done_seen，"源图送完"是假的。
            // 现在要求先看到 bmp_ready 掉下去，再看到它回来，才算真的送完。
            if (load_busy) begin
                if (!bmp_ready)
                    load_seen_busy <= 1'b1;
                if (load_seen_busy && bmp_ready)
                    source_done_seen <= 1'b1;
                if (write_finish_pulse)
                    write_done_seen <= 1'b1;
                if (write_en)
                    wr_seen <= 1'b1;

                if (sd_sec_read || sd_sec_read_end)
                    stall_cnt <= 32'd0;
                else if (stall_cnt < (CLK_FREQ_HZ * 3))
                    stall_cnt <= stall_cnt + 32'd1;
            end else begin
                load_seen_busy <= 1'b0;
                stall_cnt      <= 32'd0;
            end

            if (load_busy && source_done_seen &&
                (write_done_seen || (wr_seen && (stall_cnt >= (CLK_FREQ_HZ / 2))))) begin
                // 源图送完 → 这个槽装好了。
                // 正常路径等 write_finish_toggle；万一看不到那个握手（跨时钟域丢脉冲），
                // 只要确认写过像素、且 0.5 秒没有新扇区读，也算装好，绝不因此卡死。
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_seen_busy        <= 1'b0;
                wr_seen               <= 1'b0;
                loaded_mask[load_idx] <= 1'b1;

                if (!display_valid) begin
                    // 第一张装好就显示
                    disp_buf_idx          <= load_idx;
                    img_idx               <= load_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                    auto_play_en          <= 1'b1;   // 默认就自动轮播
                    auto_cnt              <= 32'd0;
                end else if (pend_valid && (pend_slot == load_idx)) begin
                    // 用户点名要、但当时还没装好的那张，现在装好了 → 切过去
                    disp_buf_idx <= load_idx;
                    img_idx      <= load_idx;
                    pend_valid   <= 1'b0;
                end
            end
            else if (load_busy && (stall_cnt >= (CLK_FREQ_HZ * 2))) begin
                // 2 秒一个扇区都没读出来 → 认定为读卡死锁（坏图/截断文件）。
                // ★只记失败次数，绝不把槽标成"已装好"（v33 就是因为这个永久卡死）：
                //   标成已装好的槽再也不会被预载，可 display_valid 还是 0，屏幕永远停在自检图案。
                load_busy        <= 1'b0;
                load_abort       <= 1'b1;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_seen_busy   <= 1'b0;
                wr_seen          <= 1'b0;
                stall_cnt        <= 32'd0;
                if (fail_cnt[(load_idx * 2) +: 2] != 2'd3)
                    fail_cnt[(load_idx * 2) +: 2] <= fail_cnt[(load_idx * 2) +: 2] + 2'd1;
                if (abort_cnt != 4'hF)
                    abort_cnt <= abort_cnt + 4'd1;
            end

            // ---------- 一张图都没扫到 → 每 2 秒重扫 ----------
            if (scan_done && (img_found_count == 3'd0)) begin
                if (rescan_cnt >= (CLK_FREQ_HZ * 2)) begin
                    rescan_cnt  <= 32'd0;
                    scan_kicked <= 1'b0;
                end else begin
                    rescan_cnt <= rescan_cnt + 32'd1;
                end
            end else begin
                rescan_cnt <= 32'd0;
            end

            // ---------- 自动轮播计时（切图是瞬时的，不用再等 load_busy）----------
            if (scan_done && auto_play_en && display_valid && (img_found_count > 3'd1)) begin
                if (!auto_tick)
                    auto_cnt <= auto_cnt + 32'd1;
            end else begin
                auto_cnt <= 32'd0;
            end

            // ---------- 切图请求：目标槽装好了就立刻切 ----------
            // 手动按键优先于自动轮播（两个都在这一条里，按下按键就把周期清零）
            if (scan_done && display_valid && (img_found_count > 3'd1) &&
                (key_next_press || (auto_play_en && auto_tick))) begin
                if (loaded_mask[next_from_current]) begin
                    disp_buf_idx <= next_from_current;
                    img_idx      <= next_from_current;
                    pend_valid   <= 1'b0;
                end else begin
                    pend_slot  <= next_from_current;
                    pend_valid <= 1'b1;
                end
                auto_cnt <= 32'd0;
            end

            // ---------- 自动 / 手动切换（v33 漏掉的一块，v34 补回）----------
            // 用户口径：KEY1 = 自动/手动。OSD 第 3 行末尾会显示「自动」或「手动」。
            if (key_auto_press && scan_done && ((img_found_count > 3'd1) || vsrc_en)) begin
                auto_play_en <= ~auto_play_en;
                auto_cnt     <= 32'd0;
            end

            // ---------- 一轮都试过还是没装好 → 清失败计数，从头再来（永不死锁）----------
            if (scan_done && !load_busy && (img_found_count != 3'd0) &&
                all_tried && ((loaded_mask & need_mask) != need_mask)) begin
                fail_cnt  <= 8'd0;
                if (abort_cnt != 4'h0)
                    abort_cnt <= 4'h0;
            end

            // ---------- 后台预载：把还没装过的槽依次装好（每张图一辈子只读一次卡）----------
            if (preload_go && !all_tried) begin
                load_idx         <= next_unloaded_f(loaded_mask | fail_mask, need_mask);
                load_sector      <= sector_lut(next_unloaded_f(loaded_mask | fail_mask, need_mask));
                write_buf_idx    <= next_unloaded_f(loaded_mask | fail_mask, need_mask);
                load_start_pulse <= 1'b1;
                load_busy        <= 1'b1;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                load_seen_busy   <= 1'b0;
                wr_seen          <= 1'b0;
                stall_cnt        <= 32'd0;
            end

            // ---------- 上电后发起第一次扫描 ----------
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                loaded_mask           <= 4'd0;
                fail_cnt              <= 8'd0;
                abort_cnt             <= 4'd0;
                wr_seen               <= 1'b0;
                pend_valid            <= 1'b0;
            end
        end
    end
end


bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_abort             (load_abort),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code_i),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .img_width              (bmp_img_width),
    .img_height             (bmp_img_height),
    .hdr_ready              (bmp_hdr_ready),
    .o_scan_sector          (scan_sector_dbg),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done_raw),
    .sd_state               (sd_fsm_state),
    .sd_r1                  (sd_r1_s),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data_raw),
    .sd_sec_read_data_valid (sd_sec_read_data_valid_raw),
    .sd_sec_read_end        (sd_sec_read_end_raw),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

// ---------------------------------------------------------------------------
//  虚拟 SD 卡实例 + 二选一
//  vsrc_en = 1 时：bmp_read 看到的是内部生成的假 BMP 流（不走真卡）
//  vsrc_en = 0 时：完全透传真实卡
// ---------------------------------------------------------------------------
wire [7:0] vsrc_byte;
wire       vsrc_dvalid;
wire       vsrc_dend;

vsrc_sd u_vsrc_sd(
    .clk                    (clk),
    .rst                    (rst),
    .en                     (vsrc_en),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (vsrc_byte),
    .sd_sec_read_data_valid (vsrc_dvalid),
    .sd_sec_read_end        (vsrc_dend),
    .o_w                    (vsrc_w),
    .o_h                    (vsrc_h),
    .o_idx                  (vsrc_idx)
);

assign o_vsrc_w   = vsrc_w;
assign o_vsrc_h   = vsrc_h;
assign o_vsrc_idx = vsrc_idx;

assign sd_sec_read_data       = vsrc_en ? vsrc_byte       : sd_sec_read_data_raw;
assign sd_sec_read_data_valid = vsrc_en ? vsrc_dvalid     : sd_sec_read_data_valid_raw;
assign sd_sec_read_end        = vsrc_en ? vsrc_dend       : sd_sec_read_end_raw;

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
