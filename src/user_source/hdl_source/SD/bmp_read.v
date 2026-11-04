module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input                       load_abort,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,      // 现在表示"允许的最大宽"（缩放器决定真正上限）
    input  [15:0]               bmp_height,     // 现在表示"允许的最大高"
    output wire [15:0]          img_width,      // 从文件头解析出的原始宽
    output wire [15:0]          img_height,     // 从文件头解析出的原始高
    output reg                  hdr_ready,      // 头部解析完成，1 拍；缩放器据此初始化
    output reg  [31:0]          o_scan_sector,  // 调试用：当前扫描到的扇区号（已寄存）

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;

reg [2:0]  state;
reg [9:0]  rd_cnt;

// 扇区读超时保护：SD 卡偶尔不回 sd_sec_read_end（CRC 错、卡抽风、掉卡），
// 早期版本会永久卡在 ST_SCAN 里——屏幕永远停在自检图案且没有任何提示。
// 现在每个状态最多等 2ms（200_000 拍 @100MHz），超时就跳到下一个扇区；
// 连续 64 次超时说明卡/接线有问题，直接结束扫描（已找到的图照样能用）。
reg [23:0] tmo_cnt;
reg [6:0]  tmo_fail;
wire       tmo_hit = (tmo_cnt == 24'd200_000);

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;

wire header_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

// ---------------------------------------------------------------------------
// 引导扇区（FAT BPB）解析：把"逐扇区暴力扫"换成"逐簇扫"
//
// 为什么必须这么做：卡的容量是 14.8GB，按 32KB 簇算有 45 万个簇起点。
// 逐扇区（每扇区约 0.9ms）扫 16MB 就要 30 秒，扫到能覆盖这些图片的
// 256MB 要 8 分钟 —— 用户原话是"依旧没有扫到 4 张"。
// 而**任何文件的第一个字节必然落在某个簇起点上**（文件数据总是从簇首开始），
// 所以只扫"簇起点"就够了：扇区数立刻少 SecPerClus 倍（32KB 簇 = 64 倍）。
// 簇起点 = 数据区起始扇区 + k*SecPerClus，
// 数据区起始 = RsvdSecCnt + NumFATs*FATSz —— 这三个数只能从扇区 0 读出来。
//
// 解析失败（不是 FAT32、SecPerClus 非法等）时自动退回"逐扇区 + 原来的 16MB 上限"，
// 也就是退化成旧行为，不会比原来更差。
// ---------------------------------------------------------------------------
reg [9:0]  s0_cnt;          // 扇区 0 的字节序号
reg [15:0] bpb_bps;         // BYTES_PER_SECTOR (偏移 11)
reg [7:0]  bpb_spc;         // SEC_PER_CLUSTER  (偏移 13)
reg [15:0] bpb_rsvd;        // RESERVED_SEC_CNT (偏移 14)
reg [7:0]  bpb_nfat;        // NUM_FAT          (偏移 16)
reg [15:0] bpb_fat16;       // FAT_SZ_16        (偏移 22)
reg [31:0] bpb_fat32;       // FAT_SZ_32        (偏移 36)
reg [31:0] scan_stride;     // 1 = 逐扇区；64 = 逐簇
reg [31:0] scan_limit;      // 扫描上限（扇区号）

wire [31:0] bpb_fatsz  = (bpb_fat32 != 32'd0) ? bpb_fat32 : {16'd0, bpb_fat16};
wire [31:0] bpb_nf2    = (bpb_nfat == 8'd2) ? {bpb_fatsz[30:0], 1'b0} : bpb_fatsz;
wire [31:0] bpb_data0  = {16'd0, bpb_rsvd} + bpb_nf2;
wire        bpb_ok     = (bpb_bps == 16'd512) && (bpb_spc != 8'd0) &&
                         (bpb_spc <= 8'd128) && (bpb_fatsz != 32'd0) &&
                         (bpb_data0 != 32'd0) && (bpb_data0 < 32'd1048576);
// 注意：SEC_PER_CLUSTER 存的就是"每簇多少个扇区"（32KB 簇 = 64），
// 步长直接等于它，千万不要写成 (1<<spc) —— 那会变成移 64 位得 0。
wire [31:0] bpb_stride = bpb_ok ? {24'd0, bpb_spc} : 32'd1;

// 再打一拍：bpb_data0 是 "rsvd + 2*fatsz" 的 32 位加法 + 比较，
// 直接拿它去驱动 scan_sector / sd_sec_read_addr 会形成
// bpb_fat32 -> ADDER x4 -> sd_sec_read_addr 的 12.8ns 组合链
// （上板前实测 sd_card_clk -3.055ns / 68 端点）。
// 这些值在第 40 个字节就稳定了，而 sd_sec_read_end 要等 512 字节搬完
// （至少 470 拍以后）才来，所以寄存一拍完全来得及。
reg        bpb_ok_r;
reg [31:0] bpb_data0_r;
reg [7:0]  bpb_spc_r;
reg [31:0] bpb_nf2_r;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bpb_ok_r    <= 1'b0;
        bpb_data0_r <= 32'd0;
        bpb_spc_r   <= 8'd0;
        bpb_nf2_r   <= 32'd0;
    end else begin
        // 两级流水：乘 2（左移）、加法、32 位比较分三拍做，
        // 单独一拍做不完（实测 bpb_fat32 -> ADDERx4 -> data0 要 10.5ns）。
        bpb_nf2_r   <= bpb_nf2;
        bpb_data0_r <= {16'd0, bpb_rsvd} + bpb_nf2_r;
        bpb_ok_r    <= (bpb_bps  == 16'd512) && (bpb_spc != 8'd0) &&
                       (bpb_spc  <= 8'd128) && (bpb_fatsz != 32'd0) &&
                       (bpb_data0_r != 32'd0) && (bpb_data0_r < 32'd1048576);
        bpb_spc_r   <= bpb_spc;
    end
end
wire [31:0] bpb_stride_r = bpb_ok_r ? {24'd0, bpb_spc_r} : 32'd1;

assign ready = (state == ST_IDLE);
// 官方原版把宽高直接和 640/480 比对，导致其它尺寸的图一律被当成"不是 BMP"跳过。
// 现在改为：只要是 BMP、24 位、不压缩、尺寸在合理范围内就接受，尺寸交给
// bmp_scaler 重采样到 640x480（这样才谈得上"自适应缩放"）。
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width[15:0]  != 16'd0) &&
                      (height[15:0] != 16'd0) &&
                      (width[15:0]  <= bmp_width) &&   // 上限，不再是"必须等于 640"
                      (height[15:0] <= bmp_height) &&
                      (width[1:0]   == 2'd0) &&        // 宽必须是 4 的倍数
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);
// 宽必须 4 的倍数：BMP 每行按 4 字节对齐，行尾可能有 1~3 个填充字节，
// 而下面按 file_len 连续取字节的读出逻辑不处理填充；宽是 4 的倍数时
// 3*W 恰好是 4 的倍数，永远没有填充，像素流才是干净的。
// 非 4 倍数的宽会被当成"不是合法 BMP"跳过 —— 宁可跳过，也不要显示斜的图。

// 高度为负（自上而下存储）时 height[15:0] 会是一个很大的数，上面的 <= bmp_height 会挡掉，
// 这是有意为之：官方读出逻辑按"自底向上"处理，负高度会上下颠倒。
// header_match 是一条 16 位比较链（宽/高上限），如果直接驱动
// sd_sec_read_addr 的使能，sd_card_clk 会差 -0.2ns。头 34 字节收完
// 时就算好并寄存，状态机在扇区读结束（几百拍之后）再取用，
// 相当于把这条比较链从关键路径上摘掉。
//
// ★ 2026/10/17 上板 + 真实卡数据仿真定位到的 off-by-one：
//   compression 占字节 30~33，最高字节（rd_cnt==33）比原来的采样点
//   rd_cnt==32 晚一拍，导致用【上一张图的陈旧高字节】去和 0 比较。
//   卡上 pic1/pic3/pic6 的字节 33 都是 0x00，但上一扇区的字节 33 是 0x46，
//   于是 compression 被判成 0x00460000 ≠ 0 —— 三张合法 BMP 全部漏掉，
//   只有恰好前一个扇区字节 33 为 0 的那张能被认出来（上板"只认到 1 张"）。
//   改成 rd_cnt==34：此时 compression 的四个字节都已落寄存器。
reg header_match_r;
always @(posedge clk or posedge rst) begin
    if (rst)
        header_match_r <= 1'b0;
    else if (rd_cnt == 10'd34)
        header_match_r <= header_match;
end

assign img_width  = width[15:0];
assign img_height = height[15:0];

// 头部 32 字节解析完成的 1 拍脉冲（byte31 = compression 高字节）。
// 缩放器要在这之后才能拿到正确的 img_width/img_height 去做串行除法。
// 此时离首像素还有 20 多个 SD 字节（~100 拍以上），除法（最多 6 拍）绰绰有余。
always @(posedge clk or posedge rst) begin
    if (rst)
        hdr_ready <= 1'b0;
    else
        hdr_ready <= ((state == ST_LOAD_HDR) && (sd_sec_read_data_valid == 1'b1)
                      && (rd_cnt == 10'd31));
end
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);
// file_sector_count 原先是一条 file_len -> +511 -> >>9 -> +scan_sector 的组合链，
// 直接把 sd_card_clk 拖到 -0.515ns。改成"先寄存，再用"：
// file_len 在头解析时就固定了，扫描发生在很久以后，晚一拍完全没影响。
reg [31:0] file_sec_cnt_r;
always @(posedge clk or posedge rst) begin
    if (rst)
        file_sec_cnt_r <= 32'd1;
    else
        file_sec_cnt_r <= (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
end
assign file_sector_count = file_sec_cnt_r;
// 这条 32 位调试总线要喂给 diag_overlay，如果直接 assign 成组合，
// TD 会把 diag 的译码 LUT 并进 sd_sec_read_addr 的使能锥，
// 造成 sd_sec_read_addr[7] -> diag -> sec_addr_reg 的 9.75ns 跨模块链
// （实测 sd_card_clk -0.032ns）。寄存一拍后路径终止在本级触发器。
always @(posedge clk or posedge rst) begin
    if (rst) o_scan_sector <= 32'd0;
    else     o_scan_sector <= scan_sector;
end
// 命中一张图之后：只前进一个"簇步长"，不做 file_len 跳跃。
// 理由：按 file_len 跳很容易跳过紧跟着的下一张图（文件实际占用的簇数是
// 向上取整到簇边界的，file_len/512 通常不是簇的整数倍）；而"在文件自己的身体里
// 再扫几十个簇起点"的代价只有几十次读、几十毫秒，换来的是绝不漏图。
// 文件被当成图误命中？不会 —— header_match 还要求宽高/bpp/压缩方式都对。
// ★ 时序：扫描地址的加法必须"提前一拍算好"。
//   原来 next_scan_sector_if_match = scan_sector + scan_stride 是纯组合，
//   它同时驱动 sd_sec_read_addr、scan_sector 两个 32 位寄存器，还要参与
//   "> scan_limit" 的比较 —— TD 报 sd_card_clk 组 -0.028ns（8 级逻辑：
//   LUT5x4+LUT3x3+LUT4x1，源点 sd_sec_read_addr_b[25]、终点 [9]，就是这条进位链）。
//   scan_sector 在整个扇区读出期间（512 字节、几百拍）完全不变，所以把加法
//   单独打一拍寄存器，"加法"和"寄存器/比较"不再串在同一条路上。
reg [31:0] scan_sum_r;
always @(posedge clk or posedge rst) begin
    if (rst) scan_sum_r <= 32'd0;
    else     scan_sum_r <= scan_sector + scan_stride;
end
assign next_scan_sector_if_match = scan_sum_r;
assign next_scan_sector_if_miss  = scan_sum_r;

// 扇区 0（引导扇区）里按字节偏移抓 BPB 字段。只在 scan_sector==0 时工作，
// 平时 s0_cnt 一直是 0，不占额外时序。
// ★ 时序：scan_sector==0 这个 32 位或归约原来是组合的，TD 报
//   scan_sector_reg[9] -> bpb_fat32_reg[30].ce 10.14ns（-0.422ns），
//   因为 32 位"全零判断"要 2~3 级 LUT，再和 s0_cnt 的 12 路译码一起进 CE。
//   scan_sector 一旦确定就稳定几百拍，所以把"是不是 0 号扇区"提前寄存一拍。
reg scan_at_zero;
always @(posedge clk or posedge rst) begin
    if (rst) scan_at_zero <= 1'b0;
    else     scan_at_zero <= (scan_sector == 32'd0);
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s0_cnt    <= 10'd0;
        bpb_bps   <= 16'd0;
        bpb_spc   <= 8'd0;
        bpb_rsvd  <= 16'd0;
        bpb_nfat  <= 8'd0;
        bpb_fat16 <= 16'd0;
        bpb_fat32 <= 32'd0;
    end else if (scan_start) begin
        s0_cnt    <= 10'd0;
    end else if ((state == ST_SCAN) && scan_at_zero && sd_sec_read_data_valid) begin
        s0_cnt <= s0_cnt + 10'd1;
        case (s0_cnt)
            10'd11: bpb_bps[7:0]    <= sd_sec_read_data;
            10'd12: bpb_bps[15:8]   <= sd_sec_read_data;
            10'd13: bpb_spc         <= sd_sec_read_data;
            10'd14: bpb_rsvd[7:0]   <= sd_sec_read_data;
            10'd15: bpb_rsvd[15:8]  <= sd_sec_read_data;
            10'd16: bpb_nfat        <= sd_sec_read_data;
            10'd22: bpb_fat16[7:0]  <= sd_sec_read_data;
            10'd23: bpb_fat16[15:8] <= sd_sec_read_data;
            10'd36: bpb_fat32[7:0]  <= sd_sec_read_data;
            10'd37: bpb_fat32[15:8] <= sd_sec_read_data;
            10'd38: bpb_fat32[23:16]<= sd_sec_read_data;
            10'd39: bpb_fat32[31:24]<= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

// 32 位 "已读完" 比较寄存一拍：
// 原来 (bmp_len_cnt >= file_len) 直接进状态机，组合路径 10.3ns（Logic Level 9）
// 打不过 100MHz 的 sd_card_clk。这里先寄存成 len_reached，状态机只看 1 位。
// 唯一影响是标志比计数晚 1 拍；而 sd_sec_read_end 在最后一个字节之后还要
// 等整扇区搬完（几十~几百拍）才来，所以判定时刻拿到的一定是最终值。
reg len_reached;
always @(posedge clk or posedge rst) begin
    if (rst)
        len_reached <= 1'b0;
    else
        len_reached <= (bmp_len_cnt >= file_len);
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst)
        tmo_cnt <= 24'd0;
    else if (state == ST_IDLE || sd_sec_read_end || tmo_hit)
        tmo_cnt <= 24'd0;            // 每读到一个扇区就重新计时
    else
        tmo_cnt <= tmo_cnt + 24'd1;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        scan_stride       <= 32'd1;
        scan_limit        <= scan_max_sector;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        scan_stride       <= 32'd1;
        scan_limit        <= scan_max_sector;
    end else if (load_abort) begin
        // ★ 只复位“加载”这一半，绝不碰扫描结果。
        //   旧写法把 load_abort 和 !sd_init_done 并在一起，一次加载超时就会把
        //   scan_done / scan_found_total / scan_sector 全部清零 —— 上板现象是：
        //   第一张图能显示，之后 scan_done 永远是 0，屏幕上 8 位十六进制显示
        //   "84A11100"（初始化=1、扫描结束=0、显示=1、扫到 1 张、扇区=0），
        //   key1 和轮播全都不响应，卡里另外 3 张图永远扫不到。
        //   扫描没结束时还要接着扫（从断点 scan_sector 继续），否则 scan_done
        //   永远为 0，自动轮播的条件 `scan_done && ...` 全部失效。
        state            <= scan_done ? ST_IDLE : ST_SCAN;
        state_code       <= 4'd0;
        sd_sec_read      <= 1'b0;
        write_req        <= 1'b0;
        sd_sec_read_addr <= scan_sector;
        tmo_fail         <= 7'd0;
    end else begin
        scan_found_valid <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    scan_stride      <= 32'd1;              // 先逐扇区把引导扇区读出来
                    scan_limit       <= scan_max_sector;    // 解析成功后会被放大
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (scan_sector == 32'd0) begin
                        // ── 扇区 0 有两种可能 ──
                        // (a) 就是一张 BMP（虚拟 SD 卡 vsrc_sd 模式：第 0 扇区放的就是文件头）
                        // (b) 正常的 FAT 引导扇区（真卡）
                        if (header_match_r) begin
                            scan_found_valid  <= 1'b1;
                            scan_found_sector <= 32'd0;
                            scan_found_total  <= scan_found_total + 3'd1;
                            sd_sec_read_addr  <= next_scan_sector_if_match;
                            scan_sector       <= next_scan_sector_if_match;
                        end else begin
                            // 切到"逐簇扫"：从数据区首簇开始、步长=SecPerClus、上限 256MB；
                            // 解析失败则退回旧行为（逐扇区、16MB）。
                            scan_stride      <= bpb_stride_r;
                            scan_limit       <= bpb_ok_r ? 32'd524287 : scan_max_sector;
                            scan_sector      <= bpb_ok_r ? bpb_data0_r : 32'd1;
                            sd_sec_read_addr <= bpb_ok_r ? bpb_data0_r : 32'd1;
                            tmo_fail         <= 7'd0;
                        end
                    end else if (header_match_r) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_limit)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        tmo_fail <= 7'd0;
                        if (scan_sector >= scan_limit) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
                else if (tmo_hit) begin
                    // 这一扇区 2ms 没读完：跳过去继续扫，别把整轮扫描拖死
                    sd_sec_read      <= 1'b0;
                    sd_sec_read_addr <= next_scan_sector_if_miss;
                    scan_sector      <= next_scan_sector_if_miss;
                    if (tmo_fail >= 7'd63) begin
                        scan_done <= 1'b1;      // 连续超时：放弃扫描，用已找到的图
                        state     <= ST_IDLE;
                    end else begin
                        tmo_fail <= tmo_fail + 7'd1;
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match_r) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
                else if (tmo_hit) begin
                    sd_sec_read <= 1'b0;
                    state       <= ST_IDLE;
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (len_reached) begin
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
                else if (tmo_hit) begin
                    sd_sec_read <= 1'b0;
                    state       <= ST_IDLE;      // 读卡途中卡住：放弃这一张，交给上层超时
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
