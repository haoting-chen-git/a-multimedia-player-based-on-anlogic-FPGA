// ============================================================================
//  vsrc_sd.v  ——  虚拟 SD 卡（无卡演示 / 图片自适应自测）
//
//  用途
//  ---------------------------------------------------------------------------
//  没有读卡器、甚至卡里没有 BMP 时，也能把「任意尺寸图片 -> 等比缩放 + 居中黑边
//  -> 帧缓存 -> HDMI 显示」这条完整通路跑起来并肉眼验收。
//  做法是**冒充 sd_card_top 的扇区读接口**：让工程里的 bmp_read 以为卡里第 0 个
//  扇区就躺着 24bit 未压缩 BMP 文件，它照常扫描、解析文件头、按 file_len 取像素，
//  于是一行测试代码都不用改，整条真实通路（头部解析 / 像素流 / 缩放器 / 帧缓存）
//  都被验证到。
//
//  假文件格式（全部 24bit、非压缩、宽都是 4 的倍数 -> 行尾无填充）
//  ---------------------------------------------------------------------------
//      字节 0..53   : 标准 BMP 头（宽/高/文件长度按当前尺寸实时生成）
//      字节 54..    : 像素，每像素 3 字节 B,G,R，从左上到右下连续排列
//
//  图案（一眼看出是否拉伸 / 黑边是否居中）
//  ---------------------------------------------------------------------------
//      最外 6 像素      : 纯白边框
//      64x64 平铺菱形网格: 亮黄色描边
//      其余             : 渐变底（R 随 x、G 随 y，B 固定 0x40）
//
//  尺寸轮换
//  ---------------------------------------------------------------------------
//  每当 bmp_read 从第 0 扇区开始重新读（= 每装载一张新图）就自动切到下一个尺寸，
//  配合 sd_card_bmp 的自动轮播，屏幕上的图会依次变成 8 种不同尺寸：
//      0: 800x600    1: 1280x1024   2: 1920x1080   3: 1600x400
//      4: 400x1000   5: 640x480     6: 320x240     7: 2048x1200
//
//  重要约束
//  ---------------------------------------------------------------------------
//  本模块（以及整个工程）里**不能写乘法 `*`**：TD 会把 `*` 映射成 DSP，
//  之后布线阶段会 coredump（v13 踩过）。3*W*H 等常量全部写成常数表达式。
// ============================================================================
`timescale 1ns / 1ps

module vsrc_sd (
    input               clk,
    input               rst,
    input               en,                     // 1 = 用虚拟卡；0 = 完全不干预
    input               sd_sec_read,            // bmp_read 的扇区读请求（电平）
    input      [31:0]   sd_sec_read_addr,       // 请求的扇区号
    output reg  [7:0]   sd_sec_read_data,
    output reg          sd_sec_read_data_valid,
    output reg          sd_sec_read_end,
    output reg [15:0]   o_w,                    // 当前虚拟图宽（送屏幕显示）
    output reg [15:0]   o_h,                    // 当前虚拟图高（送屏幕显示）
    output reg  [2:0]   o_idx                   // 当前尺寸编号
);

    // ------------------------------------------------------------------
    // 尺寸表（全部是常数，综合时不会有乘法器）
    // ------------------------------------------------------------------
    function [15:0] sz_w;
        input [2:0] i;
        begin
            case (i)
                3'd0: sz_w = 16'd800;
                3'd1: sz_w = 16'd1280;
                3'd2: sz_w = 16'd1920;
                3'd3: sz_w = 16'd1600;
                3'd4: sz_w = 16'd400;
                3'd5: sz_w = 16'd640;
                3'd6: sz_w = 16'd320;
                default: sz_w = 16'd2048;
            endcase
        end
    endfunction

    function [15:0] sz_h;
        input [2:0] i;
        begin
            case (i)
                3'd0: sz_h = 16'd600;
                3'd1: sz_h = 16'd1024;
                3'd2: sz_h = 16'd1080;
                3'd3: sz_h = 16'd400;
                3'd4: sz_h = 16'd1000;
                3'd5: sz_h = 16'd480;
                3'd6: sz_h = 16'd240;
                default: sz_h = 16'd1200;
            endcase
        end
    endfunction

    // 文件长度 = 54 + 3*W*H（下面直接写结果，避免综合器生成乘法器）
    function [31:0] sz_flen;
        input [2:0] i;
        begin
            case (i)
                3'd0: sz_flen = 32'd1440054;    // 800*600*3+54
                3'd1: sz_flen = 32'd3932214;    // 1280*1024*3+54
                3'd2: sz_flen = 32'd6220854;    // 1920*1080*3+54
                3'd3: sz_flen = 32'd1920054;    // 1600*400*3+54
                3'd4: sz_flen = 32'd1200054;    // 400*1000*3+54
                3'd5: sz_flen = 32'd921654;     // 640*480*3+54
                3'd6: sz_flen = 32'd230454;     // 320*240*3+54
                default: sz_flen = 32'd7372854; // 2048*1200*3+54
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // BMP 文件头字节（bc = 扇区内字节号 0..53）
    // ------------------------------------------------------------------
    function [7:0] hdr_byte;
        input [8:0]  bc;
        input [15:0] W;
        input [15:0] H;
        input [31:0] FL;
        reg   [31:0] imsz;
        begin
            imsz = FL - 32'd54;
            case (bc)
                9'd0 : hdr_byte = 8'h42;        // 'B'
                9'd1 : hdr_byte = 8'h4D;        // 'M'
                9'd2 : hdr_byte = FL[7:0];
                9'd3 : hdr_byte = FL[15:8];
                9'd4 : hdr_byte = FL[23:16];
                9'd5 : hdr_byte = FL[31:24];
                9'd10: hdr_byte = 8'd54;        // 位图数据偏移
                9'd14: hdr_byte = 8'd40;        // 信息头大小
                9'd18: hdr_byte = W[7:0];
                9'd19: hdr_byte = W[15:8];
                9'd21: hdr_byte = 8'd0;
                9'd22: hdr_byte = H[7:0];
                9'd23: hdr_byte = H[15:8];
                9'd25: hdr_byte = 8'd0;
                9'd26: hdr_byte = 8'd1;         // planes = 1
                9'd28: hdr_byte = 8'd24;        // bit count = 24
                9'd29: hdr_byte = 8'd0;
                9'd34: hdr_byte = imsz[7:0];
                9'd35: hdr_byte = imsz[15:8];
                9'd36: hdr_byte = imsz[23:16];
                9'd37: hdr_byte = imsz[31:24];
                9'd38: hdr_byte = 8'h13;        // 2835 dpi
                9'd39: hdr_byte = 8'h0B;
                9'd42: hdr_byte = 8'h13;
                9'd43: hdr_byte = 8'h0B;
                default: hdr_byte = 8'd0;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // 像素字节：先算 24bit 颜色，再按 c（0=B,1=G,2=R）取一个字节
    // ------------------------------------------------------------------
    function [7:0] pix_byte;
        input [15:0] x;
        input [15:0] y;
        input [1:0]  c;
        input [15:0] W;
        input [15:0] H;
        reg   [5:0]  lx;
        reg   [5:0]  ly;
        reg   [6:0]  dx;
        reg   [6:0]  dy;
        reg   [7:0]  dm;
        reg          border;
        reg          edge_on;
        reg   [23:0] col;
        begin
            lx = x[5:0];
            ly = y[5:0];
            dx = (lx > 6'd31) ? (lx - 6'd31) : (6'd31 - lx);
            dy = (ly > 6'd31) ? (ly - 6'd31) : (6'd31 - ly);
            dm = {1'b0, dx} + {1'b0, dy};
            border = (x < 16'd6) || (x >= (W - 16'd6)) ||
                     (y < 16'd6) || (y >= (H - 16'd6));
            edge_on = (dm > 8'd15) && (dm < 8'd23);
            if (border)
                col = 24'hFFFFFF;
            else if (edge_on)
                col = 24'hFFD000;
            else
                col = {x[9:2], y[9:2], 8'h40};
            if (c == 2'd0)
                pix_byte = col[23:16];          // BMP 里是 B,G,R
            else if (c == 2'd1)
                pix_byte = col[15:8];
            else
                pix_byte = col[7:0];
        end
    endfunction

    // ------------------------------------------------------------------
    // 状态机
    // ------------------------------------------------------------------
    localparam [1:0] ST_IDLE = 2'd0,
                     ST_DATA = 2'd1,
                     ST_END  = 2'd2,
                     ST_GAP  = 2'd3;

    localparam integer CLKDIV = 4;              // 每字节占几个时钟（真实卡约 8）

    reg  [1:0]  st;
    reg  [3:0]  div;
    reg  [8:0]  bcnt;
    reg  [15:0] px;
    reg  [15:0] py;
    reg  [1:0]  pc;
    reg         started;
    reg  [31:0] cur_addr;

    wire [15:0] cf_w = o_w;
    wire [15:0] cf_h = o_h;
    wire [31:0] cf_fl = sz_flen(o_idx);

    wire [7:0] cur_byte = (bcnt < 9'd54) ? hdr_byte(bcnt, cf_w, cf_h, cf_fl)
                                         : pix_byte(px, py, pc, cf_w, cf_h);

    always @(posedge clk) begin
        if (rst) begin
            st                     <= ST_IDLE;
            div                    <= 4'd0;
            bcnt                   <= 9'd0;
            px                     <= 16'd0;
            py                     <= 16'd0;
            pc                     <= 2'd0;
            started                <= 1'b0;
            cur_addr               <= 32'hFFFFFFFF;
            o_idx                  <= 3'd0;
            o_w                    <= sz_w(3'd0);
            o_h                    <= sz_h(3'd0);
            sd_sec_read_data       <= 8'd0;
            sd_sec_read_data_valid <= 1'b0;
            sd_sec_read_end        <= 1'b0;
        end
        else begin
            sd_sec_read_data_valid <= 1'b0;
            sd_sec_read_end        <= 1'b0;

            if (!en) begin
                st <= ST_IDLE;
                div <= 4'd0;
                started <= 1'b0;
            end
            else begin
                case (st)
                ST_IDLE: begin
                    if (sd_sec_read) begin
                        // 从第 0 扇区开始读 = 装载一张新图 -> 换尺寸
                        if (sd_sec_read_addr == 32'd0) begin
                            if (started) begin
                                o_idx <= o_idx + 3'd1;
                                o_w   <= sz_w(o_idx + 3'd1);
                                o_h   <= sz_h(o_idx + 3'd1);
                            end
                            else begin
                                started <= 1'b1;
                                o_w     <= sz_w(o_idx);
                                o_h     <= sz_h(o_idx);
                            end
                            px <= 16'd0;
                            py <= 16'd0;
                            pc <= 2'd0;
                        end
                        cur_addr <= sd_sec_read_addr;
                        bcnt     <= 9'd0;
                        div      <= 4'd0;
                        st       <= ST_DATA;
                    end
                end
                ST_DATA: begin
                    if (div == (CLKDIV - 1)) begin
                        div                    <= 4'd0;
                        sd_sec_read_data       <= cur_byte;
                        sd_sec_read_data_valid <= 1'b1;
                        bcnt                   <= bcnt + 9'd1;
                        // 推进像素指针（只有像素字节才推进）
                        if (bcnt >= 9'd54) begin
                            if (pc == 2'd2) begin
                                pc <= 2'd0;
                                if (px == (cf_w - 16'd1)) begin
                                    px <= 16'd0;
                                    py <= py + 16'd1;
                                end
                                else begin
                                    px <= px + 16'd1;
                                end
                            end
                            else begin
                                pc <= pc + 2'd1;
                            end
                        end
                        if (bcnt == 9'd511)
                            st <= ST_END;
                    end
                    else begin
                        div <= div + 4'd1;
                    end
                end
                ST_END: begin
                    sd_sec_read_end <= 1'b1;
                    div             <= 4'd0;
                    st              <= ST_GAP;
                end
                ST_GAP: begin
                    if (div == 4'd3) begin
                        div <= 4'd0;
                        st  <= ST_IDLE;
                    end
                    else begin
                        div <= div + 4'd1;
                    end
                end
                default: st <= ST_IDLE;
                endcase
            end
        end
    end

endmodule
