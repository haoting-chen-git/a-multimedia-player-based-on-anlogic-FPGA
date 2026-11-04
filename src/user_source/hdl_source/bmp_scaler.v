// ==========================================================================
// bmp_scaler.v —— BMP 图片自适应缩放（任意 W x H -> 640x480 画布，等比 + 黑边居中）
//   安路 EG4S20 (HX4S20C) HDMI 多媒体播放器
//
// 【这个模块解决什么问题】
//   参考实例只认 640x480、24 位、非压缩的 BMP，文件头宽高对不上就直接跳过。
//   本模块把任意 W x H 的图【等比】缩放到 640x480 画布中央，四周补黑边，
//   于是"图片处理"这一环就与参考实例不同了（《选题指南》的硬性要求）。
//
//   例：1920x1080(16:9) -> 640x360 居中，上下各 60 行黑边
//       800x600 (4:3)   -> 640x480 正好铺满，无黑边
//       400x1000(2:5)   -> 192x480 居中，左右各 224 列黑边
//
// ==========================================================================
// ★ 为什么第二版要重写（第一版为什么时序崩了）
//   第一版用"32 位 DDA 累加器 + 比较"在【像素级】做坐标映射，
//   而它挂在 SDRAM 写通路的组合链上，100MHz(10ns) 里根本走不完：
//       video_clk SWNS -10.866ns -> 改进到 -7.495ns -> 仍违例 -> 回退。
//
//   第二版做了三件事，把算术彻底赶出像素级关键路径：
//     1) 商/余数只算一次：hstep = W/sw、hfrac = W%sw 在装载开始时用
//        串行减法算出来（最多几十次循环），之后全程是常数。
//     2) 像素级只剩小位宽 DDA：hacc(<=DW) + hfrac 一次加法 + 一次比较
//        + 一次条件减法，全部 16 位以内 —— 比原来的 32 位链短一半以上。
//     3) 行缓存读写全部是 BSRAM 同步读（地址 -> 数据晚一拍），
//        DDA 提前一拍算地址，天然满足 BSRAM 的 1 拍延迟，没有长组合链。
//
// ★ 正确性：最近邻映射是精确的，不是近似
//     设 W = hstep*sw + hfrac，则
//         floor(k*W/sw) = hstep*k + floor(k*hfrac/sw)
//     右边第二项正好等于"小数累加器 hfrac 走 k 步产生的进位次数"，
//     所以 hacc/hidx 这一对累加器给出的是【精确】的 floor(k*W/sw)。
//     垂直方向同理（vstep/vfrac/vacc/vsrc）。
//
// --------------------------------------------------------------------------
// 数据流
//
//   源像素流（慢，来自 SD 卡）
//        |  每个 s_valid 往当前行缓存写一个像素（整W个像素 = 1 源行）
//        v
//   [行缓存 A] / [行缓存 B]   <- 乒乓，1 行 = 最多 SW_MAX 个 24bit 像素
//        |  发射一个输出行时，用 hacc/hidx 生成读地址（提前一拍）
//        v
//   输出像素流（快，1 像素/拍） -> fifo -> SDRAM 帧缓存
//
//   垂直：输出行 orow 落在 [y0, y0+sh) 时才是图像行，vsrc = vstep*(orow-y0) + 进位
//         只有 vsrc 正好等于当前源行号时才发射，因此
//         放大时同一源行会连发多遍，缩小时有些源行被整行跳过。
//         其余输出行（上/下黑边）不碰行缓存，直接发 DW 个 0。
//
//   水平：输出像素 X 落在 [x0, x0+sw) 时取源像素，否则发 0。
//
// --------------------------------------------------------------------------
// ★ 装载阶段（ST_DIV）的运算顺序 —— 全部串行减法，不占像素通路
//   ph0: p1 = W*DH
//   ph1: p2 = H*DW
//   ph2: 比较 p1 >= p2 判断"宽图"还是"高图"，确定 sw / sh 中哪个取满
//        宽图(W/H > 4/3): sw = DW, sh = H*DW/W
//        高图(W/H < 4/3): sh = DH, sw = W*DH/H
//   ph3: 除法 1 -> sw 或 sh
//   ph4: 除法 2 -> hstep/hfrac = W/sw
//   ph5: 除法 3 -> vstep/vfrac = H/sh
//   ph6: 算阈值 + 黑边偏移 x0=(DW-sw)/2、y0=(DH-sh)/2
//   一次装载最多几千个周期（相对 SD 读一帧的几百万周期可忽略）。
//
// ★ 使用约束（文档 / 演示里要写清）
//   1) 源图宽度必须是 4 的倍数。BMP 每行按 4 字节对齐，行尾可能有填充字节，
//      而官方 bmp_read 不处理填充；宽为 4 的倍数时 3*W 恰好是 4 的倍数，无填充。
//   2) 源图建议 >= 200x150。发射 1 行只要 DW 拍，而源行到达需要 W*每像素拍数；
//      源图太小时上下黑边期间源行会积压，双缓冲不够用，溢出记在 o_ovf 里。
//      真实照片远大于这个下限，不受影响。
// --------------------------------------------------------------------------
`timescale 1ns/1ps

module bmp_scaler #(
    parameter integer DW     = 640,    // 输出宽（帧缓存宽）
    parameter integer DH     = 480,    // 输出高（帧缓存高）
    parameter integer SW_MAX = 2048    // 源图最大宽度
)(
    input  wire        clk,        // sd_card_clk (~100MHz)
    input  wire        rst,        // 高有效
    input  wire        start,      // 一次装载开始（load_start_pulse），1 拍
    input  wire        i_arm,      // 写帧缓存已受理本帧写请求（write_req_ack 之后保持 1）
    input  wire [15:0] cfg_w,      // 源图宽（来自 BMP 文件头）
    input  wire [15:0] cfg_h,      // 源图高
    input  wire        s_valid,    // 源像素有效
    input  wire [23:0] s_data,     // 源像素 {R,G,B}

    output reg         m_valid,    // 输出像素有效
    output reg  [23:0] m_data,     // 输出像素 {R,G,B}
    output reg         m_done,     // 整帧 DW x DH 发完
    output wire        o_busy,     // 1 = 正在工作
    output reg  [15:0] o_ovf       // 行缓存溢出次数（正常应为 0）
);

    localparam integer SWA = (SW_MAX <= 2) ? 1 : $clog2(SW_MAX);
    localparam integer OWA = (DW     <= 2) ? 1 : $clog2(DW);
    localparam [15:0]  DW16 = DW;    // 画布宽（16 位常量）
    localparam [15:0]  DH16 = DH;    // 画布高

    // ----------------------------------------------------------------------
    localparam [1:0] ST_IDLE = 2'd0,
                     ST_DIV  = 2'd1,   // 串行除法算 sw/sh/hstep/hfrac/vstep/vfrac
                     ST_RUN  = 2'd2;

    reg [1:0]  S_st;
    reg [2:0]  S_ph;                 // ST_DIV 内的阶段 0..6
    reg [6:0]  S_ph1h;               // S_ph 的单热副本（拆分扇出，改善 sd_card_clk 时序）
    reg [15:0] S_cw, S_ch;           // 源图宽高
    reg [15:0] S_sw, S_sh;           // 图像在画布上的实际尺寸
    reg [10:0] S_x0, S_y0;           // 图像在画布上的左上角（黑边宽度）；11 位够 2048
    reg [10:0] S_xend, S_yend;       // S_x0+S_sw / S_y0+S_sh（装载时算好，省掉像素级加法）
    reg        S_wide;               // 1 = 宽图（左右取满，上下补黑）

    reg [15:0] S_hstep, S_hfrac, S_vstep, S_vfrac;

    // ----------------------------------------------------------------------
    // 常量乘法（移位相加）
    //
    // ★ 为什么不用 `*`：v3 第一版直接写 `S_cw * DH` / `S_ch * DW`，综合器把这两个
    //   16x16 乘法映射成了 DSP 块（DSP 3 -> 5），随后布局布线器在 place 结束、
    //   即将开始 route 时【崩溃】（连续两次生成 minidump-*.dmp）。
    //   这里改成按常量的二进制位做移位相加：循环变量 i 在展开时是常量，
    //   所以全是定长移位 + 加法树，不会调用 DSP，也不产生变量移位器。
    // ----------------------------------------------------------------------
    function [23:0] mul_const;
        input [15:0] x;
        input integer c;
        integer k;
        begin
            mul_const = 24'd0;
            for (k = 0; k < 24; k = k + 1)
                if (((c >> k) & 1) != 0)
                    mul_const = mul_const + ({8'd0, x} << k);
        end
    endfunction

    // ----------------------------------------------------------------------
    // 串行除法器（24 步移位减法，步数有界）
    //
    // ★ 为什么不用"反复减法"：商最大能到 DW=640（算 sh = H*DW/W 时），最坏要减
    //   640 次 = 640 拍。而 SD 的像素流在文件头之后【立刻】就开始送，这 640 拍里
    //   到达的源像素会被整段丢掉 -> 行边界整体错位 -> 最后一行永远凑不满 480 行
    //   -> 缩放器死等 -> sd_card_bmp 只能靠 1 秒超时放弃 -> 屏幕上永远只有自检
    //   图案（2026-10-17 实测就是这个现象，卡里的图一张都没出来）。
    //   移位减法固定 24 拍出一组商/余数，三次除法共 ~80 拍；再配合"ST_DIV 期间
    //   也照收像素"，除法耗时对图像完整性完全没有影响。
    // ----------------------------------------------------------------------
    reg [23:0] S_p1, S_p2;           // W*DH, H*DW
    reg [23:0] S_num;                // 被除数（移位过程中逐步生成商）
    reg [23:0] S_rem;                // 余数
    reg [23:0] S_dvs;                // 除数
    reg [4:0]  S_bcnt;               // 已完成的移位步数 0..24

    wire [23:0] div_rem_nx = {S_rem[22:0], S_num[23]};
    wire        div_ge     = (div_rem_nx >= S_dvs);
    wire [23:0] div_rem_pl = div_rem_nx - S_dvs;

    // ----------------------------------------------------------------------
    // 行缓存（两片乒乓）
    // ----------------------------------------------------------------------
    reg [23:0] rbuf0 [0:SW_MAX-1];
    reg [23:0] rbuf1 [0:SW_MAX-1];

    reg [SWA-1:0] S_wcnt;            // 当前行已写入像素数
    reg [SWA-1:0] S_rd_addr;         // 读地址（提前一拍给出）
    reg [23:0]    rd_q0, rd_q1;
    reg           S_wr_sel;
    reg           S_rd_sel;
    wire [23:0]   rd_q = S_rd_sel ? rd_q1 : rd_q0;

    always @(posedge clk) begin
        if (s_valid && (S_wr_sel == 1'b0)) rbuf0[S_wcnt] <= s_data;
    end
    always @(posedge clk) begin
        if (s_valid && (S_wr_sel == 1'b1)) rbuf1[S_wcnt] <= s_data;
    end
    always @(posedge clk) begin
        rd_q0 <= rbuf0[S_rd_addr];
        rd_q1 <= rbuf1[S_rd_addr];
    end

    // ----------------------------------------------------------------------
    // 行缓存状态
    // ----------------------------------------------------------------------
    reg        S_full0, S_full1;
    reg [15:0] S_rowi0, S_rowi1;     // 该片存的是第几个源行
    reg [15:0] S_wrow;               // 正在写第几个源行

    // 读侧选择
    // ★ 时序优化：v3 第一版写成
    //     pk_sel = (S_rowi0 <= S_rowi1) ? 0 : 1;   pk_rowi = 选择后的行号
    //     ……然后拿 pk_rowi 去和 S_vsrc 比相等
    //   这是"16 位比较 -> 16 位二选一 -> 再 16 位比较"三级串联，
    //   实测把 sd_card_clk 拖到 -0.201ns（2 个端点，Data Path 9.921ns / 8 级）。
    //   现在改成两片各自独立做"等于"判断（两条 16 位比较并行，互不依赖），
    //   选择器只用两个单比特结果，后面不再有长组合链。
    //   （具体判定信号放在垂直 DDA 之后声明，那里才定下 v_src_nx。）

    // ----------------------------------------------------------------------
    // 水平 DDA（发射用，提前一拍算地址）
    //
    // ★ 时序优化（v3）：v2 写成
    //      acc_nx = acc + frac;  carry = (acc_nx >= DW);  acc_f = carry ? acc_nx-DW : acc_nx;
    //   这是"加法 -> 比较 -> 再减法"三级串联，16 位宽下实测 9.773ns，
    //   在 100MHz(10ns) 的 sd_card_clk 里违例 -0.053ns（1 个端点）。
    //   现在改成"阈值比较 + 两条并行加法"：
    //      carry = (acc >= sw - frac)          <- 阈值在 DIV 阶段算好，是常量
    //      acc + frac     和   acc + (frac-sw) <- 两条加法并行算
    //   比较和加法互不依赖，末端只剩一个 2 选 1，链路缩短一半以上。
    //   等价性：acc+frac >= sw  <=>  acc >= sw-frac（整数、无符号）。
    // ----------------------------------------------------------------------
    reg [15:0] S_hacc, S_hidx;
    reg [15:0] S_hfrac_m;            // hfrac - DW（补码负数）
    reg [15:0] S_hthr;               // DW - hfrac
    wire [10:0] h_pl     = {1'b0, S_hacc[9:0]} + {1'b0, S_hfrac[9:0]};
    wire [10:0] h_mn     = {1'b0, S_hacc[9:0]} + S_hfrac_m[10:0];
    wire        h_carry  = (S_hacc >= S_hthr);
    wire [10:0] h_acc_f  = h_carry ? h_mn : h_pl;
    wire [11:0] h_idx_nx = {1'b0, S_hidx[10:0]} + S_hstep[11:0] + {11'd0, h_carry};

    // ----------------------------------------------------------------------
    // 垂直 DDA（按行推进）—— 同上，用阈值比较 + 并行加法
    // ----------------------------------------------------------------------
    reg [15:0] S_vacc, S_vsrc;
    reg [10:0] S_orow;
    reg [15:0] S_vfrac_m;            // vfrac - DH
    reg [15:0] S_vthr;               // DH - vfrac
    wire [9:0]  v_pl     = {1'b0, S_vacc[8:0]} + {1'b0, S_vfrac[8:0]};
    wire [9:0]  v_mn     = {1'b0, S_vacc[8:0]} + S_vfrac_m[9:0];
    wire        v_carry  = (S_vacc >= S_vthr);
    wire [9:0]  v_acc_f  = v_carry ? v_mn : v_pl;
    wire [10:0] v_src_nx = S_vsrc[10:0] + S_vstep[10:0] + {10'd0, v_carry};

    // 把 v_src_nx 打一拍再用：它带一个 11 位加法器，直接参与 next0/next1 比较
    // 会让 sd_card_clk 违例（-0.234ns）。行内 S_vsrc/S_vstep/S_vacc 都不变，
    // 所以上一拍的 v_src_nx 与这一拍的值恒等，寄存不影响功能。
    reg [10:0] S_vnx_r;
    always @(posedge clk) S_vnx_r <= v_src_nx;

    // ---- 行缓存命中 / 过期 / 复用 判定（两片各自独立，无二选一串联）----
    wire        hit0     = S_full0 && (S_vsrc == S_rowi0);
    wire        hit1     = S_full1 && (S_vsrc == S_rowi1);
    wire        pk_ok    = hit0 | hit1;          // 有一片正好存着 vsrc 这一行
    wire        pk_sel   = hit0 ? 1'b0 : 1'b1;   // 命中哪一片（只为算 rd_sel）
    wire        stale0   = S_full0 && (S_rowi0 < S_vsrc);   // 缩小跳过、可放掉
    wire        stale1   = S_full1 && (S_rowi1 < S_vsrc);
    wire        next0    = (S_vnx_r == S_rowi0);           // 放大时同一源行要复用
    wire        next1    = (S_vnx_r == S_rowi1);

    reg         S_emit;              // 正在发一行
    reg         S_blank;             // 这一行是纯黑（上下黑边）
    reg [OWA:0] S_ecnt;              // 0..DW+1

    // ----------------------------------------------------------------------
    // 输出节流（★ 必须有）
    //   官方 frame_fifo_write 前面的异步 FIFO 只有 512 深，写侧跑 sd_card_clk
    //   (100MHz)。如果一行 640 个像素背靠背地灌进去，FIFO 会在 SDRAM 忙于
    //   显示读的时候被写满，写满后 write_en 被忽略 -> 丢像素 -> 这一行的后
    //   半截留在旧帧内容上（上板看到的就是"整屏大量细亮线"）。
    //   官方 demo 的写侧来自 SD 字节流，每像素隔近百拍，从来碰不到这个边界，
    //   所以这个坑只有"先缓存一行、再高速整行输出"才会踩到。
    //   节流到 4 拍 1 像素 = 25M 像素/秒，仍比源端（SD 每像素约 960ns）快
    //   24 倍，对总耗时没有任何影响，但 FIFO 不会被撑爆。
    // ----------------------------------------------------------------------
    localparam [2:0] PACE_DIV = 3'd3;    // 每 4 拍发一个像素
    reg [2:0] S_pace;
    reg       S_armed;   // 写帧缓存已受理写请求，可以开始往外发像素

    // 当前行是否图像行 / 下一行是否图像行
    wire        row_in   = (S_orow >= S_y0) && (S_orow < S_yend);
    wire        row_nx_in= ((S_orow + 11'd1) >= S_y0) && ((S_orow + 11'd1) < S_yend);

    assign o_busy = (S_st != ST_IDLE) && !m_done;

    always @(posedge clk) begin
        if (rst) begin
            S_st     <= ST_IDLE;
            S_ph <= 3'd0;
            S_ph1h <= 7'd1 << 0;
            S_cw     <= 16'd0;
            S_ch     <= 16'd0;
            S_sw     <= 16'd0;
            S_sh     <= 16'd0;
            S_x0     <= 16'd0;
            S_y0     <= 11'd0;
            S_xend   <= 11'd0;
            S_yend   <= 11'd0;
            S_wide   <= 1'b0;
            S_p1     <= 24'd0;
            S_p2     <= 24'd0;
            S_num    <= 24'd0;
            S_rem    <= 24'd0;
            S_dvs    <= 24'd1;
            S_bcnt   <= 5'd0;
            S_hstep  <= 16'd0;
            S_hfrac  <= 16'd0;
            S_hfrac_m<= 16'd0;
            S_hthr   <= 16'd0;
            S_vstep  <= 16'd0;
            S_vfrac  <= 16'd0;
            S_vfrac_m<= 16'd0;
            S_vthr   <= 16'd0;
            S_wcnt   <= {SWA{1'b0}};
            S_rd_addr<= {SWA{1'b0}};
            S_wr_sel <= 1'b0;
            S_rd_sel <= 1'b0;
            S_full0  <= 1'b0;
            S_full1  <= 1'b0;
            S_rowi0  <= 16'd0;
            S_rowi1  <= 16'd0;
            S_wrow   <= 16'd0;
            S_hacc   <= 16'd0;
            S_hidx   <= 16'd0;
            S_vacc   <= 16'd0;
            S_vsrc   <= 16'd0;
            S_orow   <= 11'd0;
            S_emit   <= 1'b0;
            S_blank  <= 1'b0;
            S_ecnt   <= {(OWA+1){1'b0}};
            S_pace   <= 3'd0;
            S_armed  <= 1'b0;
            m_valid  <= 1'b0;
            m_data   <= 24'd0;
            m_done   <= 1'b0;
            o_ovf    <= 16'd0;
        end
        else if (start) begin
            // 重新装载：清掉一切（行缓存内容不用清，full 标志会挡住）
            S_st     <= ST_DIV;
            S_armed  <= 1'b0;   // 每次重新装载都要重新等写侧武装
            S_ph <= 3'd0;
            S_ph1h <= 7'd1 << 0;
            // cfg 为 0 时按 640x480 处理，避免除零把装载阶段卡死
            S_cw     <= (cfg_w == 16'd0) ? DW16 : cfg_w;
            S_ch     <= (cfg_h == 16'd0) ? DH16 : cfg_h;
            S_sw     <= 16'd0;
            S_sh     <= 16'd0;
            S_x0     <= 16'd0;
            S_y0     <= 11'd0;
            S_xend   <= 11'd0;
            S_yend   <= 11'd0;
            S_wide   <= 1'b0;
            S_p1     <= 24'd0;
            S_p2     <= 24'd0;
            S_num    <= 24'd0;
            S_rem    <= 24'd0;
            S_dvs    <= 24'd1;
            S_bcnt   <= 5'd0;
            S_hstep  <= 16'd0;
            S_hfrac  <= 16'd0;
            S_hfrac_m<= 16'd0;
            S_hthr   <= 16'd0;
            S_vstep  <= 16'd0;
            S_vfrac  <= 16'd0;
            S_vfrac_m<= 16'd0;
            S_vthr   <= 16'd0;
            S_wcnt   <= {SWA{1'b0}};
            S_rd_addr<= {SWA{1'b0}};
            S_wr_sel <= 1'b0;
            S_rd_sel <= 1'b0;
            S_full0  <= 1'b0;
            S_full1  <= 1'b0;
            S_rowi0  <= 16'd0;
            S_rowi1  <= 16'd0;
            S_wrow   <= 16'd0;
            S_hacc   <= 16'd0;
            S_hidx   <= 16'd0;
            S_vacc   <= 16'd0;
            S_vsrc   <= 16'd0;
            S_orow   <= 11'd0;
            S_emit   <= 1'b0;
            S_blank  <= 1'b0;
            S_ecnt   <= {(OWA+1){1'b0}};
            S_pace   <= 3'd0;
            m_valid  <= 1'b0;
            m_done   <= 1'b0;
            o_ovf    <= 16'd0;
        end
        else if (!m_done) begin
            m_valid <= 1'b0;

            // ---------------- 采集源像素 ----------------
            // ★ 必须在 ST_DIV 期间也收：SD 读到文件头之后立刻开始送像素，而装载
            //   阶段还要算三次除法（~80 拍）。如果只在 ST_RUN 收，这 80 拍里到达
            //   的像素会被丢掉，整幅图的行边界全部错位，最后一行永远凑不满 ->
            //   死等 -> 超时放弃 -> 屏幕只剩自检图案。
            if (((S_st == ST_DIV) || (S_st == ST_RUN)) && s_valid) begin
                if (S_wcnt == (S_cw - 16'd1)) begin
                    S_wcnt <= {SWA{1'b0}};
                    if (S_wr_sel == 1'b0) begin
                        S_full0 <= 1'b1;
                        S_rowi0 <= S_wrow;
                    end
                    else begin
                        S_full1 <= 1'b1;
                        S_rowi1 <= S_wrow;
                    end
                    // 另一片还满着 => 读侧没跟上，溢出
                    if ((S_wr_sel == 1'b0) ? S_full1 : S_full0)
                        o_ovf <= o_ovf + 16'd1;
                    S_wr_sel <= ~S_wr_sel;
                    S_wrow   <= S_wrow + 16'd1;
                end
                else begin
                    S_wcnt <= S_wcnt + {{(SWA-1){1'b0}}, 1'b1};
                end
            end

            case (S_st)
            // -------------------------------------------------------------
            ST_IDLE: begin
                if (start) S_st <= ST_DIV;
            end
            // -------------------------------------------------------------
            // 装载阶段：算出 sw/sh/x0/y0/hstep/hfrac/vstep/vfrac
            // -------------------------------------------------------------
            ST_DIV: begin
                case (S_ph)
                // p1 = W*DH
                3'd0: begin
                    S_p1 <= mul_const(S_cw, DH);
                    S_ph <= 3'd1;
                    S_ph1h <= 7'd1 << 1;
                end
                // p2 = H*DW
                3'd1: begin
                    S_p2 <= mul_const(S_ch, DW);
                    S_ph <= 3'd2;
                    S_ph1h <= 7'd1 << 2;
                end
                // 判宽图/高图，装第一次除法
                3'd2: begin
                    if (S_p1 >= S_p2) begin
                        // 宽图：宽取满 DW，高由 H*DW/W 算出
                        S_wide <= 1'b1;
                        S_sw   <= DW16;
                        S_num  <= S_p2;                              // 被除数 H*DW
                        S_dvs  <= (S_cw == 16'd0) ? 24'd1 : {8'd0, S_cw};
                    end
                    else begin
                        // 高图：高取满 DH，宽由 W*DH/H 算出
                        S_wide <= 1'b0;
                        S_sh   <= DH16;
                        S_num  <= S_p1;                              // 被除数 W*DH
                        S_dvs  <= (S_ch == 16'd0) ? 24'd1 : {8'd0, S_ch};
                    end
                    S_rem  <= 24'd0;
                    S_bcnt <= 5'd0;
                    S_ph <= 3'd3;
                    S_ph1h <= 7'd1 << 3;
                end
                // 除法 1/2/3：移位减法，每 24 拍出一组商与余数
                3'd3, 3'd4, 3'd5: begin
                    if (S_bcnt != 5'd24) begin
                        // 余数左移进被除数最高位，够减则减并把商的位置 1
                        S_rem  <= div_ge ? div_rem_pl : div_rem_nx;
                        S_num  <= {S_num[22:0], div_ge};
                        S_bcnt <= S_bcnt + 5'd1;
                    end
                    else if (S_ph1h[3]) begin
                        // sw(或 sh) 已算出；接着算 hstep/hfrac = W / sw
                        if (S_wide) S_sh <= S_num[15:0];
                        else        S_sw <= S_num[15:0];
                        S_num  <= {8'd0, S_cw};
                        S_dvs  <= S_wide ? {8'd0, DW16} : S_num;   // 除数 = sw
                        S_rem  <= 24'd0;
                        S_bcnt <= 5'd0;
                        S_ph <= 3'd4;
                        S_ph1h <= 7'd1 << 4;
                    end
                    else if (S_ph1h[4]) begin
                        S_hstep <= S_num[15:0];
                        S_hfrac <= S_rem[15:0];
                        // 除法 3：vstep/vfrac = H / sh
                        S_num   <= {8'd0, S_ch};
                        S_dvs   <= S_wide ? {8'd0, S_sh} : {8'd0, DH16};
                        S_rem   <= 24'd0;
                        S_bcnt  <= 5'd0;
                        S_ph <= 3'd5;
                        S_ph1h <= 7'd1 << 5;
                    end
                    else begin
                        S_vstep  <= S_num[15:0];
                        S_vfrac  <= S_rem[15:0];
                        S_hfrac_m<= S_hfrac - DW16;
                        S_hthr   <= DW16 - S_hfrac;
                        S_vfrac_m<= S_rem[15:0] - DH16;
                        S_vthr   <= DH16 - S_rem[15:0];
                        S_x0     <= (DW16 - S_sw) >> 1;
                        S_y0     <= (DH16 - S_sh) >> 1;
                        S_xend   <= ((DW16 - S_sw) >> 1) + S_sw;
                        S_yend   <= ((DH16 - S_sh) >> 1) + S_sh;
                        S_st     <= ST_RUN;
                    end
                end
                default: S_st <= ST_RUN;
                endcase
            end
            // -------------------------------------------------------------
            ST_RUN: begin
                // （源像素采集已在 case 之前统一处理，ST_DIV 期间也在收）

                // ---------------- 2. 起一行 ----------------
                if (i_arm) S_armed <= 1'b1;
                if (!S_emit) begin
                    // ★ 必须等写帧缓存受理完本帧写请求（write_req_ack）之后才发像素。
                    //   frame_fifo_write 是从 ack 之后才开始数像素的，在 ack 之前
                    //   发出去的像素会被直接丢掉。带黑边的图（1920x1080 -> 640x360，
                    //   上下各 60 行黑）一上来就发黑行、不需要等源数据，于是 ack 之前
                    //   就发走了 100 多个像素 -> 写侧永远数不满 307200 ->
                    //   write_finish 不来 -> 12 秒超时 -> 屏幕上永远换不过去。
                    if (!S_armed) begin
                        // 等写侧武装，本拍不发起新行
                    end
                    else if (S_orow < DH) begin
                        if (!row_in) begin
                            // 上/下黑边行：不碰行缓存，直接发 DW 个黑像素
                            S_emit  <= 1'b1;
                            S_blank <= 1'b1;
                            S_ecnt  <= {(OWA+1){1'b0}};
                            S_pace  <= 3'd0;
                            // ★ 黑边期间读侧一个源行都不消费，而写侧照样在收数据。
                            //   如果这里不跟住写侧，等黑边结束再去找 vsrc=0 那一行，
                            //   缓存里早就是第 N 十行了 -> 每次 stale 都把 vsrc 往前
                            //   跳一大截 -> 最后 vsrc 跑到写侧前面 -> 永远等不到 -> 死锁。
                            //   上板现象：1920x1080 那张图只发出 60 行（正好是上黑边
                            //   行数）就再也不动了，整帧凑不满 307200 像素，
                            //   sd_card_bmp 等不到 write_finish，12 秒超时后放弃，
                            //   屏幕永远停在第一张 800x600。
                            //   现在黑边期间直接跟写侧对齐，图像从"当前正在写的源行"
                            //   开始，代价是顶部裁掉几行，比卡死强。
                            S_vsrc  <= S_wrow;
                            S_full0 <= 1'b0;
                            S_full1 <= 1'b0;
                        end
                        else if (pk_ok) begin
                            // 有一片正好存着 vsrc 这一源行 -> 发射
                            S_emit    <= 1'b1;
                            S_blank   <= 1'b0;
                            S_ecnt    <= {(OWA+1){1'b0}};
                            S_hacc    <= 16'd0;
                            S_hidx    <= 16'd0;
                            S_rd_addr <= {SWA{1'b0}};
                            S_rd_sel  <= pk_sel;
                            S_pace    <= 3'd0;
                        end
                        else if (stale0) begin
                            // 缩小跳行：这片存的行已经过去了，放掉
                            S_full0 <= 1'b0;
                        end
                        else if (stale1) begin
                            S_full1 <= 1'b0;
                        end
                        else if (S_wrow >= S_ch) begin
                            // ★ 源图已经全部写完，而 vsrc 要的那一行再也不会来了
                            //   （写侧冲得太快、vsrc 落在后面）-> 余下行按黑行发完。
                            //   保证整帧一定凑满 FRAME_PIXELS，write_finish 一定会来，
                            //   上层不必等满 12 秒超时再放弃。
                            S_emit  <= 1'b1;
                            S_blank <= 1'b1;
                            S_ecnt  <= {(OWA+1){1'b0}};
                            S_pace  <= 3'd0;
                        end
                        else if (S_full0 || S_full1) begin
                            // 既没命中也没有过期行 => vsrc 落后于缓存里的行
                            // （只有上黑边期间源图特别小、写侧把两片都冲掉才会这样）。
                            // 放掉一片并把 vsrc 拉到它存的行，否则永远等不到，直接死锁。
                            // 代价是图像顶部丢几条源行，比卡死好。
                            if (S_full0) begin
                                S_vsrc  <= S_rowi0;
                                S_full0 <= 1'b0;
                            end
                            else begin
                                S_vsrc  <= S_rowi1;
                                S_full1 <= 1'b0;
                            end
                        end
                        else if (S_vsrc[10:0] < S_wrow[10:0]) begin
                            // 兜底：vsrc 落后写侧，但两片缓存里都没有它要的行（被冲掉了）。
                            // 往前挪一格，总能追上写侧，绝不在原地等死。
                            // ★ 只比低 11 位：源图行号最大 1200，11 位够。用 16 位比较时
                            //   S_vsrc[6] -> S_vsrc[0].ce 这条进位链 9.866ns（6 级 LUT + 2 个
                            //   ADDER），sd_card_clk 违例 -0.146ns；砍半后收敛。
                            S_vsrc <= S_vsrc + 16'd1;
                        end
                    end
                end
                else begin
                    // ---------------- 3. 发射中 ----------------
                    // 节流：只有 S_pace==0 的拍才真正送一个像素出去
                    // ★ 两片行缓存同时满 = 写侧已经跑到读侧前面，这时必须全速消费，
                    //   否则写侧会继续覆盖还没读过的行（仿真里 ovf 就是这么涨的）。
                    //   平时仍按 PACE_DIV 节流，避免长时间 1 像素/拍把下游写 FIFO 冲爆。
                    S_pace <= (S_full0 && S_full1) ? 3'd0
                            : ((S_pace == PACE_DIV) ? 3'd0 : (S_pace + 3'd1));
                    if ((S_full0 && S_full1) || (S_pace == 3'd0)) begin
                    // 行缓存同步读晚一拍：本拍给出的地址，下一拍才出数据。
                    // ★ 地址对齐：输出像素 p=ecnt-1 在 cycle ecnt 才被送出，而它用的
                    //   数据是 addr_active(ecnt-1) 那一拍给出的地址 —— 也就是要在
                    //   cycle ecnt-1 就把 col(p) 塞进地址寄存器。因此本拍要给的
                    //   是【下一列】的源地址，即 h_idx_nx（原始版本就是这么做的）。
                    if (S_blank) begin
                        S_rd_addr <= {SWA{1'b0}};
                    end
                    else if ((S_ecnt >= S_x0) && (S_ecnt < S_xend)) begin
                        // 图像区：本拍给出的地址供【下一拍（下一个 tick）】那个像素用，
                        // 所以要给 S_hidx（=idx(本列)），不是 h_idx_nx。
                        // 未节流时地址走"本拍给 -> 下拍出数据 -> 再下拍取用"两级，
                        // 节流后 rd_q 会在两个 tick 之间稳住，实际只隔 1 个 tick，
                        // 直接给 h_idx_nx 会让整幅图右移一个像素（仿真已验证）。
                        S_rd_addr <= S_hidx[SWA-1:0];
                        S_hidx    <= h_idx_nx;
                        S_hacc    <= h_acc_f;
                    end
                    else begin
                        // 左右黑边：地址压 0，顺便给图像首列做预热
                        S_rd_addr <= {SWA{1'b0}};
                    end

                    if ((S_ecnt >= 16'd1) && (S_ecnt <= DW)) begin
                        // 输出像素序号 p = ecnt-1，落在 [x0, x0+sw) 才取源像素
                        m_data  <= (!S_blank && (S_ecnt > S_x0) && (S_ecnt <= S_xend))
                                   ? rd_q : 24'd0;
                        m_valid <= 1'b1;
                    end

                    if (S_ecnt == (DW + 1)) begin
                        // 一行发完
                        S_emit <= 1'b0;
                        S_orow <= S_orow + 11'd1;
                        if (!S_blank) begin
                            S_vacc <= v_acc_f;
                            S_vsrc <= v_src_nx;
                            // 下一个图像行还要用同一源行（放大）就留着，否则放掉
                            if (!(row_nx_in && (pk_sel ? next1 : next0))) begin
                                if (pk_sel == 1'b0) S_full0 <= 1'b0;
                                else                S_full1 <= 1'b0;
                            end
                        end
                        if ((S_orow + 11'd1) >= DH)
                            m_done <= 1'b1;
                    end
                    else begin
                        S_ecnt <= S_ecnt + 1'b1;
                    end
                    end   // if (S_pace == 3'd0)
                end
            end
            default: S_st <= ST_IDLE;
            endcase
        end
    end

endmodule
