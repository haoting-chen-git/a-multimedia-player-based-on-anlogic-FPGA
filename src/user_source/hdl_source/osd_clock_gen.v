// ==========================================================================
// osd_clock_gen.v —— OSD 动态走时时钟 (HH:MM:SS)
//   安路 EG4S20 (HX4S20C) HDMI 多媒体播放器
//
// 用法：
//   osd_clock_gen u_clk_gen(.I_clk(video_clk), .I_rst(rst_all),
//                          .I_restart(~display_valid),   // 首图显示前停在 00:00:00
//                          .O_str(clk_str));
//   然后把 clk_str 接到 osd_text_rom 的 I_clk_str。
//
// 说明：
//   * 没有外部 RTC，所以这是“从开始播放算起”的走时，不是真实墙钟。
//   * 数 video_clk 得到 1 秒。CLK_FREQ_HZ 不精确只会让走时慢/快千分之几，
//     演示完全看不出来；想绝对准确就改成实测值。
//   * 全程比较 + 减法，没有除法器。
// ==========================================================================

module osd_clock_gen #(
    parameter CLK_FREQ_HZ = 25_175_000     // video_clk 频率
)(
    input  wire        I_clk,
    input  wire        I_rst,              // 高有效
    input  wire        I_restart,          // 高 = 清零并停表（接 ~display_valid）
    output reg  [63:0] O_str               // 8 字节，"HH:MM:SS"
                                           // O_str[7:0]='H', [15:8]='H', [23:16]=':',
                                           // [31:24]='M', [39:32]='M', [47:40]=':',
                                           // [55:48]='S', [63:56]='S'
);

    localparam [24:0] CNT_MAX = CLK_FREQ_HZ - 1;

    reg [24:0] cnt;
    reg [5:0]  sec;
    reg [5:0]  min;
    reg [5:0]  hour;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            cnt  <= 25'd0;
            sec  <= 6'd0;
            min  <= 6'd0;
            hour <= 6'd0;
        end
        else if (I_restart) begin
            cnt  <= 25'd0;
            sec  <= 6'd0;
            min  <= 6'd0;
            hour <= 6'd0;
        end
        else if (cnt == CNT_MAX) begin
            cnt <= 25'd0;
            if (sec == 6'd59) begin
                sec <= 6'd0;
                if (min == 6'd59) begin
                    min  <= 6'd0;
                    hour <= (hour == 6'd23) ? 6'd0 : (hour + 6'd1);
                end
                else
                    min <= min + 6'd1;
            end
            else
                sec <= sec + 6'd1;
        end
        else
            cnt <= cnt + 25'd1;
    end

    // 十位（0..5）：用比较代替除法
    function [3:0] tens;
        input [5:0] v;
        begin
            if      (v >= 6'd50) tens = 4'd5;
            else if (v >= 6'd40) tens = 4'd4;
            else if (v >= 6'd30) tens = 4'd3;
            else if (v >= 6'd20) tens = 4'd2;
            else if (v >= 6'd10) tens = 4'd1;
            else                 tens = 4'd0;
        end
    endfunction

    // 个位：减去对应的 10 的倍数（必须用完整的 v 减，不能只取低 4 位）
    function [3:0] ones;
        input [5:0] v;
        begin
            if      (v >= 6'd50) ones = v - 6'd50;
            else if (v >= 6'd40) ones = v - 6'd40;
            else if (v >= 6'd30) ones = v - 6'd30;
            else if (v >= 6'd20) ones = v - 6'd20;
            else if (v >= 6'd10) ones = v - 6'd10;
            else                 ones = v[3:0];
        end
    endfunction

    function [7:0] dchar;                 // 0..9 -> '0'..'9'
        input [3:0] d;
        begin
            dchar = {4'h3, d};
        end
    endfunction

    always @(*) begin
        O_str[7:0]   = dchar(tens(hour));
        O_str[15:8]  = dchar(ones(hour));
        O_str[23:16] = 8'h3A;             // ':'
        O_str[31:24] = dchar(tens(min));
        O_str[39:32] = dchar(ones(min));
        O_str[47:40] = 8'h3A;             // ':'
        O_str[55:48] = dchar(tens(sec));
        O_str[63:56] = dchar(ones(sec));
    end

endmodule
