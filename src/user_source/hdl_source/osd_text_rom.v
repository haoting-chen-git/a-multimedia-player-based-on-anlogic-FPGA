// ==========================================================================
// osd_text_rom.v —— 由 make_osd_assets.py 自动生成，请勿手工修改
// 改文案：编辑同目录 osd_text.txt，然后重跑 make_osd_assets.py
// 布局：COLS=20 列 x ROWS=4 行，单元格 32x32 px
//   row 0: {CLK}  <- 动态时钟
//   row 1: （空行，用于时钟与标题之间的分隔）
//   row 2: 安路 FPGA 播放器
//   row 3: 轮播周期 {PER} 秒 {MODE}
//
// 单元格索引 I_cell = {row[1:0], col[4:0]}，
// 用位拼接而不是 row*COLS+col —— 后者会出现除以 20，TD 会综合出除法器。
// ==========================================================================

module osd_text_rom(
    input  wire [6:0]             I_cell,      // {row, col}
    input  wire [63:0]            I_clk_str,   // HH:MM:SS 共 8 字节，低字节在前
    input  wire [7:0]             I_per_char,  // 轮播周期数字的字符编码
    input  wire [7:0]             I_mode_char0,// 轮播状态词第 1 字（自/手）
    input  wire [7:0]             I_mode_char1,// 轮播状态词第 2 字（动/动）
    output reg  [7:0]             O_char,      // 字符编码
    output reg                    O_char_en,   // 1 = 这一格写字
    output reg  [23:0]            O_char_rgb   // 字色
);

always @(*) begin
    O_char     = 8'h20;
    O_char_en  = 1'b0;
    O_char_rgb = 24'hFFFFFF;

    case (I_cell)
        7'h06: begin O_char = I_clk_str[7:0]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h07: begin O_char = I_clk_str[15:8]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h08: begin O_char = I_clk_str[23:16]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h09: begin O_char = I_clk_str[31:24]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h0A: begin O_char = I_clk_str[39:32]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h0B: begin O_char = I_clk_str[47:40]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h0C: begin O_char = I_clk_str[55:48]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h0D: begin O_char = I_clk_str[63:56]; O_char_en = 1'b1; O_char_rgb = 24'hFFD54F; end
        7'h44: begin O_char = 8'h87; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h45: begin O_char = 8'h92; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h47: begin O_char = 8'h46; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h48: begin O_char = 8'h50; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h49: begin O_char = 8'h47; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h4A: begin O_char = 8'h41; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h4C: begin O_char = 8'h8A; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h4D: begin O_char = 8'h8B; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h4E: begin O_char = 8'h83; O_char_en = 1'b1; O_char_rgb = 24'hFFEB3B; end
        7'h64: begin O_char = 8'h93; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h65: begin O_char = 8'h8A; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h66: begin O_char = 8'h82; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h67: begin O_char = 8'h8C; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h69: begin O_char = I_per_char; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h6B: begin O_char = 8'h90; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h6D: begin O_char = I_mode_char0; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        7'h6E: begin O_char = I_mode_char1; O_char_en = 1'b1; O_char_rgb = 24'h00FF80; end
        default: begin O_char = 8'h20; O_char_en = 1'b0; O_char_rgb = 24'h000000; end
    endcase
end

endmodule
