`timescale 1ns / 1ps
//======================================================================
//  audio_melody_rom.v —— 4 首旋律（每张图片一首）
//----------------------------------------------------------------------
//  由 make_audio_assets.py 自动生成，请勿手改。
//  O_note : 音高下标 0..23（C4..B5），送给 audio_note_inc
//  O_dur  : 时值，单位 = 2^11 = 2048 采样 ≈ 42.67ms
//  每首 8 个音，播放到最后一个自动回到第 0 个循环
//======================================================================
module audio_melody_rom
(
    input  wire        [1:0]  I_track,   // 0..3，可接图片缓冲区编号
    input  wire        [2:0]  I_step,    // 0..7，曲内第几个音
    output reg         [4:0]  O_note,
    output reg         [4:0]  O_dur
);

    wire [4:0] w_key = {I_track, I_step};   // 5 位 = 曲目*8 + 步

    always @(*) begin
        case (w_key)
            5'd0 : begin O_note = 5'd0 ; O_dur = 5'd6 ; end  // 曲0 第0音 C4   6 units
            5'd1 : begin O_note = 5'd4 ; O_dur = 5'd6 ; end  // 曲0 第1音 E4   6 units
            5'd2 : begin O_note = 5'd7 ; O_dur = 5'd6 ; end  // 曲0 第2音 G4   6 units
            5'd3 : begin O_note = 5'd9 ; O_dur = 5'd12; end  // 曲0 第3音 A4   12 units
            5'd4 : begin O_note = 5'd7 ; O_dur = 5'd6 ; end  // 曲0 第4音 G4   6 units
            5'd5 : begin O_note = 5'd4 ; O_dur = 5'd6 ; end  // 曲0 第5音 E4   6 units
            5'd6 : begin O_note = 5'd2 ; O_dur = 5'd6 ; end  // 曲0 第6音 D4   6 units
            5'd7 : begin O_note = 5'd0 ; O_dur = 5'd18; end  // 曲0 第7音 C4   18 units
            5'd8 : begin O_note = 5'd2 ; O_dur = 5'd6 ; end  // 曲1 第0音 D4   6 units
            5'd9 : begin O_note = 5'd6 ; O_dur = 5'd6 ; end  // 曲1 第1音 F#4  6 units
            5'd10: begin O_note = 5'd9 ; O_dur = 5'd6 ; end  // 曲1 第2音 A4   6 units
            5'd11: begin O_note = 5'd11; O_dur = 5'd12; end  // 曲1 第3音 B4   12 units
            5'd12: begin O_note = 5'd9 ; O_dur = 5'd6 ; end  // 曲1 第4音 A4   6 units
            5'd13: begin O_note = 5'd6 ; O_dur = 5'd6 ; end  // 曲1 第5音 F#4  6 units
            5'd14: begin O_note = 5'd4 ; O_dur = 5'd6 ; end  // 曲1 第6音 E4   6 units
            5'd15: begin O_note = 5'd2 ; O_dur = 5'd18; end  // 曲1 第7音 D4   18 units
            5'd16: begin O_note = 5'd4 ; O_dur = 5'd6 ; end  // 曲2 第0音 E4   6 units
            5'd17: begin O_note = 5'd7 ; O_dur = 5'd6 ; end  // 曲2 第1音 G4   6 units
            5'd18: begin O_note = 5'd11; O_dur = 5'd6 ; end  // 曲2 第2音 B4   6 units
            5'd19: begin O_note = 5'd12; O_dur = 5'd12; end  // 曲2 第3音 C5   12 units
            5'd20: begin O_note = 5'd11; O_dur = 5'd6 ; end  // 曲2 第4音 B4   6 units
            5'd21: begin O_note = 5'd7 ; O_dur = 5'd6 ; end  // 曲2 第5音 G4   6 units
            5'd22: begin O_note = 5'd9 ; O_dur = 5'd6 ; end  // 曲2 第6音 A4   6 units
            5'd23: begin O_note = 5'd4 ; O_dur = 5'd18; end  // 曲2 第7音 E4   18 units
            5'd24: begin O_note = 5'd5 ; O_dur = 5'd6 ; end  // 曲3 第0音 F4   6 units
            5'd25: begin O_note = 5'd9 ; O_dur = 5'd6 ; end  // 曲3 第1音 A4   6 units
            5'd26: begin O_note = 5'd12; O_dur = 5'd6 ; end  // 曲3 第2音 C5   6 units
            5'd27: begin O_note = 5'd14; O_dur = 5'd12; end  // 曲3 第3音 D5   12 units
            5'd28: begin O_note = 5'd12; O_dur = 5'd6 ; end  // 曲3 第4音 C5   6 units
            5'd29: begin O_note = 5'd9 ; O_dur = 5'd6 ; end  // 曲3 第5音 A4   6 units
            5'd30: begin O_note = 5'd7 ; O_dur = 5'd6 ; end  // 曲3 第6音 G4   6 units
            5'd31: begin O_note = 5'd5 ; O_dur = 5'd18; end  // 曲3 第7音 F4   18 units
            default: begin O_note = 5'd0; O_dur = 5'd6; end
        endcase
    end

endmodule
