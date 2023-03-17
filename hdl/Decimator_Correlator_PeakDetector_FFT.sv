`timescale 1ns / 1ns

module Decimator_Correlator_PeakDetector_FFT
#(
    parameter IN_DW = 32,           // input data width
    parameter OUT_DW = 32,          // correlator output data width
    parameter TAP_DW = 32,
    parameter PSS_LEN = 128,
    parameter [TAP_DW * PSS_LEN - 1 : 0] PSS_LOCAL_0 = {(PSS_LEN * TAP_DW){1'b0}},
    parameter [TAP_DW * PSS_LEN - 1 : 0] PSS_LOCAL_1 = {(PSS_LEN * TAP_DW){1'b0}},
    parameter [TAP_DW * PSS_LEN - 1 : 0] PSS_LOCAL_2 = {(PSS_LEN * TAP_DW){1'b0}},
    parameter ALGO = 1,
    parameter WINDOW_LEN = 8,
    parameter HALF_CP_ADVANCE = 1,
    parameter NFFT = 8,
    parameter USE_TAP_FILE = 1,
    parameter TAP_FILE = "",
    parameter MULT_REUSE = 0,

    localparam FFT_OUT_DW = 32,
    localparam FFT_LEN = 2 ** NFFT,
    localparam CIC_RATE = FFT_LEN / 128,
    localparam MAX_CP_LEN = 20 * FFT_LEN / 256
)
(
    input                                       clk_i,
    input                                       reset_ni,
    input   wire           [IN_DW-1:0]          s_axis_in_tdata,
    input                                       s_axis_in_tvalid,

    output                                      PBCH_valid_o,
    output                                      SSS_valid_o,
    output                 [FFT_OUT_DW-1:0]     m_axis_out_tdata,
    output                                      m_axis_out_tvalid,
    
    // debug outputs
    output  wire            [IN_DW-1:0]         m_axis_cic_debug_tdata,
    output  wire                                m_axis_cic_debug_tvalid,
    output  reg                                 peak_detected_debug_o,
    output  wire            [FFT_OUT_DW-1:0]    fft_result_debug_o,
    output  wire                                fft_sync_debug_o,
    output  wire            [15:0]              sync_wait_counter_debug_o,
    output  reg                                 fft_demod_PBCH_start_o,
    output  reg                                 fft_demod_SSS_start_o
);

wire [IN_DW - 1 : 0] m_axis_cic_tdata;
wire                 m_axis_cic_tvalid;
assign m_axis_cic_debug_tdata = m_axis_cic_tdata;
assign m_axis_cic_debug_tvalid = m_axis_cic_tvalid;

cic_d #(
    .INP_DW(IN_DW/2),
    .OUT_DW(IN_DW/2),
    .CIC_R(CIC_RATE),
    .CIC_N(3),
    .VAR_RATE(0)
)
cic_real(
    .clk(clk_i),
    .reset_n(reset_ni),
    .s_axis_in_tdata(s_axis_in_tdata[IN_DW / 2 - 1 -: IN_DW / 2]),
    .s_axis_in_tvalid(s_axis_in_tvalid),
    .m_axis_out_tdata(m_axis_cic_tdata[IN_DW / 2 - 1 -: IN_DW / 2]),
    .m_axis_out_tvalid(m_axis_cic_tvalid)
);

cic_d #(
    .INP_DW(IN_DW / 2),
    .OUT_DW(IN_DW / 2),
    .CIC_R(CIC_RATE),
    .CIC_N(3),
    .VAR_RATE(0)
)
cic_imag(
    .clk(clk_i),
    .reset_n(reset_ni),
    .s_axis_in_tdata(s_axis_in_tdata[IN_DW - 1 -: IN_DW / 2]),
    .s_axis_in_tvalid(s_axis_in_tvalid),
    .m_axis_out_tdata(m_axis_cic_tdata[IN_DW - 1 -: IN_DW / 2])
);

PSS_detector #(
    .IN_DW(IN_DW),
    .OUT_DW(OUT_DW),
    .TAP_DW(TAP_DW),
    .PSS_LEN(PSS_LEN),
    .PSS_LOCAL_0(PSS_LOCAL_0),
    .PSS_LOCAL_1(PSS_LOCAL_1),
    .PSS_LOCAL_2(PSS_LOCAL_2),
    .ALGO(ALGO),
    .USE_TAP_FILE(USE_TAP_FILE),
    .MULT_REUSE(MULT_REUSE)
)
PSS_detector_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .s_axis_in_tdata(m_axis_cic_tdata),
    .s_axis_in_tvalid(m_axis_cic_tvalid),

    .N_id_2_valid_o(peak_detected)
);

wire peak_detected;
assign peak_detected_debug_o = peak_detected;

wire [FFT_OUT_DW - 1 : 0] fft_result, fft_result_demod;
wire [FFT_OUT_DW / 2 - 1 : 0] fft_result_re, fft_result_im;
wire fft_result_demod_valid;
wire fft_sync;

assign fft_result_debug_o = fft_result;
assign fft_sync_debug_o = fft_sync;

// this delay line is needed because peak_detected goes high
// at the end of SSS symbol plus some additional delay
localparam DELAY_LINE_LEN = FFT_LEN == 256 ? 14 : (FFT_LEN == 512 ? 16 : 0);  // only defined for FFT_LEN 256 and 512 !
reg [IN_DW-1:0] delay_line_data  [0 : DELAY_LINE_LEN - 1];
reg             delay_line_valid [0 : DELAY_LINE_LEN - 1];
always @(posedge clk_i) begin
    if (!reset_ni) begin
        for (integer i = 0; i < DELAY_LINE_LEN; i = i + 1) begin
            delay_line_data[i] = '0;
            delay_line_valid[i] = '0;
        end
    end else begin
        delay_line_data[0] <= s_axis_in_tdata;
        delay_line_valid[0] <= s_axis_in_tvalid;
        for (integer i = 0; i < DELAY_LINE_LEN - 1; i = i + 1) begin
            delay_line_data[i+1] <= delay_line_data[i];
            delay_line_valid[i+1] <= delay_line_valid[i];
        end
    end
end

localparam SFN_MAX = 1023;
localparam SUBFRAMES_PER_FRAME = 20;
localparam SYM_PER_SF = 14;
localparam SFN_WIDTH = $clog2(SFN_MAX);
localparam SUBFRAME_NUMBER_WIDTH = $clog2(SUBFRAMES_PER_FRAME - 1);
localparam SYMBOL_NUMBER_WIDTH = $clog2(SYM_PER_SF - 1);
localparam USER_WIDTH = SFN_WIDTH + SUBFRAME_NUMBER_WIDTH + SYMBOL_NUMBER_WIDTH + $clog2(MAX_CP_LEN);

reg [IN_DW - 1 : 0]     fs_out_tdata;
reg [USER_WIDTH - 1 : 0] fs_out_tuser;
reg fs_out_tvalid;
reg fs_out_SSB_start;
reg fs_out_symbol_start;
wire fs_out_tlast;

frame_sync #(
    .IN_DW(IN_DW),
    .NFFT(NFFT)
)
frame_sync_i
(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .N_id_2_i(),
    .N_id_2_valid_i(peak_detected),
    .ibar_SSB_i(),
    .ibar_SSB_valid_i(),
    .s_axis_in_tdata(delay_line_data[DELAY_LINE_LEN - 1]),
    .s_axis_in_tvalid(delay_line_valid[DELAY_LINE_LEN - 1]),

    .PSS_detector_mode_o(),
    .requested_N_id_2_o(),

    .m_axis_out_tdata(fs_out_tdata),
    .m_axis_out_tuser(fs_out_tuser),
    .m_axis_out_tlast(fs_out_tlast),
    .m_axis_out_tvalid(fs_out_tvalid),
    .symbol_start_o(fs_out_symbol_start),
    .SSB_start_o(fs_out_SSB_start)
);

FFT_demod #(
    .IN_DW(IN_DW),
    .OUT_DW(FFT_OUT_DW),
    .HALF_CP_ADVANCE(HALF_CP_ADVANCE),
    .NFFT(NFFT),
    .USE_TAP_FILE(USE_TAP_FILE),
    .TAP_FILE(TAP_FILE)
)
FFT_demod_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .SSB_start_i(fs_out_SSB_start),
    .s_axis_in_tdata(fs_out_tdata),
    .s_axis_in_tlast(fs_out_tlast),
    .s_axis_in_tuser(fs_out_tuser),
    .s_axis_in_tvalid(fs_out_tvalid),
    .m_axis_out_tdata(m_axis_out_tdata),
    .m_axis_out_tuser(),
    .m_axis_out_tlast(),
    .m_axis_out_tvalid(m_axis_out_tvalid),
    .PBCH_valid_o(PBCH_valid_o),
    .SSS_valid_o(SSS_valid_o)
);

endmodule

