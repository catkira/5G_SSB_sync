`timescale 1ns / 1ns

module receiver
#(
    parameter IN_DW = 32,           // input data width
    parameter OUT_DW = 32,          // correlator output data width
    parameter TAP_DW = 32,
    parameter PSS_LEN = 128,
    parameter [TAP_DW * PSS_LEN - 1 : 0] PSS_LOCAL_0 = {(PSS_LEN * TAP_DW){1'b0}},
    parameter [TAP_DW * PSS_LEN - 1 : 0] PSS_LOCAL_1 = {(PSS_LEN * TAP_DW){1'b0}},
    parameter [TAP_DW * PSS_LEN - 1 : 0] PSS_LOCAL_2 = {(PSS_LEN * TAP_DW){1'b0}},
    parameter WINDOW_LEN = 8,
    parameter HALF_CP_ADVANCE = 1,
    parameter USE_TAP_FILE = 1,
    parameter TAP_FILE_0 = "",
    parameter TAP_FILE_1 = "",
    parameter TAP_FILE_2 = "",
    parameter TAP_FILE_PATH = "",
    parameter LLR_DW = 8,
    parameter ADDRESS_WIDTH = 16,
    parameter NFFT = 8,
    parameter XSERIES = "OLD",        // use "OLD" for Zynq7, "NEW" for MPSoC
    parameter MULT_REUSE = 0,
    parameter CLK_FREQ = 3840000,
    parameter SEPARATE_IQ_IN = 0,
    parameter VARIABLE_DETECTION_FACTOR = 1,
    parameter VARIABLE_NOISE_LIMIT = 1,
    parameter INITIAL_DETECTION_SHIFT = 4,
    parameter INITIAL_CFO_MODE = 0,

    localparam BLK_EXP_LEN = 8,
    localparam FFT_LEN = 2 ** NFFT,
    localparam MAX_CP_LEN = 20 * FFT_LEN / 256,
    localparam CIC_RATE = FFT_LEN / 128,    
    localparam FFT_OUT_DW = 16,
    localparam N_id_1_MAX = 335,
    localparam N_id_MAX = 1007,
    localparam DDS_PHASE_DW = 20,
    localparam DDS_OUT_DW = 32,
    localparam CFO_DW = 20,
    localparam COMPL_MULT_OUT_DW = 32,
    localparam AXI_ADDRESS_WIDTH = 11
)
(
    input                                           clk_i,
    input                                           reset_n,

    input                                           sample_clk_i,
    input   wire    [IN_DW - 1 : 0]                 s_axis_in_tdata,
    input   wire    [IN_DW / 2 - 1 : 0]             s_axis_in_I_tdata,
    input   wire    [IN_DW / 2 - 1 : 0]             s_axis_in_Q_tdata,
    input                                           s_axis_in_tvalid,

    output                                          PBCH_valid_o,
    output                                          SSS_valid_o,

    output          [FFT_OUT_DW - 1 : 0]            m_axis_cest_out_tdata,
    output          [1 : 0]                         m_axis_cest_out_tuser,
    output                                          m_axis_cest_out_tlast,
    output                                          m_axis_cest_out_tvalid,

    output          [LLR_DW - 1 : 0]                m_axis_llr_out_tdata,
    output          [1 : 0]                         m_axis_llr_out_tuser,
    output                                          m_axis_llr_out_tlast,
    output                                          m_axis_llr_out_tvalid,

    output          [FFT_OUT_DW - 1 : 0]            m_axis_demod_out_tdata,
    output                                          m_axis_demod_out_tvalid,
    output          [$clog2(N_id_1_MAX) - 1 : 0]    m_axis_SSS_tdata,
    output                                          m_axis_SSS_tvalid,
    output          [$clog2(N_id_MAX) - 1 : 0]      N_id_o,
    output                                          N_id_valid_o,

    // AXI stream interface to DMA core
    output          [FFT_OUT_DW - 1 : 0]            m_axis_out_tdata,
    output                                          m_axis_out_tvalid,
    output                                          m_axis_out_tlast,

    // AXI lite interface
    // write address channel
    input           [ADDRESS_WIDTH - 1 : 0]     s_axi_if_awaddr,
    input                                       s_axi_if_awvalid,
    output  reg                                 s_axi_if_awready,
    
    // write data channel
    input           [31 : 0]                    s_axi_if_wdata,
    input           [ 3 : 0]                    s_axi_if_wstrb,      // not used
    input                                       s_axi_if_wvalid,
    output  reg                                 s_axi_if_wready,

    // write response channel
    output          [ 1 : 0]                    s_axi_if_bresp,
    output  reg                                 s_axi_if_bvalid,
    input                                       s_axi_if_bready,

    // read address channel
    input           [ADDRESS_WIDTH - 1 : 0]     s_axi_if_araddr,
    input                                       s_axi_if_arvalid,
    output  reg                                 s_axi_if_arready,

    // read data channel
    output  reg     [31 : 0]                    s_axi_if_rdata,
    output          [ 1 : 0]                    s_axi_if_rresp,
    output  reg                                 s_axi_if_rvalid,
    input                                       s_axi_if_rready, 
    
    // debug outputs
    output  wire    [IN_DW - 1 : 0]                 m_axis_cic_debug_tdata,
    output  wire                                    m_axis_cic_debug_tvalid,
    output  wire    [OUT_DW - 1 : 0]                m_axis_correlator_debug_tdata,
    output  wire                                    m_axis_correlator_debug_tvalid,
    output  wire    [15:0]                          sync_wait_counter_debug_o,
    output  wire                                    peak_detected_debug_o
);

localparam OFFSET_ADDR_WIDTH = ADDRESS_WIDTH - 2;

// --------------   wires for axis_axil_fifo_i  ---------------------
wire            [OFFSET_ADDR_WIDTH - 1 : 0]     s_axi_fifo_awaddr;
wire                                        s_axi_fifo_awvalid;
wire                                        s_axi_fifo_awready;
// write data channel
wire            [31 : 0]                    s_axi_fifo_wdata;
wire            [ 3 : 0]                    s_axi_fifo_wstrb;     // not used
wire                                        s_axi_fifo_wvalid;
wire                                        s_axi_fifo_wready;
// write response channel
wire            [ 1 : 0]                    s_axi_fifo_bresp;
wire                                        s_axi_fifo_bvalid;
wire                                        s_axi_fifo_bready;
// read address channel
wire            [OFFSET_ADDR_WIDTH - 1 : 0]     s_axi_fifo_araddr;
wire                                        s_axi_fifo_arvalid;
wire                                        s_axi_fifo_arready;
// read data channel
wire            [31 : 0]                    s_axi_fifo_rdata;
wire            [ 1 : 0]                    s_axi_fifo_rresp;
wire                                        s_axi_fifo_rvalid;
wire                                        s_axi_fifo_rready;
// ------------------------------------------------------------------

// ------------------------------------------------------------------
// --------------   wires for PSS_detector_i  -----------------------
wire            [OFFSET_ADDR_WIDTH - 1 : 0]     s_axi_pss_awaddr;
wire                                            s_axi_pss_awvalid;
wire                                            s_axi_pss_awready;
// write data channel
wire            [31 : 0]                        s_axi_pss_wdata;
wire            [ 3 : 0]                        s_axi_pss_wstrb;     // not used
wire                                            s_axi_pss_wvalid;
wire                                            s_axi_pss_wready;
// write response channel
wire            [ 1 : 0]                        s_axi_pss_bresp;
wire                                            s_axi_pss_bvalid;
wire                                            s_axi_pss_bready;
// read address channel
wire            [OFFSET_ADDR_WIDTH - 1 : 0]     s_axi_pss_araddr;
wire                                            s_axi_pss_arvalid;
wire                                            s_axi_pss_arready;
// read data channel
wire            [31 : 0]                        s_axi_pss_rdata;
wire            [ 1 : 0]                        s_axi_pss_rresp;
wire                                            s_axi_pss_rvalid;
wire                                            s_axi_pss_rready;
// ------------------------------------------------------------------
// --------------   wires for receiver_regmap_i  -----------------------
wire            [OFFSET_ADDR_WIDTH - 1 : 0]     s_axi_rx_awaddr;
wire                                            s_axi_rx_awvalid;
wire                                            s_axi_rx_awready;
// write data channel
wire            [31 : 0]                        s_axi_rx_wdata;
wire            [ 3 : 0]                        s_axi_rx_wstrb;
wire                                            s_axi_rx_wvalid;
wire                                            s_axi_rx_wready;
// write response channel
wire            [ 1 : 0]                        s_axi_rx_bresp;
wire                                            s_axi_rx_bvalid;
wire                                            s_axi_rx_bready;
// read address channel
wire            [OFFSET_ADDR_WIDTH - 1 : 0]     s_axi_rx_araddr;
wire                                            s_axi_rx_arvalid;
wire                                            s_axi_rx_arready;
// read data channel
wire            [31 : 0]                        s_axi_rx_rdata;
wire            [ 1 : 0]                        s_axi_rx_rresp;
wire                                            s_axi_rx_rvalid;
wire                                            s_axi_rx_rready;
// ------------------------------------------------------------------


wire [IN_DW - 1 : 0] m_axis_cic_tdata;
wire                 m_axis_cic_tvalid;
assign m_axis_cic_debug_tdata = m_axis_cic_tdata;
assign m_axis_cic_debug_tvalid = m_axis_cic_tvalid;


reg [COMPL_MULT_OUT_DW - 1 : 0] mult_out_tdata;
reg                             mult_out_tvalid;

localparam SAMPLE_ID_WIDTH = 64;
reg [SAMPLE_ID_WIDTH - 1 : 0]  sample_id_cdc_data;
wire                            sample_id_cdc_valid;

reg [IN_DW - 1 : 0]             FIFO_out_tdata;
reg [IN_DW - 1 : 0]             FIFO_out_f;
reg                             FIFO_out_tvalid;

wire [IN_DW - 1 : 0] in_data = SEPARATE_IQ_IN ? {s_axis_in_Q_tdata, s_axis_in_I_tdata} : s_axis_in_tdata;
wire reset_ni = reset_n; // port was renamed from reset_ni to reset_n so that Vivado infers correct polarity
wire [1 : 0] fs_state;

wire CFO_mode;
wire signed [7 : 0] sample_cnt_mismatch;
wire [15: 0] missed_SSBs;

always @(posedge clk_i) begin
    if (!reset_ni)  FIFO_out_f <= '0;
    else if (FIFO_out_tvalid) FIFO_out_f <= FIFO_out_tdata;
end

receiver_regmap #(
    .ID(0),
    .ADDRESS_WIDTH(AXI_ADDRESS_WIDTH)
)
receiver_regmap_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),

    .fs_state_i(fs_state),
    .rx_signal_i(FIFO_out_f),
    .N_id_2_i(N_id_2_f),
    .N_id_i(N_id_f),
    .sample_cnt_mismatch_i(sample_cnt_mismatch),
    .missed_SSBs_i(missed_SSBs),
    .ibar_SSB_i(ibar_SSB_f),

    .s_axi_if_awaddr(s_axi_rx_awaddr),
    .s_axi_if_awvalid(s_axi_rx_awvalid),
    .s_axi_if_awready(s_axi_rx_awready),
    .s_axi_if_wdata(s_axi_rx_wdata),
    .s_axi_if_wstrb(s_axi_rx_wstrb),
    .s_axi_if_wvalid(s_axi_rx_wvalid),
    .s_axi_if_wready(s_axi_rx_wready),
    .s_axi_if_bresp(s_axi_rx_bresp),
    .s_axi_if_bvalid(s_axi_rx_bvalid),
    .s_axi_if_bready(s_axi_rx_bready),
    .s_axi_if_araddr(s_axi_rx_araddr),
    .s_axi_if_arvalid(s_axi_rx_arvalid),
    .s_axi_if_arready(s_axi_rx_arready),
    .s_axi_if_rdata(s_axi_rx_rdata),
    .s_axi_if_rresp(s_axi_rx_rresp),
    .s_axi_if_rvalid(s_axi_rx_rvalid),
    .s_axi_if_rready(s_axi_rx_rready)
);

// reset CDC from clk_i to sample_clk_i
reg reset_f, reset_ff;
wire reset_sample_clk = reset_ff;
always @(posedge sample_clk_i) begin
    reset_f <= reset_ni;
    reset_ff <= reset_f;
end

reg [SAMPLE_ID_WIDTH - 1 : 0] sample_cnt;
always @(posedge sample_clk_i) begin
    if (!reset_ff) sample_cnt <= '0;
    else if (s_axis_in_tvalid) sample_cnt <= sample_cnt + 1;
end

AXIS_FIFO #(
    .DATA_WIDTH(IN_DW),
    .USER_WIDTH(0),
    .FIFO_LEN(16),
    .ASYNC(1)
)
sample_in_cdc_i(
    .clk_i(sample_clk_i),
    .s_reset_ni(reset_sample_clk),

    .s_axis_in_tdata(in_data),
    .s_axis_in_tvalid(s_axis_in_tvalid),
    .s_axis_in_tuser(),
    .s_axis_in_tlast(),
    .s_axis_in_tfull(),

    .out_clk_i(clk_i),
    .m_reset_ni(reset_ni),
    .m_axis_out_tready(1'b1),
    .m_axis_out_tdata(FIFO_out_tdata),
    .m_axis_out_tvalid(FIFO_out_tvalid),
    .m_axis_out_tuser(),
    .m_axis_out_tlast(),
    .m_axis_out_tlevel(),
    .m_axis_out_tempty()
);

AXIS_FIFO #(
    .DATA_WIDTH(SAMPLE_ID_WIDTH),
    .USER_WIDTH(0),
    .FIFO_LEN(32),  // a bit larger, needs to cover complex_multiplier latency
    .ASYNC(1)
)
sample_id_cdc_i(
    .clk_i(sample_clk_i),
    .s_reset_ni(reset_sample_clk),

    .s_axis_in_tdata(sample_cnt),
    .s_axis_in_tvalid(s_axis_in_tvalid),
    .s_axis_in_tuser(),
    .s_axis_in_tlast(),
    .s_axis_in_tfull(),

    .out_clk_i(clk_i),
    .m_reset_ni(reset_ni),
    .m_axis_out_tready(1'b1),
    .m_axis_out_tdata(sample_id_cdc_data),
    .m_axis_out_tvalid(sample_id_cdc_valid),
    .m_axis_out_tuser(),
    .m_axis_out_tlast(),
    .m_axis_out_tlevel(),
    .m_axis_out_tempty()
);

reg signed [DDS_PHASE_DW - 1 : 0]   CFO_DDS_inc, CFO_DDS_inc_f;
reg                                 CFO_valid;
wire [DDS_OUT_DW - 1 : 0]           DDS_out;
wire DDS_out_valid;
reg [DDS_PHASE_DW - 1 : 0]          DDS_phase;
reg                                 DDS_phase_valid;

always @(posedge clk_i) begin
    if (!reset_ni) begin
        DDS_phase <= '0;
        DDS_phase_valid <= '0;
        CFO_DDS_inc_f <= '0;
        DDS_phase_valid <= 1;
    end 
    else begin
        if (CFO_mode == 0) begin
            if (CFO_valid) begin
                // CFO_DDS_inc_f <= '0; // deactive CFO correction for debugging
                CFO_DDS_inc_f <= CFO_DDS_inc_f - CFO_DDS_inc;  // incoming CFO_DDS_inc are relative to last one !
            end
            if(FIFO_out_tvalid) begin
                DDS_phase <= DDS_phase + CFO_DDS_inc_f;
            end
        end else begin
            // manual CFO mode
            CFO_DDS_inc_f <= '0;
            DDS_phase <= '0;
        end
    end
end

dds #(
    .PHASE_DW(DDS_PHASE_DW),
    .OUT_DW(DDS_OUT_DW/2),
    .USE_TAYLOR(1),
    .LUT_DW(16),
    .SIN_COS(1),
    .NEGATIVE_SINE(0),
    .NEGATIVE_COSINE(0),
    .USE_LUT_FILE(0)
)
dds_i(
    .clk(clk_i),
    .reset_n(reset_ni),

    .s_axis_phase_tdata(DDS_phase),
    .s_axis_phase_tvalid(DDS_phase_valid),

    .m_axis_out_tdata(DDS_out),
    .m_axis_out_tvalid(DDS_out_valid),

    .m_axis_out_sin_tdata(),
    .m_axis_out_sin_tvalid(),
    .m_axis_out_cos_tdata(),
    .m_axis_out_cos_tvalid()
);

complex_multiplier #(
    .OPERAND_WIDTH_A(DDS_OUT_DW/2),
    .OPERAND_WIDTH_B(IN_DW/2),
    .OPERAND_WIDTH_OUT(COMPL_MULT_OUT_DW/2),
    .BLOCKING(0),
    .GROWTH_BITS(-2),  // input is rotating vector with length 2^(IN_DW/2 - 1), therefore bit growth is 2 bits less than worst case
                       // TODO: WARNING, this can create an overflow if the DDS is not perfect (outputs vector with length > 1)
    .BYTE_ALIGNED(0)
)
complex_multiplier_i(
    .aclk(clk_i),
    .aresetn(reset_ni),

    .s_axis_a_tdata(DDS_out),
    .s_axis_a_tvalid(DDS_out_valid),
    .s_axis_b_tdata(FIFO_out_tdata),
    .s_axis_b_tvalid(FIFO_out_tvalid),

    .m_axis_dout_tdata(mult_out_tdata),
    .m_axis_dout_tvalid(mult_out_tvalid)
);

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

    .s_axis_in_tdata(mult_out_tdata[COMPL_MULT_OUT_DW / 2 - 1 -: COMPL_MULT_OUT_DW / 2]),
    .s_axis_in_tvalid(mult_out_tvalid),

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

    .s_axis_in_tdata(mult_out_tdata[COMPL_MULT_OUT_DW - 1 -: COMPL_MULT_OUT_DW / 2]),
    .s_axis_in_tvalid(mult_out_tvalid),

    .m_axis_out_tdata(m_axis_cic_tdata[IN_DW - 1 -: IN_DW / 2]),
    .m_axis_out_tvalid()
);


wire [OUT_DW - 1 : 0] correlator_tdata;
wire correlator_tvalid;
assign m_axis_correlator_debug_tdata = correlator_tdata;
assign m_axis_correlator_debug_tvalid = correlator_tvalid;

reg N_id_2_valid;
wire [1 : 0] N_id_2;
wire [1 : 0] PSS_detector_mode;
wire [1 : 0] requested_N_id_2;
assign peak_detected_debug_o = N_id_2_valid;

PSS_detector #(
    .IN_DW(IN_DW),
    .OUT_DW(OUT_DW),
    .TAP_DW(TAP_DW),
    .CFO_DW(CFO_DW),
    .DDS_DW(DDS_PHASE_DW),
    .PSS_LEN(PSS_LEN),
    .PSS_LOCAL_0(PSS_LOCAL_0),
    .PSS_LOCAL_1(PSS_LOCAL_1),
    .PSS_LOCAL_2(PSS_LOCAL_2),
    .ALGO(1'b0),
    .USE_MODE(1),
    .USE_TAP_FILE(USE_TAP_FILE),
    .TAP_FILE_0(TAP_FILE_0),
    .TAP_FILE_1(TAP_FILE_1),
    .TAP_FILE_2(TAP_FILE_2),
    .TAP_FILE_PATH(TAP_FILE_PATH),
    .MULT_REUSE(MULT_REUSE),
    .VARIABLE_DETECTION_FACTOR(VARIABLE_DETECTION_FACTOR),
    .VARIABLE_NOISE_LIMIT(VARIABLE_NOISE_LIMIT),
    .INITIAL_DETECTION_SHIFT(INITIAL_DETECTION_SHIFT),
    .INITIAL_CFO_MODE(INITIAL_CFO_MODE)
)
PSS_detector_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),

    .s_axis_in_tdata(m_axis_cic_tdata),
    .s_axis_in_tvalid(m_axis_cic_tvalid),
    .mode_i(PSS_detector_mode),
    .requested_N_id_2_i(requested_N_id_2),

    .N_id_2_valid_o(N_id_2_valid),
    .N_id_2_o(N_id_2),
    .CFO_DDS_inc_o(CFO_DDS_inc),
    .CFO_angle_o(),
    .CFO_valid_o(CFO_valid),
    .CFO_mode_o(CFO_mode),

    .s_axi_awaddr(s_axi_pss_awaddr),
    .s_axi_awvalid(s_axi_pss_awvalid),
    .s_axi_awready(s_axi_pss_awready),
    .s_axi_wdata(s_axi_pss_wdata),
    .s_axi_wstrb(s_axi_pss_wstrb),
    .s_axi_wvalid(s_axi_pss_wvalid),
    .s_axi_wready(s_axi_pss_wready),
    .s_axi_bresp(s_axi_pss_bresp),
    .s_axi_bvalid(s_axi_pss_bvalid),
    .s_axi_bready(s_axi_pss_bready),
    .s_axi_araddr(s_axi_pss_araddr),
    .s_axi_arvalid(s_axi_pss_arvalid),
    .s_axi_arready(s_axi_pss_arready),
    .s_axi_rdata(s_axi_pss_rdata),
    .s_axi_rresp(s_axi_pss_rresp),
    .s_axi_rvalid(s_axi_pss_rvalid),
    .s_axi_rready(s_axi_pss_rready)    
);

reg [1 : 0] N_id_2_f;
always @(posedge clk_i) begin
    if (!reset_ni) N_id_2_f <= '0;
    else N_id_2_f <= N_id_2_valid ? N_id_2 : N_id_2_f;
end

reg [$clog2(N_id_MAX) - 1 : 0] N_id_f;
always @(posedge clk_i) begin
    if (!reset_ni) N_id_f <= '0;
    else N_id_f <= N_id_valid ? N_id : N_id_f;
end

wire [FFT_OUT_DW - 1 : 0] fft_result, fft_result_demod;
wire [FFT_OUT_DW / 2 - 1 : 0] fft_result_re, fft_result_im;
wire fft_result_demod_valid;
wire fft_sync;

function integer calc_delay;
    input dummy;  // Vivado wants that a function has at least one argument
    begin
        // that's a bunch of magic numbers
        // TODO: make this nicer / more systematic
        if (FFT_LEN == 256) begin
            if (MULT_REUSE == 0)        calc_delay = 14;  // ok with new PSS_correlator_mr
            else if (MULT_REUSE == 1)   calc_delay = 24;  // ok with new PSS_correlator_mr, 24
            else if (MULT_REUSE == 2)   calc_delay = 25;  // ok with new PSS_correlator_mr, 24 + 1
            else if (MULT_REUSE == 4)   calc_delay = 26;  // ok with new PSS_correlator_mr, 24 + 2
            else if (MULT_REUSE == 8)   calc_delay = 28;  // ok with new PSS_correlator_mr, 24 + 4
            else if (MULT_REUSE == 16)  calc_delay = 32;  // ok with new PSS_correlator_mr, 24 + 8
            else if (MULT_REUSE == 32)  calc_delay = 40;  // ok with new PSS_correlator_mr, 24 + 16
        end else if (FFT_LEN == 512) begin
            if (MULT_REUSE == 0)        calc_delay = 16;  // ok with new PSS_correlator_mr
            else if (MULT_REUSE == 1)   calc_delay = 26;  // ok with new PSS_correlator_mr, 26
            else if (MULT_REUSE == 2)   calc_delay = 27;  // ok with new PSS_correlator_mr, 26 + 1
            else if (MULT_REUSE == 4)   calc_delay = 30;  // ok with new PSS_correlator_mr, 26 + 4
            else if (MULT_REUSE == 8)   calc_delay = 36;  // ok with new PSS_correlator_mr, 26 + 10
            else if (MULT_REUSE == 16)  calc_delay = 48;  // ok with new PSS_correlator_mr, 26 + 22
            else if (MULT_REUSE == 32)  calc_delay = 72;  // ok with new PSS_correlator_mr, 26 + 46
        end else begin
            $display("Error: FFT_LEN = %d is not supported!", FFT_LEN);
            $finish();
        end
    end
endfunction

// this delay line is needed because peak_detected goes high
// at the end of SSS symbol plus some additional delay
localparam DELAY_LINE_LEN = calc_delay(0);
reg [IN_DW-1:0] delay_line_data  [0 : DELAY_LINE_LEN - 1];
reg             delay_line_valid [0 : DELAY_LINE_LEN - 1];
always @(posedge clk_i) begin
    if (!reset_ni) begin
        for (integer i = 0; i < DELAY_LINE_LEN; i = i + 1) begin
            delay_line_data[i] = '0;
            delay_line_valid[i] = '0;
        end
    end else begin
        delay_line_data[0] <= mult_out_tdata;
        delay_line_valid[0] <= mult_out_tvalid;
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
wire [2 : 0] ibar_SSB;
wire ibar_SSB_valid;
reg [2 : 0] ibar_SSB_f;
localparam N_ID_MAX = 1007;
reg [$clog2(N_ID_MAX) - 1 : 0] N_id;
reg N_id_valid;
wire sample_id_fifo_valid_in;
wire reset_fft_n;
wire [1 : 0] fs_N_id_2;
wire fs_N_id_2_valid;

frame_sync #(
    .IN_DW(IN_DW),
    .NFFT(NFFT),
    .CLK_FREQ(CLK_FREQ)
)
frame_sync_i
(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .N_id_2_i(N_id_2),
    .N_id_2_valid_i(N_id_2_valid),
    .ibar_SSB_i(ibar_SSB),
    .ibar_SSB_valid_i(ibar_SSB_valid),
    .s_axis_in_tdata(delay_line_data[DELAY_LINE_LEN - 1]),
    .s_axis_in_tvalid(delay_line_valid[DELAY_LINE_LEN - 1]),

    .PSS_detector_mode_o(PSS_detector_mode),
    .requested_N_id_2_o(requested_N_id_2),

    .sample_id_valid(sample_id_fifo_valid_in),

    .m_axis_out_tdata(fs_out_tdata),
    .m_axis_out_tuser(fs_out_tuser),
    .m_axis_out_tlast(fs_out_tlast),
    .m_axis_out_tvalid(fs_out_tvalid),
    .symbol_start_o(fs_out_symbol_start),
    .SSB_start_o(fs_out_SSB_start),
    .reset_fft_no(reset_fft_n),
    .state_o(fs_state),
    .sample_cnt_mismatch_o(sample_cnt_mismatch),
    .missed_SSBs_o(missed_SSBs),
    .N_id_2_o(fs_N_id_2),
    .N_id_2_valid_o(fs_N_id_2_valid)
);

wire [SAMPLE_ID_WIDTH - 1 : 0] sample_id_fifo_out_data;
wire sample_id_fifo_out_ready;
wire sample_id_fifo_out_valid;
AXIS_FIFO #(
    .DATA_WIDTH(SAMPLE_ID_WIDTH),
    .USER_WIDTH(0),
    .FIFO_LEN(32),  // needs to be large enough to cover FFT_demod latency
    .ASYNC(0)
)
sample_id_fifo_i(
    .clk_i(clk_i),
    .s_reset_ni(reset_ni),

    .s_axis_in_tdata(sample_id_cdc_data),
    .s_axis_in_tvalid(sample_id_fifo_valid_in),
    .s_axis_in_tuser(),
    .s_axis_in_tlast(),
    .s_axis_in_tfull(),

    .out_clk_i(clk_i),
    .m_reset_ni(reset_ni),
    .m_axis_out_tready(sample_id_fifo_out_ready),
    .m_axis_out_tdata(sample_id_fifo_out_data),
    .m_axis_out_tvalid(sample_id_fifo_out_valid),
    .m_axis_out_tuser(),
    .m_axis_out_tlast(),
    .m_axis_out_tlevel(),
    .m_axis_out_tempty()
);

localparam FFT_DEMOD_OUT_USER_WIDTH = SFN_WIDTH + SUBFRAME_NUMBER_WIDTH + SYMBOL_NUMBER_WIDTH + BLK_EXP_LEN + 1;
wire [FFT_OUT_DW - 1 : 0]                   fft_demod_out_tdata;
wire [FFT_DEMOD_OUT_USER_WIDTH - 1 : 0]     fft_demod_out_tuser;
wire                                        fft_demod_out_tvalid;
wire                                        fft_demod_out_tlast;
assign m_axis_demod_out_tdata = fft_demod_out_tdata;
assign m_axis_demod_out_tvalid = fft_demod_out_tvalid;
wire reset_fft_demod_n = reset_fft_n && reset_ni;
FFT_demod #(
    .IN_DW(IN_DW),
    .OUT_DW(FFT_OUT_DW),
    .HALF_CP_ADVANCE(HALF_CP_ADVANCE),
    .NFFT(NFFT),
    .BLK_EXP_LEN(BLK_EXP_LEN),
    .XSERIES(XSERIES),
    .USE_TAP_FILE(USE_TAP_FILE),
    .TAP_FILE(""),
    .TAP_FILE_PATH(TAP_FILE_PATH)
)
FFT_demod_i(
    .clk_i(clk_i),
    .reset_ni(reset_fft_demod_n),
    .SSB_start_i(fs_out_SSB_start),
    .s_axis_in_tdata(fs_out_tdata),
    .s_axis_in_tlast(fs_out_tlast),
    .s_axis_in_tuser(fs_out_tuser),
    .s_axis_in_tvalid(fs_out_tvalid),
    .m_axis_out_tdata(fft_demod_out_tdata),
    .m_axis_out_tuser(fft_demod_out_tuser),
    .m_axis_out_tlast(fft_demod_out_tlast),
    .m_axis_out_tvalid(fft_demod_out_tvalid),
    .PBCH_valid_o(PBCH_valid_o),
    .SSS_valid_o(SSS_valid_o)
);

wire rgs_overflow;
ressource_grid_subscriber #(
    .IQ_WIDTH(FFT_OUT_DW),
    .BLK_EXP_LEN(BLK_EXP_LEN)
)
ressource_grid_subscriber_i(
    .clk_i(clk_i),
    .reset_ni(reset_fft_demod_n),

    .s_axis_iq_tdata(fft_demod_out_tdata),
    .s_axis_iq_tuser(fft_demod_out_tuser),
    .s_axis_iq_tvalid(fft_demod_out_tvalid),
    .s_axis_iq_tlast(fft_demod_out_tlast),

    .sample_id_data(sample_id_fifo_out_data),
    .sample_id_valid(sample_id_fifo_out_valid),
    .sample_id_ready(sample_id_fifo_out_ready),

    .m_axis_fifo_tdata(m_axis_out_tdata),
    .m_axis_fifo_tvalid(m_axis_out_tvalid),
    .m_axis_fifo_tlast(m_axis_out_tlast),
    .overflow_o(rgs_overflow)
);

SSS_detector #(
    .IN_DW(FFT_OUT_DW)
)
SSS_detector_i(
    .clk_i(clk_i),
    .reset_ni(reset_fft_demod_n),
    .N_id_2_i(fs_N_id_2),
    .N_id_2_valid_i(fs_N_id_2_valid),
    .s_axis_in_tdata(fft_demod_out_tdata),
    .s_axis_in_tvalid(SSS_valid_o),
    .m_axis_out_tdata(m_axis_SSS_tdata),
    .m_axis_out_tvalid(m_axis_SSS_tvalid),
    .N_id_o(N_id),
    .N_id_valid_o(N_id_valid)
);

assign N_id_o = N_id;
assign N_id_valid_o = N_id_valid;

channel_estimator #(
    .IN_DW(FFT_OUT_DW)
)
channel_estimator_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .N_id_i(N_id),
    .N_id_valid_i(N_id_valid),
    .s_axis_in_tdata(fft_demod_out_tdata),
    .s_axis_in_tuser(fft_demod_out_tuser[0]),
    .s_axis_in_tvalid(fft_demod_out_tvalid),

    .m_axis_out_tdata(m_axis_cest_out_tdata),
    .m_axis_out_tuser(m_axis_cest_out_tuser),
    .m_axis_out_tlast(m_axis_cest_out_tlast),
    .m_axis_out_tvalid(m_axis_cest_out_tvalid),

    .debug_ibar_SSB_o(ibar_SSB),
    .debug_ibar_SSB_valid_o(ibar_SSB_valid)
);

always @(posedge clk_i) begin
    if (!reset_ni) ibar_SSB_f <= '0;
    else if (ibar_SSB_valid) ibar_SSB_f <= ibar_SSB;
end

demap #(
    .IQ_DW(FFT_OUT_DW / 2),
    .LLR_DW(LLR_DW)
)
demap_i(
    .clk_i(clk_i),
    .reset_ni(reset_fft_demod_n),

    .s_axis_in_tdata(m_axis_cest_out_tdata),
    .s_axis_in_tuser(m_axis_cest_out_tuser),
    .s_axis_in_tlast(m_axis_cest_out_tlast),
    .s_axis_in_tvalid(m_axis_cest_out_tvalid),

    .m_axis_out_tdata(m_axis_llr_out_tdata),
    .m_axis_out_tuser(m_axis_llr_out_tuser),
    .m_axis_out_tlast(m_axis_llr_out_tlast),
    .m_axis_out_tvalid(m_axis_llr_out_tvalid)
);

axis_axil_fifo #(
    .DATA_WIDTH(LLR_DW),
    .FIFO_LEN(2048),
    .USER_WIDTH(1),
    .ADDRESS_WIDTH(OFFSET_ADDR_WIDTH)
)
axis_axil_fifo_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),

    .s_axis_in_tdata(m_axis_llr_out_tdata),
    .s_axis_in_tuser(m_axis_llr_out_tuser[0]),
    .s_axis_in_tlast(m_axis_llr_out_tlast),
    .s_axis_in_tvalid(m_axis_llr_out_tvalid && (m_axis_llr_out_tuser == 1)),  // only store PBCH bits in FIFO
    .s_axis_in_tfull(),

    .s_axi_awaddr(s_axi_fifo_awaddr),
    .s_axi_awvalid(s_axi_fifo_awvalid),
    .s_axi_awready(s_axi_fifo_awready),
    .s_axi_wdata(s_axi_fifo_wdata),
    .s_axi_wstrb(s_axi_fifo_wstrb),
    .s_axi_wvalid(s_axi_fifo_wvalid),
    .s_axi_wready(s_axi_fifo_wready),
    .s_axi_bresp(s_axi_fifo_bresp),
    .s_axi_bvalid(s_axi_fifo_bvalid),
    .s_axi_bready(s_axi_fifo_bready),
    .s_axi_araddr(s_axi_fifo_araddr),
    .s_axi_arvalid(s_axi_fifo_arvalid),
    .s_axi_arready(s_axi_fifo_arready),
    .s_axi_rdata(s_axi_fifo_rdata),
    .s_axi_rresp(s_axi_fifo_rresp),
    .s_axi_rvalid(s_axi_fifo_rvalid),
    .s_axi_rready(s_axi_fifo_rready)
);

// ------------------------------------------------------------------

axil_interconnect_wrap_1x4 #(
    .DATA_WIDTH(32),
    .ADDR_WIDTH(ADDRESS_WIDTH),
    .M_REGIONS(1),
    .M00_ADDR_WIDTH(OFFSET_ADDR_WIDTH),
    .M01_ADDR_WIDTH(OFFSET_ADDR_WIDTH),
    .M02_ADDR_WIDTH(OFFSET_ADDR_WIDTH),
    .M03_ADDR_WIDTH(OFFSET_ADDR_WIDTH),
    .M00_BASE_ADDR(0 << OFFSET_ADDR_WIDTH),
    .M01_BASE_ADDR(1 << OFFSET_ADDR_WIDTH),
    .M02_BASE_ADDR(2 << OFFSET_ADDR_WIDTH),
    .M03_BASE_ADDR(3 << OFFSET_ADDR_WIDTH)
)
axil_interconnect_wrap_1x4_i(
    .clk(clk_i),
    .rst(!reset_ni),

    .s00_axil_awaddr(s_axi_if_awaddr),
    .s00_axil_awvalid(s_axi_if_awvalid),
    .s00_axil_awready(s_axi_if_awready),
    .s00_axil_wdata(s_axi_if_wdata),
    .s00_axil_wstrb(s_axi_if_wstrb),
    .s00_axil_wvalid(s_axi_if_wvalid),
    .s00_axil_wready(s_axi_if_wready),
    .s00_axil_bresp(s_axi_if_bresp),
    .s00_axil_bvalid(s_axi_if_bvalid),
    .s00_axil_bready(s_axi_if_bready),
    .s00_axil_araddr(s_axi_if_araddr),
    .s00_axil_arvalid(s_axi_if_arvalid),
    .s00_axil_arready(s_axi_if_arready),
    .s00_axil_rdata(s_axi_if_rdata),
    .s00_axil_rresp(s_axi_if_rresp),
    .s00_axil_rvalid(s_axi_if_rvalid),
    .s00_axil_rready(s_axi_if_rready),

    .m00_axil_awaddr(s_axi_fifo_awaddr),
    .m00_axil_awvalid(s_axi_fifo_awvalid),
    .m00_axil_awready(s_axi_fifo_awready),
    .m00_axil_wdata(s_axi_fifo_wdata),
    .m00_axil_wstrb(s_axi_fifo_wstrb),
    .m00_axil_wvalid(s_axi_fifo_wvalid),
    .m00_axil_wready(s_axi_fifo_wready),
    .m00_axil_bresp(s_axi_fifo_bresp),
    .m00_axil_bvalid(s_axi_fifo_bvalid),
    .m00_axil_bready(s_axi_fifo_bready),
    .m00_axil_araddr(s_axi_fifo_araddr),
    .m00_axil_arvalid(s_axi_fifo_arvalid),
    .m00_axil_arready(s_axi_fifo_arready),
    .m00_axil_rdata(s_axi_fifo_rdata),
    .m00_axil_rresp(s_axi_fifo_rresp),
    .m00_axil_rvalid(s_axi_fifo_rvalid),
    .m00_axil_rready(s_axi_fifo_rready),

    .m01_axil_awaddr(s_axi_pss_awaddr),
    .m01_axil_awvalid(s_axi_pss_awvalid),
    .m01_axil_awready(s_axi_pss_awready),
    .m01_axil_wdata(s_axi_pss_wdata),
    .m01_axil_wstrb(s_axi_pss_wstrb),
    .m01_axil_wvalid(s_axi_pss_wvalid),
    .m01_axil_wready(s_axi_pss_wready),
    .m01_axil_bresp(s_axi_pss_bresp),
    .m01_axil_bvalid(s_axi_pss_bvalid),
    .m01_axil_bready(s_axi_pss_bready),
    .m01_axil_araddr(s_axi_pss_araddr),
    .m01_axil_arvalid(s_axi_pss_arvalid),
    .m01_axil_arready(s_axi_pss_arready),
    .m01_axil_rdata(s_axi_pss_rdata),
    .m01_axil_rresp(s_axi_pss_rresp),
    .m01_axil_rvalid(s_axi_pss_rvalid),
    .m01_axil_rready(s_axi_pss_rready),

    .m02_axil_awaddr(s_axi_rx_awaddr),
    .m02_axil_awvalid(s_axi_rx_awvalid),
    .m02_axil_awready(s_axi_rx_awready),
    .m02_axil_wdata(s_axi_rx_wdata),
    .m02_axil_wstrb(s_axi_rx_wstrb),
    .m02_axil_wvalid(s_axi_rx_wvalid),
    .m02_axil_wready(s_axi_rx_wready),
    .m02_axil_bresp(s_axi_rx_bresp),
    .m02_axil_bvalid(s_axi_rx_bvalid),
    .m02_axil_bready(s_axi_rx_bready),
    .m02_axil_araddr(s_axi_rx_araddr),
    .m02_axil_arvalid(s_axi_rx_arvalid),
    .m02_axil_arready(s_axi_rx_arready),
    .m02_axil_rdata(s_axi_rx_rdata),
    .m02_axil_rresp(s_axi_rx_rresp),
    .m02_axil_rvalid(s_axi_rx_rvalid),
    .m02_axil_rready(s_axi_rx_rready)
);

endmodule