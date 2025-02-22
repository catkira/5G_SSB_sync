module BWP_extractor #(
    parameter IN_DW = 16,           // input data width
    parameter NFFT = 8,
    parameter BLK_EXP_LEN = 8,

    localparam SFN_MAX = 1023,
    localparam SUBFRAMES_PER_FRAME = 20,
    localparam SYM_PER_SF = 14,
    localparam SFN_WIDTH = $clog2(SFN_MAX),
    localparam SUBFRAME_NUMBER_WIDTH = $clog2(SUBFRAMES_PER_FRAME - 1),
    localparam SYMBOL_NUMBER_WIDTH = $clog2(SYM_PER_SF - 1),
    localparam USER_WIDTH_IN = SFN_WIDTH + SUBFRAME_NUMBER_WIDTH + SYMBOL_NUMBER_WIDTH + BLK_EXP_LEN,
    localparam USER_WIDTH_OUT = SFN_WIDTH + SUBFRAME_NUMBER_WIDTH + SYMBOL_NUMBER_WIDTH + BLK_EXP_LEN + 1
)
(
    input                                       clk_i,
    input                                       reset_ni,
    input   wire       [IN_DW - 1 : 0]          s_axis_in_tdata,
    input   wire       [USER_WIDTH_IN - 1 : 0]  s_axis_in_tuser,    
    input                                       s_axis_in_tlast,
    input                                       s_axis_in_tvalid,

    output  reg        [IN_DW - 1 : 0]          m_axis_out_tdata,
    output  reg        [USER_WIDTH_OUT - 1 : 0] m_axis_out_tuser,
    output  reg                                 m_axis_out_tlast,
    output  reg                                 m_axis_out_tvalid,
    output  reg                                 PBCH_valid_o,
    output  reg                                 SSS_valid_o
);

localparam SYMBOLS_PER_PRB = 12;
wire [SYMBOL_NUMBER_WIDTH - 1 : 0] sym = s_axis_in_tuser[SYMBOL_NUMBER_WIDTH + BLK_EXP_LEN - 1 -: SYMBOL_NUMBER_WIDTH];
wire [SUBFRAME_NUMBER_WIDTH - 1 : 0] subframe = s_axis_in_tuser[SUBFRAME_NUMBER_WIDTH + SYMBOL_NUMBER_WIDTH + BLK_EXP_LEN - 1 -: SUBFRAME_NUMBER_WIDTH];
localparam FFT_LEN = 2 ** NFFT;
reg [$clog2(FFT_LEN) - 1 : 0] sc_cnt;
wire is_PBCH_symbol = (sym == 3 || sym == 4 || sym == 5) && (subframe == 0);
wire is_SSS_symbol = (sym == 4) && (subframe == 0);
localparam SSS_LEN = 127;
localparam SSS_START = FFT_LEN / 2 - (SSS_LEN + 1) / 2;
wire valid_SSS_SC = (sc_cnt >= SSS_START) && (sc_cnt <= SSS_START + SSS_LEN - 1);
localparam PBCH_LEN = 20 * SYMBOLS_PER_PRB;
localparam PBCH_START = FFT_LEN / 2 - PBCH_LEN / 2;
wire valid_PBCH_SC = (sc_cnt >= PBCH_START) && (sc_cnt <= PBCH_START + PBCH_LEN - 1);

function integer calc_num_prb;
    input integer NFFT;
begin
    case (NFFT)
        8 : calc_num_prb = 20;
        9 : calc_num_prb = 25;
        10 : calc_num_prb = 52;
        11 : calc_num_prb = 106;
        default: $display("NFFT = %d is not supported!", NFFT);
    endcase
end
endfunction

localparam BWP_LEN = calc_num_prb(NFFT) * SYMBOLS_PER_PRB;

localparam SC_START = FFT_LEN / 2 - BWP_LEN / 2;
localparam SC_END = SC_START + BWP_LEN;
wire valid_SC = (sc_cnt >= SC_START) && (sc_cnt <= SC_END - 1);

always @(posedge clk_i) begin
    if (!reset_ni) begin
        m_axis_out_tvalid <= '0;
        m_axis_out_tdata <= '0;
        m_axis_out_tuser <= '0;
        m_axis_out_tlast <= '0;
        sc_cnt <= '0;
        PBCH_valid_o <= '0;
        SSS_valid_o <= '0;
    end else begin
        SSS_valid_o <= valid_SSS_SC && is_SSS_symbol;
        PBCH_valid_o <= valid_PBCH_SC && is_PBCH_symbol;
        m_axis_out_tvalid <= valid_SC && s_axis_in_tvalid;
        m_axis_out_tdata <= s_axis_in_tdata;
        m_axis_out_tuser <= {s_axis_in_tuser, is_PBCH_symbol};
        m_axis_out_tlast <= s_axis_in_tvalid && (sc_cnt == (FFT_LEN - 1 - SC_START));

        if (s_axis_in_tvalid) begin
            if (sc_cnt == FFT_LEN - 1)  sc_cnt <= '0;
            else                        sc_cnt <= sc_cnt + 1;
        end
    end
end

endmodule