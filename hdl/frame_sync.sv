`timescale 1ns / 1ns

module frame_sync #(
    parameter IN_DW = 32,
    parameter NFFT = 8,
    parameter CLK_FREQ = 3840000,

    localparam FFT_LEN = 2 ** NFFT,
    localparam CP1_LEN = 20 * FFT_LEN / 256,
    localparam CP2_LEN = 18 * FFT_LEN / 256,
    localparam OUT_DW = IN_DW,
    localparam MAX_CP_LEN = CP1_LEN,
    localparam SFN_MAX = 1023,
    localparam SUBFRAMES_PER_FRAME = 20,
    localparam SYM_PER_SF = 14,
    localparam SFN_WIDTH = $clog2(SFN_MAX),
    localparam SUBFRAME_NUMBER_WIDTH = $clog2(SUBFRAMES_PER_FRAME - 1),
    localparam SYMBOL_NUMBER_WIDTH = $clog2(SYM_PER_SF - 1),
    localparam USER_WIDTH = SFN_WIDTH + SUBFRAME_NUMBER_WIDTH + SYMBOL_NUMBER_WIDTH + $clog2(MAX_CP_LEN)
)
(
    input                                           clk_i,
    input                                           reset_ni,
    input   wire       [IN_DW - 1 : 0]              s_axis_in_tdata,
    input                                           s_axis_in_tvalid,

    input              [1 : 0]                      N_id_2_i,
    input                                           N_id_2_valid_i,
    input              [2 : 0]                      ibar_SSB_i,
    input                                           ibar_SSB_valid_i,

    output  reg        [1 : 0]                      PSS_detector_mode_o,
    output  reg        [1 : 0]                      requested_N_id_2_o,

    // interface to sample_id FIFO
    output  wire                                    sample_id_valid,

    // output to FFT_demod
    output  reg        [OUT_DW - 1 : 0]             m_axis_out_tdata,
    output  reg        [USER_WIDTH - 1 : 0]         m_axis_out_tuser,
    output  reg                                     m_axis_out_tlast,
    output  reg                                     m_axis_out_tvalid,
    output  reg                                     symbol_start_o,
    output  reg                                     SSB_start_o,
    output  reg                                     reset_fft_no,
    output  reg        [1 : 0]                      N_id_2_o,
    output  reg                                     N_id_2_valid_o,

    // output to regmap
    output  wire       [1 : 0]                      state_o,
    output  wire signed   [7 : 0]                   sample_cnt_mismatch_o,
    output  wire       [15 : 0]                     missed_SSBs_o
);

reg [$clog2(MAX_CP_LEN) - 1: 0] CP_len;
reg [SFN_WIDTH - 1 : 0] sfn;
reg [SUBFRAME_NUMBER_WIDTH - 1 : 0] subframe_number;
reg [SYMBOL_NUMBER_WIDTH - 1 : 0] sym_cnt;
reg [$clog2(MAX_CP_LEN) - 1 : 0] current_CP_len;
reg [IN_DW - 1 : 0] s_axis_in_tdata_f;
always @(posedge clk_i) begin
    if (!reset_ni) begin
        m_axis_out_tdata <= '0;
        m_axis_out_tuser <= '0;
        s_axis_in_tdata_f <= '0;
    end else begin
        s_axis_in_tdata_f <= s_axis_in_tdata;
        m_axis_out_tdata <= s_axis_in_tdata_f;
        m_axis_out_tuser <= {sfn, subframe_number, sym_cnt, current_CP_len};
    end
end

always @(posedge clk_i) begin
    if (!reset_ni)  requested_N_id_2_o <= '0;
    else if (N_id_2_valid_i)  requested_N_id_2_o <= N_id_2_i;
end

// process that forwards N_id_2
always @(posedge clk_i) begin
    N_id_2_o <= reset_ni ? N_id_2_i : '0;
    N_id_2_valid_o <= reset_ni ? N_id_2_valid_i : '0;
end

reg              [2 : 0]                      ibar_SSB;
always @(posedge clk_i) begin
    if (!reset_ni) ibar_SSB <= '0;
    else ibar_SSB <= ibar_SSB_valid_i ? ibar_SSB_i : ibar_SSB;
end

// ---------------------------------------------------------------------------------------------------//
// FSM for controlling PSS detector
localparam [1 : 0] PSS_DETECTOR_MODE_SEARCH = 0;
localparam [1 : 0] PSS_DETECTOR_MODE_FIND   = 1;
localparam [1 : 0] PSS_DETECTOR_MODE_PAUSE  = 1;
localparam CLKS_20MS = $rtoi(CLK_FREQ * 0.02);
localparam CLKS_PSS_EARLY_WAKEUP = $rtoi(CLK_FREQ * 0.0001); // start PSS detector 0.1 ms before expected SSB
localparam CLKS_PSS_LATE_TOLERANCE = $rtoi(CLK_FREQ * 0.0001); // keep PSS detector running until 0.1ms after expected SSB
reg [$clog2(CLKS_20MS + CLKS_PSS_LATE_TOLERANCE) - 1 : 0] clks_since_SSB;
reg [1 : 0] PSS_state;
localparam [1 : 0] SEARCH_PSS = 0;
localparam [1 : 0] FIND_PSS = 1;
localparam [1 : 0] PAUSE_PSS = 2;
reg [15 : 0] missed_SSBs;
assign missed_SSBs_o = missed_SSBs;
always @(posedge clk_i) begin
    if (!reset_ni) begin
        PSS_state <= SEARCH_PSS;
        clks_since_SSB <= '0;
        PSS_detector_mode_o <= '0;
        missed_SSBs <= '0;
    end else begin
        PSS_detector_mode_o <= PSS_state;
        case (PSS_state)
            SEARCH_PSS : begin  // search PSS with any N_id_2
                if (N_id_2_valid_i) begin
                    missed_SSBs <= '0;
                    PSS_state <= PAUSE_PSS;
                    clks_since_SSB <= 1;
                end else begin
                    clks_since_SSB <= clks_since_SSB + 1;
                end
            end
            PAUSE_PSS : begin // PAUSE until next PSS is expected    
                if (clks_since_SSB > (CLKS_20MS - CLKS_PSS_EARLY_WAKEUP)) begin
                    PSS_state <= FIND_PSS;
                end else begin
                    clks_since_SSB <= clks_since_SSB + 1;
                end
            end
            FIND_PSS : begin  // FIND PSS with same N_id_2 as last one
                if (clks_since_SSB > (CLKS_20MS + CLKS_PSS_LATE_TOLERANCE)) begin
                    $display("frame_sync: did not find PSS, going back to SEARCH mode!");
                    PSS_state <= SEARCH_PSS;
                    missed_SSBs <= missed_SSBs + 1;
                end else if (N_id_2_valid_i) begin
                    $display("frame_sync: found PSS in FIND mode, putting PSS detectore in PAUSE mode");
                    PSS_state <= PAUSE_PSS;
                    clks_since_SSB <= 1;
                end else begin
                    clks_since_SSB <= clks_since_SSB + 1;
                end
            end
        endcase
    end
end

// This function is only used for debugging for now
// assumes SSB pattern case A (TS 38.213)
// N_id_2_valid arrives here 1 symbol late, therefore start with 3 instead of 2
function is_SSB_location;
    input [SFN_WIDTH - 1 : 0] sfn;
    input [SUBFRAME_NUMBER_WIDTH - 1 : 0] subframe;
    input [SYMBOL_NUMBER_WIDTH - 1 : 0] sym;
    input [$clog2(FFT_LEN + MAX_CP_LEN) - 1 : 0] sample_cnt;
    input [2 : 0] ibar_SSB;
    input [$clog2(MAX_CP_LEN) - 1 : 0] current_CP_len;
    input [3 : 0] sample_ahead;
    begin
        if ((sample_cnt + sample_ahead) % (FFT_LEN + current_CP_len) != 0) is_SSB_location = 0;
        else begin
            case (ibar_SSB)
                0 : begin
                    is_SSB_location = (sample_cnt == 0) && (sym == 3) && (subframe == 0);
                end
                1 : begin
                    is_SSB_location = (sample_cnt == 0) && (sym == 9) && (subframe == 0);
                end
                2 : begin
                    is_SSB_location = (sample_cnt == 0) && (sym == 3) && (subframe == 1);
                end
                3 : begin
                    is_SSB_location = (sample_cnt == 0) && (sym == 9) && (subframe == 1);
                end
            endcase
        end
    end
endfunction

// ---------------------------------------------------------------------------------------------------//
// FSM for keeping track of current subframe number and symbol number within a subframe 
// and sending the current CP length to the FFT_demod core
//
// sfn is the current system frame number
// subframe_number is current subframe number within the current frame
// sym_cnt is the current symbol number within the current subframe
//
// TODO:  - add timeout to WAIT_FOR_IBAR state
//        - make SYNCED and WAIT_FOR_IBAR substates of a single state and reduce duplicate code
reg [$clog2(FFT_LEN + MAX_CP_LEN) - 1 : 0] sample_cnt;
reg find_SSB;

localparam SYMS_BTWN_SSB = SUBFRAMES_PER_FRAME * SYM_PER_SF;
reg [$clog2(SYMS_BTWN_SSB + 100) - 1 : 0] syms_since_last_SSB;

reg [1 : 0] state;
assign state_o = state;
localparam [1 : 0] WAIT_FOR_SSB = 0;
localparam [1 : 0] WAIT_FOR_IBAR = 1;
localparam [1 : 0] SYNCED = 2;
reg [SYMBOL_NUMBER_WIDTH - 1 : 0] sym_cnt_next;
localparam FIND_EARLY_SAMPLES = 4;
reg signed [7 : 0] sample_cnt_mismatch;
assign sample_cnt_mismatch_o = sample_cnt_mismatch;

always @(posedge clk_i) begin
    if (!reset_ni) begin
        sfn <= '0;
        subframe_number <= '0;
        sym_cnt <= '0;
        sym_cnt_next = '0;
        sample_cnt <= '0;
        state <= WAIT_FOR_SSB;
        current_CP_len <= CP2_LEN;
        find_SSB <= '0;
        SSB_start_o <= '0;
        syms_since_last_SSB <= '0;
        m_axis_out_tvalid <= '0;
        m_axis_out_tlast <= '0;
        sample_cnt_mismatch <= '0;
        reset_fft_no <= '0;
    end else begin
        case (state)
            WAIT_FOR_SSB: begin
                SSB_start_o <= '0;
                find_SSB <= '0;                
                if (N_id_2_valid_i) begin
                    sample_cnt <= 1;
                    m_axis_out_tvalid <= s_axis_in_tvalid;
                    // SSB_pattern for case A is [2, 8, 16, 22]
                    // whether we are on symbol 2 or symbol 8 depends on ibar_SSB
                    // assume for now that we are at symbol 2
                    // it might have to be corrected once ibar_SSB arrives
                    // N_id_2_valid arrives here 1 symbol late, therefore start with 3 instead of 2
                    current_CP_len <= CP2_LEN;
                    sym_cnt <= 3;
                    state <= WAIT_FOR_IBAR;
                    syms_since_last_SSB <= '0;
                    // SSB_start_o <= 1;
                    reset_fft_no <= 1;
                end else begin
                    // SSB_start_o <= '0;
                    m_axis_out_tvalid <= '0;
                    reset_fft_no <= '0;
                end
            end
            WAIT_FOR_IBAR: begin
                SSB_start_o <= '0;
                if (ibar_SSB_valid_i) begin
                    $display("frame_sync: received ibar_SSB = %d", ibar_SSB_i);
                    if (ibar_SSB_i != 0) begin
                        // sym_cnt needs to be corrected for ibar_SSB != 0
                        case (ibar_SSB_i)
                            0: begin 
                                // no adjustment needed here
                            end
                            1: begin
                                // sym_cnt <= sym_cnt + 6 < SYM_PER_SF ? sym_cnt + 6 : sym_cnt + 6 - SYM_PER_SF;
                            end
                            2: begin
                                // sym_cnt <= sym_cnt + 14 < SYM_PER_SF ? sym_cnt + 6 : sym_cnt + 6 - SYM_PER_SF;
                                // if (subframe_number == SUBFRAMES_PER_FRAME - 1) begin
                                //     subframe_number <= '0;
                                //     // inc sfn with modulo SFN_MAX if current subframe_number is SYM_PER_SF - 1
                                //     sfn <= sfn == SFN_MAX - 1 ? 0 : sfn + 1;
                                // end else begin
                                //     subframe_number <= subframe_number + 1;
                                end       //                     end
                            3: begin
                                // sym_cnt <= sym_cnt + 20 < SYM_PER_SF ? sym_cnt + 6 : sym_cnt + 6 - SYM_PER_SF;
                                // if (subframe_number == SUBFRAMES_PER_FRAME - 1) begin
                                //     subframe_number <= '0;
                                //     // inc sfn with modulo SFN_MAX if current subframe_number is SYM_PER_SF - 1
                                //     sfn <= sfn == SFN_MAX - 1 ? 0 : sfn + 1;
                                // end else begin
                                //     subframe_number <= subframe_number + 1;
                                // end
                            end
                        endcase
                    end
                    state <= SYNCED;
                end

                m_axis_out_tvalid <= s_axis_in_tvalid;

                if (s_axis_in_tvalid) begin
                    if (sample_cnt == (FFT_LEN + current_CP_len - 1)) begin
                        m_axis_out_tlast <= 1;

                        if (sym_cnt == SYM_PER_SF - 1) begin
                            sym_cnt <= 0;
                            sym_cnt_next = 0;
                            subframe_number <= subframe_number == SYM_PER_SF - 1 ? 0 : subframe_number + 1;
                            if (subframe_number == SUBFRAMES_PER_FRAME - 1) begin
                                subframe_number <= '0;
                                // inc sfn with modulo SFN_MAX if current subframe_number is SYM_PER_SF - 1
                                sfn <= sfn == SFN_MAX - 1 ? 0 : sfn + 1;
                            end else begin
                                subframe_number <= subframe_number + 1;
                            end
                        end else begin
                            sym_cnt <= sym_cnt + 1;
                            sym_cnt_next = sym_cnt + 1;
                        end

                        sample_cnt <= '0;
                        if ((sym_cnt_next == 0) || (sym_cnt_next == 7))   current_CP_len <= CP1_LEN;
                        else                                              current_CP_len <= CP2_LEN;
                        
                        if ((syms_since_last_SSB == SYMS_BTWN_SSB - 1) || N_id_2_valid_i)   syms_since_last_SSB <= 0;
                        else                                            syms_since_last_SSB <= syms_since_last_SSB + 1;                        
                    end else begin
                        sample_cnt <= sample_cnt + 1;
                        m_axis_out_tlast <= '0;
                    end
                end else if (N_id_2_valid_i) begin
                    syms_since_last_SSB <= 0;                    
                end

                if (N_id_2_valid_i) begin
                    // output of SSB_start_o in WAIT_FOR_IBAR state is needed, because channel_estimator needs it to detect ibar_SSB
                    // SSB_start_o <= 1;
                end else begin
                    // SSB_start_o <= '0;
                end                
            end
            SYNCED: begin
                if (ibar_SSB_valid_i) begin
                    // TODO throw error if ibar_SSB does not match expected ibar_SSB
                end

                if (find_SSB) begin
                    if (N_id_2_valid_i) begin
                        // expected sample_cnt is 0, if actual sample_cnt deviates +-1, perform realignment
                        $display("frame_sync: SSB at sfn = %d, subframe = %d, symbol = %d, sample = %d", sfn, subframe_number, sym_cnt, sample_cnt);
                        $display("frame_sync: is_SSB_location = %d", is_SSB_location(sfn, subframe_number, sym_cnt, sample_cnt, 0, current_CP_len, 0));
                        if (sample_cnt == 0) begin
                            sample_cnt_mismatch <= 0;
                            // SSB arrives as expected, no STO correction needed
                            $display("frame_sync: SSB is on time");
                        end else if (sample_cnt < 3) begin
                            sample_cnt_mismatch <= sample_cnt;
                            // SSB arrives too late
                            // correct this STO by outputting symbol_start and SSB_start a bit later
                            $display("frame_sync: SSB is %d samples too late", sample_cnt);
                        end else if (sample_cnt > (FFT_LEN + current_CP_len - 1 - FIND_EARLY_SAMPLES)) begin
                            sample_cnt_mismatch <= sample_cnt - (FFT_LEN + current_CP_len);
                            // SSB arrives too early
                            // correct this STO by outputting symbol_start and SSB_start a bit earlier
                            $display("frame_sync: SSB is %d samples too early", (FFT_LEN + current_CP_len) - sample_cnt);
                        end
                        find_SSB <= '0;
                    end

                    if (sample_cnt == 3) begin
                        // could not find SSB, connection is lost 
                        // go back to search mode (state 0)
                        $display("frame_sync: could not find SSB, connection is lost!");
                        state <= WAIT_FOR_SSB;
                    end
                end else begin
                    if (N_id_2_valid_i) begin
                        $display("frame_sync: ignoring SSB outside FIND mode");
                    end
                    if (s_axis_in_tvalid) begin
                        if ((sample_cnt == FFT_LEN + current_CP_len - FIND_EARLY_SAMPLES) && (syms_since_last_SSB == (SYMS_BTWN_SSB - 1))) begin
                            find_SSB <= 1;  // go into find state one SC before the symbol ends
                            // $display("find SSB ...");
                        end
                    end                    
                end

                if (N_id_2_valid_i && find_SSB) SSB_start_o <= '1;
                else                            SSB_start_o <= '0;

                m_axis_out_tvalid <= s_axis_in_tvalid;

                // set sfn, subframe_number, sym_cnt, sample_cnt
                // set m_axis_out_tlast
                if (s_axis_in_tvalid) begin
                    if (sample_cnt != 0 && ((sample_cnt == (FFT_LEN + current_CP_len - 1)) || ((sample_cnt < (FFT_LEN + current_CP_len - 1)) && find_SSB && N_id_2_valid_i))) begin
                        m_axis_out_tlast <= 1;
                        if (sym_cnt == SYM_PER_SF - 1) begin
                            sym_cnt <= 0;
                            sym_cnt_next = '0;
                            subframe_number <= subframe_number == SYM_PER_SF - 1 ? 0 : subframe_number + 1;
                            if (subframe_number == SUBFRAMES_PER_FRAME - 1) begin
                                subframe_number <= '0;
                                // inc sfn with modulo SFN_MAX if current subframe_number is SYM_PER_SF - 1
                                sfn <= sfn == SFN_MAX - 1 ? 0 : sfn + 1;
                            end else begin
                                subframe_number <= subframe_number + 1;
                            end
                        end else begin
                            sym_cnt <= sym_cnt + 1;
                            sym_cnt_next = sym_cnt + 1;
                        end

                        sample_cnt <= '0;
                        if ((sym_cnt_next == 0) || (sym_cnt_next == 7))     current_CP_len <= CP1_LEN;
                        else                                                current_CP_len <= CP2_LEN;
                        
                        if (find_SSB && N_id_2_valid_i) syms_since_last_SSB <= '0;
                        else                syms_since_last_SSB <= syms_since_last_SSB + 1;
                    end else begin
                        sample_cnt <= sample_cnt + 1;
                        m_axis_out_tlast <= '0;
                        if (find_SSB && N_id_2_valid_i) syms_since_last_SSB <= '0;
                    end
                end else if (find_SSB && N_id_2_valid_i) begin
                    syms_since_last_SSB <= '0;
                end
            end
        endcase
    end
end

// ----------------------------------------------------------------
// This process sets symbol_start_o to 1 at the beginning of every symbol,
// but only when the FSM above is in SYNCED state
reg symbol_state;
always @(posedge clk_i) begin
    if (!reset_ni) begin
        symbol_start_o <= '0;
        symbol_state <= '0;
    end else begin
        case (symbol_state)
            0: begin
                // first symbol of SSB can arrive a bit earlier or later,
                // therefore need a special check for this case
                if (find_SSB) begin
                    if ((state != WAIT_FOR_SSB)&& N_id_2_valid_i) begin
                        symbol_state <= 1;
                        symbol_start_o <= 1;
                    end
                end else begin
                    if ((state != WAIT_FOR_SSB) && (sample_cnt == 0) && (s_axis_in_tvalid)) begin
                        symbol_state <= 1;
                        symbol_start_o <= 1;
                    end
                end
            end
            1: begin
                symbol_start_o <= '0;
                if ((sample_cnt == 1) || (state == WAIT_FOR_SSB)) symbol_state <= '0;
            end
        endcase
    end
end

// store sample_id into FIFO at the beginning of each symbol
assign sample_id_valid = symbol_start_o;

endmodule