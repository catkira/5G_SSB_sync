`timescale 1ns / 1ns

module channel_estimator #(
    parameter IN_DW = 32,           // input data width
    localparam OUT_DW = IN_DW,
    localparam NFFT = 8,
    localparam FFT_LEN = 2 ** NFFT,
    localparam MAX_CELL_ID = 1007
)
(
    input                                           clk_i,
    input                                           reset_ni,
    input   wire       [IN_DW - 1 : 0]              s_axis_in_tdata,
    input   wire                                    s_axis_in_tuser,     // 1 bit, 1 -> PBCH message, 0 -> other
    input                                           s_axis_in_tvalid,
    input   wire       [$clog2(MAX_CELL_ID) - 1: 0] N_id_i,
    input                                           N_id_valid_i,

    output  reg        [OUT_DW - 1 : 0]             m_axis_out_tdata,
    output  reg        [1 : 0]                      m_axis_out_tuser,    // 2 bits, 1 -> PBCH message, 0 -> other
    output  reg                                     m_axis_out_tlast,
    output  reg                                     m_axis_out_tvalid,

    // debug ports
    output  reg        [1 : 0]                      debug_PBCH_DMRS_o,
    output  reg                                     debug_PBCH_DMRS_valid_o,
    output  reg        [2 : 0]                      debug_ibar_SSB_o,
    output  reg                                     debug_ibar_SSB_valid_o
);

reg [$clog2(MAX_CELL_ID) - 1: 0] N_id, N_id_used;
reg [3 : 0] state_PBCH_DMRS;
localparam PBCH_DMRS_LEN = 144;
localparam NUM_PBCH_DMRS_TYPES = 8;
reg [1 : 0] PBCH_DMRS [0 : NUM_PBCH_DMRS_TYPES - 1][0 : PBCH_DMRS_LEN - 1];
reg [$clog2(1601) - 1 : 0] PBCH_DMRS_cnt;  // PBCH_DMRS_cnt runs to 1601, see code below
reg PBCH_DMRS_ready;

localparam LFSR_N = 31;
reg lfsr_out_0, lfsr_out_1;
reg lfsr_valid;
reg LFSR_reset_n;
reg LFSR_load_config;

LFSR #(
    .N(LFSR_N),
    .TAPS('h11),
    .START_VALUE(1),
    .VARIABLE_CONFIG(1)
)
lfsr_0_i
(
    .clk_i(clk_i),
    .reset_ni(LFSR_reset_n),
    .load_config_i(LFSR_load_config),
    .taps_i('b1001),
    .start_value_i(1),
    .data_o(lfsr_out_0),
    .valid_o(lfsr_valid)
);

genvar ii;
for (ii = 0; ii < 8; ii = ii + 1) begin : LFSR_1

    reg [LFSR_N - 1 : 0] c_init;
    reg out;
    localparam [2 : 0] ibar_SSB = ii;
    
    LFSR #(
        .N(LFSR_N),
        .TAPS('b1001),
        .START_VALUE(1),
        .VARIABLE_CONFIG(1)
    )
    lfsr_i
    (
        .clk_i(clk_i),
        .reset_ni(LFSR_reset_n),
        .load_config_i(LFSR_load_config),
        .taps_i('b1111),
        .start_value_i(c_init),
        .data_o(out)
    );

    always @(posedge clk_i) begin
        if (!reset_ni) begin
            for (integer i = 0; i < NUM_PBCH_DMRS_TYPES; i = i + 1) begin
                PBCH_DMRS[ii][i] = '0;
            end
        end else begin
            if (N_id_valid_i) begin
                N_id_used <= N_id_i;
                c_init = (((ibar_SSB + 1) * ((N_id_i >> 2) + 1)) << 11) +  ((ibar_SSB + 1) << 6) + (N_id_i[1 : 0] % 4);
                // $display("cinit = %x", c_init);
            end
            if (state_PBCH_DMRS == 3) begin
                if (PBCH_DMRS_cnt[0] == 0) PBCH_DMRS[ii][PBCH_DMRS_cnt >> 1][1] = (lfsr_out_0 ^ out);
                else                       PBCH_DMRS[ii][PBCH_DMRS_cnt >> 1][0] = (lfsr_out_0 ^ out);

                // if ((ii == 1) && !PBCH_DMRS_cnt[0] && (PBCH_DMRS_cnt > 1)) begin
                //     $display("dmrs[%d] = %b", (PBCH_DMRS_cnt >> 1) - 1,  PBCH_DMRS[ii][(PBCH_DMRS_cnt >> 1) - 1]);
                // end
            end
        end
    end
end

reg [1 : 0] PBCH_DMRS_start_idx;
always @(posedge clk_i) begin
    if (!reset_ni) begin
        N_id <= '0;
        PBCH_DMRS_start_idx <= '0;
    end else begin
        if (N_id_valid_i)  begin
            N_id <= N_id_i;
            PBCH_DMRS_start_idx <= N_id_i[1 : 0];
        end
    end
end

// this process updates the PBCH_DMRS after a N_id was set
always @(posedge clk_i) begin
    if (!reset_ni) begin
        state_PBCH_DMRS <= '0;
        PBCH_DMRS_cnt <= '0;
        LFSR_reset_n <= '0;
        LFSR_load_config <= '0;
        debug_PBCH_DMRS_o <= '0;
        debug_PBCH_DMRS_valid_o <= '0;
        PBCH_DMRS_ready <= '0;
    end else begin
        case (state_PBCH_DMRS)
            0: begin  // wait for SSB
                if (N_id_valid_i) state_PBCH_DMRS <= 1;
            end
            1: begin  // load LFSRs with start values
                state_PBCH_DMRS <= 1;
                PBCH_DMRS_cnt <= '0;
                LFSR_reset_n <= '1;
                LFSR_load_config <= 1;
                PBCH_DMRS_ready <= '0;
                state_PBCH_DMRS <= 2;
            end
            2: begin  // skip first 1600 bits from LFSRs
                LFSR_load_config <= '0;
                if (PBCH_DMRS_cnt == 1601) begin
                    PBCH_DMRS_cnt <= '0;
                    state_PBCH_DMRS <= 3;
                end else begin
                    PBCH_DMRS_cnt <= PBCH_DMRS_cnt + 1;
                end
            end
            3: begin // store PBCH DMRS in 4 separate processes (see generate for loop above)
                // PBCH_DMRS can be used for channel estimation starting from now
                // assuming that the usage starts at the lowest index
                PBCH_DMRS_ready <= 1;
                if (PBCH_DMRS_cnt == (2*PBCH_DMRS_LEN - 1)) begin
                    state_PBCH_DMRS <= 4;
                end
                PBCH_DMRS_cnt <= PBCH_DMRS_cnt + 1;

                if (PBCH_DMRS_cnt[0] == 0) debug_PBCH_DMRS_o[1] <= (lfsr_out_0 ^ LFSR_1[2].out);
                else                       debug_PBCH_DMRS_o[0] <= (lfsr_out_0 ^ LFSR_1[2].out);
                debug_PBCH_DMRS_valid_o <= PBCH_DMRS_cnt[0];
            end
            4: begin  // FINISH, stay in this state until a reset happens
                debug_PBCH_DMRS_valid_o <= '0;
                // state_PBCH_DMRS <= '0;
                // LFSR_reset_n <= '0;
            end
        endcase
    end
end

reg [3 : 0] state_det_ibar;
reg [2 : 0] PBCH_sym_idx;
reg [$clog2(256) : 0] PBCH_SC_idx;
reg [$clog2(256) : 0] PBCH_SC_used_idx;
wire [$clog2(256) : 0] PBCH_SC_idx_plus_start = PBCH_SC_used_idx - PBCH_DMRS_start_idx;
reg [$clog2(64) : 0] PBCH_DMRS_idx;
reg signed [$clog2(60*2) + 1 : 0] DMRS_corr [0 : 7];  // one symbol has max 60 pilots
reg signed [$clog2(60*2) + 1 : 0] DMRS_corr_rot [0 : 7];  // one symbol has max 60 pilots
reg [$clog2(60*2) : 0] tmp_corr;
wire signed [IN_DW / 2 - 1 : 0] in_re, in_im;
assign in_re = s_axis_in_tdata[IN_DW / 2 - 1 : 0];
assign in_im = s_axis_in_tdata[IN_DW - 1 : IN_DW / 2];
reg [$clog2(8) - 1 : 0] ibar_SSB_detected;

function [$clog2(60*2) : 0] abs_DMRS_corr;
input signed [$clog2(60*2) + 1 : 0] arg;
begin
    abs_DMRS_corr = arg[$clog2(60*2) + 1] ? ~arg + 1: arg;  // discard MSB
end
endfunction

// hard BPSK demodulation of incoming signal
// this is used for PBCH DMRS detection
wire [1 : 0] in_demod = {data_in[IN_DW / 2 - 1], data_in[IN_DW - 1]};
wire [1 : 0] in_demod_rot = {~data_in[IN_DW - 1], data_in[IN_DW / 2 - 1]};  // also test with 90 deg rotated input signal to handle phase offsets
reg [IN_DW - 1 : 0] data_in;
always @(posedge clk_i) data_in <= s_axis_in_tdata;
reg valid_in;
always @(posedge clk_i) valid_in <= s_axis_in_tvalid;

// This process detects which PBCH DMRS is used for the current PBCH symbol
// There are 8 possible PBCH DMRSs, the number of the used PBCH DMRS is called ibar_SSB
// and determines the subframe number of the current symbol
always @(posedge clk_i) begin
    if (!reset_ni) begin
        state_det_ibar <= '0;
        PBCH_sym_idx <= '0;
        PBCH_SC_idx <= '0;
        PBCH_SC_used_idx <= '0;
        PBCH_DMRS_idx <= 0;
        for (integer i = 0; i < 8; i = i + 1)  begin
            DMRS_corr[i] <= '0;
            DMRS_corr_rot[i] <= '0;
        end
        ibar_SSB_detected = '0;
        debug_ibar_SSB_o <= '0;
        debug_ibar_SSB_valid_o <= '0;
        pilots_ready <= '0;
    end else begin
        // if (s_axis_in_tvalid) $display("rx %d + j%d -> %b", in_re, in_im, {s_axis_in_tdata[IN_DW/2-1], s_axis_in_tdata[IN_DW-1]});

        case (state_det_ibar)
            0: begin
                debug_ibar_SSB_valid_o <= 0;
                if (s_axis_in_tuser && PBCH_DMRS_ready) begin
                    pilots_ready <= '0;
                    state_det_ibar <= 1;
                    PBCH_sym_idx <= '0;
                    PBCH_SC_idx <= '0;
                    PBCH_SC_used_idx <= '0;
                    PBCH_DMRS_idx <= '0;
                end
            end
            1: begin // compare 1st PBCH symbol
                if (PBCH_SC_idx == 255) begin
                    if (PBCH_sym_idx == 0) begin
                        state_det_ibar <= 2;
                        PBCH_sym_idx <= PBCH_sym_idx + 1;
                        PBCH_SC_used_idx <= '0;
                        PBCH_DMRS_idx <= '0;
                    end else begin
                        state_det_ibar <= 0;
                    end
                end else begin
                    if (valid_in) begin
                        // $display("rx 0/2 %d + j%d -> %b", in_re, in_im, in_demod);
                        if (PBCH_SC_idx_plus_start[1 : 0] == 0) begin
                            // $display("compare [%d] %b to %b, %b, %b, %b, %b, %b, %b, %b", PBCH_DMRS_idx, in_demod, 
                            //     PBCH_DMRS[0][PBCH_DMRS_idx], PBCH_DMRS[1][PBCH_DMRS_idx], PBCH_DMRS[2][PBCH_DMRS_idx], PBCH_DMRS[3][PBCH_DMRS_idx],
                            //     PBCH_DMRS[4][PBCH_DMRS_idx], PBCH_DMRS[5][PBCH_DMRS_idx], PBCH_DMRS[6][PBCH_DMRS_idx], PBCH_DMRS[7][PBCH_DMRS_idx]);
                            PBCH_DMRS_idx <= PBCH_DMRS_idx + 1;
                            for (integer i = 0; i < 8; i = i + 1) begin
                                DMRS_corr[i] = DMRS_corr[i] + ((PBCH_DMRS[i][PBCH_DMRS_idx][0] == in_demod[0]) ? 1 : -1);
                                DMRS_corr[i] = DMRS_corr[i] + ((PBCH_DMRS[i][PBCH_DMRS_idx][1] == in_demod[1]) ? 1 : -1);
                                DMRS_corr_rot[i] = DMRS_corr_rot[i] + ((PBCH_DMRS[i][PBCH_DMRS_idx][0] == in_demod_rot[0]) ? 1 : -1);
                                DMRS_corr_rot[i] = DMRS_corr_rot[i] + ((PBCH_DMRS[i][PBCH_DMRS_idx][1] == in_demod_rot[1]) ? 1 : -1);
                            end
                        end
                        PBCH_SC_used_idx <= PBCH_SC_used_idx + 1;
                    end
                end
                if (valid_in) PBCH_SC_idx <= PBCH_SC_idx + 1;
            end
            2: begin
                // for(integer i = 0; i < 8; i = i + 1)  $display("corr[%d] = %d", i, DMRS_corr[i]);
                ibar_SSB_detected = '0;
                tmp_corr = '0;
                for(integer i = 0; i < 8; i = i + 1) begin
                    if ((abs_DMRS_corr(DMRS_corr[i]) > tmp_corr) || (abs_DMRS_corr(DMRS_corr_rot[i]) > tmp_corr)) begin
                        tmp_corr = abs_DMRS_corr(DMRS_corr_rot[i]) > abs_DMRS_corr(DMRS_corr[i]) ? abs_DMRS_corr(DMRS_corr_rot[i]) : abs_DMRS_corr(DMRS_corr[i]);
                        ibar_SSB_detected = i;
                    end
                end
                $display("detected ibar_SSB = %d with correlation = %d", ibar_SSB_detected, tmp_corr);
                pilots_ready <= 1;
                debug_ibar_SSB_o <= ibar_SSB_detected;
                debug_ibar_SSB_valid_o <= 1;
                state_det_ibar <= '0;
                for(integer i = 0; i < 8; i = i + 1)  DMRS_corr[i] <= '0;
            end
        endcase
    end
end

// Put incoming data into a FIFO to give enough time for PBCH DRMS detection
// Length of FIFO has to be at least FFT_LEN samples, because 1 symbol is used 
// to detect the PBCH DRMS
reg [IN_DW - 1 : 0] in_fifo_data;
reg                 in_fifo_valid;
reg                 in_fifo_ready;
reg                 in_fifo_user;
localparam EXTRA_LEN = FFT_LEN;  // for the atan2 latency, FIFO_LEN has to be power of 2, therefore increase by FFT_LEN !
reg [$clog2(FFT_LEN + EXTRA_LEN) - 1 : 0]  in_fifo_level;
AXIS_FIFO #(
    .DATA_WIDTH(IN_DW),
    .FIFO_LEN(FFT_LEN + EXTRA_LEN),
    .USER_WIDTH(1),
    .ASYNC(0)
)
data_FIFO_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),

    .s_axis_in_tdata(s_axis_in_tdata),
    .s_axis_in_tuser(s_axis_in_tuser),
    .s_axis_in_tvalid(s_axis_in_tvalid),

    .m_axis_out_tready(in_fifo_ready),
    .m_axis_out_tdata(in_fifo_data),
    .m_axis_out_tuser(in_fifo_user),
    .m_axis_out_tvalid(in_fifo_valid),
    .m_axis_out_tlevel(in_fifo_level)
);

// This atan2 instance is used to calculate angle(received_i),
// which is the phase of each subcarrier of the current symbol
localparam PHASE_DW = 18;
reg signed [PHASE_DW - 1 : 0]   SC_phase;
reg                             SC_phase_valid;
atan2 #(
    .INPUT_WIDTH(IN_DW / 2),
    .LUT_DW(14),
    .OUTPUT_WIDTH(PHASE_DW)
)
atan2_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    
    .numerator_i(s_axis_in_tdata[IN_DW - 1 : IN_DW / 2]),
    .denominator_i(s_axis_in_tdata[IN_DW / 2 - 1 : 0]),
    .valid_i(s_axis_in_tvalid),

    .angle_o(SC_phase),
    .valid_o(SC_phase_valid)
);

// Put all calculated phases into a FIFO
reg  angle_FIFO_ready;
reg  angle_FIFO_valid;
reg  [$clog2(FFT_LEN) - 1 : 0] angle_FIFO_level;
reg  signed [PHASE_DW - 1 : 0] angle_FIFO_data;
AXIS_FIFO #(
    .DATA_WIDTH(PHASE_DW),
    .FIFO_LEN(FFT_LEN),
    .ASYNC(0)
)
angle_FIFO_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),

    .s_axis_in_tdata(SC_phase),
    .s_axis_in_tvalid(SC_phase_valid),

    .m_axis_out_tready(angle_FIFO_ready),
    .m_axis_out_tdata(angle_FIFO_data),
    .m_axis_out_tvalid(angle_FIFO_valid),
    .m_axis_out_tlevel(angle_FIFO_level)
);


// This process calculates phase correction factors
// by calculating angle(pilot_i) - angle(received_i)
// 
// TODO: for now it supports only phase correction for PBCH symbols
reg [2 : 0]  state_corrector;
localparam [2 : 0]  WAIT_FOR_INPUTS = 0;
localparam [2 : 0]  CALC_CORRECTION = 1;
localparam [2 : 0]  PASS_THROUGH = 2;
reg          pilots_ready;
reg          in_data_ready;
reg          angles_ready;
reg [$clog2(FFT_LEN) - 1 : 0]         SC_cnt;

localparam MAX_PHASE = (2**(PHASE_DW - 1) - 1);
localparam signed [PHASE_DW - 1 : 0] DEG45 = MAX_PHASE / 4;
localparam signed [PHASE_DW - 1 : 0] DEG135 = 3 * DEG45;
reg signed [PHASE_DW - 1 : 0] pilot_angle;
reg [2 : 0] ibar_SSB_buf;

localparam MAX_SYM_PER_BURST = 3;
localparam MIN_PILOT_SPACING = 4;
reg [$clog2(FFT_LEN * MAX_SYM_PER_BURST / MIN_PILOT_SPACING) - 1 : 0] pilot_SC_idx;
reg [1 : 0] start_idx;  // buffer start idx so that it cannot change within one symbol
wire [$clog2(FFT_LEN) : 0] SC_idx_plus_start = SC_cnt - start_idx;
reg [10 : 0]    symbol_cnt;
localparam SYMS_BTWN_SSB = 14 * 20;
localparam ZERO_CARRIERS = 16;
reg  signed [PHASE_DW - 1 : 0] corr_angle_DDS_in;
reg                            corr_angle_DDS_valid_in;
localparam  [1 : 0]            SYMBOL_TYPE_OTHER = 0;
localparam  [1 : 0]            SYMBOL_TYPE_PBCH = 1;
reg         [1 : 0]            symbol_type;
reg         [2 : 0]            remaining_syms;
localparam                     SYMS_PER_PBCH = 3;
localparam                     SYMS_PER_OTHER = 3;

always @(posedge clk_i) begin
    if (!reset_ni) begin
        state_corrector <= WAIT_FOR_INPUTS;
        SC_cnt <= '0;
        pilot_SC_idx <= '0;
        symbol_cnt <= '0;
        corr_angle_DDS_in <= '0;
        corr_angle_DDS_valid_in <= '0;
        symbol_type <= SYMBOL_TYPE_OTHER;
        remaining_syms <= '0;
        corr_data_fifo_in_valid <= '0;
        corr_data_fifo_in_data <= '0;
        corr_data_fifo_in_last <= '0;
        start_idx <= '0;
        ibar_SSB_buf <= '0;
    end else begin
        case(state_corrector)
            WAIT_FOR_INPUTS : begin
                SC_cnt <= '0;
                pilot_SC_idx <= '0;
                corr_data_fifo_in_valid <= '0;
                corr_angle_DDS_valid_in <= '0;
                start_idx <= PBCH_DMRS_start_idx;
                if ((in_fifo_level > 0) && (!PBCH_DMRS_ready)) begin
                    // $display("calculate_phase: PBCH DMRS not ready, pass through without correction");
                    symbol_type <= SYMBOL_TYPE_OTHER;
                    state_corrector <= PASS_THROUGH;
                    remaining_syms <= SYMS_PER_OTHER - 1;
                end else if ((in_fifo_level > 0) && PBCH_DMRS_ready) begin
                    // in_fifo_user signals start of a new burst if its != 0
                    // the symbol type depends on the position of the set bit
                    if ((in_fifo_user == 1) && pilots_ready)  begin  // pilots become ready withing 256 clks, so we can wait here, FIFOs are large enough
                        $display("calculate_phase: PBCH symbol");
                        remaining_syms <= SYMS_PER_PBCH - 1;  
                        symbol_type <= SYMBOL_TYPE_PBCH;
                        state_corrector <= CALC_CORRECTION;
                        ibar_SSB_buf <= ibar_SSB_detected;
                    end else if (in_fifo_user == 0) begin
                        // $display("calculate_phase: data symbol");
                        remaining_syms <= SYMS_PER_OTHER - 1;
                        symbol_type <= SYMBOL_TYPE_OTHER;
                        state_corrector <= PASS_THROUGH;
                    end
                end
            end
            CALC_CORRECTION : begin
                // only need to check angle_FIFO because, it becomes always later ready than data_FIFO
                if ((angle_FIFO_level > 0) && (SC_cnt < FFT_LEN - 2 - ZERO_CARRIERS)) begin
                    in_fifo_ready <= 1;
                    angle_FIFO_ready <= 1;
                end else begin
                    in_fifo_ready <= '0;
                    angle_FIFO_ready <= '0;
                end
                
                if (angle_FIFO_valid != in_fifo_valid) begin
                    $display("ERROR: in_FIFO and angle_FIFO don't output in sync !");
                end

                if (angle_FIFO_valid) begin
                    corr_data_fifo_in_data <= in_fifo_data;
                    // if (symbol_cnt == 0)  $display("data %x  angle %f", in_fifo_data, $itor(angle_FIFO_data) / DEG45 * 45);
                    if (SC_idx_plus_start[1:0] == 0) begin
                        // we are at a pilot location, calculate correction factor
                        // one corr_angle has to be used for 4 corr_data, because pilots are every 4th SC
                        // use simple piecewise const. corr. angle for now, it can be improved with linear interpolation later                            
                        case(PBCH_DMRS[ibar_SSB_buf][pilot_SC_idx])
                            2'b00 : pilot_angle = DEG45;
                            2'b01 : pilot_angle = -DEG45;
                            2'b10 : pilot_angle = DEG135;
                            2'b11 : pilot_angle = -DEG135;
                        endcase
                        // if (symbol_cnt == 0)  $display("pilot = %x", PBCH_DMRS[ibar_SSB_buf][pilot_SC_idx]);
                        
                        if ((remaining_syms == 1) && ((SC_cnt >= 48) && (SC_cnt <= 191))) begin
                            // This is a special case for the 2nd PBCH symbol
                            // some SCs in the middle of the symbol are the SSS, which needs to be skipped here
                            corr_angle_DDS_in <= 0;
                            corr_angle_DDS_valid_in <= 0;
                            corr_data_fifo_in_valid <= 0;                    
                        end else begin
                            corr_angle_DDS_in <= -(angle_FIFO_data - pilot_angle);
                            corr_angle_DDS_valid_in <= 1;
                            corr_data_fifo_in_valid <= 0;                            
                            if (symbol_cnt == 0)  $display("pilot at SC %d, rx angle = %f deg, ideal angle = %f, delta = %f", SC_cnt,
                                $itor(angle_FIFO_data) / DEG45 * 45, $itor(pilot_angle) / DEG45 * 45, ($itor(angle_FIFO_data - pilot_angle)) / DEG45 * 45);
                            pilot_SC_idx <= pilot_SC_idx + 1;
                        end
                    end else begin
                        corr_angle_DDS_valid_in <= 0;
                        if ((remaining_syms == 1) && ((SC_cnt >= 48) && (SC_cnt <= 191))) corr_data_fifo_in_valid <= 0;
                        else begin
                            corr_data_fifo_in_valid <= 1;
                            // $display("store data sample from SC %d", SC_cnt);
                        end
                    end

                    if (SC_cnt == FFT_LEN - 2 - ZERO_CARRIERS) begin
                        corr_data_fifo_in_last <= remaining_syms == 0;
                    end else begin
                        corr_data_fifo_in_last <= '0;
                    end

                    if (SC_cnt == FFT_LEN - 1 - ZERO_CARRIERS) begin
                        if (remaining_syms > 0) begin
                            // if (symbol_type == SYMBOL_TYPE_PBCH)  $display("starting with PBCH symbol %d", SYMS_PER_PBCH - remaining_syms);
                            // stay in CALC_CORRECTION state and proces next symbol of burst
                            remaining_syms <= remaining_syms - 1;
                            SC_cnt <= '0;
                        end else begin
                            state_corrector <= WAIT_FOR_INPUTS;
                            symbol_cnt <= symbol_cnt + 1;
                        end
                    end else begin
                        corr_data_fifo_in_tuser <= symbol_type;
                        SC_cnt <= SC_cnt + 1;
                    end
                end
                else begin
                    corr_angle_DDS_in <= 0;
                    corr_data_fifo_in_valid <= '0;
                    corr_angle_DDS_valid_in <= '0;
                end

            end
            PASS_THROUGH : begin
                // only need to check angle_FIFO because, it becomes always later ready than data_FIFO
                if ((angle_FIFO_level > 0) && (SC_cnt < FFT_LEN - 2 - ZERO_CARRIERS)) begin
                    in_fifo_ready <= 1;
                    angle_FIFO_ready <= 1;
                end else begin
                    in_fifo_ready <= '0;
                    angle_FIFO_ready <= '0;
                end

                if (angle_FIFO_valid) begin
                    corr_data_fifo_in_data <= in_fifo_data;
                    if (SC_idx_plus_start[1:0] == 0) begin
                        corr_data_fifo_in_valid <= 0;
                        corr_angle_DDS_in <= 0;
                        corr_angle_DDS_valid_in <= 1;
                        pilot_SC_idx <= pilot_SC_idx + 1;
                    end else begin
                        corr_angle_DDS_valid_in <= 0;
                        corr_data_fifo_in_valid <= 1;          
                    end

                    if (SC_cnt == FFT_LEN - 2 - ZERO_CARRIERS) begin
                        corr_data_fifo_in_last <= remaining_syms == 0;
                    end else begin
                        corr_data_fifo_in_last <= '0;
                    end                    

                    if (SC_cnt == FFT_LEN - 1 - ZERO_CARRIERS) begin
                        state_corrector <= WAIT_FOR_INPUTS;
                        if (symbol_cnt == SYMS_BTWN_SSB - 1)    symbol_cnt <= '0;
                        else                                    symbol_cnt <= symbol_cnt + 1;
                    end else begin
                        corr_data_fifo_in_tuser <= symbol_type;
                        SC_cnt <= SC_cnt + 1;
                    end
                end
                else begin
                    corr_angle_DDS_in <= 0;
                    corr_data_fifo_in_last <= '0;
                    corr_data_fifo_in_valid <= '0;
                    corr_angle_DDS_valid_in <= '0;
                end
            end
        endcase
    end
end


localparam DDS_OUT_DW = 32;
reg signed [DDS_OUT_DW - 1 : 0] DDS_out;
reg                             DDS_out_valid;
dds #(
    .PHASE_DW(PHASE_DW),
    .OUT_DW(DDS_OUT_DW / 2),
    .USE_TAYLOR(0),
    .SIN_COS(1),
    .NEGATIVE_SINE(0),
    .NEGATIVE_COSINE(0),
    .USE_LUT_FILE(0)
)
DDS_i(
    .clk(clk_i),
    .reset_n(reset_ni),
    .s_axis_phase_tdata(corr_angle_DDS_in),
    .s_axis_phase_tvalid(corr_angle_DDS_valid_in),
    .m_axis_out_tdata(DDS_out),
    .m_axis_out_tvalid(DDS_out_valid)   
);


// This fifo stores correction angles for each pilot
reg signed [DDS_OUT_DW - 1 : 0] corr_angle_fifo_out_data;
reg                             corr_angle_fifo_out_valid;
reg                             corr_angle_fifo_out_empty;
reg                             corr_angle_fifo_out_ready;
AXIS_FIFO #(
    .DATA_WIDTH(DDS_OUT_DW),
    .FIFO_LEN(FFT_LEN),
    .USER_WIDTH(0),
    .ASYNC(0)
)
corr_angle_fifo_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .s_axis_in_tdata(DDS_out),
    .s_axis_in_tvalid(DDS_out_valid),

    .m_axis_out_tready(corr_angle_fifo_out_ready),
    .m_axis_out_tdata(corr_angle_fifo_out_data),
    .m_axis_out_tvalid(corr_angle_fifo_out_valid),
    .m_axis_out_tempty(corr_angle_fifo_out_empty)
);

// This fifo stores uncorrected IQ samples
reg [IN_DW - 1 : 0] corr_data_fifo_in_data;
reg corr_data_fifo_in_valid;
reg [IN_DW - 1 : 0] corr_data_fifo_out_data;
reg corr_data_fifo_out_valid;
reg corr_data_fifo_out_empty;
reg corr_data_fifo_out_ready;
reg corr_data_fifo_out_last;
reg corr_data_fifo_in_last;
reg [1 : 0] corr_data_fifo_in_tuser;
reg [$clog2(FFT_LEN) - 1 : 0] corr_data_fifo_out_level;
reg [1 : 0] corr_data_fifo_out_symbol_type;
AXIS_FIFO #(
    .DATA_WIDTH(IN_DW),
    .FIFO_LEN(FFT_LEN),
    .USER_WIDTH(2),
    .ASYNC(0)
)
corr_data_fifo_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),
    .s_axis_in_tdata(corr_data_fifo_in_data),
    .s_axis_in_tuser(corr_data_fifo_in_tuser),
    .s_axis_in_tlast(corr_data_fifo_in_last),
    .s_axis_in_tvalid(corr_data_fifo_in_valid),

    .m_axis_out_tready(corr_data_fifo_out_ready),
    .m_axis_out_tdata(corr_data_fifo_out_data),
    .m_axis_out_tvalid(corr_data_fifo_out_valid),
    .m_axis_out_tempty(corr_data_fifo_out_empty),
    .m_axis_out_tuser(corr_data_fifo_out_symbol_type),
    .m_axis_out_tlast(corr_data_fifo_out_last),
    .m_axis_out_tlevel(corr_data_fifo_out_level)
);

// corr_data_fifo_out_symbol_type needs to be connected to m_axis_out_tuser with delay
// because the delay line and complex multiplier needs some clks
localparam COMPLEX_MULT_DELAY = 6;
localparam CORR_DATA_DELAY = 4;
localparam CORR_DELAY = CORR_DATA_DELAY + COMPLEX_MULT_DELAY;
reg [1 : 0] symbol_type_delayed [0 : CORR_DELAY - 1];
reg         tlast_delayed [0 : CORR_DELAY - 1];
assign m_axis_out_tuser = symbol_type_delayed[CORR_DELAY - 1];
assign m_axis_out_tlast = tlast_delayed[CORR_DELAY - 1];
always @(posedge clk_i) begin
    if (!reset_ni) begin
        for (integer i = 0; i < CORR_DELAY; i = i + 1) begin
            symbol_type_delayed[i] <= '0;
            tlast_delayed[i] <= '0;
        end
    end else begin
        symbol_type_delayed[0] <= corr_data_fifo_out_symbol_type;
        tlast_delayed[0] <= corr_data_fifo_out_last;
        for (integer i = 0; i < CORR_DELAY - 1; i = i + 1)  begin
            symbol_type_delayed[i + 1] <= symbol_type_delayed[i];
            tlast_delayed[i + 1] <= tlast_delayed[i];
        end
    end
end

// apply corr_angle to corr_data
// output of corr_data_fifo_out_data has to be delayed by 0 .. 3 cycles, depending on the pilot location
reg signed [IN_DW - 1 : 0] corr_data_delayed [0 : CORR_DATA_DELAY - 1];
reg                        corr_valid_delayed [0 : CORR_DATA_DELAY - 1];
reg                        corr_last_delayed [0 : CORR_DATA_DELAY - 1];
always @(posedge clk_i) begin
    if (!reset_ni) begin
        for (integer i = 0; i < 4; i = i + 1) begin
            corr_data_delayed[i] = '0;
            corr_valid_delayed[i] = '0;
            corr_last_delayed[i] = '0;
        end
    end else begin
        for (integer i = 0; i < 4; i = i + 1) begin
            if (i == 0)  begin
                corr_data_delayed[0] <= corr_data_fifo_out_data;
                corr_last_delayed[0] <= corr_data_fifo_out_last;
                corr_valid_delayed[0] <= corr_data_fifo_out_valid;
            end else begin
                corr_data_delayed[i] = corr_data_delayed[i - 1];
                corr_last_delayed[i] <= corr_last_delayed[i - 1];
                corr_valid_delayed[i] = corr_valid_delayed[i - 1];
            end
        end
    end
end
wire [IN_DW - 1 : 0] delayed_data = corr_data_delayed[CORR_DATA_DELAY - 1];
wire                 delayed_valid = corr_valid_delayed[CORR_DATA_DELAY - 1];

// This is a simple piecewise constant interpolator
// read one pilot every 4 data carrier
// angle fifo get filled later than data fifo, therefore check corr_angle_fifo_out_empty
reg signed [DDS_OUT_DW - 1 : 0] corr_angle_buf;
always @(posedge clk_i) begin
    if (!reset_ni) corr_angle_buf <= '0;
    else if (corr_angle_fifo_out_valid)  corr_angle_buf <= corr_angle_fifo_out_data;
end

reg [1 : 0] div3_cnt;
reg [1 : 0] state_interp;
reg [10 : 0] out_sample_cnt;
localparam [1 : 0]  STATE_INTERP_WAIT_FOR_START     = 0;
localparam [1 : 0]  STATE_INTERP_INTERPOLATE        = 1;
always @(posedge clk_i) begin
    if (!reset_ni) begin
        div3_cnt <= '0;
        corr_angle_fifo_out_ready <= '0;
        corr_data_fifo_out_ready <= '0;
        state_interp <= STATE_INTERP_WAIT_FOR_START;
    end else begin
        case(state_interp)
            STATE_INTERP_WAIT_FOR_START : begin
                corr_angle_fifo_out_ready <= '0;
                corr_data_fifo_out_ready <= '0;
                div3_cnt <= '0;
                if (!corr_angle_fifo_out_empty) begin
                    state_interp <= STATE_INTERP_INTERPOLATE;
                end
            end
            STATE_INTERP_INTERPOLATE : begin
                if (corr_data_fifo_out_last && corr_data_fifo_out_valid) begin
                    corr_data_fifo_out_ready <= '0;
                    corr_angle_fifo_out_ready <= '0;
                    state_interp <= STATE_INTERP_WAIT_FOR_START;
                end else if (!corr_angle_fifo_out_empty) begin
                    corr_data_fifo_out_ready <= '1;
                    out_sample_cnt <= out_sample_cnt + 1;
                    corr_angle_fifo_out_ready <= div3_cnt == 0;
                    if (div3_cnt == 2)  div3_cnt <= '0;
                    else div3_cnt <= div3_cnt + 1;
                end else begin
                    corr_angle_fifo_out_ready <= '0;
                    corr_data_fifo_out_ready <= '0;
                end
            end
        endcase
    end
end

complex_multiplier #(
    .OPERAND_WIDTH_A(DDS_OUT_DW / 2),
    .OPERAND_WIDTH_B(IN_DW / 2),
    .OPERAND_WIDTH_OUT(IN_DW / 2),
    .BLOCKING(0),
    .GROWTH_BITS(-2)
)
complex_multiplier_i(
    .aclk(clk_i),
    .aresetn(reset_ni),

    .s_axis_a_tdata(corr_angle_buf),
    .s_axis_a_tvalid(1'b1),
    .s_axis_b_tdata(delayed_data),
    .s_axis_b_tvalid(delayed_valid),
    
    .m_axis_dout_tready(1'b0),
    .m_axis_dout_tdata(m_axis_out_tdata),
    .m_axis_dout_tvalid(m_axis_out_tvalid)
);

endmodule