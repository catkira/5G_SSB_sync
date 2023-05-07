`timescale 1ns / 1ns
// This core connects AXI lite mapped registers from the Open5G receiver
// Copyright (C) 2023  Benjamin Menkuec
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

module receiver_regmap #(
    parameter ID = 0,
    parameter ADDRESS_WIDTH = 11,

    localparam N_id_MAX = 1007
)
(
    input clk_i,
    input reset_ni,

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

    // mapped registers
    input           [1 : 0]                     fs_state_i,
    input           [31 : 0]                    rx_signal_i,
    input           [1 : 0]                     N_id_2_i,
    input           [$clog2(N_id_MAX) - 1 : 0]  N_id_i,
    input   signed  [7 : 0]                     sample_cnt_mismatch_i,
    input           [15 : 0]                    missed_SSBs_i,
    input           [2 : 0]                     ibar_SSB_i
);

localparam PCORE_VERSION = 'h00040069;

wire rreq;
wire [8:0] raddr;
reg [31:0] rdata;
reg rack;

always @(posedge clk_i) begin
    if (!reset_ni)  rack <= '0;
    else            rack <= rreq;   // ack immediately after req
end

always @(posedge clk_i) begin
    if (!reset_ni)  rdata <= '0;
    else begin
        if (rreq == 1'b1) begin
            case (raddr)
                9'h000: rdata <= PCORE_VERSION;
                9'h001: rdata <= ID;
                9'h002: rdata <= '0;
                9'h003: rdata <= 32'h52587E7E; // "RX~~"
                9'h004: rdata <= 32'h69696969;
                9'h005: rdata <= {30'd0, fs_state_i};
                9'h006: rdata <= rx_signal_i;
                9'h007: rdata <= {30'd0, N_id_2_i};
                9'h008: rdata <= {22'd0, N_id_i};
                9'h009: rdata <= {24'd0, sample_cnt_mismatch_i};
                9'h00A: rdata <= {16'd0, missed_SSBs_i};
                9'h00B: rdata <= {29'd0, ibar_SSB_i};
                default: rdata <= '0;
            endcase
        end
    end
end

wire wreq;
wire [8:0] waddr;
reg [31:0] wdata;
reg wack;

always @(posedge clk_i) begin
    if (!reset_ni)  wack <= '0;
    else            wack <= wreq;   // ack immediately after req
end

always @(posedge clk_i) begin
    if (!reset_ni) begin
    end else begin
        if (wreq) begin
            case (waddr)
                default: begin end
            endcase
        end
    end
end

AXI_lite_interface #(
    .ADDRESS_WIDTH(ADDRESS_WIDTH)
)
AXI_lite_interface_i(
    .clk_i(clk_i),
    .reset_ni(reset_ni),

    .s_axi_awaddr(s_axi_if_awaddr),
    .s_axi_awvalid(s_axi_if_awvalid),
    .s_axi_awready(s_axi_if_awready),
    .s_axi_wdata(s_axi_if_wdata),
    .s_axi_wstrb(s_axi_if_wstrb),
    .s_axi_wvalid(s_axi_if_wvalid),
    .s_axi_wready(s_axi_if_wready),
    .s_axi_bresp(s_axi_if_bresp),
    .s_axi_bvalid(s_axi_if_bvalid),
    .s_axi_bready(s_axi_if_bready),
    .s_axi_araddr(s_axi_if_araddr),
    .s_axi_arvalid(s_axi_if_arvalid),
    .s_axi_arready(s_axi_if_arready),
    .s_axi_rdata(s_axi_if_rdata),
    .s_axi_rresp(s_axi_if_rresp),
    .s_axi_rvalid(s_axi_if_rvalid),
    .s_axi_rready(s_axi_if_rready),

    .wreq_o(wreq),
    .waddr_o(waddr),
    .wdata_o(wdata),
    .wack(wack),
    .rreq_o(rreq),
    .raddr_o(raddr),
    .rdata(rdata),
    .rack(rack)  
);

endmodule