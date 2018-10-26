module bram_driver #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE) (
	input clk, reset,
	input read_req, input [clog2(RAM_SIZE)-1:0] read_addr,
	input write_enable,
	input [clog2(RAM_SIZE)-1:0] write_addr,
	input [BYTE_LEN-1:0] write_val,
	output read_ready, output [BYTE_LEN-1:0] read_out
	);

`include "params.vh"

delay #(.DELAY_LEN(PACKET_BUFFER_READ_LATENCY)) delay_inst(
	.clk(clk), .in(read_req), .out(read_ready));
packet_buffer_ram packet_buffer_ram_inst(
	.clka(clk), .ena(1), .wea(write_enable),
	.addra(write_addr), .dina(write_val),
	.clkb(clk), .enb(1), .addrb(read_addr), .doutb(read_out));

endmodule

module ddr2_ram_driver(
	input sys_clk,
	inout [15:0] ddr2_dq, inout [1:0] ddr2_dqs_n, ddr2_dqs_p,
	output [12:0] ddr2_addr, output[2:0] ddr2_ba,
	output ddr2_ras_n, ddr2_cas_n, ddr2_we_n,
	output [0:0] ddr2_ck_p, ddr2_ck_n, ddr2_cke, ddr2_cs_n,
	output [1:0] ddr2_dm, output [0:0] ddr2_odt);

nexys4_ddr2 ddr2_ram_inst(
	.ddr2_dq(ddr2_dq), .ddr2_dqs_n(ddr2_dqs_n), .ddr2_dqs_p(ddr2_dqs_p),
	.ddr2_addr(ddr2_addr), .ddr2_ba(ddr2_ba),
	.ddr2_ras_n(ddr2_ras_n), .ddr2_cas_n(ddr2_cas_n),
	.ddr2_we_n(ddr2_we_n), .ddr2_ck_p(ddr2_ck_p), .ddr2_ck_n(ddr2_ck_n),
	.ddr2_cke(ddr2_cke), .ddr2_cs_n(ddr2_cs_n),
	.ddr2_dm(ddr2_dm), .ddr2_odt(ddr2_odt), .sys_clk_i(sys_clk));

endmodule
