module video_cache_ram_driver #(
	parameter RAM_SIZE = VIDEO_CACHE_RAM_SIZE,
	parameter READ_LATENCY = VIDEO_CACHE_RAM_LATENCY) (
	input clk, rst,
	input readclk, input [clog2(RAM_SIZE)-1:0] raddr,
	input we, input [clog2(RAM_SIZE)-1:0] waddr,
	input [COLOR_LEN-1:0] win,
	output outclk, output [COLOR_LEN-1:0] out
	);

`include "params.vh"

delay #(.DELAY_LEN(READ_LATENCY)) delay_inst(
	.clk(clk), .reset(rst), .in(readclk), .out(outclk));
video_cache_ram video_cache_ram_inst(
	.clka(clk), .wea(we),
	.addra(waddr), .dina(win),
	.clkb(clk), .addrb(raddr), .doutb(out));

endmodule

module packet_synth_rom_driver #(
	parameter RAM_SIZE = PACKET_SYNTH_ROM_SIZE,
	parameter READ_LATENCY = PACKET_SYNTH_ROM_LATENCY) (
	input clk, rst,
	input readclk, input [clog2(RAM_SIZE)-1:0] raddr,
	output outclk, output [BYTE_LEN-1:0] out
	);

`include "params.vh"

delay #(.DELAY_LEN(READ_LATENCY)) delay_inst(
	.clk(clk), .reset(rst), .in(readclk), .out(outclk));
packet_synth_rom packet_synth_rom_inst(
	.clka(clk),
	.addra(raddr), .douta(out));

endmodule

module packet_buffer_ram_driver #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE,
	parameter READ_LATENCY = PACKET_BUFFER_READ_LATENCY) (
	input clk, rst,
	input readclk, input [clog2(RAM_SIZE)-1:0] raddr,
	input we, input [clog2(RAM_SIZE)-1:0] waddr,
	input [BYTE_LEN-1:0] win,
	output outclk, output [BYTE_LEN-1:0] out
	);

`include "params.vh"

delay #(.DELAY_LEN(READ_LATENCY)) delay_inst(
	.clk(clk), .reset(rst), .in(readclk), .out(outclk));
packet_buffer_ram packet_buffer_ram_inst(
	.clka(clk), .wea(we),
	.addra(waddr), .dina(win),
	.clkb(clk), .addrb(raddr), .doutb(out));

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
