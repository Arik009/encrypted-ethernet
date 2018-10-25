module bram_manager #(
	parameter RAM_SIZE_LOG2 = PACKET_BUFFER_SIZE_LOG2) (
	input clk, reset,
	input read_req, input [RAM_SIZE_LOG2-1:0] read_addr,
	input write_enable,
	input [RAM_SIZE_LOG2-1:0] write_addr,
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
