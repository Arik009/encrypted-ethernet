module graphics_main #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE) (
	input clk, reset, blank,
	input [clog2(VGA_WIDTH)-1:0] vga_x,
	input [clog2(VGA_HEIGHT)-1:0] vga_y,
	input ram_read_ready, input [clog2(BYTE_LEN)-1:0] ram_read_val,
	output ram_read_req, output [clog2(RAM_SIZE)-1:0] ram_read_addr);

`include "params.vh"

endmodule
