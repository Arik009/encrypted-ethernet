module graphics_main #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE) (
	input clk, rst, blank,
	input [clog2(VGA_WIDTH)-1:0] vga_x,
	input [clog2(VGA_HEIGHT)-1:0] vga_y,
	input vga_hsync_in, vga_vsync_in,
	input ram_outclk, input [COLOR_LEN-1:0] ram_out,
	output ram_readclk, output [clog2(RAM_SIZE)-1:0] ram_raddr,
	output [COLOR_LEN-1:0] vga_col,
	output vga_hsync_out, vga_vsync_out);

`include "params.vh"

wire blank_delayed;
delay #(.DELAY_LEN(VIDEO_CACHE_RAM_LATENCY)) hsync_delay(
	.clk(clk), .rst(rst), .in(vga_hsync_in), .out(vga_hsync_out));
delay #(.DELAY_LEN(VIDEO_CACHE_RAM_LATENCY)) vsync_delay(
	.clk(clk), .rst(rst), .in(vga_vsync_in), .out(vga_vsync_out));
delay #(.DELAY_LEN(VIDEO_CACHE_RAM_LATENCY)) blank_delay(
	.clk(clk), .rst(rst), .in(blank), .out(blank_delayed));

wire [6:0] image_x, image_y;
assign image_x = vga_x[8:2];
assign image_y = vga_y[8:2];
assign ram_readclk = !blank && !vga_x[9] && !vga_y[9];
assign ram_raddr = {image_y, image_x};
assign vga_col = blank_delayed ? 12'h0 :
	ram_outclk ? ram_out : 12'hfff;

endmodule
