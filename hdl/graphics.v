module graphics_main #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE) (
	input clk, reset, blank,
	input [clog2(VGA_WIDTH)-1:0] vga_x,
	input [clog2(VGA_HEIGHT)-1:0] vga_y,
	input vga_hsync_in, vga_vsync_in,
	input ram_read_ready, input [COLOR_LEN-1:0] ram_read_val,
	output ram_read_req, output [clog2(RAM_SIZE)-1:0] ram_read_addr,
	output [COLOR_LEN-1:0] vga_col,
	output vga_hsync_out, vga_vsync_out);

`include "params.vh"

delay #(.DELAY_LEN(VIDEO_CACHE_RAM_LATENCY)) hsync_delay(
	.clk(clk), .in(vga_hsync_in), .out(vga_hsync_out));
delay #(.DELAY_LEN(VIDEO_CACHE_RAM_LATENCY)) vsync_delay(
	.clk(clk), .in(vga_vsync_in), .out(vga_vsync_out));

wire [4:0] image_x, image_y;
assign image_x = vga_x[8:4];
assign image_y = vga_y[8:4];
assign ram_read_req = !blank;
assign ram_read_addr = {image_y, image_x};
assign vga_col = ram_read_ready ? ram_read_val : 0;

endmodule
