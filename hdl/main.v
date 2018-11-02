// streams ram memory over uart
// TODO: upgrade this to generic fifo
module ram_to_uart #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE) (
	input clk, reset, start,
	// read_end points to one byte after the end, like in C
	input [clog2(RAM_SIZE)-1:0] read_start, read_end,
	input ram_read_ready,
	input [BYTE_LEN-1:0] ram_read_out,
	output uart_txd, reg ram_read_req = 0,
	output [clog2(RAM_SIZE)-1:0] ram_read_addr);

`include "params.vh"

localparam STATE_IDLE = 0;
localparam STATE_READING = 1;
localparam STATE_WRITING = 2;

reg [1:0] state = STATE_IDLE;

reg [clog2(RAM_SIZE)-1:0] curr_addr;
assign ram_read_addr = curr_addr;
reg [BYTE_LEN-1:0] data_buff;
reg data_ready = 0;
wire uart_driver_done;
uart_driver uart_driver_inst(
	.clk(clk), .reset(reset), .data_ready(data_ready),
	.data(data_buff), .txd(uart_txd), .done(uart_driver_done));

always @(posedge clk) begin
	if (reset) begin
		state <= STATE_IDLE;
		data_ready <= 0;
		ram_read_req <= 0;
	end else if (state == STATE_IDLE && start) begin
		curr_addr <= read_start;
		state <= STATE_READING;
		ram_read_req <= 1;
	end else if (state == STATE_READING && ram_read_ready) begin
		curr_addr <= curr_addr + 1;
		state <= STATE_WRITING;
		ram_read_req <= 0;
		data_ready <= 1;
		data_buff <= ram_read_out;
	end else if (state == STATE_WRITING && uart_driver_done) begin
		if (curr_addr == read_end)
			state <= STATE_IDLE;
		else begin
			state <= STATE_READING;
			ram_read_req <= 1;
		end
		data_ready <= 0;
	end
end

endmodule

module ram_to_uart_tester(
	input clk, reset, start,
	output uart_txd);

`include "params.vh"

localparam RAM_SIZE = PACKET_BUFFER_SIZE;

wire ram_read_req, ram_read_ready;
wire [BYTE_LEN-1:0] ram_read_out;
wire [clog2(RAM_SIZE)-1:0] ram_read_addr;
packet_buffer_ram_driver packet_buffer_ram_driver_inst(
	.clk(clk), .reset(reset),
	.read_req(ram_read_req), .read_addr(ram_read_addr),
	.read_ready(ram_read_ready), .read_out(ram_read_out),
	.write_enable(0));
ram_to_uart ram_to_uart_inst(
	.clk(clk), .reset(reset), .start(start),
	.read_start(0), .read_end(RAM_SIZE),
	.ram_read_ready(ram_read_ready), .ram_read_out(ram_read_out),
	.uart_txd(uart_txd), .ram_read_req(ram_read_req),
	.ram_read_addr(ram_read_addr));

endmodule

// SW[0]: reset
// BTNC: dump ram
module main(
	input CLK100MHZ,
	input [15:0] SW,
	input BTNC, BTNU, BTNL, BTNR, BTND,
	output [7:0] JB,
	output [3:0] VGA_R,
	output [3:0] VGA_B,
	output [3:0] VGA_G,
	output VGA_HS,
	output VGA_VS,
	output LED16_B, LED16_G, LED16_R,
	output LED17_B, LED17_G, LED17_R,
	output [15:0] LED,
	output [7:0] SEG,  // segments A-G (0-6), DP (7)
	output [7:0] AN,	// Display 0-7
	inout ETH_CRSDV, ETH_RXERR,
	inout [1:0] ETH_RXD,
	output ETH_REFCLK, ETH_INTN, ETH_RSTN,
	output UART_RXD_OUT, UART_CTS,
	inout [15:0] ddr2_dq,
	inout [1:0] ddr2_dqs_n, ddr2_dqs_p,
	output [12:0] ddr2_addr,
	output [2:0] ddr2_ba,
	output ddr2_ras_n, ddr2_cas_n, ddr2_we_n,
	output [0:0] ddr2_ck_p, ddr2_ck_n, ddr2_cke, ddr2_cs_n,
	output [1:0] ddr2_dm,
	output [0:0] ddr2_odt
	);

`include "params.vh"

parameter RAM_SIZE = PACKET_BUFFER_SIZE;

wire clk_50mhz;

// the main clock for FPGA logic will be 50MHz
wire clk;
assign clk = clk_50mhz;

// 50MHz clock for Ethernet receiving
clk_wiz_0 clk_wiz_inst(
	.reset(0),
	.clk_in1(CLK100MHZ), .clk_out1(clk_50mhz));

wire reset;
synchronize(.clk(clk), .in(SW[0]), .out(reset));

wire [31:0] hex_display_data;
wire [6:0] segments;

display_8hex display(
	.clk(clk), .data(hex_display_data), .seg(segments), .strobe(AN));

assign SEG[7] = 1'b1;
assign SEG[6:0] = segments;

assign LED16_R = BTNL; // left button -> red led
assign LED16_G = BTNC; // center button -> green led
assign LED16_B = BTNR; // right button -> blue led
assign LED17_R = BTNL;
assign LED17_G = BTNC;
assign LED17_B = BTNR;

wire [10:0] hcount;
wire [9:0] vcount;
wire hsync, vsync, blank;

xvga xvga_inst(
	.vclock(clk), .hcount(hcount), .vcount(vcount),
	.hsync(hsync), .vsync(vsync), .blank(blank));

wire [3:0] vga_r, vga_g, vga_b;
assign {vga_r, vga_g, vga_b} = 0;
assign VGA_R = ~blank ? vga_r : 0;
assign VGA_G = ~blank ? vga_g : 0;
assign VGA_B = ~blank ? vga_b : 0;

assign VGA_HS = ~hsync;
assign VGA_VS = ~vsync;

assign UART_CTS = 1;

wire btnc, btnl;
sync_debounce sd_btnc(
	.reset(reset), .clk(clk), .in(BTNC), .out(btnc));
sync_debounce sd_btnl(
	.reset(reset), .clk(clk), .in(BTNL), .out(btnl));

wire ram_read_req, ram_read_ready, ram_write_enable;
wire [clog2(RAM_SIZE)-1:0] ram_read_addr, ram_write_addr;
wire [BYTE_LEN-1:0] ram_read_out, ram_write_val;
packet_buffer_ram_driver packet_buffer_ram_driver_inst(
	.clk(clk), .reset(reset),
	.read_req(ram_read_req), .read_addr(ram_read_addr),
	.read_ready(ram_read_ready), .read_out(ram_read_out),
	.write_enable(ram_write_enable),
	.write_addr(ram_write_addr), .write_val(ram_write_val));
ram_to_uart ram_to_uart_inst(
	.clk(clk), .reset(reset), .start(btnc),
	.read_start(0), .read_end(RAM_SIZE),
	.ram_read_ready(ram_read_ready), .ram_read_out(ram_read_out),
	.uart_txd(UART_RXD_OUT), .ram_read_req(ram_read_req),
	.ram_read_addr(ram_read_addr));

assign ETH_REFCLK = clk;
wire eth_outclk, eth_done, eth_byte_outclk, eth_dtb_done;
wire [1:0] eth_out;
ethernet_driver eth_driv_inst(
	.clk(clk), .reset(reset),
	.crsdv(ETH_CRSDV), .rxerr(ETH_RXERR),
	.rxd(ETH_RXD),
	.intn(ETH_INTN), .rstn(ETH_RSTN),
	.out(eth_out),
	.outclk(eth_outclk), .done(eth_done));
dibits_to_bytes eth_dtb(
	.clk(clk), .reset(reset),
	.inclk(eth_outclk), .in(eth_out), .done_in(eth_done),
	.out(ram_write_val), .outclk(eth_byte_outclk), .done_out(eth_dtb_done));
assign ram_write_enable = eth_byte_outclk;

// maximum ethernet frame length is 1522 bytes
localparam MAX_ETH_FRAME_LEN = 1522;
reg [clog2(MAX_ETH_FRAME_LEN)-1:0] eth_byte_cnt = 0;
reg record = 1;
always @(posedge clk) begin
	if (reset) begin
		eth_byte_cnt <= 0;
		record <= 1;
	end else if (eth_done) begin
		eth_byte_cnt <= 0;
		record <= 0;
	end else if (eth_byte_outclk && record)
		eth_byte_cnt <= eth_byte_cnt + 1;
end
assign ram_write_addr = eth_byte_cnt;

// DEBUGGING SIGNALS

wire blink;
blinker blinker_inst(
	.clk(clk), .reset(reset),
	.enable(1), .out(blink));

assign LED = {
	SW[15:2],
	blink,
	reset
};

assign hex_display_data = {
	4'h0, ram_write_addr, 4'h0, ram_read_addr
};

assign JB = {
	4'h0,
	ETH_RXD,
	UART_RXD_OUT,
	ETH_CRSDV
};

endmodule
