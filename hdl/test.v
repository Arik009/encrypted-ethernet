`timescale 1ns / 1ps

module test_ram_to_uart();

reg clk = 0;
// 50MHz clock
initial forever #10 clk = ~clk;

reg reset = 1;
reg start = 0;
wire txd;
ram_to_uart_tester ram_to_uart_tester_inst(
	.clk(clk), .reset(reset), .start(start), .uart_txd(txd));

initial begin
	#100
	reset = 0;
	start = 1;
	#1000000
	$stop();
end

endmodule

module test_ethernet_driver();

`include "params.vh"

reg clk = 0;
// 50MHz clock
initial forever #10 clk = ~clk;

reg reset = 1;
wire crsdv_tr = 1'bz, rxerr_tr = 1'bz;
wire [1:0] rxd_tr = 2'bzz;
reg crsdv, rxerr;
reg [1:0] rxd;
reg reset_done = 0;
assign crsdv_tr = reset_done ? crsdv : 1'bz;
assign rxerr_tr = reset_done ? rxerr : 1'bz;
assign rxd_tr = reset_done ? rxd : 2'bzz;
wire intn, rstn;
wire [1:0] eth_out;
wire eth_outclk, eth_done;
wire [BYTE_LEN-1:0] eth_byte_out;
wire eth_byte_outclk, eth_dtb_done;
ethernet_driver eth_driv_inst(
	.clk(clk), .reset(reset),
	.crsdv(crsdv_tr), .rxerr(rxerr_tr),
	.rxd(rxd_tr),
	.intn(intn), .rstn(rstn),
	.out(eth_out),
	.outclk(eth_outclk), .done(eth_done));
dibits_to_bytes eth_dtb(
	.clk(clk), .reset(reset),
	.inclk(eth_outclk), .in(eth_out), .done_in(eth_done),
	.out(eth_byte_out), .outclk(eth_byte_outclk), .done_out(eth_dtb_done));

initial begin
	#100
	reset = 0;
	// reset sequence
	#400
	reset_done = 1;
	rxerr = 0;
	rxd = 0;
	crsdv = 0;
	#100
	crsdv = 1;
	#100

	// 28 cycles = 56 bits
	rxd = 2'b01;
	#540
	rxd = 2'b11;
	#20

	// send 10101010 twice (2 bytes)
	rxd = 2'b10;
	#160

	rxd = 0;
	crsdv = 0;
	#160

	$stop();
end

endmodule

module test_main();

reg clk;
// 50MHz clock
initial forever #10 clk = ~clk;

initial begin
	$stop();
end

endmodule
