// FGP: FPGA Graphics Protocol
// simple invented-here DMA protocol used to transmit graphics information
// sits just above the ethernet layer
// should be encrypted in actual packet
// the padding makes it a multiple of 128, for AES
// format: [ offset (1 byte) | padding (127 bytes) | data (768 bytes) ]
// 768 bytes = 512 colors

// produces outputs with a latency of LATENCY
// essentially the same structure as eth_tx
module fgp_tx #(
	parameter LATENCY = PACKET_SYNTH_ROM_LATENCY) (
	input clk, rst, start, in_done,
	input inclk, input [BYTE_LEN-1:0] in,
	input [BYTE_LEN-1:0] offset,
	input readclk,
	output outclk, output [BYTE_LEN-1:0] out,
	output upstream_readclk, done);

`include "params.vh"
`include "networking.vh"

reg [BYTE_LEN-1:0] offset_buf;

wire outclk_pd;
wire [BYTE_LEN-1:0] out_pd, out_premux;
assign out =
	inclk ? in :
	out_premux;
wire outclk_internal;
assign outclk = outclk_internal || inclk;
delay #(.DELAY_LEN(LATENCY)) outclk_delay(
	.clk(clk), .rst(rst || start),
	.in(outclk_pd), .out(outclk_internal));
delay #(.DELAY_LEN(LATENCY),
	.DATA_WIDTH(BYTE_LEN)) out_delay(
	.clk(clk), .rst(rst || start), .in(out_pd), .out(out_premux));
assign done = in_done;

localparam STATE_OFFSET = 0;
localparam STATE_DATA = 1;

reg [0:0] state = STATE_OFFSET;
reg [9:0] cnt = 0;

assign upstream_readclk = (state == STATE_DATA) && readclk;
assign outclk_pd =
	(state == STATE_OFFSET) &&
	readclk;
// if we're not in offset, then we're in padding
assign out_pd =
	(state == STATE_OFFSET) ? offset : 0;

always @(posedge clk) begin
	if (rst || start) begin
		state <= STATE_OFFSET;
		offset_buf <= offset;
		cnt <= 0;
	end else if (readclk) begin
		if (state == STATE_OFFSET && cnt == FGP_OFFSET_LEN-1) begin
			state <= STATE_DATA;
			cnt <= 0;
		end else
			cnt <= cnt + 1;
	end
end

endmodule

// parser drives ram directly for now, no error detection
module fgp_rx(
	input clk, rst, inclk,
	input [BYTE_LEN-1:0] in,
	output done,
	output offset_outclk,
	output [BYTE_LEN-1:0] offset_out,
	output outclk, output [BYTE_LEN-1:0] out);

`include "params.vh"
`include "networking.vh"

localparam STATE_OFFSET = 0;
localparam STATE_DATA = 1;

reg [0:0] state = STATE_OFFSET;
reg [9:0] cnt = 0;

assign done = inclk && state == STATE_DATA && cnt == FGP_DATA_LEN-1;
assign offset_outclk = inclk && state == STATE_OFFSET &&
	cnt == FGP_OFFSET_LEN-1;
assign offset_out = in;
assign outclk = inclk && state == STATE_DATA;
assign out = in;

always @(posedge clk) begin
	if (rst || done) begin
		state <= STATE_OFFSET;
		cnt <= 0;
	end else if (inclk) begin
		if (state == STATE_OFFSET && cnt == FGP_OFFSET_LEN-1) begin
			state <= STATE_DATA;
			cnt <= 0;
		end else
			cnt <= cnt + 1;
	end
end

endmodule
