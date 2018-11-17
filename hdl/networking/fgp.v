// FGP: FPGA Graphics Protocol
// simple invented-here DMA protocol used to transmit graphics information
// sits just above the ethernet layer
// should be encrypted in actual packet
// the padding makes it a multiple of 128, for AES
// format: [ offset (1 byte) | padding (127 bytes) | data (768 bytes) ]
// 768 bytes = 512 colors

// parser drives ram directly for now, no error detection
module fgp_parser(
	input clk, rst, inclk,
	input [BYTE_LEN-1:0] in,
	output done,
	output setoff_req,
	output [BYTE_LEN+clog2(FGP_DATA_LEN_COLORS)-1:0] setoff_val,
	output outclk, output [BYTE_LEN-1:0] out);

`include "params.vh"
`include "networking.vh"

localparam STATE_OFFSET = 0;
localparam STATE_PADDING = 1;
localparam STATE_DATA = 2;

reg [1:0] state = STATE_OFFSET;
reg [9:0] cnt = 0;

assign done = inclk && state == STATE_DATA && cnt == FGP_DATA_LEN-1;
assign setoff_req = inclk && state == STATE_OFFSET &&
	cnt == FGP_OFFSET_LEN-1;
assign setoff_val = {in, {clog2(FGP_DATA_LEN_COLORS){1'b0}}};
assign outclk = inclk && state == STATE_DATA;
assign out = in;

always @(posedge clk) begin
	if (rst) begin
		state <= STATE_OFFSET;
		cnt <= 0;
	end else if (inclk) begin
		if (state == STATE_OFFSET && cnt == FGP_OFFSET_LEN-1) begin
			state <= STATE_PADDING;
			cnt <= 0;
		end else if (state == STATE_PADDING &&
				cnt == FGP_PADDING_LEN-1) begin
			state <= STATE_DATA;
			cnt <= 0;
		end else
			cnt <= cnt + 1;
	end
end

endmodule
