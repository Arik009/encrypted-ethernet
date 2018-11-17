// generate an ethernet frame on the level of bytes
// CRC needs to be generated on the level of dibits,
// which is handled in eth_frame_generator
// expects payload and rom delay of PACKET_SYNTH_ROM_DELAY
// exposes readclk to out latency of PACKET_SYNTH_ROM_DELAY
module eth_frame_byte_generator #(
	parameter RAM_SIZE = PACKET_SYNTH_ROM_SIZE) (
	input clk, rst, start, in_done,
	input inclk, input [BYTE_LEN-1:0] in,
	input readclk, input ram_outclk, input [BYTE_LEN-1:0] ram_out,
	output ram_readclk,
	output reg [clog2(RAM_SIZE)-1:0] ram_raddr,
	output outclk, output [BYTE_LEN-1:0] out,
	output upstream_readclk, done);

`include "params.vh"
`include "packet_synth_rom_layout.vh"

localparam STATE_IDLE = 0;
localparam STATE_MAC_DST = 1;
localparam STATE_MAC_SRC = 2;
localparam STATE_ETHERTYPE = 3;
localparam STATE_PAYLOAD = 4;

localparam MAC_LEN = 6;
localparam ETHERTYPE_LEN = 2;

// "pre-delayed" versions of outputs, to be combined with
// ram read results
wire outclk_pd;
wire [BYTE_LEN-1:0] out_pd;
// out_premux is out, but may be overwritten with ram read result
wire [BYTE_LEN-1:0] out_premux;
// if we requested a payload read, then we want to use the payload
// read result instead
// if we requested a read from ram, then we want to use the ram read
// result instead
assign out = inclk ? in : (ram_outclk ? ram_out : out_premux);

wire outclk_internal;
assign outclk = outclk_internal || ram_outclk || inclk;
delay #(.DELAY_LEN(PACKET_SYNTH_ROM_LATENCY)) outclk_delay(
	.clk(clk), .rst(rst || start),
	.in(outclk_pd), .out(outclk_internal));
delay #(.DELAY_LEN(PACKET_SYNTH_ROM_LATENCY),
	.DATA_WIDTH(BYTE_LEN)) out_delay(
	.clk(clk), .rst(rst || start), .in(out_pd), .out(out_premux));
delay #(.DELAY_LEN(PACKET_SYNTH_ROM_LATENCY)) done_delay(
	.clk(clk), .rst(rst || start), .in(in_done), .out(done));

reg [2:0] state = STATE_IDLE;
reg [2:0] cnt;

assign upstream_readclk = state == STATE_PAYLOAD && readclk;
assign ram_readclk = (
	(state == STATE_MAC_DST) ||
	(state == STATE_MAC_SRC)) &&
	readclk;
assign outclk_pd =
	(state == STATE_ETHERTYPE) &&
	readclk;
// make EtherType 0x0000 for now
assign out_pd = 0;

always @(posedge clk) begin
	if (rst)
		state <= STATE_IDLE;
	else if (start) begin
		state <= STATE_MAC_DST;
		ram_raddr <= MAC_RECV_OFF;
		cnt <= 0;
	end else if (readclk) begin
		case (state)
		STATE_MAC_DST:
			if (cnt == MAC_LEN -1) begin
				cnt <= 0;
				state <= STATE_MAC_SRC;
				ram_raddr <= MAC_SEND_OFF;
			end else begin
				cnt <= cnt + 1;
				ram_raddr <= ram_raddr + 1;
			end
		STATE_MAC_SRC:
			if (cnt == MAC_LEN -1) begin
				cnt <= 0;
				state <= STATE_ETHERTYPE;
			end else begin
				cnt <= cnt + 1;
				ram_raddr <= ram_raddr + 1;
			end
		STATE_ETHERTYPE:
			if (cnt == ETHERTYPE_LEN-1) begin
				state <= STATE_PAYLOAD;
			end else
				cnt <= cnt + 1;
		STATE_PAYLOAD:
			if (in_done) begin
				cnt <= 0;
				state <= STATE_IDLE;
			end else
				cnt <= cnt + 1;
		endcase
	end
end

endmodule

module crc32(
	input clk, rst,
	// allow application to reuse out as a shift register
	input shift, inclk, input [1:0] in,
	output [31:0] out);

localparam CRC_INIT = 32'hffffffff;
// polynomial is reflected since we operate in
// least-significant-bit first for simplicity
localparam CRC_POLY = 32'hedb88320;

reg [31:0] curr;
// leave at least significant bit first, since crc will be
// shifted out from the end
assign out = ~curr;

// optimized dibit CRC step
// step1: XOR in both inputs at once
// step2: first division by poly
// step3: second division by poly
wire [31:0] step1, step2, step3;
assign step1 = {curr[2+:30], curr[0+:2] ^ in};
assign step2 = step1[1+:31] ^ (step1[0] ? CRC_POLY : 0);
assign step3 = step2[1+:31] ^ (step2[0] ? CRC_POLY : 0);

always @(posedge clk) begin
	if (rst)
		curr <= CRC_INIT;
	else if (shift)
		curr <= {2'b11, curr[2+:30]};
	else if (inclk)
		curr <= step3;
end

endmodule

// creates continuous dibit stream for an ethernet frame
module eth_frame_generator #(
	parameter RAM_SIZE = PACKET_SYNTH_ROM_SIZE) (
	input clk, rst, start, in_done,
	input inclk, input [BYTE_LEN-1:0] in,
	input ram_outclk, input [BYTE_LEN-1:0] ram_out,
	output ram_readclk,
	output [clog2(RAM_SIZE)-1:0] ram_raddr,
	output outclk, output [1:0] out,
	output upstream_readclk, done);

`include "params.vh"

// main processing ready for frame body
wire main_rdy;
wire efg_readclk, efg_outclk, efg_done;
wire [BYTE_LEN-1:0] efg_out;
wire btd_inclk, btd_outclk, btd_in_done, btd_rdy, btd_done;
wire [BYTE_LEN-1:0] btd_in;
wire [1:0] btd_out;
wire crc_rst, crc_shift;
wire [31:0] crc_out;
eth_frame_byte_generator efg_inst(
	.clk(clk), .rst(rst), .start(start), .in_done(in_done),
	.inclk(inclk), .in(in), .readclk(efg_readclk),
	.ram_outclk(ram_outclk), .ram_out(ram_out),
	.ram_readclk(ram_readclk), .ram_raddr(ram_raddr),
	.outclk(efg_outclk), .out(efg_out),
	.upstream_readclk(upstream_readclk), .done(efg_done));
stream_coord_buf #(.DATA_WIDTH(BYTE_LEN)) btd_scb_inst(
	.clk(clk), .rst(rst || start),
	.inclk(efg_outclk), .in(efg_out),
	.in_done(efg_done), .downstream_rdy(btd_rdy && main_rdy),
	.outclk(btd_inclk), .out(btd_in), .done(btd_in_done),
	.readclk(efg_readclk));
bytes_to_dibits btd_inst(
	.clk(clk), .rst(rst || start),
	.inclk(btd_inclk), .in(btd_in),
	.in_done(btd_in_done), .outclk(btd_outclk), .out(btd_out),
	.rdy(btd_rdy), .done(btd_done));
crc32 crc32_inst(
	.clk(clk), .rst(crc_rst), .shift(crc_shift),
	.inclk(btd_outclk), .in(btd_out), .out(crc_out));

// measured in dibits
localparam PREAMBLE_LEN = 32;
localparam CRC_LEN = 16;

localparam STATE_IDLE = 0;
localparam STATE_PREAMBLE = 1;
localparam STATE_BODY = 2;
localparam STATE_CRC = 3;

reg [1:0] state = STATE_IDLE;
reg [4:0] cnt;
// sfd indicates that we have reached the end of the preamble
wire sfd, end_of_crc;
assign sfd = (state == STATE_PREAMBLE) && (cnt == PREAMBLE_LEN-1);
assign end_of_crc = (state == STATE_CRC) && (cnt == CRC_LEN-1);
assign crc_shift = state == STATE_CRC;
assign out =
	(state == STATE_BODY) ? btd_out :
	sfd ? 2'b11 :
	(state == STATE_PREAMBLE) ? 2'b01 :
	crc_out[0+:2];
assign crc_rst = rst || sfd;
assign main_rdy = sfd || (state == STATE_BODY);
assign done = end_of_crc;
assign outclk = btd_outclk ||
	(state == STATE_PREAMBLE) ||
	(state == STATE_CRC);

always @(posedge clk) begin
	if (rst)
		state <= STATE_IDLE;
	else if (start) begin
		cnt <= 0;
		state <= STATE_PREAMBLE;
	end else if (sfd)
		state <= STATE_BODY;
	else if (state == STATE_BODY && btd_done) begin
		state <= STATE_CRC;
		cnt <= 0;
	end else if (end_of_crc)
		state <= STATE_IDLE;
	else
		cnt <= cnt + 1;
end

endmodule
