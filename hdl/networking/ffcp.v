// FFCP: FGPA Flow Control Protocol
// simple invented-here protocol for flow control
// format: [ type (2 bits) | index (6 bits) | FGP data (769 bytes) ]
// type is 0 (syn), 1 (msg) or 2 (ack)
// The index works like the sequence number in TCP
// FFCP's flow control is a very simplified version of TCP's, where
// the window size is fixed and data flows in only one direction

module ffcp_tx #(
	parameter LATENCY = PACKET_SYNTH_ROM_LATENCY) (
	input clk, rst, start, in_done,
	input inclk, input [BYTE_LEN-1:0] in,
	input [FFCP_TYPE_LEN-1:0] ffcp_type,
	input [FFCP_INDEX_LEN:0] ffcp_index,
	input readclk,
	output outclk, output [BYTE_LEN-1:0] out,
	output upstream_readclk, done);

`include "params.vh"
`include "networking.vh"

reg [BYTE_LEN-1:0] metadata_buf;

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

localparam STATE_METADATA = 0;
localparam STATE_DATA = 1;

reg [0:0] state = STATE_METADATA;
reg [9:0] cnt = 0;

assign upstream_readclk = (state == STATE_DATA) && readclk;
assign outclk_pd = (state == STATE_METADATA) && readclk;
assign out_pd =
	(state == STATE_METADATA) ? metadata_buf : 0;

always @(posedge clk) begin
	if (rst || start) begin
		state <= STATE_METADATA;
		metadata_buf <= {ffcp_type, ffcp_index};
		cnt <= 0;
	end else if (readclk) begin
		if (state == STATE_METADATA && cnt == FFCP_METADATA_LEN-1) begin
			state <= STATE_DATA;
			cnt <= 0;
		end else
			cnt <= cnt + 1;
	end
end

endmodule

module ffcp_rx(
	input clk, rst, inclk,
	input [BYTE_LEN-1:0] in,
	output done,
	output metadata_outclk,
	output [FFCP_TYPE_LEN-1:0] ffcp_type,
	output [FFCP_INDEX_LEN-1:0] ffcp_index,
	output outclk, output [BYTE_LEN-1:0] out);

`include "params.vh"
`include "networking.vh"

localparam STATE_METADATA = 0;
localparam STATE_DATA = 1;

reg [0:0] state = STATE_METADATA;
reg [9:0] cnt = 0;

wire metadata_done;
assign metadata_done =
	state == STATE_METADATA && cnt == FFCP_METADATA_LEN-1;
assign done = inclk && state == STATE_DATA && cnt == FFCP_DATA_LEN-1;
assign metadata_outclk = inclk && metadata_done;
assign ffcp_type = in[FFCP_INDEX_LEN+:FFCP_TYPE_LEN];
assign ffcp_index = in[0+:FFCP_INDEX_LEN];
assign outclk = inclk && state == STATE_DATA;
assign out = in;

always @(posedge clk) begin
	if (rst || done) begin
		state <= STATE_METADATA;
		cnt <= 0;
	end else if (inclk) begin
		if (metadata_done) begin
			state <= STATE_DATA;
			cnt <= 0;
		end else
			cnt <= cnt + 1;
	end
end

endmodule

module ffcp_rx_server(
	input clk, rst, syn,
	input inclk, input [FFCP_INDEX_LEN-1:0] in_index,
	input downstream_done,
	output outclk, output [FFCP_INDEX_LEN-1:0] out_index);

`include "params.vh"
`include "networking.vh"

localparam FFCP_BUFFER_LEN = 2**FFCP_INDEX_LEN;

reg received[FFCP_BUFFER_LEN-1:0];
reg [clog2(FFCP_BUFFER_LEN)-1:0] rst_cnt = 0;
reg [clog2(FFCP_BUFFER_LEN)-1:0] queue_head;
wire curr_received;
assign curr_received = received[queue_head];
wire rst_done;
assign rst_done = rst_cnt == FFCP_BUFFER_LEN-1;

reg downstream_rdy = 1;
always @(posedge clk) begin
	if (rst)
		downstream_rdy <= 1;
	else if (outclk)
		downstream_rdy <= 0;
	else if (downstream_done)
		downstream_rdy <= 1;
end

// ignore all messages from FFCP_WINDOW_LEN indices before head
// be careful of wraparound
wire [clog2(FFCP_BUFFER_LEN)-1:0] ignore_start;
wire ignore, receiving;
assign ignore_start = queue_head - FFCP_WINDOW_LEN;
assign ignore =
	(ignore_start < queue_head) ?
		(in_index >= ignore_start && in_index < queue_head)
	: (in_index >= ignore_start || in_index < queue_head);
assign receiving = inclk && !ignore;

// try to ack as many indices as possible, so wait until the queue head
// has advanced past all received packets before acking
// also wait until self and downstream is ready
reg ack_buf = 0;
assign outclk = ack_buf && !curr_received &&
	rst_done && !receiving && downstream_rdy;
assign out_index = queue_head;

always @(posedge clk) begin
	if (rst || syn) begin
		// if we're resetting because of a syn, we've already received
		// the message at index 0
		queue_head <= syn ? 1 : 0;
		rst_cnt <= 0;
		ack_buf <= syn;
		// clear this now so we don't need to waste a rst_cnt bit for
		// an extra clear cycle
		received[FFCP_BUFFER_LEN-1] <= 0;
	end else if (!rst_done) begin
		received[rst_cnt] <= 0;
		rst_cnt <= rst_cnt + 1;
	end else if (receiving)
		received[in_index] <= 1;
	else if (curr_received) begin
		ack_buf <= 1;
		received[queue_head] <= 0;
		queue_head <= queue_head + 1;
	end else if (outclk)
		ack_buf <= 0;
end

endmodule
