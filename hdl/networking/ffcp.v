// FFCP: FGPA Flow Control Protocol
// simple invented-here protocol for flow control
// format: [ type (2 bits) | index (6 bits) | FGP data (769 bytes) ]
// type is 0 (syn), 1 (msg) or 2 (ack)
// The index works like the sequence number in TCP
// FFCP's flow control is a very simplified version of TCP's, where
// the window size is fixed and data flows in only one direction
// FGP data is omitted in ack

module ffcp_tx #(
	parameter LATENCY = PACKET_SYNTH_ROM_LATENCY) (
	input clk, rst, start, in_done,
	input inclk, input [BYTE_LEN-1:0] in,
	input [FFCP_TYPE_LEN-1:0] ffcp_type,
	input [FFCP_INDEX_LEN-1:0] ffcp_index,
	input readclk,
	output outclk, output [BYTE_LEN-1:0] out,
	output upstream_readclk, done);

`include "networking.vh"

reg [BYTE_LEN-1:0] metadata_buf;
wire [FFCP_TYPE_LEN-1:0] metadata_buf_type;
assign metadata_buf_type = metadata_buf[FFCP_INDEX_LEN+:FFCP_TYPE_LEN];
wire is_ack;
assign is_ack = metadata_buf_type == FFCP_TYPE_ACK;

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

localparam STATE_METADATA = 0;
localparam STATE_DATA = 1;

reg [0:0] state = STATE_METADATA;
reg [9:0] cnt = 0;

wire metadata_done;
assign metadata_done =
	state == STATE_METADATA && cnt == FFCP_METADATA_LEN-1;
wire ack_done;
delay #(.DELAY_LEN(LATENCY),
	.DATA_WIDTH(BYTE_LEN)) done_delay(
	.clk(clk), .rst(rst || start),
	.in(readclk && metadata_done && is_ack), .out(ack_done));
assign done = in_done || ack_done;

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
		if (metadata_done) begin
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

`include "networking.vh"

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

// ignore all messages other than those in receive window// be careful of wraparound
wire ignore, receiving;
assign ignore = in_index - queue_head >= FFCP_WINDOW_LEN;
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

// the ffcp_tx_queue manages the packet buffer (PB), intended for use
// with ffcp_tx_server
// we only need to be able to advance the tail by one index each time,
// or overwrite the head
module ffcp_tx_queue(
	input clk, rst,
	input advance_tail,
	input inclk, input [clog2(PB_QUEUE_LEN)-1:0] in_head,
	output almost_full,
	output reg [clog2(PB_QUEUE_LEN)-1:0] head, tail);

`include "networking.vh"

assign almost_full = tail - head >= PB_QUEUE_ALMOST_FULL_THRES;

always @(posedge clk) begin
	if (rst) begin
		tail <= 0;
		head <= 0;
	end else begin
		if (inclk)
			head <= in_head;
		if (advance_tail)
			tail <= tail + 1;
	end
end

endmodule

// inclk: ack indices
// outclk: ffcp index and payload index in PB
// outclk_pb: update PB queue head
module ffcp_tx_server(
	input clk, rst,
	input [clog2(PB_QUEUE_LEN)-1:0] pb_head, pb_tail,
	input downstream_done,
	input inclk, input [FFCP_INDEX_LEN-1:0] in_index,
	output outclk, out_syn,
	output [FFCP_INDEX_LEN-1:0] out_index,
	output [clog2(PB_QUEUE_LEN)-1:0] out_buf_pos,
	// the pb outputs are the interface to ffcp_tx_queue
	output outclk_pb,
	output [clog2(PB_QUEUE_LEN)-1:0] out_pb_head);

`include "networking.vh"

reg downstream_rdy = 1;
always @(posedge clk) begin
	if (rst)
		downstream_rdy <= 1;
	else if (outclk)
		downstream_rdy <= 0;
	else if (downstream_done)
		downstream_rdy <= 1;
end

reg [clog2(FFCP_BUFFER_LEN)-1:0] queue_head = 0;
reg [clog2(FFCP_BUFFER_LEN)-1:0] curr_index = 0;

wire [clog2(FFCP_BUFFER_LEN)-1:0] window_end;
wire at_end;
assign window_end = queue_head + FFCP_WINDOW_LEN;
assign at_end = curr_index == window_end ||
	curr_index - queue_head + pb_head == pb_tail;

// ignore all acks other than those in transmit window
// be careful of wraparound
wire ignore, receiving;
assign ignore = in_index - queue_head >= FFCP_WINDOW_LEN;
assign receiving = inclk && !ignore;

assign outclk_pb = receiving;
assign out_pb_head = in_index - queue_head + pb_head;

// first packet should be a syn once reset is done
reg syn_buf = 1;
assign out_syn = syn_buf && out_index == 0;
assign out_index = curr_index;
assign out_buf_pos = curr_index - queue_head + pb_head;
assign outclk = !rst && downstream_rdy && !at_end;

localparam TESTING = 0;
localparam RESEND_TIMEOUT = TESTING ? 4 : 500000;
localparam RESYN_TIMEOUT = TESTING ? 20 : 50000000;

wire resend_disable, resyn_disable, resyn;
// wait for 10ms before trying again
pulse_extender #(.EXTEND_LEN(RESEND_TIMEOUT)) resend_timer (
	.clk(clk), .rst(rst), .in(!at_end), .out(resend_disable));
// wait for 1s before trying to re-establish the connection
pulse_extender #(.EXTEND_LEN(RESYN_TIMEOUT)) resyn_timer (
	.clk(clk), .rst(rst), .in(inclk), .out(resyn_disable));
pulse_generator resyn_pg (
	.clk(clk), .rst(rst), .in(!resyn_disable), .out(resyn));

always @(posedge clk) begin
	if (rst || resyn) begin
		queue_head <= 0;
		curr_index <= 0;
		syn_buf <= 1;
	end else begin
		if (receiving) begin
			syn_buf <= 0;
			queue_head <= in_index;
		end
		// if in_index is "more than" curr_index, just skip ahead
		if (receiving && in_index - curr_index < FFCP_WINDOW_LEN)
			curr_index <= in_index;
		else if (outclk)
			curr_index <= curr_index + 1;
		else if (!resend_disable)
			curr_index <= queue_head;
	end
end

endmodule
