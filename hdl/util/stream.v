// convert a dibit stream to a bytestream
// no latency between in and out
module dibits_to_bytes(
	// inclk is pulsed when a dibit is presented on in
	// outclk is pulsed when a byte is presented on out
	input clk, rst, inclk,
	input [1:0] in,
	input in_done,
	output outclk,
	output [BYTE_LEN-1:0] out,
	output done);

`include "params.vh"

// scratch space to shift dibits in
reg [BYTE_LEN-3:0] shifted;
// only need half as much since we get 2 bits at a time
reg [clog2(BYTE_LEN)-2:0] cnt = 0;

assign out = {in, shifted};
assign outclk = inclk && cnt == BYTE_LEN/2-1;
assign done = in_done;

always @(posedge clk) begin
	if (rst || in_done) begin
		cnt <= 0;
	end else if (outclk)
		cnt <= 0;
	else if (inclk) begin
		shifted <= {in, shifted[2+:BYTE_LEN-4]};
		cnt <= cnt + 1;
	end
end

endmodule

// convert a bytestream to a dibit stream
// assumes that bytes are inserted no faster than once every 4 clock cycles
module bytes_to_dibits(
	// inclk is pulsed when a byte is presented on in
	// outclk is pulsed when a dibit is presented on out
	input clk, rst, inclk, input [BYTE_LEN-1:0] in,
	input in_done,
	output reg outclk = 0, output [1:0] out,
	output rdy,
	// done is pulsed after done_in when buffer has been cleared
	output done);

`include "params.vh"

// scratch space to shift dibits out
reg [BYTE_LEN-1:0] shifted;
assign out = shifted[1:0];
// only need half as much since we output 2 bits at a time
reg [clog2(BYTE_LEN)-2:0] cnt = BYTE_LEN/2-1;
assign idle = cnt == BYTE_LEN/2-1;
assign rdy = idle;

// in_done asserted, should assert done when buffer clears
reg in_done_found = 0;
assign done = in_done_found && idle;

always @(posedge clk) begin
	if (rst) begin
		cnt <= BYTE_LEN/2-1;
		outclk <= 0;
		in_done_found <= 0;
	end else begin
		if (in_done)
			in_done_found <= 1;
		else if (in_done_found && !idle)
			in_done_found <= 0;

		if (inclk) begin
			shifted <= in;
			cnt <= 0;
			outclk <= 1;
		end else if (!idle) begin
			outclk <= 1;
			shifted <= {2'b00, shifted[2+:BYTE_LEN-2]};
			cnt <= cnt + 1;
		end else
			outclk <= 0;
	end
end

endmodule

module bytes_to_colors(
	input clk, rst,
	input inclk, input [BYTE_LEN-1:0] in,
	output reg outclk, output reg [COLOR_LEN-1:0] out);

`include "params.vh"

// three states to convert three bytes into two colors
reg [1:0] state = 0;
reg [BYTE_LEN-1:0] prev_in;

always @(posedge clk) begin
	prev_in <= in;
	if (rst)
		state <= 0;
	else if (inclk) begin
		case (state)
		1: begin
			outclk <= 1;
			out <= {prev_in, in[BYTE_LEN/2+:BYTE_LEN/2]};
		end
		2: begin
			outclk <= 1;
			out <= {prev_in[0+:BYTE_LEN/2], in};
		end
		default:
			outclk <= 0;
		endcase

		if (state == 2)
			state <= 0;
		else
			state <= state + 1;
	end else
		outclk <= 0;
end

endmodule

// stream data out of memory, rate-limited by ready
module stream_from_memory #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE,
	parameter RAM_READ_LATENCY = PACKET_BUFFER_READ_LATENCY) (
	input clk, rst, start,
	// as usual, read_end is one byte after the last byte
	input [clog2(RAM_SIZE)-1:0] read_start, read_end,
	input readclk,
	input ram_outclk, input [BYTE_LEN-1:0] ram_out,
	output reg ram_readclk = 0,
	output reg [clog2(RAM_SIZE)-1:0] ram_raddr,
	output outclk, output [BYTE_LEN-1:0] out, output done);

`include "params.vh"

assign outclk = ram_outclk;
assign out = ram_out;

// save read_end so that it can be changed after start
reg [clog2(RAM_SIZE)-1:0] read_end_buf;

// disambiguate reading first and last word
reg first_word = 0;

wire idle;
assign idle = !first_word && ram_raddr == read_end_buf;

wire prev_readclk;
delay readclk_delay(
	.clk(clk), .rst(rst), .in(readclk), .out(prev_readclk));

// delay done so it appears when data comes out of ram
wire done_pd;
assign done_pd = outclk && (ram_raddr + 1 == read_end_buf);
delay #(.DELAY_LEN(RAM_READ_LATENCY)) done_delay(
	.clk(clk), .rst(rst), .in(done_pd), .out(done));

always @(posedge clk) begin
	if (rst) begin
		ram_readclk <= 0;
		// stop stream even if readclk is asserted
		// (i.e. make idle = 1)
		ram_raddr <= 0;
		read_end_buf <= 0;
		first_word <= 0;
	end else begin
		if (start) begin
			ram_raddr <= read_start;
			read_end_buf <= read_end;
			first_word <= 1;
		end else if (!idle && prev_readclk) begin
			// only advance read address on next clock cycle
			// so that ram reads from current address
			ram_raddr <= ram_raddr + 1;
			first_word <= 0;
		end
		if ((start || !idle) && readclk)
			ram_readclk <= 1;
		else
			ram_readclk <= 0;
	end
end

endmodule

// create a memory write stream
// for testing purposes
module stream_to_memory #(
	parameter RAM_SIZE = PACKET_BUFFER_SIZE,
	parameter WORD_LEN = BYTE_LEN) (
	input clk, rst,
	// used to set the offset for a new write stream
	input set_offset_req,
	input [clog2(RAM_SIZE)-1:0] set_offset_val,
	input inclk, input [WORD_LEN-1:0] in,
	output reg ram_we = 0,
	output reg [clog2(RAM_SIZE)-1:0] ram_waddr,
	output reg [WORD_LEN-1:0] ram_win);

`include "params.vh"

reg [clog2(RAM_SIZE)-1:0] curr_addr = 0;
always @(posedge clk) begin
	if (rst) begin
		ram_we <= 0;
		curr_addr <= 0;
	end else begin
		if (set_offset_req)
			curr_addr <= set_offset_val;
		else if (inclk)
			curr_addr <= curr_addr + 1;

		if (inclk) begin
			ram_we <= 1;
			ram_waddr <= curr_addr;
			ram_win <= in;
		end else
			ram_we <= 0;
	end
end

endmodule

// coordinate two stream modules so that upstream data is requested
// only after downstream has received the previous word
module stream_coord(
	input clk, rst,
	input downstream_rdy, downstream_inclk,
	output readclk);

reg waiting = 0;
assign readclk = !rst && (waiting ? 0 : downstream_rdy);

always @(posedge clk) begin
	if (rst)
		waiting <= 0;
	else if (downstream_inclk)
		waiting <= 0;
	else if (readclk)
		waiting <= 1;
end

endmodule

// buffered version of stream_coord, ensures that a word is passed
// out immediately when downstream is ready
module stream_coord_buf #(
	parameter DATA_WIDTH = 1) (
	input clk, rst,
	input inclk, input [DATA_WIDTH-1:0] in,
	input in_done,
	input downstream_rdy,
	output outclk, output [DATA_WIDTH-1:0] out,
	output done,
	output readclk);

wire swb_empty;
stream_coord sc_inst(
	.clk(clk), .rst(rst),
	// if downstream is ready, the buffer will be cleared,
	// so the buffer is ready
	.downstream_rdy(swb_empty || downstream_rdy),
	.downstream_inclk(inclk),
	.readclk(readclk));
single_word_buffer #(.DATA_WIDTH(DATA_WIDTH+1)) swb_inst(
	.clk(clk), .rst(rst), .clear(downstream_rdy),
	.inclk(inclk), .in({in, in_done}),
	.empty(swb_empty), .out({out, done}));
assign outclk = !rst && !swb_empty && downstream_rdy;

endmodule

// coordinated, buffered version of bytes_to_dibits
module bytes_to_dibits_coord_buf(
	input clk, rst, inclk,
	input [BYTE_LEN-1:0] in,
	input in_done, downstream_rdy,
	output readclk, outclk,
	output [1:0] out,
	output done);

`include "params.vh"

wire btd_rdy, btd_inclk, btd_in_done;
wire [BYTE_LEN-1:0] btd_in;
stream_coord_buf #(.DATA_WIDTH(BYTE_LEN)) btd_scb_inst(
	.clk(clk), .rst(rst),
	.inclk(inclk), .in(in),
	.in_done(in_done), .downstream_rdy(btd_rdy && downstream_rdy),
	.outclk(btd_inclk), .out(btd_in), .done(btd_in_done),
	.readclk(readclk));
bytes_to_dibits btd_inst(
	.clk(clk), .rst(rst),
	.inclk(btd_inclk), .in(btd_in), .in_done(btd_in_done),
	.outclk(outclk), .out(out),
	.rdy(btd_rdy), .done(done));

endmodule
