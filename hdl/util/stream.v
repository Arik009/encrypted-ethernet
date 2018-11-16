// convert a dibit stream to a bytestream
module dibits_to_bytes(
	// inclk is pulsed when a dibit is presented on in
	// outclk is pulsed when a byte is presented on out
	input clk, reset, inclk,
	input [1:0] in,
	input done_in,
	output reg [BYTE_LEN-1:0] out = 0,
	output reg outclk = 0,
	output done_out);

`include "params.vh"

delay #(.DELAY_LEN(1)) done_delay(
	.clk(clk), .reset(reset), .in(done_in), .out(done_out));

// scratch space to shift dibits in
reg [BYTE_LEN-3:0] shifted;
// only need half as much since we get 2 bits at a time
reg [clog2(BYTE_LEN)-2:0] cnt = 0;

always @(posedge clk) begin
	if (reset) begin
		cnt <= 0;
		outclk <= 0;
		out <= 0;
	end else if (inclk) begin
		if (cnt == BYTE_LEN/2 - 1) begin
			out <= {in, shifted};
			outclk <= 1;
		end else
			outclk <= 0;
		shifted <= {in, shifted[2+:BYTE_LEN-4]};
		if (done_in)
			cnt <= 0;
		else
			// assumes BYTE_LEN is a power of 2 so it wraps around
			cnt <= cnt + 1;
	end else begin
		if (done_in)
			cnt <= 0;
		outclk <= 0;
	end
end

endmodule

// convert a bytestream to a dibit stream
// assumes that bytes are inserted no faster than once every 4 clock cycles
module bytes_to_dibits(
	// inclk is pulsed when a byte is presented on in
	// outclk is pulsed when a dibit is presented on out
	input clk, reset, inclk,
	input [BYTE_LEN-1:0] in,
	input done_in,
	output [1:0] out,
	output reg outclk = 0,
	// idle is asserted when no byte is in queue
	output idle,
	// done_out is pulsed after done_in when buffer has been cleared
	output done_out);

`include "params.vh"

// scratch space to shift dibits out
reg [BYTE_LEN-1:0] shifted;
assign out = shifted[1:0];
// only need half as much since we output 2 bits at a time
reg [clog2(BYTE_LEN)-2:0] cnt = BYTE_LEN/2-1;
assign idle = cnt == BYTE_LEN/2-1;

// done_in asserted, should assert done_out when buffer clears
reg done_in_found = 0;
assign done_out = done_in_found && idle;

always @(posedge clk) begin
	if (reset) begin
		cnt <= BYTE_LEN/2-1;
		outclk <= 0;
		done_in_found <= 0;
	end else if (inclk) begin
		if (done_in)
			done_in_found <= 1;
		shifted <= in;
		cnt <= 0;
		outclk <= 1;
	end else if (!idle) begin
		if (done_in)
			done_in_found <= 1;
		outclk <= 1;
		shifted <= {2'b00, shifted[2+:BYTE_LEN-2]};
		cnt <= cnt + 1;
	end else begin
		if (done_in)
			done_in_found <= 1;
		else if (done_in_found)
			done_in_found <= 0;
		outclk <= 0;
	end
end

endmodule

module bytes_to_colors(
	input clk, reset,
	input inclk, input [BYTE_LEN-1:0] in,
	output reg outclk, output reg [COLOR_LEN-1:0] out);

`include "params.vh"

// three states to convert three bytes into two colors
reg [1:0] state = 0;
reg [BYTE_LEN-1:0] prev_in;

always @(posedge clk) begin
	prev_in <= in;
	if (reset)
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
	parameter RAM_SIZE = PACKET_BUFFER_SIZE) (
	input clk, rst, start,
	// as usual, read_end is one byte after the last byte
	input [clog2(RAM_SIZE)-1:0] read_start, read_end,
	// ready is asserted when the next read should be initiated
	input downstream_rdy,
	input ram_outclk, input [BYTE_LEN-1:0] ram_out,
	output reg ram_readclk = 0,
	output reg [clog2(RAM_SIZE)-1:0] ram_raddr,
	output outclk, output [BYTE_LEN-1:0] out);

`include "params.vh"

assign outclk = ram_outclk;
assign out = ram_out;

// save read_end so that it can be changed after start
reg [clog2(RAM_SIZE)-1:0] read_end_buf;

// disambiguate reading first and last word
reg first_word = 0;

wire idle;
assign idle = !first_word && ram_raddr == read_end_buf;

reg prev_downstream_rdy = 0;

always @(posedge clk) begin
	if (rst) begin
		ram_readclk <= 0;
		// stop stream even if downstream_rdy is asserted
		// (i.e. make idle = 1)
		ram_raddr <= 0;
		read_end_buf <= 0;
		prev_downstream_rdy <= 0;
		first_word <= 0;
	end else begin
		prev_downstream_rdy <= downstream_rdy;
		if (start) begin
			ram_raddr <= read_start;
			read_end_buf <= read_end;
			first_word <= 1;
		end else if (!idle && prev_downstream_rdy) begin
			// only advance read address on next clock cycle
			// so that ram reads from current address
			ram_raddr <= ram_raddr + 1;
			first_word <= 0;
		end
		if ((start || !idle) && downstream_rdy)
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
assign readclk = waiting ? 0 : downstream_rdy;

always @(posedge clk) begin
	if (rst)
		waiting <= 0;
	else if (downstream_inclk)
		waiting <= 0;
	else if (readclk)
		waiting <= 1;
end

endmodule
