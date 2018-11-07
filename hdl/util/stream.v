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

delay #(.DELAY_LEN(1)) done_delay(.clk(clk), .in(done_in), .out(done_out));

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
