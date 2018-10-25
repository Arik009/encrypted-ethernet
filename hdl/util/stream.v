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
reg [BYTE_LEN_LOG2-2:0] cnt;

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
		// assumes BYTE_LEN is a power of 2 so it wraps around
		cnt <= cnt + 1;
	end else
		outclk <= 0;
end

endmodule
