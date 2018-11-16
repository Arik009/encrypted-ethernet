module delay #(
	// number of delay cycles
	parameter DELAY_LEN = 1,
	parameter DATA_WIDTH = 1) (
	input clk, [DATA_WIDTH-1:0] in,
	output [DATA_WIDTH-1:0] out);

reg [DELAY_LEN*DATA_WIDTH-1:0] queue;
assign out = queue[0+:DATA_WIDTH];

always @ (posedge clk)
begin
	if (DELAY_LEN == 1)
		queue <= in;
	else
		queue <= {in, queue[DATA_WIDTH+:(DELAY_LEN-1)*DATA_WIDTH]};
end

endmodule

module debounce (
	input reset, clk, noisy,
	output reg clean);

reg [19:0] count;
reg prev;

always @(posedge clk) begin
	if (reset) begin
		prev <= noisy;
		clean <= noisy;
		count <= 0;
	end else if (noisy != prev) begin
		prev <= noisy; count <= 0;
	end else if (count == 650000)
		clean <= prev;
	else
		count <= count+1;
end

endmodule

module sync_debounce (
	input reset, clk, in,
	output out);

`include "params.vh"

wire synced;
delay #(.DELAY_LEN(SYNC_DELAY_LEN)) delay_inst(
	.clk(clk), .in(in), .out(synced));
debounce debounce_inst(
	.reset(reset), .clk(clk), .noisy(synced), .clean(out));

endmodule

module blinker #(
	parameter BLINK_PERIOD = 50000000) (
	input clk, reset, enable,
	output reg out = 0);

`include "util.vh"

reg [clog2(BLINK_PERIOD)-1:0] cnt = 0;

always @(posedge clk) begin
	if (reset || !enable) begin
		cnt <= 0;
		out <= 0;
	end else if (cnt == BLINK_PERIOD-1) begin
		cnt <= 0;
		out <= ~out;
	end else
		cnt <= cnt + 1;
end

endmodule

module pulse_extender #(
	// time to extend pulse by, default 0.1s
	parameter EXTEND_LEN = 5000000) (
	input clk, reset, in, output reg out = 0);

`include "util.vh"

reg [clog2(EXTEND_LEN)-1:0] cnt = 0;

always @(posedge clk) begin
	if (reset) begin
		out <= 0;
		cnt <= 0;
	end else if (in) begin
		out <= 1;
		cnt <= 0;
	end else if (cnt == EXTEND_LEN)
		out <= 0;
	else
		cnt <= cnt + 1;
end

endmodule

module pulse_generator (
	input clk, reset, in, output out);

reg pulsed = 0;
assign out = in && !pulsed;

always @(posedge clk) begin
	if (reset)
		pulsed <= 0;
	else if (in)
		pulsed <= 1;
	else
		pulsed <= 0;
end

endmodule

// pulses for a single clock cycle every PULSE_PERIOD
module clock_divider #(
	parameter PULSE_PERIOD = 4) (
	// only pulses if en is asserted
	input clk, start, en, output out);

`include "util.vh"

reg [clog2(PULSE_PERIOD)-1:0] cnt = 0;
assign out = en && cnt == 0;

always @(posedge clk) begin
	if (start)
		cnt <= 0;
	else if (cnt == PULSE_PERIOD-1)
		cnt <= 0;
	else
		cnt <= cnt + 1;
end

endmodule
