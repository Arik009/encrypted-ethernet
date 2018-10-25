module delay #(
	// number of delay cycles
	parameter DELAY_LEN = 1) (
	input clk, in,
	output out);

reg [DELAY_LEN-1:0] queue;
assign out = queue[0];

always @ (posedge clk)
begin
	if (DELAY_LEN == 1)
		queue <= in;
	else
		queue <= {in, queue[DELAY_LEN-1:1]};
end

endmodule

module synchronize #(
	// number of sync flops
	parameter NSYNC = 3) (
	input clk, in, output out);

delay #(.DELAY_LEN(NSYNC)) delay_inst (.clk(clk), .in(in), .out(out));

endmodule

module debounce (
	input reset, clk, noisy,
	output reg clean);

reg [19:0] count;
reg new;

always @(posedge clk) begin
	if (reset) begin
		new <= noisy;
		clean <= noisy;
		count <= 0;
	end else if (noisy != new) begin
		new <= noisy; count <= 0;
	end else if (count == 650000)
		clean <= new;
	else
		count <= count+1;
end

endmodule

module sync_debounce (
	input reset, clk, in,
	output out);

synchronize sync_inst(.clk(clk), .in(in), .out(synced));
debounce debounce_inst(
	.reset(reset), .clk(clk), .noisy(synced), .clean(out));

endmodule

module blinker #(
	parameter BLINK_PERIOD_LOG2 = 26,
	parameter BLINK_PERIOD = 50000000) (
	input clk, reset, enable,
	output reg out = 0);

reg [BLINK_PERIOD_LOG2-1:0] cnt = 0;

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
	parameter EXTEND_LEN_LOG2 = 23,
	parameter EXTEND_LEN = 5000000) (
	input clk, reset, in, output reg out);

reg [EXTEND_LEN_LOG2-1:0] cnt = 0;

always @(posedge clk) begin
	if (reset)
		out <= 0;
	else if (in) begin
		out <= 1;
		cnt <= 0;
	end else if (cnt == EXTEND_LEN)
		out <= 0;
	else
		cnt <= cnt + 1;
end

endmodule
