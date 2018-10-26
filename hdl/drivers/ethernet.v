module ethernet_driver(
	input clk, reset,
	inout crsdv, rxerr,
	inout [1:0] rxd,
	output intn, reg rstn = 1,
	// done is pulsed once a full packet has been received
	// this will never be asserted at the same time as outclk
	output reg [1:0] out = 0,
	output reg outclk = 0, done = 0);

`include "params.vh"

// should have 25ms delay from power supply up before
// nRST assertion, but we assume that power supplies have
// long been set up already
// according to spec: need 200ns setup before nRST deassertion
// and 100ns hold time afterwards
localparam RESET_SETUP = 10;
localparam RESET_HOLD = 5;
localparam RESET_SEQUENCE_LEN = RESET_SETUP + RESET_HOLD;
reg [clog2(RESET_SEQUENCE_LEN)-1:0] reset_cnt = 0;
assign reset_done = reset_cnt == RESET_SEQUENCE_LEN;

// 100Base-TX Full Duplex, auto-negotiation disabled,
// CRS active during receive
localparam DEFAULT_MODE = 3'b011;
// PHY address, leave at zero
localparam DEFAULT_PHYAD = 0;
// REF_CLK in mode
localparam DEFAULT_NINTSEL = 1;
assign crsdv = reset_done ? 1'bz : DEFAULT_MODE[2];
assign rxd = reset_done ? 2'bzz : DEFAULT_MODE[1:0];
assign rxerr = reset_done ? 1'bz : DEFAULT_PHYAD;
assign intn = reset_done ? 1'bz : DEFAULT_NINTSEL;

localparam STATE_IDLE = 2'b00;
localparam STATE_WAITING = 2'b01;
localparam STATE_PREAMBLE = 2'b11;
localparam STATE_RECEIVING = 2'b10;
reg [1:0] state = STATE_IDLE;

reg prev_crsdv;
always @(posedge clk) begin
	if (reset_done)
		prev_crsdv <= crsdv;
end

// assertion of CRS_DV is async wrt the REF_CLK, so synchronization needed
// note that the delay causes CRS to appear asserted just when it finishes
// receiving a packet, but this is okay because it will immediately
// revert to the idle state
wire crsdv_init;
synchronize #(.NSYNC(2)) crsdv_init_synchronizer(
	.clk(clk), .in(prev_crsdv), .out(crsdv_init));

// distinguish crs and dv signals
// only to be used when state == STATE_RECEIVING
assign crsdv_toggling = prev_crsdv != crsdv;
assign crs = crsdv_toggling ? 0 : crsdv;
assign dv = crsdv_toggling ? 1 : crsdv;

always @(posedge clk) begin
	if (reset) begin
		reset_cnt <= 0;
		rstn <= 0;
		state <= STATE_IDLE;
		out <= 0;
		outclk <= 0;
		done <= 0;
	end else if (~reset_done) begin
		if (reset_cnt == RESET_SETUP - 1)
			rstn <= 1;
		reset_cnt <= reset_cnt + 1;
	end else case(state)
	STATE_IDLE: begin
		done <= 0;
		if (crsdv_init)
			state <= STATE_WAITING;
	end
	STATE_WAITING:
		if (!crsdv)
			state <= STATE_IDLE;
		else if (rxd == 2'b01) begin
			state <= STATE_PREAMBLE;
		end
	STATE_PREAMBLE:
		if (!crsdv)
			state <= STATE_IDLE;
		else if (rxd == 2'b11) begin
			state <= STATE_RECEIVING;
		end
	STATE_RECEIVING: begin
		if (!dv) begin
			state <= STATE_IDLE;
			outclk <= 0;
			done <= 1;
		end else begin
			outclk <= 1;
			out <= rxd;
		end
	end
	endcase
end

endmodule
