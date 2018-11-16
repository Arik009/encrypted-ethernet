module rmii_driver(
	input clk, reset,
	inout crsdv_in,
	inout [1:0] rxd_in,
	output rxerr, intn,
	output reg rstn = 1,
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
wire reset_done;
assign reset_done = reset_cnt == RESET_SEQUENCE_LEN;

// 100Base-TX Full Duplex, auto-negotiation disabled,
// CRS active during receive
localparam DEFAULT_MODE = 3'b011;
// PHY address, leave at zero
localparam DEFAULT_PHYAD = 0;
// REF_CLK in mode
localparam DEFAULT_NINTSEL = 1;
assign crsdv_in = reset_done ? 1'bz : DEFAULT_MODE[2];
assign rxd_in = reset_done ? 2'bzz : DEFAULT_MODE[1:0];
assign rxerr = reset_done ? 1'bz : DEFAULT_PHYAD;
assign intn = reset_done ? 1'bz : DEFAULT_NINTSEL;

wire crsdv;
wire [1:0] rxd;

// assertion of CRS_DV is async wrt the REF_CLK, so synchronization needed
delay #(.DELAY_LEN(SYNC_DELAY_LEN-1)) crsdv_sync(
	.clk(clk), .reset(reset), .in(crsdv_in), .out(crsdv));
// delay rxd to be in time with crsdv
delay #(.DELAY_LEN(SYNC_DELAY_LEN-1), .DATA_WIDTH(2)) rxd_sync(
	.clk(clk), .reset(reset), .in(rxd_in), .out(rxd));

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

// distinguish crs and dv signals
// only to be used when state == STATE_RECEIVING
wire crsdv_toggling, crs, dv;
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
		if (crsdv)
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
