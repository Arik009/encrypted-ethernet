module rmii_driver(
	input clk, rst,
	inout crsdv_in,
	inout [1:0] rxd_in,
	output rxerr, intn,
	output reg rstn = 0,
	output reg [1:0] out = 0,
	// done is pulsed at the same time as the last dibit of a full packet
	output reg outclk = 0, output done);

`include "params.vh"

// should have 25ms delay from power supply up before
// nRST assertion, but we assume that power supplies have
// long been set up already
// according to spec: need 100us before nRST deassertion
// and 800ns afterwards
localparam RESET_BEFORE = 5000;
localparam RESET_AFTER = 40;
localparam RESET_SEQUENCE_LEN = RESET_BEFORE + RESET_AFTER;
reg [clog2(RESET_SEQUENCE_LEN)-1:0] rst_cnt = 0;
wire rst_done;
assign rst_done = rst_cnt == RESET_SEQUENCE_LEN;

// 100Base-TX Full Duplex, auto-negotiation disabled,
// CRS active during receive
localparam DEFAULT_MODE = 3'b011;
// PHY address, leave at zero
localparam DEFAULT_PHYAD = 0;
// REF_CLK in mode
localparam DEFAULT_NINTSEL = 1;
assign crsdv_in = rst_done ? 1'bz : DEFAULT_MODE[2];
assign rxd_in = rst_done ? 2'bzz : DEFAULT_MODE[1:0];
assign rxerr = rst_done ? 1'bz : DEFAULT_PHYAD;
assign intn = rst_done ? 1'bz : DEFAULT_NINTSEL;

wire crsdv;
wire [1:0] rxd;

// assertion of CRS_DV is async wrt the REF_CLK, so synchronization needed
delay #(.DELAY_LEN(SYNC_DELAY_LEN-1)) crsdv_sync(
	.clk(clk), .rst(rst), .in(crsdv_in), .out(crsdv));
// delay rxd to be in time with crsdv
delay #(.DELAY_LEN(SYNC_DELAY_LEN-1), .DATA_WIDTH(2)) rxd_sync(
	.clk(clk), .rst(rst), .in(rxd_in), .out(rxd));

localparam STATE_IDLE = 0;
localparam STATE_WAITING = 1;
localparam STATE_PREAMBLE = 2;
localparam STATE_RECEIVING = 3;
reg [1:0] state = STATE_IDLE;

reg prev_crsdv;
always @(posedge clk) begin
	if (rst_done)
		prev_crsdv <= crsdv;
end

// distinguish crs and dv signals
// only to be used when state == STATE_RECEIVING
wire crsdv_toggling, crs, dv;
assign crsdv_toggling = prev_crsdv != crsdv;
assign crs = crsdv_toggling ? 0 : crsdv;
assign dv = crsdv_toggling ? 1 : crsdv;
assign done = (state == STATE_RECEIVING) && !dv;

always @(posedge clk) begin
	if (rst) begin
		rst_cnt <= 0;
		rstn <= 0;
		state <= STATE_IDLE;
		out <= 0;
		outclk <= 0;
	end else if (~rst_done) begin
		if (rst_cnt == RESET_BEFORE - 1)
			rstn <= 1;
		rst_cnt <= rst_cnt + 1;
	end else case(state)
	STATE_IDLE:
		if (crsdv)
			state <= STATE_WAITING;
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
	STATE_RECEIVING:
		if (!dv) begin
			state <= STATE_IDLE;
			outclk <= 0;
		end else begin
			outclk <= 1;
			out <= rxd;
		end
	endcase
end

endmodule
