// fast receiving, designed for 12MBaud
// 120/50MHz clock boundary
module uart_rx_fast_driver #(
	// 120MBaud * 10 = 120MHz
	parameter CYCLES_PER_BIT = 10) (
	input clk, clk_120mhz, reset,
	input rxd,
	output [7:0] out, output outclk);

`include "params.vh"

// count the number of consecutive zeroes to detect start bit
reg [clog2(CYCLES_PER_BIT/2)-1:0] start_bit_cnt = 0;

reg [BYTE_LEN-1:0] curr_byte_shifted;

reg [clog2(CYCLES_PER_BIT)-1:0] cycle_in_bit_cnt = 0;
// allow bit_in_byte_cnt to go to BYTE_LEN + 2 to detect start/stop bit
reg [clog2(BYTE_LEN+2)-1:0] bit_in_byte_cnt = 0;

reg outclk_120mhz = 0;
reg [BYTE_LEN-1:0] out_120mhz;

wire fifo_empty, fifo_rden;
assign fifo_rden = !fifo_empty;
byte_stream_fifo data_fifo(
	.rst(reset),
	.wr_clk(clk_120mhz), .rd_clk(clk),
	.din(out_120mhz), .wr_en(outclk_120mhz),
	.rd_en(fifo_rden), .dout(out),
	.empty(fifo_empty));
delay fifo_read_delay(
	.clk(clk), .reset(reset), .in(fifo_rden), .out(outclk));
wire reset_120mhz;
reset_stream_fifo reset_fifo_inst(
	.clka(clk), .clkb(clk_120mhz),
	.reseta(reset), .resetb(reset_120mhz));

always @(posedge clk_120mhz) begin
	if (reset_120mhz) begin
		bit_in_byte_cnt <= 0;
		start_bit_cnt <= 0;
		outclk_120mhz <= 0;
	end else begin
		if (rxd)
			start_bit_cnt <= 0;
		else if (start_bit_cnt != CYCLES_PER_BIT/2-1)
			start_bit_cnt <= start_bit_cnt + 1;

		if (bit_in_byte_cnt == 0) begin
			// wait for start bit
			if (start_bit_cnt == CYCLES_PER_BIT/2-1) begin
				curr_byte_shifted <= 0;
				bit_in_byte_cnt <= 1;
				cycle_in_bit_cnt <= 0;
			end
			outclk_120mhz <= 0;
		end else begin
			if (cycle_in_bit_cnt == CYCLES_PER_BIT-1) begin
				if (bit_in_byte_cnt == BYTE_LEN + 1) begin
					// rxd should be high at this point
					// but we can't do anything about it otherwise
					out_120mhz <= curr_byte_shifted;
					outclk_120mhz <= 1;
					bit_in_byte_cnt <= 0;
				end else begin
					bit_in_byte_cnt <= bit_in_byte_cnt + 1;
					curr_byte_shifted <=
						{rxd, curr_byte_shifted[1+:BYTE_LEN-1]};
				end
				cycle_in_bit_cnt <= 0;
			end else
				cycle_in_bit_cnt <= cycle_in_bit_cnt + 1;
		end
	end
end

endmodule

// expose a stream interface to request one byte at a time
module uart_tx_fast_stream_driver(
	input clk, clk_120mhz, reset, start,
	input inclk, input [7:0] in,
	output txd, output ready);

wire driv_rdy;
uart_tx_fast_driver uart_driv_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .reset(reset),
	.inclk(inclk), .in(in), .txd(txd), .ready(driv_rdy));
stream_coord sc_inst(
	.clk(clk), .rst(reset || start),
	.downstream_rdy(driv_rdy), .downstream_inclk(inclk),
	.readclk(ready));

endmodule

// fast transmitting, designed for 12MBaud
// 120/50MHz clock boundary
module uart_tx_fast_driver #(
	// 120MBaud * 10 = 120MHz
	parameter CYCLES_PER_BIT = 10) (
	input clk, clk_120mhz, reset,
	input inclk, input [7:0] in,
	output txd,
	// ready is asserted to request for a new byte to transmit
	output ready);

`include "params.vh"

wire tx_clk;
clock_divider #(.PULSE_PERIOD(CYCLES_PER_BIT)) tx_clock_divider(
	.clk(clk_120mhz), .start(1'b0), .en(1'b1), .out(tx_clk));

// two extra bits for start/stop bits
reg [BYTE_LEN+2-1:0] curr_byte_shifted = ~0;
assign txd = curr_byte_shifted[0];

// allow bits_left_cnt to go to BYTE_LEN + 2 to send start/stop bits
reg [clog2(BYTE_LEN+2)-1:0] bits_left_cnt = 0;

wire [BYTE_LEN-1:0] in_120mhz;
wire inclk_120mhz;

wire reset_120mhz;
reset_stream_fifo reset_fifo_inst(
	.clka(clk), .clkb(clk_120mhz),
	.reseta(reset), .resetb(reset_120mhz));
wire fifo_empty, fifo_full, fifo_rden;
assign fifo_rden = !fifo_empty && tx_clk && bits_left_cnt == 0;
byte_stream_fifo data_fifo(
	.rst(reset),
	.wr_clk(clk), .rd_clk(clk_120mhz),
	.din(in), .wr_en(inclk),
	.rd_en(fifo_rden), .dout(in_120mhz),
	.full(fifo_full), .empty(fifo_empty));
delay fifo_read_delay(
	.clk(clk_120mhz), .reset(reset_120mhz),
	.in(fifo_rden), .out(inclk_120mhz));
// delay tx_clk by one cycle to account for fifo read time
wire tx_clk_delayed;
delay tx_clk_delay(
	.clk(clk_120mhz), .reset(reset_120mhz),
	.in(tx_clk), .out(tx_clk_delayed));

assign ready = !fifo_full;

always @(posedge clk_120mhz) begin
	if (reset_120mhz) begin
		bits_left_cnt <= 0;
		curr_byte_shifted <= ~0;
	end else if (tx_clk_delayed) begin
		if (bits_left_cnt == 0 && inclk_120mhz) begin
			bits_left_cnt <= BYTE_LEN + 2 - 1;
			curr_byte_shifted <= {1'b1, in_120mhz, 1'b0};
		end else begin
			if (bits_left_cnt != 0)
				bits_left_cnt <= bits_left_cnt - 1;
			curr_byte_shifted <= {1'b1, curr_byte_shifted[1+:BYTE_LEN+1]};
		end
	end
end

endmodule
