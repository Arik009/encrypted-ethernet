module uart_tx_driver #(
	// number of cycles per bit
	// 434 cycles at 50MHz = 115200 baud
	parameter BAUD_PERIOD = 434) (
	input clk, reset, data_ready, [BYTE_LEN-1:0] data,
	// done is pulsed when the current input data is consumed
	output reg txd = 1, reg done = 0);

`include "params.vh"

reg [clog2(BAUD_PERIOD):0] cnt = 0;

// including start and stop bits
localparam TX_LEN = BYTE_LEN + 2;
// will never hold start bit
reg [TX_LEN-2:0] data_shifted;

// amount of data shifted out so far, not including current one
reg [clog2(TX_LEN)-1:0] data_cnt = TX_LEN - 1;

always @(posedge clk) begin
	if (reset) begin
		cnt <= 0;
		txd <= 1;
		done <= 0;
		// reset to idle
		data_cnt <= TX_LEN - 1;
	end else if (cnt == BAUD_PERIOD-1) begin
		cnt <= 0;
		if (data_cnt != TX_LEN - 1) begin
			{data_shifted, txd} <= {1'b1, data_shifted};
			data_cnt <= data_cnt + 1;
		end else if (data_ready) begin
			{data_shifted, txd} <= {1'b1, data, 1'b0};
			data_cnt <= 0;
			done <= 1;
		end
	end else begin
		done <= 0;
		cnt <= cnt + 1;
	end
end

endmodule

// fast receiving, designed for 12MBaud
// 120/50MHz clock boundary
module uart_rx_fast_driver #(
	// 120MBaud * 10 = 120MHz
	parameter CYCLES_PER_BIT = 10) (
	input clk, clk_120mhz, reset,
	input rxd,
	output [7:0] out,
	output out_ready);

`include "params.vh"

// count the number of consecutive zeroes to detect start bit
reg [clog2(CYCLES_PER_BIT/2)-1:0] start_bit_cnt = 0;

reg [BYTE_LEN-1:0] curr_byte_shifted;

reg [clog2(CYCLES_PER_BIT)-1:0] cycle_in_bit_cnt = 0;
// allow bit_in_byte_cnt to go to BYTE_LEN + 2 to detect start/stop bit
reg [clog2(BYTE_LEN+2)-1:0] bit_in_byte_cnt = 0;

reg out_ready_120mhz = 0;
reg [BYTE_LEN-1:0] out_120mhz;

wire fifo_empty, fifo_rden;
assign fifo_rden = !fifo_empty;
small_sync_fifo data_fifo(
	.rst(reset),
	.wr_clk(clk_120mhz), .rd_clk(clk),
	.din(out_120mhz), .wr_en(out_ready_120mhz),
	.rd_en(fifo_rden), .dout(out),
	.empty(fifo_empty));
delay fifo_read_delay(
	.clk(clk), .in(fifo_rden), .out(out_ready));

// buffer signals to meet timing constraints
reg reset_buf = 0;
always @(posedge clk_120mhz) begin
	reset_buf <= reset;
	if (reset_buf) begin
		bit_in_byte_cnt <= 0;
		start_bit_cnt <= 0;
		out_ready_120mhz <= 0;
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
			out_ready_120mhz <= 0;
		end else begin
			if (cycle_in_bit_cnt == CYCLES_PER_BIT-1) begin
				if (bit_in_byte_cnt == BYTE_LEN + 1) begin
					// rxd should be high at this point
					// but we can't do anything about it otherwise
					out_120mhz <= curr_byte_shifted;
					out_ready_120mhz <= 1;
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
	input in_ready, input [7:0] in,
	output txd, output ready);

wire driv_ready;
uart_tx_fast_driver uart_driv_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .reset(reset),
	.in_ready(in_ready), .in(in), .txd(txd), .ready(driv_ready));

// Wait for data to arrive before checking ready
reg waiting_for_data = 0;
assign ready = waiting_for_data ? 0 : driv_ready;

always @(posedge clk) begin
	if (reset || start)
		waiting_for_data <= 0;
	else if (in_ready)
		waiting_for_data <= 0;
	else if (ready)
		waiting_for_data <= 1;
end

endmodule

// fast transmitting, designed for 12MBaud
// 120/50MHz clock boundary
module uart_tx_fast_driver #(
	// 120MBaud * 10 = 120MHz
	parameter CYCLES_PER_BIT = 10) (
	input clk, clk_120mhz, reset,
	input in_ready,
	input [7:0] in,
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
wire in_ready_120mhz;

wire fifo_empty, fifo_full, fifo_rden;
assign fifo_rden = !fifo_empty && tx_clk && bits_left_cnt == 0;
small_sync_fifo data_fifo(
	.rst(reset),
	.wr_clk(clk), .rd_clk(clk_120mhz),
	.din(in), .wr_en(in_ready),
	.rd_en(fifo_rden), .dout(in_120mhz),
	.full(fifo_full), .empty(fifo_empty));
delay fifo_read_delay(
	.clk(clk_120mhz), .in(fifo_rden), .out(in_ready_120mhz));
// delay tx_clk by one cycle to account for fifo read time
wire tx_clk_delayed;
delay tx_clk_delay(
	.clk(clk_120mhz), .in(tx_clk), .out(tx_clk_delayed));

assign ready = !fifo_full;

reg reset_buf = 0;
always @(posedge clk_120mhz) begin
	reset_buf <= reset;
	if (reset_buf) begin
		bits_left_cnt <= 0;
		curr_byte_shifted <= ~0;
	end else if (tx_clk_delayed) begin
		if (bits_left_cnt == 0 && in_ready_120mhz) begin
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
