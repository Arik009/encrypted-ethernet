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
	output reg [7:0] out,
	output reg out_ready = 0);

`include "params.vh"

// count the number of consecutive zeroes to detect start bit
reg [clog2(CYCLES_PER_BIT/2)-1:0] start_bit_cnt = 0;

reg [BYTE_LEN-1:0] curr_byte_shifted;

reg [clog2(CYCLES_PER_BIT)-1:0] cycle_in_bit_cnt = 0;
// allow bit_in_byte_cnt to go to BYTE_LEN + 2 to detect start/stop bit
reg [clog2(BYTE_LEN+2)-1:0] bit_in_byte_cnt = 0;

reg [BYTE_LEN-1:0] curr_byte_buffer_120mhz;
// new_byte_indicator_120mhz is flipped each time a new byte appears
// on the 50mhz side, use new_byte_indicator to keep track of
// where we are
reg new_byte_indicator_120mhz = 0;
reg new_byte_indicator = 0;

always @(posedge clk) begin
	if (reset) begin
		out_ready <= 0;
		new_byte_indicator <= 0;
	end else if (new_byte_indicator_120mhz != new_byte_indicator) begin
		new_byte_indicator <= ~new_byte_indicator;
		out <= curr_byte_buffer_120mhz;
		out_ready <= 1;
	end else
		out_ready <= 0;
end

always @(posedge clk_120mhz) begin
	if (reset) begin
		bit_in_byte_cnt <= 0;
		start_bit_cnt <= 0;
		new_byte_indicator_120mhz <= 0;
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
		end else begin
			if (cycle_in_bit_cnt == CYCLES_PER_BIT-1) begin
				if (bit_in_byte_cnt == BYTE_LEN + 1) begin
					// rxd should be high at this point
					// but we can't do anything about it otherwise
					curr_byte_buffer_120mhz <= curr_byte_shifted;
					new_byte_indicator_120mhz <= ~new_byte_indicator_120mhz;
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
	output txd,
	output ready);

wire driv_ready;
uart_tx_fast_driver uart_driv_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .reset(reset),
	.in_ready(in_ready), .in(in), .txd(txd), .ready(driv_ready));

// "start" is treated like an abort, so everything is reset.
wire driv_ready_pulse;
pulse_generator driv_ready_pg(
	.clk(clk), .reset(reset || start), .in(driv_ready),
	.out(driv_ready_pulse));

// Initially, the driver should be ready and driv_ready should
// be asserted, and waiting_for_data will be deasserted.
// The pulse generator emits a pulse on the falling edge of reset
// since driv_ready would be asserted.
// Then we wait for data to arrive. When data arrives with in_ready,
// we deassert waiting_for_data. We will send out the next ready pulse on
// the rising edge of driv_ready.
reg waiting_for_data = 0;
assign ready = waiting_for_data ? 0 : driv_ready_pulse;

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
	output reg ready = 1);

`include "params.vh"

wire tx_clk;
clock_divider #(.PULSE_PERIOD(CYCLES_PER_BIT)) tx_clock_divider(
	.clk(clk_120mhz), .start(1'b0), .en(1'b1), .out(tx_clk));

// two extra bits for start/stop bits
reg [BYTE_LEN+2-1:0] curr_byte_shifted = ~0;
assign txd = curr_byte_shifted[0];

// allow bits_left_cnt to go to BYTE_LEN + 2 to send start/stop bits
reg [clog2(BYTE_LEN+2)-1:0] bits_left_cnt = 0;

reg [BYTE_LEN-1:0] new_byte_buffer;
// new_byte_indicator is flipped when a new byte is to be sent
// when the 120mhz side buffers it, it flips new_byte_indicator_120mhz
// to indicate that it is ready for another byte
reg new_byte_indicator_120mhz = 0;
reg new_byte_indicator = 0;
// when 120mhz side has caught up, it is ready for the next byte
// buffer it to keep boundary path short
wire ready_120mhz;
assign ready_120mhz = new_byte_indicator_120mhz == new_byte_indicator;

always @(posedge clk) begin
	if (reset) begin
		new_byte_indicator <= 0;
		ready <= 1;
	end else if (in_ready) begin
		new_byte_buffer <= in;
		new_byte_indicator <= ~new_byte_indicator;
		ready <= 0;
	end else
		ready <= ready_120mhz;
end

always @(posedge clk_120mhz) begin
	if (reset) begin
		bits_left_cnt <= 0;
		new_byte_indicator_120mhz <= 0;
		curr_byte_shifted <= ~0;
	end else if (tx_clk) begin
		if (bits_left_cnt == 0 &&
				new_byte_indicator_120mhz != new_byte_indicator) begin
			new_byte_indicator_120mhz <= ~new_byte_indicator_120mhz;
			bits_left_cnt <= BYTE_LEN + 2 - 1;
			curr_byte_shifted <= {1'b1, new_byte_buffer, 1'b0};
		end else begin
			if (bits_left_cnt != 0)
				bits_left_cnt <= bits_left_cnt - 1;
			curr_byte_shifted <= {1'b1, curr_byte_shifted[1+:BYTE_LEN+1]};
		end
	end
end

endmodule
