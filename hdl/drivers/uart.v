module uart_driver #(
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
