module reset_stream_fifo(
	input clka, clkb,
	input reseta, output resetb);

wire fifo_empty, fifo_out;
wire fifo_rden, fifo_prev_rden;
bit_stream_fifo reset_fifo(
	.rst(0),
	.wr_clk(clka), .rd_clk(clkb),
	.din(reseta), .wr_en(reseta),
	.rd_en(fifo_rden), .dout(fifo_out),
	.empty(fifo_empty));
assign fifo_rden = !fifo_empty;
delay reset_fifo_read_delay(
	.clk(clkb), .in(fifo_rden), .out(fifo_prev_rden));
assign resetb = fifo_prev_rden && fifo_out;

endmodule
