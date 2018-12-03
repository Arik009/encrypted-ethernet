// generates a new random number every 0.01s, not very good quality
module rng (
	input clk, rst,
	input acl_miso,
	output acl_mosi, acl_csn);

localparam COORD_WIDTH = 16;

wire adxl362_outclk;
wire [COORD_WIDTH-1:0] adxl362_x, adxl362_y;
adxl362_reader adxl362_reader_inst(
	.clk(clk), .rst(rst), .miso(acl_miso), .mosi(acl_mosi),
	.csn(acl_csn), .outclk(adxl362_outclk), .x(adxl362_x), .y(adxl362_y));

endmodule

module adxl362_reader #(
	// number of clock cycles between samples
	// this is different from the one defined in imu_reader
	// here sample is defined as reading a single frame
	// 100Hz = 650000 cycles per sample
	parameter SAMPLE_PERIOD = 650000,
	parameter COORD_WIDTH = 16) (
	input clk,
	input rst,
	input miso,
	output mosi,
	output reg csn = 0,
	output reg outclk = 0,
	output signed [COORD_WIDTH-1:0] x, y);

// according to the adxl362 spec
parameter INSTRUCTION_READ = 8'h0B;
parameter DATA_REGISTERS_START = 8'h0E;

wire sample_clk;
clock_divider #(.PULSE_PERIOD(SAMPLE_PERIOD)) clock_div_inst (
	.clk(clk), .rst(rst), .en(1'b1), .out(sample_clk));

// order of operation: send 16 bits of instruction, read 32 bits of data
parameter STATE_IDLE = 2'b00;
parameter STATE_SENDING_INSTRUCTION = 2'b01;
parameter STATE_RECEIVING = 2'b11;

parameter INSTRUCTION_WIDTH = 16;
parameter FRAME_LENGTH_LOG2 = 5;
parameter FRAME_LENGTH = 32;

reg [1:0] state = STATE_IDLE;

// used for both the sending and receiving stages
reg [FRAME_LENGTH_LOG2-1:0] count;
// holds the instruction as it is shifted into mosi
reg [INSTRUCTION_WIDTH-1:0] instruction_buffer;
// holds the frame as it is being shifted in
reg [FRAME_LENGTH-1:0] partial_frame;
// always holds a valid frame
reg [FRAME_LENGTH-1:0] frame;

// convert little endian to big endian
assign x = {frame[16+:4], frame[24+:8], 4'h0};
assign y = {frame[0+:4], frame[8+:8], 4'h0};

assign mosi = instruction_buffer[INSTRUCTION_WIDTH-1];

always @(posedge clk) begin
	if (rst) begin
		state <= STATE_IDLE;
		outclk <= 0;
		csn <= 0;
	else case (state)
	STATE_IDLE: begin
		if (sample_clk) begin
			instruction_buffer <= {INSTRUCTION_READ, DATA_REGISTERS_START};
			count <= 0;
			state <= STATE_SENDING_INSTRUCTION;
			csn <= 0;
			outclk <= 0;
		end
	end
	STATE_SENDING_INSTRUCTION: begin
		if (count == INSTRUCTION_WIDTH - 1) begin
			state <= STATE_RECEIVING;
			count <= 0;
		end else begin
			count <= count + 1;
			instruction_buffer <=
				{instruction_buffer[INSTRUCTION_WIDTH-2:0], 1'b0};
		end
	end
	STATE_RECEIVING: begin
		if (count == FRAME_LENGTH-1) begin
			state <= STATE_IDLE;
			csn <= 1;
			frame <= {partial_frame[FRAME_LENGTH-2:0], miso};
			outclk <= 1;
		end else begin
			count <= count + 1;
			partial_frame <= {partial_frame[FRAME_LENGTH-2:0], miso};
		end
	end
	endcase
end

endmodule
