module crc32(
	input clk, reset,
	input inclk, input [1:0] in,
	output [31:0] out);

localparam CRC_INIT = 32'hffffffff;
// polynomial is reflected since we operate in
// least-significant-bit first for simplicity
localparam CRC_POLY = 32'hedb88320;

reg [31:0] curr;
// most significant byte first
assign out = ~{curr[0+:8], curr[8+:8], curr[16+:8], curr[24+:8]};

// optimized dibit CRC step
// step1: XOR in both inputs at once
// step2: first division by poly
// step3: second division by poly
wire [31:0] step1, step2, step3;
assign step1 = {curr[2+:30], curr[0+:2] ^ in};
assign step2 = step1[1+:31] ^ (step1[0] ? CRC_POLY : 0);
assign step3 = step2[1+:31] ^ (step2[0] ? CRC_POLY : 0);

always @(posedge clk) begin
	if (reset)
		curr <= CRC_INIT;
	else if (inclk)
		curr <= step3;
end

endmodule

