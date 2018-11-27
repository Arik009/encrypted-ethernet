// inclk is asserted when a block is presented on in
// outclk should be asserted when an encrypted block is presented on out
// always takes the same number of clock cycles per block
module aes_encrypt(
	input clk, rst,
	input inclk, input [BLOCK_LEN-1:0] in, key,
	output outclk, output [BLOCK_LEN-1:0] out);

`include "params.vh"

// TODO: not yet implemented, test a simple delay
delay #(.DELAY_LEN(4), .DATA_WIDTH(1+BLOCK_LEN)) delay_inst(
	.clk(clk), .rst(rst), .in({inclk, in}), .out({outclk, out}));

endmodule

// inclk is asserted when a block is presented on in
// outclk should be asserted when an decrypted block is presented on out
// always takes the same number of clock cycles per block (at most 64)
module aes_decrypt(
	input clk, rst,
	input inclk, input [BLOCK_LEN-1:0] in, key,
	output outclk, output [BLOCK_LEN-1:0] out);

`include "params.vh"

// TODO: not yet implemented, test with a simple delay
delay #(.DELAY_LEN(4), .DATA_WIDTH(1+BLOCK_LEN)) delay_inst(
	.clk(clk), .rst(rst), .in({inclk, in}), .out({outclk, out}));

endmodule

// bytes interface for aes_encrypt
module aes_encrypt_bytes(
	input clk, rst,
	input inclk, input [BYTE_LEN-1:0] in, input [BLOCK_LEN-1:0] key,
	output outclk, output [BYTE_LEN-1:0] out);

`include "params.vh"

wire btbl_outclk;
wire [BLOCK_LEN-1:0] btbl_out;
bytes_to_blocks btbl_inst(
	.clk(clk), .rst(rst),
	.inclk(inclk), .in(in),
	.in_done(1'b0),
	.outclk(btbl_outclk), .out(btbl_out));
wire encr_outclk;
wire [BLOCK_LEN-1:0] encr_out;
aes_encrypt encr_inst(
	.clk(clk), .rst(rst),
	.inclk(btbl_outclk), .in(btbl_out), .key(key),
	.outclk(encr_outclk), .out(encr_out));
wire bltb_outclk;
wire [BYTE_LEN-1:0] bltb_out;
blocks_to_bytes bltb_inst(
	.clk(clk), .rst(rst),
	.inclk(encr_outclk), .in(encr_out),
	.in_done(1'b0),
	.outclk(outclk), .out(out));

endmodule

// bytes interface for aes_decrypt
module aes_decrypt_bytes(
	input clk, rst,
	input inclk, input [BYTE_LEN-1:0] in, input [BLOCK_LEN-1:0] key,
	output outclk, output [BYTE_LEN-1:0] out);

`include "params.vh"

wire btbl_outclk;
wire [BLOCK_LEN-1:0] btbl_out;
bytes_to_blocks btbl_inst(
	.clk(clk), .rst(rst),
	.inclk(inclk), .in(in),
	.in_done(1'b0),
	.outclk(btbl_outclk), .out(btbl_out));
wire decr_outclk;
wire [BLOCK_LEN-1:0] decr_out;
aes_decrypt decr_inst(
	.clk(clk), .rst(rst),
	.inclk(btbl_outclk), .in(btbl_out), .key(key),
	.outclk(decr_outclk), .out(decr_out));
wire bltb_outclk;
wire [BYTE_LEN-1:0] bltb_out;
blocks_to_bytes bltb_inst(
	.clk(clk), .rst(rst),
	.inclk(decr_outclk), .in(decr_out),
	.in_done(1'b0),
	.outclk(outclk), .out(out));

endmodule

module aes_modules(wire done);

      reg [127:0] in;
      reg [127:0] key;
      wire [127:0] out;

      assign done=1;

      aes_encrypt_block block(.in(in), .out(out), .key(key));
endmodule

module aes_encrypt_block(input [127:0] in, 
                   input [127:0] key,
                   output [127:0] out);
    wire [127:0] sb_out;
    wire [127:0] sr_out;
    wire [127:0] mc_out;


    // we're currently not generating correct round keys, TODO

    subbytes a(.in(in), .out(sb_out));                   
    shiftrows b(.in(sb_out), .out(sr_out));
    mixcolumns c(.in(sr_out), .out(mc_out));
    addroundkey d(.in(mc_out), .out(out), .key(key));
endmodule

module addroundkey(input [127:0] in,
                   input [127:0] key, 
                   output [127:0] out);
    assign out = in ^ key;
endmodule

module inv_sbox(input [7:0] in,
            output [7:0] out);

            reg [7:0] inv_sbox;
            // TODO: replace this with a clock?
            // or consider muxing the whole thing using an array...
            always @(*) begin
                case (in)
					8'h0 : inv_sbox = 8'h52;
					8'h1 : inv_sbox = 8'h9;
					8'h2 : inv_sbox = 8'h6A;
					8'h3 : inv_sbox = 8'hD5;
					8'h4 : inv_sbox = 8'h30;
					8'h5 : inv_sbox = 8'h36;
					8'h6 : inv_sbox = 8'hA5;
					8'h7 : inv_sbox = 8'h38;
					8'h8 : inv_sbox = 8'hBF;
					8'h9 : inv_sbox = 8'h40;
					8'hA : inv_sbox = 8'hA3;
					8'hB : inv_sbox = 8'h9E;
					8'hC : inv_sbox = 8'h81;
					8'hD : inv_sbox = 8'hF3;
					8'hE : inv_sbox = 8'hD7;
					8'hF : inv_sbox = 8'hFB;
					8'h10 : inv_sbox = 8'h7C;
					8'h11 : inv_sbox = 8'hE3;
					8'h12 : inv_sbox = 8'h39;
					8'h13 : inv_sbox = 8'h82;
					8'h14 : inv_sbox = 8'h9B;
					8'h15 : inv_sbox = 8'h2F;
					8'h16 : inv_sbox = 8'hFF;
					8'h17 : inv_sbox = 8'h87;
					8'h18 : inv_sbox = 8'h34;
					8'h19 : inv_sbox = 8'h8E;
					8'h1A : inv_sbox = 8'h43;
					8'h1B : inv_sbox = 8'h44;
					8'h1C : inv_sbox = 8'hC4;
					8'h1D : inv_sbox = 8'hDE;
					8'h1E : inv_sbox = 8'hE9;
					8'h1F : inv_sbox = 8'hCB;
					8'h20 : inv_sbox = 8'h54;
					8'h21 : inv_sbox = 8'h7B;
					8'h22 : inv_sbox = 8'h94;
					8'h23 : inv_sbox = 8'h32;
					8'h24 : inv_sbox = 8'hA6;
					8'h25 : inv_sbox = 8'hC2;
					8'h26 : inv_sbox = 8'h23;
					8'h27 : inv_sbox = 8'h3D;
					8'h28 : inv_sbox = 8'hEE;
					8'h29 : inv_sbox = 8'h4C;
					8'h2A : inv_sbox = 8'h95;
					8'h2B : inv_sbox = 8'hB;
					8'h2C : inv_sbox = 8'h42;
					8'h2D : inv_sbox = 8'hFA;
					8'h2E : inv_sbox = 8'hC3;
					8'h2F : inv_sbox = 8'h4E;
					8'h30 : inv_sbox = 8'h8;
					8'h31 : inv_sbox = 8'h2E;
					8'h32 : inv_sbox = 8'hA1;
					8'h33 : inv_sbox = 8'h66;
					8'h34 : inv_sbox = 8'h28;
					8'h35 : inv_sbox = 8'hD9;
					8'h36 : inv_sbox = 8'h24;
					8'h37 : inv_sbox = 8'hB2;
					8'h38 : inv_sbox = 8'h76;
					8'h39 : inv_sbox = 8'h5B;
					8'h3A : inv_sbox = 8'hA2;
					8'h3B : inv_sbox = 8'h49;
					8'h3C : inv_sbox = 8'h6D;
					8'h3D : inv_sbox = 8'h8B;
					8'h3E : inv_sbox = 8'hD1;
					8'h3F : inv_sbox = 8'h25;
					8'h40 : inv_sbox = 8'h72;
					8'h41 : inv_sbox = 8'hF8;
					8'h42 : inv_sbox = 8'hF6;
					8'h43 : inv_sbox = 8'h64;
					8'h44 : inv_sbox = 8'h86;
					8'h45 : inv_sbox = 8'h68;
					8'h46 : inv_sbox = 8'h98;
					8'h47 : inv_sbox = 8'h16;
					8'h48 : inv_sbox = 8'hD4;
					8'h49 : inv_sbox = 8'hA4;
					8'h4A : inv_sbox = 8'h5C;
					8'h4B : inv_sbox = 8'hCC;
					8'h4C : inv_sbox = 8'h5D;
					8'h4D : inv_sbox = 8'h65;
					8'h4E : inv_sbox = 8'hB6;
					8'h4F : inv_sbox = 8'h92;
					8'h50 : inv_sbox = 8'h6C;
					8'h51 : inv_sbox = 8'h70;
					8'h52 : inv_sbox = 8'h48;
					8'h53 : inv_sbox = 8'h50;
					8'h54 : inv_sbox = 8'hFD;
					8'h55 : inv_sbox = 8'hED;
					8'h56 : inv_sbox = 8'hB9;
					8'h57 : inv_sbox = 8'hDA;
					8'h58 : inv_sbox = 8'h5E;
					8'h59 : inv_sbox = 8'h15;
					8'h5A : inv_sbox = 8'h46;
					8'h5B : inv_sbox = 8'h57;
					8'h5C : inv_sbox = 8'hA7;
					8'h5D : inv_sbox = 8'h8D;
					8'h5E : inv_sbox = 8'h9D;
					8'h5F : inv_sbox = 8'h84;
					8'h60 : inv_sbox = 8'h90;
					8'h61 : inv_sbox = 8'hD8;
					8'h62 : inv_sbox = 8'hAB;
					8'h63 : inv_sbox = 8'h0;
					8'h64 : inv_sbox = 8'h8C;
					8'h65 : inv_sbox = 8'hBC;
					8'h66 : inv_sbox = 8'hD3;
					8'h67 : inv_sbox = 8'hA;
					8'h68 : inv_sbox = 8'hF7;
					8'h69 : inv_sbox = 8'hE4;
					8'h6A : inv_sbox = 8'h58;
					8'h6B : inv_sbox = 8'h5;
					8'h6C : inv_sbox = 8'hB8;
					8'h6D : inv_sbox = 8'hB3;
					8'h6E : inv_sbox = 8'h45;
					8'h6F : inv_sbox = 8'h6;
					8'h70 : inv_sbox = 8'hD0;
					8'h71 : inv_sbox = 8'h2C;
					8'h72 : inv_sbox = 8'h1E;
					8'h73 : inv_sbox = 8'h8F;
					8'h74 : inv_sbox = 8'hCA;
					8'h75 : inv_sbox = 8'h3F;
					8'h76 : inv_sbox = 8'hF;
					8'h77 : inv_sbox = 8'h2;
					8'h78 : inv_sbox = 8'hC1;
					8'h79 : inv_sbox = 8'hAF;
					8'h7A : inv_sbox = 8'hBD;
					8'h7B : inv_sbox = 8'h3;
					8'h7C : inv_sbox = 8'h1;
					8'h7D : inv_sbox = 8'h13;
					8'h7E : inv_sbox = 8'h8A;
					8'h7F : inv_sbox = 8'h6B;
					8'h80 : inv_sbox = 8'h3A;
					8'h81 : inv_sbox = 8'h91;
					8'h82 : inv_sbox = 8'h11;
					8'h83 : inv_sbox = 8'h41;
					8'h84 : inv_sbox = 8'h4F;
					8'h85 : inv_sbox = 8'h67;
					8'h86 : inv_sbox = 8'hDC;
					8'h87 : inv_sbox = 8'hEA;
					8'h88 : inv_sbox = 8'h97;
					8'h89 : inv_sbox = 8'hF2;
					8'h8A : inv_sbox = 8'hCF;
					8'h8B : inv_sbox = 8'hCE;
					8'h8C : inv_sbox = 8'hF0;
					8'h8D : inv_sbox = 8'hB4;
					8'h8E : inv_sbox = 8'hE6;
					8'h8F : inv_sbox = 8'h73;
					8'h90 : inv_sbox = 8'h96;
					8'h91 : inv_sbox = 8'hAC;
					8'h92 : inv_sbox = 8'h74;
					8'h93 : inv_sbox = 8'h22;
					8'h94 : inv_sbox = 8'hE7;
					8'h95 : inv_sbox = 8'hAD;
					8'h96 : inv_sbox = 8'h35;
					8'h97 : inv_sbox = 8'h85;
					8'h98 : inv_sbox = 8'hE2;
					8'h99 : inv_sbox = 8'hF9;
					8'h9A : inv_sbox = 8'h37;
					8'h9B : inv_sbox = 8'hE8;
					8'h9C : inv_sbox = 8'h1C;
					8'h9D : inv_sbox = 8'h75;
					8'h9E : inv_sbox = 8'hDF;
					8'h9F : inv_sbox = 8'h6E;
					8'hA0 : inv_sbox = 8'h47;
					8'hA1 : inv_sbox = 8'hF1;
					8'hA2 : inv_sbox = 8'h1A;
					8'hA3 : inv_sbox = 8'h71;
					8'hA4 : inv_sbox = 8'h1D;
					8'hA5 : inv_sbox = 8'h29;
					8'hA6 : inv_sbox = 8'hC5;
					8'hA7 : inv_sbox = 8'h89;
					8'hA8 : inv_sbox = 8'h6F;
					8'hA9 : inv_sbox = 8'hB7;
					8'hAA : inv_sbox = 8'h62;
					8'hAB : inv_sbox = 8'hE;
					8'hAC : inv_sbox = 8'hAA;
					8'hAD : inv_sbox = 8'h18;
					8'hAE : inv_sbox = 8'hBE;
					8'hAF : inv_sbox = 8'h1B;
					8'hB0 : inv_sbox = 8'hFC;
					8'hB1 : inv_sbox = 8'h56;
					8'hB2 : inv_sbox = 8'h3E;
					8'hB3 : inv_sbox = 8'h4B;
					8'hB4 : inv_sbox = 8'hC6;
					8'hB5 : inv_sbox = 8'hD2;
					8'hB6 : inv_sbox = 8'h79;
					8'hB7 : inv_sbox = 8'h20;
					8'hB8 : inv_sbox = 8'h9A;
					8'hB9 : inv_sbox = 8'hDB;
					8'hBA : inv_sbox = 8'hC0;
					8'hBB : inv_sbox = 8'hFE;
					8'hBC : inv_sbox = 8'h78;
					8'hBD : inv_sbox = 8'hCD;
					8'hBE : inv_sbox = 8'h5A;
					8'hBF : inv_sbox = 8'hF4;
					8'hC0 : inv_sbox = 8'h1F;
					8'hC1 : inv_sbox = 8'hDD;
					8'hC2 : inv_sbox = 8'hA8;
					8'hC3 : inv_sbox = 8'h33;
					8'hC4 : inv_sbox = 8'h88;
					8'hC5 : inv_sbox = 8'h7;
					8'hC6 : inv_sbox = 8'hC7;
					8'hC7 : inv_sbox = 8'h31;
					8'hC8 : inv_sbox = 8'hB1;
					8'hC9 : inv_sbox = 8'h12;
					8'hCA : inv_sbox = 8'h10;
					8'hCB : inv_sbox = 8'h59;
					8'hCC : inv_sbox = 8'h27;
					8'hCD : inv_sbox = 8'h80;
					8'hCE : inv_sbox = 8'hEC;
					8'hCF : inv_sbox = 8'h5F;
					8'hD0 : inv_sbox = 8'h60;
					8'hD1 : inv_sbox = 8'h51;
					8'hD2 : inv_sbox = 8'h7F;
					8'hD3 : inv_sbox = 8'hA9;
					8'hD4 : inv_sbox = 8'h19;
					8'hD5 : inv_sbox = 8'hB5;
					8'hD6 : inv_sbox = 8'h4A;
					8'hD7 : inv_sbox = 8'hD;
					8'hD8 : inv_sbox = 8'h2D;
					8'hD9 : inv_sbox = 8'hE5;
					8'hDA : inv_sbox = 8'h7A;
					8'hDB : inv_sbox = 8'h9F;
					8'hDC : inv_sbox = 8'h93;
					8'hDD : inv_sbox = 8'hC9;
					8'hDE : inv_sbox = 8'h9C;
					8'hDF : inv_sbox = 8'hEF;
					8'hE0 : inv_sbox = 8'hA0;
					8'hE1 : inv_sbox = 8'hE0;
					8'hE2 : inv_sbox = 8'h3B;
					8'hE3 : inv_sbox = 8'h4D;
					8'hE4 : inv_sbox = 8'hAE;
					8'hE5 : inv_sbox = 8'h2A;
					8'hE6 : inv_sbox = 8'hF5;
					8'hE7 : inv_sbox = 8'hB0;
					8'hE8 : inv_sbox = 8'hC8;
					8'hE9 : inv_sbox = 8'hEB;
					8'hEA : inv_sbox = 8'hBB;
					8'hEB : inv_sbox = 8'h3C;
					8'hEC : inv_sbox = 8'h83;
					8'hED : inv_sbox = 8'h53;
					8'hEE : inv_sbox = 8'h99;
					8'hEF : inv_sbox = 8'h61;
					8'hF0 : inv_sbox = 8'h17;
					8'hF1 : inv_sbox = 8'h2B;
					8'hF2 : inv_sbox = 8'h4;
					8'hF3 : inv_sbox = 8'h7E;
					8'hF4 : inv_sbox = 8'hBA;
					8'hF5 : inv_sbox = 8'h77;
					8'hF6 : inv_sbox = 8'hD6;
					8'hF7 : inv_sbox = 8'h26;
					8'hF8 : inv_sbox = 8'hE1;
					8'hF9 : inv_sbox = 8'h69;
					8'hFA : inv_sbox = 8'h14;
					8'hFB : inv_sbox = 8'h63;
					8'hFC : inv_sbox = 8'h55;
					8'hFD : inv_sbox = 8'h21;
					8'hFE : inv_sbox = 8'hC;
					8'hFF : inv_sbox = 8'h7D;
					default : inv_sbox = 8'h0;
				endcase
            end
            assign out = inv_sbox;
endmodule

module sbox(input [7:0] in,
            output [7:0] out);

            reg [7:0] sbox;
            // TODO: replace this with a clock?
            // or consider muxing the whole thing using an array...
            always @(*) begin
                case (in)
                    8'h0 : sbox = 8'h63;
                    8'h1 : sbox = 8'h7C;
                    8'h2 : sbox = 8'h77;
                    8'h3 : sbox = 8'h7B;
                    8'h4 : sbox = 8'hF2;
                    8'h5 : sbox = 8'h6B;
                    8'h6 : sbox = 8'h6F;
                    8'h7 : sbox = 8'hC5;
                    8'h8 : sbox = 8'h30;
                    8'h9 : sbox = 8'h1;
                    8'hA : sbox = 8'h67;
                    8'hB : sbox = 8'h2B;
                    8'hC : sbox = 8'hFE;
                    8'hD : sbox = 8'hD7;
                    8'hE : sbox = 8'hAB;
                    8'hF : sbox = 8'h76;
                    8'h10 : sbox = 8'hCA;
                    8'h11 : sbox = 8'h82;
                    8'h12 : sbox = 8'hC9;
                    8'h13 : sbox = 8'h7D;
                    8'h14 : sbox = 8'hFA;
                    8'h15 : sbox = 8'h59;
                    8'h16 : sbox = 8'h47;
                    8'h17 : sbox = 8'hF0;
                    8'h18 : sbox = 8'hAD;
                    8'h19 : sbox = 8'hD4;
                    8'h1A : sbox = 8'hA2;
                    8'h1B : sbox = 8'hAF;
                    8'h1C : sbox = 8'h9C;
                    8'h1D : sbox = 8'hA4;
                    8'h1E : sbox = 8'h72;
                    8'h1F : sbox = 8'hC0;
                    8'h20 : sbox = 8'hB7;
                    8'h21 : sbox = 8'hFD;
                    8'h22 : sbox = 8'h93;
                    8'h23 : sbox = 8'h26;
                    8'h24 : sbox = 8'h36;
                    8'h25 : sbox = 8'h3F;
                    8'h26 : sbox = 8'hF7;
                    8'h27 : sbox = 8'hCC;
                    8'h28 : sbox = 8'h34;
                    8'h29 : sbox = 8'hA5;
                    8'h2A : sbox = 8'hE5;
                    8'h2B : sbox = 8'hF1;
                    8'h2C : sbox = 8'h71;
                    8'h2D : sbox = 8'hD8;
                    8'h2E : sbox = 8'h31;
                    8'h2F : sbox = 8'h15;
                    8'h30 : sbox = 8'h4;
                    8'h31 : sbox = 8'hC7;
                    8'h32 : sbox = 8'h23;
                    8'h33 : sbox = 8'hC3;
                    8'h34 : sbox = 8'h18;
                    8'h35 : sbox = 8'h96;
                    8'h36 : sbox = 8'h5;
                    8'h37 : sbox = 8'h9A;
                    8'h38 : sbox = 8'h7;
                    8'h39 : sbox = 8'h12;
                    8'h3A : sbox = 8'h80;
                    8'h3B : sbox = 8'hE2;
                    8'h3C : sbox = 8'hEB;
                    8'h3D : sbox = 8'h27;
                    8'h3E : sbox = 8'hB2;
                    8'h3F : sbox = 8'h75;
                    8'h40 : sbox = 8'h9;
                    8'h41 : sbox = 8'h83;
                    8'h42 : sbox = 8'h2C;
                    8'h43 : sbox = 8'h1A;
                    8'h44 : sbox = 8'h1B;
                    8'h45 : sbox = 8'h6E;
                    8'h46 : sbox = 8'h5A;
                    8'h47 : sbox = 8'hA0;
                    8'h48 : sbox = 8'h52;
                    8'h49 : sbox = 8'h3B;
                    8'h4A : sbox = 8'hD6;
                    8'h4B : sbox = 8'hB3;
                    8'h4C : sbox = 8'h29;
                    8'h4D : sbox = 8'hE3;
                    8'h4E : sbox = 8'h2F;
                    8'h4F : sbox = 8'h84;
                    8'h50 : sbox = 8'h53;
                    8'h51 : sbox = 8'hD1;
                    8'h52 : sbox = 8'h0;
                    8'h53 : sbox = 8'hED;
                    8'h54 : sbox = 8'h20;
                    8'h55 : sbox = 8'hFC;
                    8'h56 : sbox = 8'hB1;
                    8'h57 : sbox = 8'h5B;
                    8'h58 : sbox = 8'h6A;
                    8'h59 : sbox = 8'hCB;
                    8'h5A : sbox = 8'hBE;
                    8'h5B : sbox = 8'h39;
                    8'h5C : sbox = 8'h4A;
                    8'h5D : sbox = 8'h4C;
                    8'h5E : sbox = 8'h58;
                    8'h5F : sbox = 8'hCF;
                    8'h60 : sbox = 8'hD0;
                    8'h61 : sbox = 8'hEF;
                    8'h62 : sbox = 8'hAA;
                    8'h63 : sbox = 8'hFB;
                    8'h64 : sbox = 8'h43;
                    8'h65 : sbox = 8'h4D;
                    8'h66 : sbox = 8'h33;
                    8'h67 : sbox = 8'h85;
                    8'h68 : sbox = 8'h45;
                    8'h69 : sbox = 8'hF9;
                    8'h6A : sbox = 8'h2;
                    8'h6B : sbox = 8'h7F;
                    8'h6C : sbox = 8'h50;
                    8'h6D : sbox = 8'h3C;
                    8'h6E : sbox = 8'h9F;
                    8'h6F : sbox = 8'hA8;
                    8'h70 : sbox = 8'h51;
                    8'h71 : sbox = 8'hA3;
                    8'h72 : sbox = 8'h40;
                    8'h73 : sbox = 8'h8F;
                    8'h74 : sbox = 8'h92;
                    8'h75 : sbox = 8'h9D;
                    8'h76 : sbox = 8'h38;
                    8'h77 : sbox = 8'hF5;
                    8'h78 : sbox = 8'hBC;
                    8'h79 : sbox = 8'hB6;
                    8'h7A : sbox = 8'hDA;
                    8'h7B : sbox = 8'h21;
                    8'h7C : sbox = 8'h10;
                    8'h7D : sbox = 8'hFF;
                    8'h7E : sbox = 8'hF3;
                    8'h7F : sbox = 8'hD2;
                    8'h80 : sbox = 8'hCD;
                    8'h81 : sbox = 8'hC;
                    8'h82 : sbox = 8'h13;
                    8'h83 : sbox = 8'hEC;
                    8'h84 : sbox = 8'h5F;
                    8'h85 : sbox = 8'h97;
                    8'h86 : sbox = 8'h44;
                    8'h87 : sbox = 8'h17;
                    8'h88 : sbox = 8'hC4;
                    8'h89 : sbox = 8'hA7;
                    8'h8A : sbox = 8'h7E;
                    8'h8B : sbox = 8'h3D;
                    8'h8C : sbox = 8'h64;
                    8'h8D : sbox = 8'h5D;
                    8'h8E : sbox = 8'h19;
                    8'h8F : sbox = 8'h73;
                    8'h90 : sbox = 8'h60;
                    8'h91 : sbox = 8'h81;
                    8'h92 : sbox = 8'h4F;
                    8'h93 : sbox = 8'hDC;
                    8'h94 : sbox = 8'h22;
                    8'h95 : sbox = 8'h2A;
                    8'h96 : sbox = 8'h90;
                    8'h97 : sbox = 8'h88;
                    8'h98 : sbox = 8'h46;
                    8'h99 : sbox = 8'hEE;
                    8'h9A : sbox = 8'hB8;
                    8'h9B : sbox = 8'h14;
                    8'h9C : sbox = 8'hDE;
                    8'h9D : sbox = 8'h5E;
                    8'h9E : sbox = 8'hB;
                    8'h9F : sbox = 8'hDB;
                    8'hA0 : sbox = 8'hE0;
                    8'hA1 : sbox = 8'h32;
                    8'hA2 : sbox = 8'h3A;
                    8'hA3 : sbox = 8'hA;
                    8'hA4 : sbox = 8'h49;
                    8'hA5 : sbox = 8'h6;
                    8'hA6 : sbox = 8'h24;
                    8'hA7 : sbox = 8'h5C;
                    8'hA8 : sbox = 8'hC2;
                    8'hA9 : sbox = 8'hD3;
                    8'hAA : sbox = 8'hAC;
                    8'hAB : sbox = 8'h62;
                    8'hAC : sbox = 8'h91;
                    8'hAD : sbox = 8'h95;
                    8'hAE : sbox = 8'hE4;
                    8'hAF : sbox = 8'h79;
                    8'hB0 : sbox = 8'hE7;
                    8'hB1 : sbox = 8'hC8;
                    8'hB2 : sbox = 8'h37;
                    8'hB3 : sbox = 8'h6D;
                    8'hB4 : sbox = 8'h8D;
                    8'hB5 : sbox = 8'hD5;
                    8'hB6 : sbox = 8'h4E;
                    8'hB7 : sbox = 8'hA9;
                    8'hB8 : sbox = 8'h6C;
                    8'hB9 : sbox = 8'h56;
                    8'hBA : sbox = 8'hF4;
                    8'hBB : sbox = 8'hEA;
                    8'hBC : sbox = 8'h65;
                    8'hBD : sbox = 8'h7A;
                    8'hBE : sbox = 8'hAE;
                    8'hBF : sbox = 8'h8;
                    8'hC0 : sbox = 8'hBA;
                    8'hC1 : sbox = 8'h78;
                    8'hC2 : sbox = 8'h25;
                    8'hC3 : sbox = 8'h2E;
                    8'hC4 : sbox = 8'h1C;
                    8'hC5 : sbox = 8'hA6;
                    8'hC6 : sbox = 8'hB4;
                    8'hC7 : sbox = 8'hC6;
                    8'hC8 : sbox = 8'hE8;
                    8'hC9 : sbox = 8'hDD;
                    8'hCA : sbox = 8'h74;
                    8'hCB : sbox = 8'h1F;
                    8'hCC : sbox = 8'h4B;
                    8'hCD : sbox = 8'hBD;
                    8'hCE : sbox = 8'h8B;
                    8'hCF : sbox = 8'h8A;
                    8'hD0 : sbox = 8'h70;
                    8'hD1 : sbox = 8'h3E;
                    8'hD2 : sbox = 8'hB5;
                    8'hD3 : sbox = 8'h66;
                    8'hD4 : sbox = 8'h48;
                    8'hD5 : sbox = 8'h3;
                    8'hD6 : sbox = 8'hF6;
                    8'hD7 : sbox = 8'hE;
                    8'hD8 : sbox = 8'h61;
                    8'hD9 : sbox = 8'h35;
                    8'hDA : sbox = 8'h57;
                    8'hDB : sbox = 8'hB9;
                    8'hDC : sbox = 8'h86;
                    8'hDD : sbox = 8'hC1;
                    8'hDE : sbox = 8'h1D;
                    8'hDF : sbox = 8'h9E;
                    8'hE0 : sbox = 8'hE1;
                    8'hE1 : sbox = 8'hF8;
                    8'hE2 : sbox = 8'h98;
                    8'hE3 : sbox = 8'h11;
                    8'hE4 : sbox = 8'h69;
                    8'hE5 : sbox = 8'hD9;
                    8'hE6 : sbox = 8'h8E;
                    8'hE7 : sbox = 8'h94;
                    8'hE8 : sbox = 8'h9B;
                    8'hE9 : sbox = 8'h1E;
                    8'hEA : sbox = 8'h87;
                    8'hEB : sbox = 8'hE9;
                    8'hEC : sbox = 8'hCE;
                    8'hED : sbox = 8'h55;
                    8'hEE : sbox = 8'h28;
                    8'hEF : sbox = 8'hDF;
                    8'hF0 : sbox = 8'h8C;
                    8'hF1 : sbox = 8'hA1;
                    8'hF2 : sbox = 8'h89;
                    8'hF3 : sbox = 8'hD;
                    8'hF4 : sbox = 8'hBF;
                    8'hF5 : sbox = 8'hE6;
                    8'hF6 : sbox = 8'h42;
                    8'hF7 : sbox = 8'h68;
                    8'hF8 : sbox = 8'h41;
                    8'hF9 : sbox = 8'h99;
                    8'hFA : sbox = 8'h2D;
                    8'hFB : sbox = 8'hF;
                    8'hFC : sbox = 8'hB0;
                    8'hFD : sbox = 8'h54;
                    8'hFE : sbox = 8'hBB;
                    8'hFF : sbox = 8'h16;
                    default : sbox = 8'h0;
                endcase
            end
            assign out = sbox;
endmodule

//TODO 
module subbytes(input [127:0] in, 
                input decrypt, // flag, when set to 1 work in decrypt mode
                output [127:0] out);
                
     sbox q0( .in(in[127:120]),.out(out[127:120]) );
     sbox q1( .in(in[119:112]),.out(out[119:112]) );
     sbox q2( .in(in[111:104]),.out(out[111:104]) );
     sbox q3( .in(in[103:96]),.out(out[103:96]) );
     
     sbox q4( .in(in[95:88]),.out(out[95:88]) );
     sbox q5( .in(in[87:80]),.out(out[87:80]) );
     sbox q6( .in(in[79:72]),.out(out[79:72]) );
     sbox q7( .in(in[71:64]),.out(out[71:64]) );
     
     sbox q8( .in(in[63:56]),.out(out[63:56]) );
     sbox q9( .in(in[55:48]),.out(out[55:48]) );
     sbox q10(.in(in[47:40]),.out(out[47:40]) );
     sbox q11(.in(in[39:32]),.out(out[39:32]) );
     
     sbox q12(.in(in[31:24]),.out(out[31:24]) );
     sbox q13(.in(in[23:16]),.out(out[23:16]) );
     sbox q14(.in(in[15:8]),.out(out[15:8]) );
     sbox q16(.in(in[7:0]),.out(out[7:0]) );
endmodule

module shiftrows(input [127:0] in, 
                input decrypt, // flag, when set to 1 work in decrypt mode
                output [127:0] out);
                
                wire [7:0] bytes [15:0];  
                        
                genvar i;
                generate
                    for (i = 0; i < 16; i=i+1) begin : gen_bytes 
                        assign bytes[i] = in[127-i:127-i-7];
                    end
                endgenerate
                
                assign out = {bytes[0], bytes[1], bytes[2], bytes[3], 
                              bytes[5], bytes[6], bytes[7], bytes[4],
                              bytes[10], bytes[11], bytes[8], bytes[9],
                              bytes[15], bytes[12], bytes[13], bytes[14]};
endmodule

//TODO
module mixcolumns(input [127:0] in, 
                input decrypt, // flag, when set to 1 work in decrypt mode
                output [127:0] out);
                
        wire [7:0] bytes [15:0];  
        wire [7:0] dbytes [15:0];       // doubled bytes
        wire [7:0] mixed_bytes [15:0];  
        
        genvar i;
        generate
            for (i = 0; i < 16; i=i+1) begin : gen_bytes 
                assign bytes[i] = in[127-i*8:127-i*8-7];
                assign dbytes[i] = bytes[i] << 1;
            end
        endgenerate
       
       /*
       We left multiply each column by

       2 3 1 1 
       1 2 3 1
       1 1 2 3
       3 1 1 2

       in order to generate a new matrix. This means we can go column by column,
       taking every fourth byte.
       
       multiplication by 2 is equivalent to a left shift of 1,
       multiplication by 3 is a left shift of 1 xored with the initial value. 
       using this, we can implement this transformation with no multipliers.       
       */
       
        // TODO: check if i need to process mods here. 
        genvar j;
        generate
            for (j = 0; j < 4; j=j+1) begin : gen_column_mix 
                assign mixed_bytes[i] = dbytes[i]^dbytes[i+4]^bytes[i+4]^bytes[i+8]^bytes[i+12];
                assign mixed_bytes[i+4] = bytes[i]^dbytes[i+4]^dbytes[i+8]^bytes[i+8]^bytes[i+12];
                assign mixed_bytes[i+8] = bytes[i]^bytes[i+4]^dbytes[i+8]^dbytes[i+12]^bytes[i+12];
                assign mixed_bytes[i+12] = dbytes[i]^bytes[i]^bytes[i+4]^bytes[i+8]^dbytes[i+12];
            end
        endgenerate
        
        assign out = {bytes[0], bytes[1], bytes[2], bytes[3],
                      bytes[4], bytes[5], bytes[6], bytes[7],
                      bytes[8], bytes[9], bytes[10], bytes[11], 
                      bytes[12], bytes[13], bytes[14], bytes[15]};
        
endmodule
