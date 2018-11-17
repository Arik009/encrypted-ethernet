// SW[0]: reset
// SW[1]: master configure: on for transmit, off for receive
// BTNC: dump ram
// BTNL: send sample packet
module main(
	input CLK100MHZ,
	input [15:0] SW,
	input BTNC, BTNU, BTNL, BTNR, BTND,
	output [7:0] JB,
	output [3:0] VGA_R,
	output [3:0] VGA_B,
	output [3:0] VGA_G,
	output VGA_HS,
	output VGA_VS,
	output LED16_B, LED16_G, LED16_R,
	output LED17_B, LED17_G, LED17_R,
	output [15:0] LED,
	output [7:0] SEG,  // segments A-G (0-6), DP (7)
	output [7:0] AN,	// Display 0-7
	inout ETH_CRSDV, ETH_RXERR,
	inout [1:0] ETH_RXD,
	output ETH_REFCLK, ETH_INTN, ETH_RSTN,
	input UART_TXD_IN, UART_RTS,
	output UART_RXD_OUT, UART_CTS,
	output ETH_TXEN,
	output [1:0] ETH_TXD,
	output ETH_MDC, ETH_MDIO,
	inout [15:0] ddr2_dq,
	inout [1:0] ddr2_dqs_n, ddr2_dqs_p,
	output [12:0] ddr2_addr,
	output [2:0] ddr2_ba,
	output ddr2_ras_n, ddr2_cas_n, ddr2_we_n,
	output [0:0] ddr2_ck_p, ddr2_ck_n, ddr2_cke, ddr2_cs_n,
	output [1:0] ddr2_dm,
	output [0:0] ddr2_odt
	);

////// INCLUDES

`include "params.vh"
`include "networking.vh"

localparam RAM_SIZE = PACKET_BUFFER_SIZE;
localparam VRAM_SIZE = VIDEO_CACHE_RAM_SIZE;

////// CLOCKING

wire clk_50mhz;

// the main clock for FPGA logic will be 50MHz
wire clk;
assign clk = clk_50mhz;

wire clk_120mhz, clk_65mhz;

// 50MHz clock for Ethernet receiving
clk_wiz_0 clk_wiz_inst(
	.reset(0),
	.clk_in1(CLK100MHZ),
	.clk_out1(clk_50mhz),
	.clk_out3(clk_120mhz));

////// RESET

wire sw0, sw1;
delay #(.DELAY_LEN(SYNC_DELAY_LEN)) sw0_sync(
	.clk(clk), .rst(0), .in(SW[0]), .out(sw0));
delay #(.DELAY_LEN(SYNC_DELAY_LEN)) sw1_sync(
	.clk(clk), .rst(0), .in(SW[1]), .out(sw1));

wire config_transmit;
assign config_transmit = sw1;

reg prev_sw1 = 0;
always @(posedge clk) begin
	prev_sw1 <= sw1;
end

// reset device when configuration is changed
wire config_change_reset;
assign config_change_reset = sw1 != prev_sw1;

wire rst;
// ensure that reset pulse lasts a sufficient long amount of time
pulse_extender reset_pe(
	.clk(clk), .rst(0), .in(sw0 || config_change_reset), .out(rst));

////// HEX DISPLAY

wire [31:0] hex_display_data;
wire [6:0] segments;

display_8hex display(
	.clk(clk), .data(hex_display_data), .seg(segments), .strobe(AN));

assign SEG[7] = 1'b1;
assign SEG[6:0] = segments;

////// LEDS

assign LED16_R = BTNL; // left button -> red led
assign LED16_G = BTNC; // center button -> green led
assign LED16_B = BTNR; // right button -> blue led
assign LED17_R = BTNL;
assign LED17_G = BTNC;
assign LED17_B = BTNR;

////// BUTTONS

wire btnc_raw, btnl_raw, btnc, btnl;
sync_debounce sd_btnc(
	.rst(rst), .clk(clk), .in(BTNC), .out(btnc_raw));
sync_debounce sd_btnl(
	.rst(rst), .clk(clk), .in(BTNL), .out(btnl_raw));

pulse_generator pg_btnc(
	.clk(clk), .rst(rst), .in(btnc_raw), .out(btnc));
pulse_generator pg_btnl(
	.clk(clk), .rst(rst), .in(btnl_raw), .out(btnl));

////// VGA

wire [clog2(VGA_WIDTH)-1:0] vga_x;
wire [clog2(VGA_HEIGHT)-1:0] vga_y;
// allow for hsync and vsync to be delayed before sending on wire
wire vga_hsync, vga_vsync, vga_hsync_predelay, vga_vsync_predelay, blank;

xvga xvga_inst(
	.vclock(clk), .hcount(vga_x), .vcount(vga_y),
	.vga_hsync(vga_hsync_predelay), .vga_vsync(vga_vsync_predelay),
	.blank(blank));

wire [3:0] vga_r_out, vga_g_out, vga_b_out;
wire [COLOR_LEN-1:0] vga_col;
assign {vga_r_out, vga_g_out, vga_b_out} = vga_col;
assign vga_hs_out = vga_hsync;
assign vga_vs_out = vga_vsync;

// buffer all outputs
delay #(.DATA_WIDTH(4)) vga_r_sync(
	.clk(clk), .rst(rst), .in(vga_r_out), .out(VGA_R));
delay #(.DATA_WIDTH(4)) vga_g_sync(
	.clk(clk), .rst(rst), .in(vga_g_out), .out(VGA_G));
delay #(.DATA_WIDTH(4)) vga_b_sync(
	.clk(clk), .rst(rst), .in(vga_b_out), .out(VGA_B));
delay vga_hs_sync(
	.clk(clk), .rst(rst), .in(vga_hsync), .out(VGA_HS));
delay vga_vs_sync(
	.clk(clk), .rst(rst), .in(vga_vsync), .out(VGA_VS));

////// BRAM

wire ram_readclk, ram_outclk, ram_we;
wire [clog2(RAM_SIZE)-1:0] ram_raddr, ram_waddr;
wire [BYTE_LEN-1:0] ram_out, ram_win;
packet_buffer_ram_driver ram_driv_inst(
	.clk(clk), .rst(rst),
	.readclk(ram_readclk), .raddr(ram_raddr),
	.we(ram_we), .waddr(ram_waddr), .win(ram_win),
	.outclk(ram_outclk), .out(ram_out));

wire vram_readclk, vram_outclk, vram_we;
wire [clog2(VIDEO_CACHE_RAM_SIZE)-1:0] vram_raddr, vram_waddr;
wire [COLOR_LEN-1:0] vram_out, vram_win;
video_cache_ram_driver vram_driv_inst(
	.clk(clk), .rst(rst),
	.readclk(vram_readclk), .raddr(vram_raddr),
	.we(vram_we), .waddr(vram_waddr), .win(vram_win),
	.outclk(vram_outclk), .out(vram_out));

////// RAM MULTIPLEXING

wire uart_ram_we;
wire [clog2(RAM_SIZE)-1:0] uart_ram_waddr;
wire [BYTE_LEN-1:0] uart_ram_win;
wire eth_ram_we;
wire [clog2(RAM_SIZE)-1:0] eth_ram_waddr;
wire [BYTE_LEN-1:0] eth_ram_win;
assign ram_we =
	config_transmit ? uart_ram_we : eth_ram_we;
assign ram_waddr =
	config_transmit ? uart_ram_waddr : eth_ram_waddr;
assign ram_win =
	config_transmit ? uart_ram_win : eth_ram_win;

////// RMII

assign ETH_REFCLK = clk;
assign ETH_MDC = 0;
assign ETH_MDIO = 0;
wire rmii_outclk, rmii_done;
wire [1:0] rmii_out;
rmii_driver rmii_driv_inst(
	.clk(clk), .rst(rst),
	.crsdv_in(ETH_CRSDV), .rxd_in(ETH_RXD),
	.rxerr(ETH_RXERR),
	.intn(ETH_INTN), .rstn(ETH_RSTN),
	.out(rmii_out),
	.outclk(rmii_outclk), .done(rmii_done));

wire eth_txen;
wire [1:0] eth_txd;
// to be connected to eth_frame_generator
assign eth_txen = 0;
assign eth_txd = 2'b0;

// buffer the outputs so that eth_txd calculation would be
// under timing constraints
delay eth_txen_delay(
	.clk(clk), .rst(rst), .in(eth_txen), .out(ETH_TXEN));
delay #(.DATA_WIDTH(2)) eth_txd_delay(
	.clk(clk), .rst(rst), .in(eth_txd), .out(ETH_TXD));

////// UART RX => RAM

assign UART_CTS = 1;
wire [7:0] uart_rx_out;
wire uart_rx_outclk;
uart_rx_fast_driver uart_rx_inst (
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst),
	.rxd(UART_TXD_IN), .out(uart_rx_out), .outclk(uart_rx_outclk));
stream_to_memory uart_stm_inst(
	.clk(clk), .rst(rst),
	.setoff_req(1'b0), .setoff_val(0),
	.inclk(uart_rx_outclk), .in(uart_rx_out),
	.ram_we(uart_ram_we), .ram_waddr(uart_ram_waddr),
	.ram_win(uart_ram_win));

////// UART TX <= RAM

wire uart_tx_inclk, uart_tx_ready;
wire [BYTE_LEN-1:0] uart_tx_in;
wire uart_txd;
stream_from_memory uart_sfm_inst(
	.clk(clk), .rst(rst), .start(uart_sfm_start),
	.read_start(0), .read_end(RAM_SIZE),
	.readclk(uart_tx_ready),
	.ram_outclk(ram_outclk), .ram_out(ram_out),
	.ram_readclk(ram_readclk), .ram_raddr(ram_raddr),
	.outclk(uart_tx_inclk), .out(uart_tx_in));
uart_tx_fast_stream_driver uart_tx_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst),
	.start(btnc),
	.inclk(uart_tx_inclk), .in(uart_tx_in), .txd(UART_RXD_OUT),
	.ready(uart_tx_ready));

////// ETHERNET RX => VRAM

wire eth_parse_downstream_done, eth_parse_outclk, eth_parse_err;
wire [BYTE_LEN-1:0] eth_parse_out;
eth_parser eth_parser_inst(
	.clk(clk), .rst(rst),
	.inclk(rmii_outclk), .in(rmii_out),
	.in_done(rmii_done),
	.downstream_done(eth_parse_downstream_done),
	.outclk(eth_parse_outclk), .out(eth_parse_out),
	.err(eth_parse_err));
wire fgp_parse_setoff_req, fgp_parse_outclk;
wire [BYTE_LEN+clog2(FGP_DATA_LEN_COLORS)-1:0] fgp_parse_setoff_val;
wire [BYTE_LEN-1:0] fgp_parse_out;
wire eth_parse_downstream_rst;
assign eth_parse_downstream_rst = rst || eth_parse_err;
fgp_parser fgp_parser_inst(
	.clk(clk), .rst(eth_parse_downstream_rst),
	.inclk(eth_parse_outclk), .in(eth_parse_out),
	.done(eth_parse_downstream_done),
	.setoff_req(fgp_parse_setoff_req), .setoff_val(fgp_parse_setoff_val),
	.outclk(fgp_parse_outclk), .out(fgp_parse_out));
wire fgp_btc_outclk;
wire [COLOR_LEN-1:0] fgp_btc_out;
bytes_to_colors fgp_btc_inst(
	.clk(clk), .rst(eth_parse_downstream_rst),
	.inclk(fgp_parse_outclk), .in(fgp_parse_out),
	.outclk(fgp_btc_outclk), .out(fgp_btc_out));
stream_to_memory
	#(.RAM_SIZE(VRAM_SIZE), .WORD_LEN(COLOR_LEN)) fgp_stm_inst(
	.clk(clk), .rst(eth_parse_downstream_rst),
	.setoff_req(fgp_parse_setoff_req),
	.setoff_val(fgp_parse_setoff_val[clog2(VRAM_SIZE)-1:0]),
	.inclk(fgp_btc_outclk), .in(fgp_btc_out),
	.ram_we(vram_we), .ram_waddr(vram_waddr),
	.ram_win(vram_win));

////// VRAM => VGA

graphics_main graphics_main_inst(
	.clk(clk), .rst(rst), .blank(blank),
	.vga_x(vga_x), .vga_y(vga_y),
	.vga_hsync_in(vga_hsync_predelay), .vga_vsync_in(vga_vsync_predelay),
	.ram_read_ready(vram_outclk), .ram_read_val(vram_out),
	.ram_read_req(vram_readclk), .ram_read_addr(vram_raddr),
	.vga_col(vga_col),
	.vga_hsync_out(vga_hsync), .vga_vsync_out(vga_vsync));

////// ETHERNET RX => RAM

wire eth_dtb_outclk, eth_dtb_done;
wire [BYTE_LEN-1:0] eth_dtb_out;
dibits_to_bytes eth_dtb(
	.clk(clk), .rst(rst),
	.inclk(rmii_outclk), .in(rmii_out), .in_done(rmii_done),
	.outclk(eth_dtb_outclk), .out(eth_dtb_out),
	.done(eth_dtb_done));
stream_to_memory eth_stm_inst(
	.clk(clk), .rst(rst),
	.setoff_req(eth_dtb_done), .setoff_val(0),
	.inclk(eth_dtb_outclk), .in(eth_dtb_out),
	.ram_we(eth_ram_we), .ram_waddr(eth_ram_waddr),
	.ram_win(eth_ram_win));

////// DEBUGGING SIGNALS

wire blink;
blinker blinker_inst(
	.clk(clk), .rst(rst),
	.enable(1), .out(blink));

assign LED = {
	SW[15:2],
	blink,
	rst
};

assign hex_display_data = {
	4'h0, ram_waddr, 4'h0, ram_raddr
};

assign JB = {
	8'h0
};

endmodule

module main_test_aes(
	input CLK100MHZ,
	input [15:0] SW,
	input BTNC, BTNU, BTNL, BTNR, BTND,
	output [7:0] JB,
	output [3:0] VGA_R,
	output [3:0] VGA_B,
	output [3:0] VGA_G,
	output VGA_HS,
	output VGA_VS,
	output LED16_B, LED16_G, LED16_R,
	output LED17_B, LED17_G, LED17_R,
	output [15:0] LED,
	output [7:0] SEG,  // segments A-G (0-6), DP (7)
	output [7:0] AN,	// Display 0-7
	inout ETH_CRSDV, ETH_RXERR,
	inout [1:0] ETH_RXD,
	output ETH_REFCLK, ETH_INTN, ETH_RSTN,
	input UART_TXD_IN, UART_RTS,
	output UART_RXD_OUT, UART_CTS,
	output ETH_TXEN,
	output [1:0] ETH_TXD,
	output ETH_MDC, ETH_MDIO,
	inout [15:0] ddr2_dq,
	inout [1:0] ddr2_dqs_n, ddr2_dqs_p,
	output [12:0] ddr2_addr,
	output [2:0] ddr2_ba,
	output ddr2_ras_n, ddr2_cas_n, ddr2_we_n,
	output [0:0] ddr2_ck_p, ddr2_ck_n, ddr2_cke, ddr2_cs_n,
	output [1:0] ddr2_dm,
	output [0:0] ddr2_odt
	);

wire clk_50mhz;

// the main clock for FPGA logic will be 50MHz
wire clk;
assign clk = clk_50mhz;

// 50MHz clock for Ethernet receiving
clk_wiz_0 clk_wiz_inst(
	.reset(0),
	.clk_in1(CLK100MHZ),
	.clk_out1(clk_50mhz));

reg [127:0] aes_in, aes_key;
wire [127:0] aes_out;
reg [6:0] aes_cnt = 0;
aes_encrypt_block block(.in(aes_in), .out(aes_out), .key(aes_key));
reg [127:0] aes_out_shift;
reg jb_out;
assign JB[0] = jb_out;

wire tx_clk;
clock_divider #(.PULSE_PERIOD(128)) cd(.clk(clk), .start(0), .en(1), .out(block_clk));

always @(posedge clk) begin
	aes_in <= {aes_in[126:0], SW[0]};
	aes_key <= {aes_key[126:0], SW[1]};
	if (block_clk)
		aes_out_shift <= aes_out;
	{aes_out_shift[126:0], jb_out} <= aes_out_shift;
	aes_cnt <= aes_cnt + 1;
end

endmodule
