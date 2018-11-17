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

`include "params.vh"

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

wire [31:0] hex_display_data;
wire [6:0] segments;

display_8hex display(
	.clk(clk), .data(hex_display_data), .seg(segments), .strobe(AN));

assign SEG[7] = 1'b1;
assign SEG[6:0] = segments;

assign LED16_R = BTNL; // left button -> red led
assign LED16_G = BTNC; // center button -> green led
assign LED16_B = BTNR; // right button -> blue led
assign LED17_R = BTNL;
assign LED17_G = BTNC;
assign LED17_B = BTNR;

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

assign UART_CTS = 1;

wire btnc_raw, btnl_raw, btnc, btnl;
sync_debounce sd_btnc(
	.rst(rst), .clk(clk), .in(BTNC), .out(btnc_raw));
sync_debounce sd_btnl(
	.rst(rst), .clk(clk), .in(BTNL), .out(btnl_raw));

pulse_generator pg_btnc(
	.clk(clk), .rst(rst), .in(btnc_raw), .out(btnc));
pulse_generator pg_btnl(
	.clk(clk), .rst(rst), .in(btnl_raw), .out(btnl));

wire ram_readclk, ram_outclk, ram_we;
wire [clog2(PACKET_BUFFER_SIZE)-1:0] ram_raddr, ram_waddr;
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

wire uart_ram_we;
wire [clog2(PACKET_BUFFER_SIZE)-1:0] uart_ram_waddr;
wire [BYTE_LEN-1:0] uart_ram_win;
wire eth_ram_we;
wire [clog2(PACKET_BUFFER_SIZE)-1:0] eth_ram_waddr;
wire [BYTE_LEN-1:0] eth_ram_win;
assign ram_we =
	config_transmit ? uart_ram_we : eth_ram_we;
assign ram_waddr =
	config_transmit ? uart_ram_waddr : eth_ram_waddr;
assign ram_win =
	config_transmit ? uart_ram_win : eth_ram_win;

wire [7:0] uart_rx_out;
wire uart_rx_outclk;
uart_rx_fast_driver uart_rx_inst (
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst),
	.rxd(UART_TXD_IN), .out(uart_rx_out), .outclk(uart_rx_outclk));
stream_to_memory uart_stm_inst(
	.clk(clk), .rst(rst),
	.set_offset_req(1'b0), .set_offset_val(0),
	.inclk(uart_rx_outclk), .in(uart_rx_out),
	.ram_we(uart_ram_we), .ram_waddr(uart_ram_waddr),
	.ram_win(uart_ram_win));

wire uart_tx_inclk, uart_tx_ready;
wire [BYTE_LEN-1:0] uart_tx_in;
wire uart_txd;
wire uart_sfm_start;
assign uart_sfm_start = btnc;
uart_tx_fast_stream_driver uart_tx_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst),
	.start(uart_sfm_start),
	.inclk(uart_tx_inclk), .in(uart_tx_in), .txd(UART_RXD_OUT),
	.ready(uart_tx_ready));
stream_from_memory uart_sfm_inst(
	.clk(clk), .rst(rst), .start(uart_sfm_start),
	.read_start(0), .read_end(PACKET_BUFFER_SIZE),
	.readclk(uart_tx_ready),
	.ram_outclk(ram_outclk), .ram_out(ram_out),
	.ram_readclk(ram_readclk), .ram_raddr(ram_raddr),
	.outclk(uart_tx_inclk), .out(uart_tx_in));

wire btc_vram_outclk;
wire [COLOR_LEN-1:0] btc_vram_out;
bytes_to_colors btc_vram(
	.clk(clk), .rst(rst), .inclk(uart_rx_outclk), .in(uart_rx_out),
	.outclk(btc_vram_outclk), .out(btc_vram_out));
stream_to_memory
	#(.RAM_SIZE(VIDEO_CACHE_RAM_SIZE), .WORD_LEN(COLOR_LEN)) stm_vram(
	.clk(clk), .rst(rst), .set_offset_req(0), .set_offset_val(0),
	.inclk(btc_vram_outclk), .in(btc_vram_out),
	.ram_we(vram_we), .ram_waddr(vram_waddr),
	.ram_win(vram_win));

graphics_main graphics_main_inst(
	.clk(clk), .rst(rst), .blank(blank),
	.vga_x(vga_x), .vga_y(vga_y),
	.vga_hsync_in(vga_hsync_predelay), .vga_vsync_in(vga_vsync_predelay),
	.ram_read_ready(vram_outclk), .ram_read_val(vram_out),
	.ram_read_req(vram_readclk), .ram_read_addr(vram_raddr),
	.vga_col(vga_col),
	.vga_hsync_out(vga_hsync), .vga_vsync_out(vga_vsync));

assign ETH_REFCLK = clk;
assign ETH_MDC = 0;
assign ETH_MDIO = 0;
wire eth_outclk, eth_done, eth_byte_outclk, eth_dtb_done;
wire [1:0] eth_out;
rmii_driver rmii_driv_inst(
	.clk(clk), .rst(rst),
	.crsdv_in(ETH_CRSDV), .rxd_in(ETH_RXD),
	.rxerr(ETH_RXERR),
	.intn(ETH_INTN), .rstn(ETH_RSTN),
	.out(eth_out),
	.outclk(eth_outclk), .done(eth_done));
dibits_to_bytes eth_dtb(
	.clk(clk), .rst(rst),
	.inclk(eth_outclk), .in(eth_out), .done_in(eth_done),
	.out(eth_ram_win), .outclk(eth_byte_outclk),
	.done_out(eth_dtb_done));
assign eth_ram_we = eth_byte_outclk;

// maximum ethernet frame length is 1522 bytes
localparam MAX_ETH_FRAME_LEN = 1522;
reg [clog2(MAX_ETH_FRAME_LEN)-1:0] eth_byte_cnt = 0;
reg record = 1;
always @(posedge clk) begin
	if (rst) begin
		eth_byte_cnt <= 0;
		record <= 1;
	end else if (eth_done) begin
		eth_byte_cnt <= 0;
		record <= 0;
	end else if (eth_byte_outclk && record)
		eth_byte_cnt <= eth_byte_cnt + 1;
end
assign eth_ram_waddr = eth_byte_cnt;

wire eth_txen;
wire [1:0] eth_txd;
// to be connected to eth_frame_generator
assign eth_txen = 0;

// buffer the outputs so that eth_txd calculation would be
// under timing constraints
delay eth_txen_delay(
	.clk(clk), .rst(rst), .in(eth_txen), .out(ETH_TXEN));
delay #(.DATA_WIDTH(2)) eth_txd_delay(
	.clk(clk), .rst(rst), .in(eth_txd), .out(ETH_TXD));

// DEBUGGING SIGNALS

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
