`timescale 1ns / 1ps

module test_daisy_chain();

////// INCLUDES

`include "networking.vh"
`include "packet_synth_rom_layout.vh"

localparam RAM_SIZE = PACKET_BUFFER_SIZE;
localparam VRAM_SIZE = VIDEO_CACHE_RAM_SIZE;
localparam ROM_SIZE = PACKET_SYNTH_ROM_SIZE;

localparam KEY = 128'h4b42_4410_770a_ee13_094d_d0da_1217_7bb0;

////// CLOCKING

reg clk_100mhz = 0;
// 100MHz clock
initial forever #5 clk_100mhz = ~clk_100mhz;

wire clk_50mhz;

// the main clock for FPGA logic will be 50MHz
wire clk;
assign clk = clk_50mhz;

wire clk_120mhz, clk_65mhz;

// 50MHz clock for Ethernet receiving
clk_wiz_0 clk_wiz_inst(
	.reset(1'b0),
	.clk_in1(clk_100mhz),
	.clk_out1(clk_50mhz),
	.clk_out3(clk_120mhz));

wire config_transmit;
assign config_transmit = 1;

reg rst = 1;

////// BRAM

// the ram_rst signals allow us to clear any pending reads
wire ram_rst;
wire ram_readclk, ram_outclk, ram_we;
wire [clog2(RAM_SIZE)-1:0] ram_raddr, ram_waddr;
wire [BYTE_LEN-1:0] ram_out, ram_win;
packet_buffer_ram_driver ram_driv_inst(
	.clk(clk), .rst(rst || ram_rst),
	.readclk(ram_readclk), .raddr(ram_raddr),
	.we(ram_we), .waddr(ram_waddr), .win(ram_win),
	.outclk(ram_outclk), .out(ram_out));

wire vram_readclk, vram_outclk, vram_we;
wire [clog2(VRAM_SIZE)-1:0] vram_raddr, vram_waddr;
wire [COLOR_LEN-1:0] vram_out, vram_win;
video_cache_ram_driver vram_driv_inst(
	.clk(clk), .rst(rst),
	.readclk(vram_readclk), .raddr(vram_raddr),
	.we(vram_we), .waddr(vram_waddr), .win(vram_win),
	.outclk(vram_outclk), .out(vram_out));

wire rom_rst, rom_readclk, rom_outclk;
wire [clog2(ROM_SIZE)-1:0] rom_raddr;
wire [BYTE_LEN-1:0] rom_out;
packet_synth_rom_driver psr_inst(
	.clk(clk), .rst(rst || rom_rst),
	.readclk(rom_readclk), .raddr(rom_raddr),
	.outclk(rom_outclk), .out(rom_out));

wire uart_rom_rst, uart_rom_readclk, uart_rom_outclk;
wire [clog2(ROM_SIZE)-1:0] uart_rom_raddr;
wire [BYTE_LEN-1:0] uart_rom_out;
packet_synth_rom_driver uart_psr_inst(
	.clk(clk), .rst(rst || uart_rom_rst),
	.readclk(uart_rom_readclk), .raddr(uart_rom_raddr),
	.outclk(uart_rom_outclk), .out(uart_rom_out));

////// RAM MULTIPLEXING

wire uart_ram_rst, uart_ram_we, uart_ram_readclk, uart_ram_outclk;
wire [clog2(RAM_SIZE)-1:0] uart_ram_waddr, uart_ram_raddr;
wire [BYTE_LEN-1:0] uart_ram_win, uart_ram_out;
wire eth_ram_rst, eth_ram_we, eth_ram_readclk, eth_ram_outclk;
wire [clog2(RAM_SIZE)-1:0] eth_ram_waddr, eth_ram_raddr;
wire [BYTE_LEN-1:0] eth_ram_win, eth_ram_out;
assign ram_rst = config_transmit ? uart_ram_rst : eth_ram_rst;
assign ram_we = config_transmit ? uart_ram_we : eth_ram_we;
assign ram_waddr = config_transmit ? uart_ram_waddr : eth_ram_waddr;
assign ram_win = config_transmit ? uart_ram_win : eth_ram_win;
assign ram_readclk = config_transmit ? eth_ram_readclk : uart_ram_readclk;
assign ram_raddr = config_transmit ? eth_ram_raddr : uart_ram_raddr;
assign uart_ram_outclk = config_transmit ? 0 : ram_outclk;
assign eth_ram_outclk = config_transmit ? ram_outclk : 0;
assign uart_ram_out = config_transmit ? 0 : ram_out;
assign eth_ram_out = config_transmit ? ram_out : 0;

////// AES

wire [BLOCK_LEN-1:0] aes_key;
assign aes_key = {KEY[8+:BLOCK_LEN-8], 8'b0};

wire aes_encr_rst, aes_decr_rst;
wire aes_encr_inclk, aes_decr_inclk, aes_encr_outclk, aes_decr_outclk;
wire [BYTE_LEN-1:0] aes_encr_in, aes_decr_in, aes_encr_out, aes_decr_out;
aes_combined_bytes aes_encr_inst(
	.clk(clk), .rst(rst || aes_encr_rst),
	.inclk(aes_encr_inclk), .in(aes_encr_in), .key(aes_key),
	.outclk(aes_encr_outclk), .out(aes_encr_out),
	.decr_select(1'b0));
aes_combined_bytes aes_decr_inst(
	.clk(clk), .rst(rst || aes_decr_rst),
	.inclk(aes_decr_inclk), .in(aes_decr_in), .key(aes_key),
	.outclk(aes_decr_outclk), .out(aes_decr_out),
	.decr_select(1'b1));

////// UART TX <= ROM

wire uart_tx_inclk, uart_tx_readclk;
wire [BYTE_LEN-1:0] uart_tx_in;
wire uart_tx_start;
assign uart_rom_rst = uart_tx_start;
stream_from_memory uart_sfm_inst(
	.clk(clk), .rst(rst), .start(uart_tx_start),
	.read_start(SAMPLE_PAYLOAD_OFF),
	.read_end(SAMPLE_PAYLOAD_OFF + SAMPLE_PAYLOAD_LEN),
	.readclk(uart_tx_readclk),
	.ram_outclk(uart_rom_outclk), .ram_out(uart_rom_out),
	.ram_readclk(uart_rom_readclk), .ram_raddr(uart_rom_raddr),
	.outclk(uart_tx_inclk), .out(uart_tx_in));
wire uart_txd;
uart_tx_fast_stream_driver uart_tx_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst), .start(uart_tx_start),
	.inclk(uart_tx_inclk), .in(uart_tx_in), .txd(uart_txd),
	.readclk(uart_tx_readclk));

////// UART RX => RAM

wire [7:0] uart_rx_out;
wire uart_rx_outclk;
uart_rx_fast_driver uart_rx_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst),
	.rxd(uart_txd), .out(uart_rx_out), .outclk(uart_rx_outclk));
wire uart_rx_active;
// reset downstream modules if nothing is received for 1ms
pulse_extender #(.EXTEND_LEN(50000)) uart_rx_active_pe(
	.clk(clk), .rst(rst), .in(uart_rx_outclk), .out(uart_rx_active));
wire uart_rx_downstream_rst;
assign uart_rx_downstream_rst = rst || !uart_rx_active;

wire uart_rx_fgp_offset_outclk;
wire [BYTE_LEN-1:0] uart_rx_fgp_offset_out;
wire uart_rx_fgp_outclk;
wire [BYTE_LEN-1:0] uart_rx_fgp_out;
// use an fgp_rx to split the data from the offset
fgp_rx encr_fgp_rx(
	.clk(clk), .rst(uart_rx_downstream_rst),
	.inclk(uart_rx_outclk), .in(uart_rx_out),
	.offset_outclk(uart_rx_fgp_offset_outclk),
	.offset_out(uart_rx_fgp_offset_out),
	.outclk(uart_rx_fgp_outclk), .out(uart_rx_fgp_out));

assign aes_encr_rst = uart_rx_downstream_rst;
assign aes_encr_inclk = uart_rx_fgp_outclk;
assign aes_encr_in = uart_rx_fgp_out;

wire encr_fgp_outclk;
assign encr_fgp_outclk = uart_rx_fgp_offset_outclk || aes_encr_outclk;
wire [BYTE_LEN-1:0] encr_fgp_out;
assign encr_fgp_out = uart_rx_fgp_offset_outclk ?
	uart_rx_fgp_offset_out : aes_encr_out;

reg [clog2(FGP_LEN)-1:0] uart_rx_cnt = 0;
reg [clog2(PB_QUEUE_LEN)-1:0] pb_queue_head = 0, pb_queue_tail = 0;
assign uart_ram_waddr = {pb_queue_tail, uart_rx_cnt};
assign uart_ram_we = encr_fgp_outclk;
assign uart_ram_win = encr_fgp_out;
always @(posedge clk) begin
	if (rst) begin
		pb_queue_tail <= 0;
		uart_rx_cnt <= 0;
	end else if (uart_rx_downstream_rst)
		uart_rx_cnt <= 0;
	else if (encr_fgp_outclk) begin
		if (uart_rx_cnt == FGP_LEN-1) begin
			uart_rx_cnt <= 0;
			// if queue overflows, drop the current packet
			if (pb_queue_tail + 1 != pb_queue_head)
				pb_queue_tail <= pb_queue_tail + 1;
		end else
			uart_rx_cnt <= uart_rx_cnt + 1;
	end
end

////// ETHERNET TX <= RAM

wire eth_txen;
wire [1:0] eth_txd;

wire eth_tx_done;
reg eth_tx_active = 0, eth_tx_start = 0;
always @(posedge clk) begin
	if (rst) begin
		eth_tx_active <= 0;
		eth_tx_start <= 0;
		pb_queue_head <= 0;
	end else if (eth_tx_start) begin
		eth_tx_start <= 0;
		eth_tx_active <= 1;
	end else if (eth_tx_active && eth_tx_done) begin
		eth_tx_active <= 0;
		pb_queue_head <= pb_queue_head + 1;
	end else if (!eth_tx_active && pb_queue_head != pb_queue_tail)
		eth_tx_start <= 1;
end

assign eth_ram_rst = eth_tx_start;
wire [clog2(RAM_SIZE)-1:0] eth_tx_sfm_read_start;
assign eth_tx_sfm_read_start = {pb_queue_head, {clog2(FGP_LEN){1'b0}}};
wire eth_tx_sfm_readclk, eth_tx_sfm_outclk, eth_tx_sfm_done;
wire [BYTE_LEN-1:0] eth_tx_sfm_out;
stream_from_memory #(.RAM_SIZE(RAM_SIZE),
	.RAM_READ_LATENCY(PACKET_BUFFER_READ_LATENCY)) eth_tx_sfm_inst(
	.clk(clk), .rst(rst), .start(eth_tx_start),
	.read_start(eth_tx_sfm_read_start),
	.read_end(eth_tx_sfm_read_start + FGP_LEN),
	.readclk(eth_tx_sfm_readclk),
	.ram_outclk(eth_ram_outclk), .ram_out(eth_ram_out),
	.ram_readclk(eth_ram_readclk), .ram_raddr(eth_ram_raddr),
	.outclk(eth_tx_sfm_outclk), .out(eth_tx_sfm_out),
	.done(eth_tx_sfm_done));
assign rom_rst = eth_tx_start;
eth_tx eth_tx_inst(
	.clk(clk), .rst(rst), .start(eth_tx_start),
	.in_done(eth_tx_sfm_done),
	.inclk(eth_tx_sfm_outclk), .in(eth_tx_sfm_out),
	.ram_outclk(rom_outclk), .ram_out(rom_out),
	.ram_readclk(rom_readclk), .ram_raddr(rom_raddr),
	.outclk(eth_txen), .out(eth_txd),
	.upstream_readclk(eth_tx_sfm_readclk), .done(eth_tx_done));

////// RMII

wire rmii_outclk, rmii_done;
wire [1:0] rmii_out;
wire rmii_rxerr, rmii_intn, rmii_rstn;
rmii_driver rmii_driv_inst(
	.clk(clk), .rst(rst),
	.crsdv_in(eth_txen), .rxd_in(eth_txd),
	.rxerr(rmii_rxerr),
	.intn(rmii_intn), .rstn(rmii_rstn),
	.out(rmii_out),
	.outclk(rmii_outclk), .done(rmii_done));

////// ETHERNET RX => VRAM

wire eth_rx_downstream_done, eth_rx_outclk, eth_rx_err;
wire [BYTE_LEN-1:0] eth_rx_out;
wire eth_rx_ethertype_outclk;
wire [ETH_ETHERTYPE_LEN*BYTE_LEN-1:0] eth_rx_ethertype_out;
eth_rx eth_rx_inst(
	.clk(clk), .rst(rst),
	.inclk(rmii_outclk), .in(rmii_out),
	.in_done(rmii_done),
	.downstream_done(eth_rx_downstream_done),
	.outclk(eth_rx_outclk), .out(eth_rx_out),
	.ethertype_outclk(eth_rx_ethertype_outclk),
	.ethertype_out(eth_rx_ethertype_out),
	.err(eth_rx_err));
wire eth_rx_downstream_rst;
assign eth_rx_downstream_rst = rst || eth_rx_err;

reg fgp_rx_en = 0;
always @(posedge clk) begin
	if (eth_rx_downstream_rst)
		fgp_rx_en <= 0;
	else if (eth_rx_ethertype_outclk)
		fgp_rx_en <= eth_rx_ethertype_out == ETHERTYPE_FGP;
end

wire fgp_rx_setoff_req;
wire [BYTE_LEN+clog2(FGP_DATA_LEN_COLORS)-1:0] fgp_rx_setoff_val;
wire fgp_rx_outclk;
wire [BYTE_LEN-1:0] fgp_rx_out, fgp_rx_offset_out;
fgp_rx fgp_rx_inst(
	.clk(clk), .rst(eth_rx_downstream_rst),
	.inclk(eth_rx_outclk && fgp_rx_en), .in(eth_rx_out),
	.done(eth_rx_downstream_done),
	.offset_outclk(fgp_rx_setoff_req), .offset_out(fgp_rx_offset_out),
	.outclk(fgp_rx_outclk), .out(fgp_rx_out));
assign fgp_rx_setoff_val = {fgp_rx_offset_out,
	{clog2(FGP_DATA_LEN_COLORS){1'b0}}};

assign aes_decr_rst = eth_rx_downstream_rst;
assign aes_decr_inclk = fgp_rx_outclk;
assign aes_decr_in = fgp_rx_out;

wire fgp_btc_outclk;
wire [COLOR_LEN-1:0] fgp_btc_out;
bytes_to_colors fgp_btc_inst(
	.clk(clk), .rst(eth_rx_downstream_rst),
	.inclk(aes_decr_outclk), .in(aes_decr_out),
	.outclk(fgp_btc_outclk), .out(fgp_btc_out));
stream_to_memory
	#(.RAM_SIZE(VRAM_SIZE), .WORD_LEN(COLOR_LEN)) fgp_stm_inst(
	.clk(clk), .rst(eth_rx_downstream_rst),
	.setoff_req(fgp_rx_setoff_req),
	.setoff_val(fgp_rx_setoff_val[clog2(VRAM_SIZE)-1:0]),
	.inclk(fgp_btc_outclk), .in(fgp_btc_out),
	.ram_we(vram_we), .ram_waddr(vram_waddr),
	.ram_win(vram_win));

reg uart_tx_start_manual = 0;
assign uart_tx_start = uart_tx_start_manual || eth_tx_done;

initial begin
	#2000
	rst = 0;
	#400
	uart_tx_start_manual = 1;
	#20
	uart_tx_start_manual = 0;
	// 12mbaud = 84ns per bit, for a frame of 914 bytes
	// multiply by 2 to account for overhead
	// run for two cycles
	#(2 * 2 * 84 * 914 * 8)
	$stop();
end

endmodule
