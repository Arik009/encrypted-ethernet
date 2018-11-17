`timescale 1ns / 1ps

module test_ethernet_driver();

`include "params.vh"

reg clk = 0;
// 50MHz clock
initial forever #10 clk = ~clk;

reg rst = 1;
wire crsdv_tr = 1'bz, rxerr_tr = 1'bz;
wire [1:0] rxd_tr = 2'bzz;
reg crsdv, rxerr;
reg [1:0] rxd;
reg rst_done = 0;
assign crsdv_tr = rst_done ? crsdv : 1'bz;
assign rxerr_tr = rst_done ? rxerr : 1'bz;
assign rxd_tr = rst_done ? rxd : 2'bzz;
wire intn, rstn;
wire [1:0] eth_out;
wire eth_outclk, eth_done;
wire [BYTE_LEN-1:0] eth_byte_out;
wire eth_byte_outclk, eth_dtb_done;
rmii_driver rmii_driv_inst(
	.clk(clk), .rst(rst),
	.crsdv_in(crsdv_tr), .rxd_in(rxd_tr),
	.rxerr(rxerr_tr),
	.intn(intn), .rstn(rstn),
	.out(eth_out),
	.outclk(eth_outclk), .done(eth_done));
dibits_to_bytes eth_dtb(
	.clk(clk), .rst(rst),
	.inclk(eth_outclk), .in(eth_out), .in_done(eth_done),
	.out(eth_byte_out), .outclk(eth_byte_outclk), .done(eth_dtb_done));

initial begin
	#100
	rst = 0;
	// reset sequence
	#400
	rst_done = 1;
	rxerr = 0;
	rxd = 0;
	crsdv = 0;
	#100
	crsdv = 1;
	#100

	// 28 cycles = 56 bits
	rxd = 2'b01;
	#540
	rxd = 2'b11;
	#20

	// send 10101010 twice (2 bytes)
	rxd = 2'b10;
	#160

	rxd = 0;
	crsdv = 0;
	#160

	$stop();
end

endmodule

module test_ddr2_ram();

`include "params.vh"

reg clk_100mhz = 0;
// 100MHz clock
initial forever #5 clk_100mhz = ~clk_100mhz;

// clk will be 50MHz
wire clk, clk_200mhz;
clk_wiz_0 clk_wiz_inst(
	.rst(0),
	.clk_in1(clk_100mhz), .clk_out1(clk), .clk_out2(clk_200mhz));

wire [15:0] ddr2_dq;
wire [1:0] ddr2_dqs_n, ddr2_dqs_p;
wire [12:0] ddr2_addr;
wire [2:0] ddr2_ba;
wire ddr2_ras_n, ddr2_cas_n, ddr2_we_n;
wire [0:0] ddr2_ck_p, ddr2_ck_n, ddr2_cke, ddr2_cs_n;
wire [1:0] ddr2_dm;
wire [0:0] ddr2_odt;
reg [26:0] app_addr = 0;
reg [2:0] app_cmd = 3'b111;
reg app_en = 0;
reg [63:0] app_wdf_data = 0;
reg app_wdf_end = 0;
reg [7:0] app_wdf_mask = 0;
reg app_wdf_wren = 0;
wire [63:0] app_rd_data;
wire app_rd_data_end, app_rd_data_valid, app_rdy, app_wdf_rdy;
wire ui_clk, ui_clk_sync_rst, init_calib_complete;
reg sys_rst = 0;
`define x16
`define sg25
ddr2_mcp ddr2_mcp_inst(
	.ck(ddr2_ck_p), .ck_n(ddr2_ck_n), .cke(ddr2_cke),
	.cs_n(ddr2_cs_n),
	.ras_n(ddr2_ras_n), .cas_n(ddr2_cas_n), .we_n(ddr2_we_n),
	.dm_rdqs(ddr2_dm), .ba(ddr2_ba), .addr(ddr2_addr),
	.dq(ddr2_dq), .dqs(ddr2_dqs_p), .dqs_n(ddr2_dqs_n),
	.odt(ddr2_odt));
nexys4_ddr2 ddr2_ram_inst(
	.ddr2_dq(ddr2_dq), .ddr2_dqs_n(ddr2_dqs_n), .ddr2_dqs_p(ddr2_dqs_p),
	.ddr2_addr(ddr2_addr), .ddr2_ba(ddr2_ba),
	.ddr2_ras_n(ddr2_ras_n), .ddr2_cas_n(ddr2_cas_n), .ddr2_we_n(ddr2_we_n),
	.ddr2_ck_p(ddr2_ck_p), .ddr2_ck_n(ddr2_ck_n), .ddr2_cke(ddr2_cke),
	.ddr2_cs_n(ddr2_cs_n),
	.ddr2_dm(ddr2_dm), .ddr2_odt(ddr2_odt), .sys_clk_i(clk_200mhz),
	.sys_rst(sys_rst), .app_sr_req(0), .app_ref_req(0), .app_zq_req(0),
	.app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en),
	.app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end),
	.app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
	.app_rd_data(app_rd_data), .app_rd_data_end(app_rd_data_end),
	.app_rd_data_valid(app_rd_data_valid),
	.app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
	.ui_clk(ui_clk), .ui_clk_sync_rst(ui_clk_sync_rst),
	.init_calib_complete(init_calib_complete));

initial begin
	#400
	sys_rst = 1;
	#1000000
	$stop();
end

endmodule

module test_crc();

`include "params.vh"

localparam RAM_SIZE = PACKET_SYNTH_ROM_SIZE;

reg clk = 0;
// 50MHz clock
initial forever #10 clk = ~clk;

reg rst = 1;
wire read_req;
reg [clog2(RAM_SIZE)-1:0] read_addr = 0;
wire read_ready;
wire [BYTE_LEN-1:0] read_out;
packet_synth_rom_driver packet_synth_rom_driver_inst(
	.clk(clk), .rst(rst), .readclk(read_req), .raddr(read_addr),
	.outclk(read_ready), .out(read_out));
reg done_in = 0;
wire [1:0] dibit_out;
wire byte_clk, done_out;
wire btd_idle;
bytes_to_dibits btd_inst(
	.clk(clk), .rst(rst), .inclk(read_ready),
	.in(read_out), .in_done(done_in),
	.out(dibit_out), .outclk(byte_clk), .rdy(btd_idle),
	.done(done_out));
wire [31:0] crc;
crc32 crc32_inst(
	.clk(clk), .rst(rst), .shift(0),
	.inclk(byte_clk), .in(dibit_out), .out(crc));

reg reading = 0;
reg [clog2(BYTE_LEN)-2:0] dibit_cnt;
always @(posedge clk) begin
	if (rst)
		dibit_cnt = 0;
	else if (reading) begin
		if (dibit_cnt == BYTE_LEN/2-1)
			read_addr <= read_addr + 1;
		dibit_cnt <= dibit_cnt + 1;
	end
end
assign read_req = reading && dibit_cnt == 0;

initial begin
	#100
	rst = 0;

	// reset sequence
	#400

	reading = 1;
	// read out sample packet
	// 62 bytes * 4 dibits * 20ns
	#4960
	reading = 0;

	#100

	$stop();
end

endmodule

module test_packet_synth();

`include "params.vh"
`include "packet_synth_rom_layout.vh"

reg clk_100mhz = 0;
// 100MHz clock
initial forever #5 clk_100mhz = ~clk_100mhz;

wire clk;
clk_wiz_0 clk_wiz_inst(
	.reset(1'b0),
	.clk_in1(clk_100mhz),
	.clk_out1(clk));

localparam RAM_SIZE = PACKET_SYNTH_ROM_SIZE;
reg rst = 1, start = 0;
wire efg_inclk, efg_outclk, efg_done;
wire [BYTE_LEN-1:0] efg_in;
wire [1:0] efg_out;
wire rom1_readclk, rom2_readclk, rom1_outclk, rom2_outclk;
wire [clog2(RAM_SIZE)-1:0] rom1_raddr, rom2_raddr;
wire [BYTE_LEN-1:0] rom1_out, rom2_out;
packet_synth_rom_driver psr_driv_1(
	.clk(clk), .rst(rst),
	.readclk(rom1_readclk), .raddr(rom1_raddr),
	.outclk(rom1_outclk), .out(rom1_out));
packet_synth_rom_driver psr_driv_2(
	.clk(clk), .rst(rst),
	.readclk(rom2_readclk), .raddr(rom2_raddr),
	.outclk(rom2_outclk), .out(rom2_out));
wire sfm_readclk, sfm_done;
stream_from_memory #(
	.RAM_SIZE(RAM_SIZE), .RAM_READ_LATENCY(PACKET_SYNTH_ROM_LATENCY))
	sfm_inst(
	.clk(clk), .rst(rst), .start(start),
	.read_start(SAMPLE_PAYLOAD_OFF),
	.read_end(SAMPLE_PAYLOAD_OFF + SAMPLE_PAYLOAD_LEN),
	.readclk(sfm_readclk),
	.ram_outclk(rom1_outclk), .ram_out(rom1_out),
	.ram_readclk(rom1_readclk), .ram_raddr(rom1_raddr),
	.outclk(efg_inclk), .out(efg_in), .done(sfm_done));
eth_generator efg_inst(
	.clk(clk), .rst(rst), .start(start), .in_done(sfm_done),
	.inclk(efg_inclk), .in(efg_in),
	.ram_outclk(rom2_outclk), .ram_out(rom2_out),
	.ram_readclk(rom2_readclk), .ram_raddr(rom2_raddr),
	.outclk(efg_outclk), .out(efg_out),
	.upstream_readclk(sfm_readclk), .done(efg_done));

initial begin
	#500
	rst = 0;
	start = 1;
	#20
	start = 0;

	// bytes * dibits/byte * ns/dibit
	#(88 * 4 * 20)

	#400

	$stop();
end

endmodule

module test_aes_one_block_encrypt();

reg clk_100mhz = 0;
// 100MHz clock
initial forever #5 clk_100mhz = ~clk_100mhz;

wire clk;
clk_wiz_0 clk_wiz_inst(
	.reset(0),
	.clk_in1(clk_100mhz),
	.clk_out1(clk));

reg [127:0] in;
reg [127:0] key;
wire [127:0] out;

aes_encrypt_block aes(
	.in(in), .key(key), .out(out));

initial begin
	#100
	in = 200;
    key = 0;
	#1000

	$stop();
end

endmodule

module test_fast_uart();

`include "params.vh"

reg clk_100mhz = 0;
// 100MHz clock
initial forever #5 clk_100mhz = ~clk_100mhz;

reg clk_12mbaud = 0;
initial forever #41.6667 clk_12mbaud = ~clk_12mbaud;

wire clk, clk_120mhz;
clk_wiz_0 clk_wiz_inst(
	.reset(1'b0),
	.clk_in1(clk_100mhz),
	.clk_out1(clk),
	.clk_out3(clk_120mhz));

reg rst = 1;

localparam RAM_SIZE = PACKET_BUFFER_SIZE;

wire ram_readclk, ram_we, ram_outclk;
wire [clog2(RAM_SIZE)-1:0] ram_raddr, ram_waddr;
wire [BYTE_LEN-1:0] ram_win, ram_out;
packet_buffer_ram_driver ram_driv_inst(
	.clk(clk), .rst(rst),
	.readclk(ram_readclk), .raddr(ram_raddr),
	.we(ram_we), .waddr(ram_waddr), .win(ram_win),
	.outclk(ram_outclk), .out(ram_out));

wire uart_rxd;
wire [7:0] uart_out;
wire uart_outclk;
uart_rx_fast_driver uart_rx_inst (
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst),
	.rxd(uart_rxd), .out(uart_out), .outclk(uart_outclk));
stream_to_memory uart_stm_inst(
	.clk(clk), .rst(rst),
	.set_offset_req(1'b0), .set_offset_val(0),
	.inclk(uart_outclk), .in(uart_out),
	.ram_we(ram_we), .ram_waddr(ram_waddr),
	.ram_win(ram_win));

reg sfm_start = 0;

wire uart_inclk;
wire [BYTE_LEN-1:0] uart_in;
wire uart_txd, uart_tx_ready;
uart_tx_fast_stream_driver uart_tx_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst), .start(sfm_start),
	.inclk(uart_inclk), .in(uart_in), .txd(uart_txd),
	.ready(uart_tx_ready));
stream_from_memory uart_sfm_inst(
	.clk(clk), .rst(rst), .start(sfm_start),
	.read_start(0), .read_end(3),
	.downstream_rdy(uart_tx_ready),
	.ram_outclk(ram_outclk), .ram_out(ram_out),
	.ram_readclk(ram_readclk), .ram_raddr(ram_raddr),
	.outclk(uart_inclk), .out(uart_in));

reg [31:0] test_data = 32'b10_10011011_10_11010000_110_10001111_1;
assign uart_rxd = test_data[31];
reg sending = 0;
always @(posedge clk_12mbaud) begin
	if (sending)
		test_data <= {test_data[30:0], test_data[31]};
end

initial begin
	#2000
	rst = 0;
	#20
	sending = 1;
	#(32 * 2 * 41.6667)
	#40
	sfm_start = 1;
	#20
	sfm_start = 0;
	#(32 * 2 * 41.6667)
	#40
	$stop();
end

endmodule

module test_bytes_to_colors();

`include "params.vh"

reg clk = 0;
// 50MHz clock
initial forever #10 clk = ~clk;

reg rst = 1;
reg btc_inclk = 0;
wire [BYTE_LEN-1:0] btc_in;
wire btc_outclk;
wire [COLOR_LEN-1:0] btc_out;

bytes_to_colors btc_inst(
	.clk(clk), .rst(rst), .inclk(btc_inclk), .in(btc_in),
	.outclk(btc_outclk), .out(btc_out));
reg [4*COLOR_LEN-1:0] in_data_shifted = 48'hDEADBEEFCAFE;
assign btc_in = in_data_shifted[0+:BYTE_LEN];

wire vram_readclk, vram_outclk, vram_we;
wire [clog2(VIDEO_CACHE_RAM_SIZE)-1:0] vram_raddr, vram_waddr;
wire [COLOR_LEN-1:0] vram_out, vram_win;
video_cache_ram_driver vram_driv_inst(
	.clk(clk), .rst(rst),
	.readclk(vram_readclk), .raddr(vram_raddr),
	.we(vram_we), .waddr(vram_waddr), .win(vram_win),
	.outclk(vram_outclk), .out(vram_out));
stream_to_memory
	#(.RAM_SIZE(VIDEO_CACHE_RAM_SIZE), .WORD_LEN(COLOR_LEN)) stm_inst(
	.clk(clk), .rst(rst), .set_offset_req(0), .set_offset_val(0),
	.inclk(btc_outclk), .in(btc_out),
	.ram_we(vram_we), .ram_waddr(vram_waddr),
	.ram_win(vram_win));
assign vram_readclk = 0;
assign vram_raddr = 0;

always @(posedge clk) begin
	if (btc_inclk)
		in_data_shifted <= {in_data_shifted[0+:BYTE_LEN],
			in_data_shifted[BYTE_LEN+:BYTE_LEN]};
end

initial begin
	#100
	rst = 0;
	#100
	btc_inclk = 1;
	#(6 * 20)
	#100
	$stop();
end

endmodule

module test_packet_parse();

`include "params.vh"
`include "packet_synth_rom_layout.vh"

reg clk = 0;
// 50MHz clock
initial forever #10 clk = ~clk;

localparam RAM_SIZE = PACKET_SYNTH_ROM_SIZE;

reg rst = 1, start = 0;
wire rom_readclk, rom_outclk;
wire [clog2(RAM_SIZE)-1:0] rom_raddr;
wire [BYTE_LEN-1:0] rom_out;
wire sfm_readclk;
wire btd_inclk, btd_in_done;
wire [BYTE_LEN-1:0] btd_in;
wire eth_parse_inclk, eth_parse_in_done;
wire [1:0] eth_parse_in;
packet_synth_rom_driver psr_inst(
	.clk(clk), .rst(rst),
	.readclk(rom_readclk), .raddr(rom_raddr),
	.outclk(rom_outclk), .out(rom_out));
stream_from_memory #(
	.RAM_SIZE(RAM_SIZE), .RAM_READ_LATENCY(PACKET_SYNTH_ROM_LATENCY))
	sfm_inst(
	.clk(clk), .rst(rst), .start(start),
	.read_start(SAMPLE_FRAME_OFF),
	.read_end(SAMPLE_FRAME_OFF + SAMPLE_FRAME_LEN),
	.readclk(sfm_readclk),
	.ram_outclk(rom_outclk), .ram_out(rom_out),
	.ram_readclk(rom_readclk), .ram_raddr(rom_raddr),
	.outclk(btd_inclk), .out(btd_in), .done(btd_in_done));
bytes_to_dibits_coord_buf btd_inst(
	.clk(clk), .rst(rst),
	.inclk(btd_inclk), .in(btd_in), .in_done(btd_in_done),
	.downstream_rdy(1), .readclk(sfm_readclk),
	.outclk(eth_parse_inclk), .out(eth_parse_in),
	.done(eth_parse_in_done));
eth_parser eth_parser_inst(
	.clk(clk), .rst(rst),
	.inclk(eth_parse_inclk), .in(eth_parse_in),
	.in_done(eth_parse_in_done));

initial begin
	#100
	rst = 0;
	start = 1;
	#20
	start = 0;
	#(SAMPLE_FRAME_LEN * 4 * 20)
	#400
	$stop();
end

endmodule

module test_main();

reg clk_100mhz = 0;
// 100MHz clock
initial forever #5 clk_100mhz = ~clk_100mhz;

wire clk, clk_120mhz;
clk_wiz_0 clk_wiz_inst(
	.reset(0),
	.clk_in1(clk_100mhz),
	.clk_out1(clk),
	.clk_out3(clk_120mhz));

initial begin
	#10000
	$stop();
end

endmodule
