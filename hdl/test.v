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
	.reset(0),
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
wire [2:0] app_cmd;
wire app_en;
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
nexys4_ddr2 #(.SIM_BYPASS_INIT_CAL("FAST")) ddr2_ram_inst(
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

reg ui_en = 0;
wire reading;
assign reading = ui_en && app_addr != 5 && app_rdy;
assign app_cmd = reading ? 3'b001 : 3'b111;
assign app_en = reading;
always @(posedge ui_clk) begin
	if (init_calib_complete && app_rdy)
		ui_en <= 1;
	if (reading) begin
		app_addr <= app_addr + 1;
	end
end

initial begin
	#400
	sys_rst = 1;
	#500
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
wire eth_tx_inclk, eth_tx_outclk, eth_tx_done;
wire [BYTE_LEN-1:0] eth_tx_in;
wire [1:0] eth_tx_out;
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
wire sfm_readclk, sfm_done, sfm_outclk;
wire [BYTE_LEN-1:0] sfm_out;
stream_from_memory #(.RAM_SIZE(RAM_SIZE),
	.RAM_READ_LATENCY(PACKET_SYNTH_ROM_LATENCY)) sfm_inst(
	.clk(clk), .rst(rst), .start(start),
	.read_start(SAMPLE_IMG_DATA_OFF),
	.read_end(SAMPLE_IMG_DATA_OFF + SAMPLE_IMG_DATA_LEN),
	.readclk(sfm_readclk),
	.ram_outclk(rom1_outclk), .ram_out(rom1_out),
	.ram_readclk(rom1_readclk), .ram_raddr(rom1_raddr),
	.outclk(sfm_outclk), .out(sfm_out), .done(sfm_done));
wire fgp_readclk, fgp_done;
fgp_tx fgp_tx_inst(
	.clk(clk), .rst(rst), .start(start), .in_done(sfm_done),
	.inclk(sfm_outclk), .in(sfm_out),
	// set an arbitrary offset for testing
	.offset(8'hc), .readclk(fgp_readclk),
	.outclk(eth_tx_inclk), .out(eth_tx_in),
	.upstream_readclk(sfm_readclk), .done(fgp_done));
eth_tx eth_tx_inst(
	.clk(clk), .rst(rst), .start(start), .in_done(fgp_done),
	.inclk(eth_tx_inclk), .in(eth_tx_in),
	.ram_outclk(rom2_outclk), .ram_out(rom2_out),
	.ram_readclk(rom2_readclk), .ram_raddr(rom2_raddr),
	.outclk(eth_tx_outclk), .out(eth_tx_out),
	.upstream_readclk(fgp_readclk), .done(eth_tx_done));

initial begin
	#500
	rst = 0;
	start = 1;
	#20
	start = 0;

	// bytes * dibits/byte * ns/dibit
	#(1512 * 4 * 20)

	#400

	$stop();
end

endmodule

module test_aes_full_encrypt();

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
wire [127:0] out_enc, dec;
reg rst; 
reg in_clk;
wire out_clk, out_clk_2; 

aes_combined enc_block(.clk(clk), .rst(rst), .key(key), .inclk(in_clk), .in(in), .outclk(out_clk), .out(out_enc), .decr_select(0));
aes_combined dec_block(.clk(clk), .rst(rst), .key(key), .inclk(out_clk), .in(out_enc), .outclk(out_clk_2), .out(dec), .decr_select(1));


initial begin
    rst = 1;
    in_clk = 0;
	#250
	rst = 0;
	in = 213412334;
    key = 0;
    #10
    in_clk = 1;
    #10
    in_clk = 0;
	#800
	
	
	

	$stop();
end

endmodule

module test_aes_inversion();

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
wire [127:0] out_enc_s, dec_s;
wire [127:0] out_enc_sr, dec_sr;
wire [127:0] out_enc_m, dec_m;
wire [127:0] out_enc_a, dec_a;

reg rst; 
reg in_clk;
wire out_clk, out_clk_2; 

subbytes a(.in(in), .out(out_enc_s), .decrypt(0));
subbytes b(.in(out_enc_s),.out(dec_s), .decrypt(1));
shiftrows c(.in(in), .out(out_enc_sr), .decrypt(0));
shiftrows d(.in(out_enc_sr), .out(dec_sr), .decrypt(1));
mixcolumns e(.in(in), .out(out_enc_m), .decrypt(0));
mixcolumns f(.in(out_enc_m), .out(dec_m), .decrypt(1));
addroundkey g(.in(in), .out(out_enc_a), .key(key));
addroundkey h(.in(out_enc_a), .out(dec_a), .key(key));


initial begin
	#250
	rst = 0;
	in = 2009789435;
    key = 1234343;
    #10
    in_clk = 1;
    #10
    in_clk = 0;
	#800

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

aes_block aes(
	.in(in), .key(key), .out(out), .decr_select(0));

initial begin
	#100
	in = 200;
    key = 0;
	#40

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
	.setoff_req(1'b0), .setoff_val(0),
	.inclk(uart_outclk), .in(uart_out),
	.ram_we(ram_we), .ram_waddr(ram_waddr),
	.ram_win(ram_win));

reg sfm_start = 0;

wire uart_inclk;
wire [BYTE_LEN-1:0] uart_in;
wire uart_txd, uart_tx_ready;
wire sfm_done;
uart_tx_fast_stream_driver uart_tx_inst(
	.clk(clk), .clk_120mhz(clk_120mhz), .rst(rst), .start(sfm_start),
	.inclk(uart_inclk), .in(uart_in), .txd(uart_txd),
	.ready(uart_tx_ready));
stream_from_memory uart_sfm_inst(
	.clk(clk), .rst(rst), .start(sfm_start),
	.read_start(0), .read_end(3),
	.readclk(uart_tx_ready),
	.ram_outclk(ram_outclk), .ram_out(ram_out),
	.ram_readclk(ram_readclk), .ram_raddr(ram_raddr),
	.outclk(uart_inclk), .out(uart_in), .done(sfm_done));

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
	.clk(clk), .rst(rst), .setoff_req(0), .setoff_val(0),
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
`include "networking.vh"
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
wire eth_parse_inclk, eth_parse_in_done, eth_parse_outclk, eth_parse_err;
wire [1:0] eth_parse_in;
wire [BYTE_LEN-1:0] eth_parse_out;
wire fgp_parse_done;
wire btc_inclk, btc_outclk;
wire [BYTE_LEN-1:0] btc_in;
wire [COLOR_LEN-1:0] btc_out;
wire stm_setoff_req;
wire [clog2(VIDEO_CACHE_RAM_SIZE)-1:0] stm_setoff_val;
wire vram_readclk, vram_outclk, vram_we;
wire [clog2(VIDEO_CACHE_RAM_SIZE)-1:0] vram_raddr, vram_waddr;
wire [COLOR_LEN-1:0] vram_out, vram_win;
packet_synth_rom_driver psr_inst(
	.clk(clk), .rst(rst),
	.readclk(rom_readclk), .raddr(rom_raddr),
	.outclk(rom_outclk), .out(rom_out));
stream_from_memory #(.RAM_SIZE(RAM_SIZE),
	.RAM_READ_LATENCY(PACKET_SYNTH_ROM_LATENCY)) sfm_inst(
	.clk(clk), .rst(rst), .start(start),
	.read_start(SAMPLE_FRAME_OFF),
	.read_end(SAMPLE_FRAME_OFF + SAMPLE_FRAME_LEN),
	.readclk(sfm_readclk),
	.ram_outclk(rom_outclk), .ram_out(rom_out),
	.ram_readclk(rom_readclk), .ram_raddr(rom_raddr),
	.outclk(btd_inclk), .out(btd_in), .done(btd_in_done));
bytes_to_dibits_coord_buf btd_inst(
	.clk(clk), .rst(rst || start),
	.inclk(btd_inclk), .in(btd_in), .in_done(btd_in_done),
	.downstream_rdy(1), .readclk(sfm_readclk),
	.outclk(eth_parse_inclk), .out(eth_parse_in),
	.done(eth_parse_in_done));
wire eth_rx_ethertype_outclk;
wire [ETH_ETHERTYPE_LEN*BYTE_LEN-1:0] eth_rx_ethertype_out;
eth_rx eth_rx_inst(
	.clk(clk), .rst(rst),
	.inclk(eth_parse_inclk), .in(eth_parse_in),
	.in_done(eth_parse_in_done),
	.downstream_done(fgp_parse_done),
	.outclk(eth_parse_outclk), .out(eth_parse_out),
	.ethertype_outclk(eth_rx_ethertype_outclk),
	.ethertype_out(eth_rx_ethertype_out),
	.err(eth_parse_err));
wire eth_parse_downstream_rst;
assign eth_parse_downstream_rst = rst || eth_parse_err;
fgp_rx fgp_rx_inst(
	.clk(clk), .rst(eth_parse_downstream_rst),
	.inclk(eth_parse_outclk), .in(eth_parse_out),
	.done(fgp_parse_done),
	.setoff_req(stm_setoff_req), .setoff_val(stm_setoff_val),
	.outclk(btc_inclk), .out(btc_in));
bytes_to_colors btc_inst(
	.clk(clk), .rst(eth_parse_downstream_rst),
	.inclk(btc_inclk), .in(btc_in),
	.outclk(btc_outclk), .out(btc_out));
stream_to_memory
	#(.RAM_SIZE(VIDEO_CACHE_RAM_SIZE), .WORD_LEN(COLOR_LEN)) stm_inst(
	.clk(clk), .rst(eth_parse_downstream_rst),
	.setoff_req(stm_setoff_req), .setoff_val(stm_setoff_val),
	.inclk(btc_outclk), .in(btc_out),
	.ram_we(vram_we), .ram_waddr(vram_waddr),
	.ram_win(vram_win));
video_cache_ram_driver vram_driv_inst(
	.clk(clk), .rst(rst),
	.readclk(vram_readclk), .raddr(vram_raddr),
	.we(vram_we), .waddr(vram_waddr), .win(vram_win),
	.outclk(vram_outclk), .out(vram_out));

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

module test_daisy_chain();

////// INCLUDES

`include "params.vh"
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

wire aes_encr_rst, aes_decr_rst;
wire aes_encr_inclk, aes_decr_inclk, aes_encr_outclk, aes_decr_outclk;
wire [BYTE_LEN-1:0] aes_encr_in, aes_decr_in, aes_encr_out, aes_decr_out;
aes_combined_bytes aes_encr_inst(
	.clk(clk), .rst(rst || aes_encr_rst),
	.inclk(aes_encr_inclk), .in(aes_encr_in), .key(KEY),
	.outclk(aes_encr_outclk), .out(aes_encr_out),
	.decr_select(0));
aes_combined_bytes aes_decr_inst(
	.clk(clk), .rst(rst || aes_decr_rst),
	.inclk(aes_decr_inclk), .in(aes_decr_in), .key(KEY),
	.outclk(aes_decr_outclk), .out(aes_decr_out),
	.decr_select(0));

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

localparam PB_PARTITION_LEN = 2**clog2(FGP_LEN);
localparam PB_QUEUE_LEN = PACKET_BUFFER_SIZE / PB_PARTITION_LEN;
reg [clog2(FGP_LEN)-1:0] uart_rx_cnt = 0;
reg [clog2(PB_QUEUE_LEN)-1:0] pb_queue_head = 0, pb_queue_tail = 0;
assign uart_ram_waddr = {pb_queue_tail, uart_rx_cnt};
assign uart_ram_we = encr_fgp_outclk;
assign uart_ram_win = encr_fgp_out;
always @(posedge clk) begin
	if (uart_rx_downstream_rst) begin
		uart_rx_cnt <= 0;
		pb_queue_head <= 0;
		pb_queue_tail <= 0;
	end else if (encr_fgp_outclk) begin
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
