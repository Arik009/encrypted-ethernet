`include "util.vh"

localparam SYNC_DELAY_LEN = 3;

localparam BYTE_LEN = 8;
localparam COLOR_CHANNEL_LEN = 4;
localparam COLOR_LEN = COLOR_CHANNEL_LEN * 3;

localparam PACKET_BUFFER_SIZE = 4096;
// taken from ip summary
localparam PACKET_BUFFER_READ_LATENCY = 2;

localparam PACKET_SYNTH_ROM_SIZE = 4096;
localparam PACKET_SYNTH_ROM_LATENCY = 2;

localparam VGA_WIDTH = 800;
localparam VGA_HEIGHT = 600;
localparam VGA_H_FRONT_PORCH = 56;
localparam VGA_H_SYNC = 120;
localparam VGA_H_BACK_PORCH = 64;
localparam VGA_V_FRONT_PORCH = 37;
localparam VGA_V_SYNC = 6;
localparam VGA_V_BACK_PORCH = 23;
localparam VGA_POLARITY = 0;

// localparam VGA_WIDTH = 1024;
// localparam VGA_HEIGHT = 768;
// localparam VGA_H_FRONT_PORCH = 24;
// localparam VGA_H_SYNC = 136;
// localparam VGA_H_BACK_PORCH = 160;
// localparam VGA_V_FRONT_PORCH = 9;
// localparam VGA_V_SYNC = 6;
// localparam VGA_V_BACK_PORCH = 23;
// localparam VGA_POLARITY = 1;

// localparam VGA_WIDTH = 640;
// localparam VGA_HEIGHT = 480;
// localparam VGA_H_FRONT_PORCH = 16;
// localparam VGA_H_SYNC = 96;
// localparam VGA_H_BACK_PORCH = 48;
// localparam VGA_V_FRONT_PORCH = 10;
// localparam VGA_V_SYNC = 2;
// localparam VGA_V_BACK_PORCH = 33;
// localparam VGA_POLARITY = 1;

localparam VGA_H_TOT = VGA_WIDTH +
	VGA_H_FRONT_PORCH + VGA_H_FRONT_PORCH + VGA_H_SYNC;
localparam VGA_V_TOT = VGA_HEIGHT +
	VGA_V_FRONT_PORCH + VGA_V_FRONT_PORCH + VGA_V_SYNC;
