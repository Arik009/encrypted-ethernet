`timescale 1ns / 1ps

module xvga(
	input vclock,
	output reg [clog2(VGA_WIDTH)-1:0] hcount,
	output reg [clog2(VGA_HEIGHT)-1:0] vcount,
	output reg vsync, hsync,
	// vga outputs account for polarity
	output vga_vsync, vga_hsync,
	output reg blank);

`include "params.vh"

localparam VGA_H_FRONT_PORCH = 56;
localparam VGA_H_SYNC = 120;
localparam VGA_H_BACK_PORCH = 64;
localparam VGA_V_FRONT_PORCH = 37;
localparam VGA_V_SYNC = 6;
localparam VGA_V_BACK_PORCH = 23;
localparam VGA_POLARITY = 0;

// 1024x768
// localparam VGA_H_FRONT_PORCH = 24;
// localparam VGA_H_SYNC = 136;
// localparam VGA_H_BACK_PORCH = 160;
// localparam VGA_V_FRONT_PORCH = 9;
// localparam VGA_V_SYNC = 6;
// localparam VGA_V_BACK_PORCH = 23;
// localparam VGA_POLARITY = 1;

// 640x480
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

assign vga_hsync = hsync ^ VGA_POLARITY;
assign vga_vsync = vsync ^ VGA_POLARITY;

// horizontal: 1344 pixels total
// display 1024 pixels per line
reg hblank,vblank;
wire hsyncon,hsyncoff,hreset,hblankon;
assign hblankon = (hcount == VGA_WIDTH-1);
assign hsyncon = (hcount == VGA_WIDTH+VGA_H_FRONT_PORCH-1);
assign hsyncoff = (hcount == VGA_WIDTH+VGA_H_FRONT_PORCH+VGA_H_SYNC-1);
assign hreset = (hcount == VGA_H_TOT-1);

// vertical: 806 lines total
// display 768 lines
wire vsyncon,vsyncoff,vreset,vblankon;
assign vblankon = hreset & (vcount == VGA_HEIGHT-1);
assign vsyncon = hreset & (vcount == VGA_HEIGHT+VGA_V_FRONT_PORCH-1);
assign vsyncoff = hreset &
	(vcount == VGA_HEIGHT+VGA_V_FRONT_PORCH+VGA_V_SYNC-1);
assign vreset = hreset & (vcount == VGA_V_TOT-1);

// sync and blanking
wire next_hblank,next_vblank;
assign next_hblank = hreset ? 0 : hblankon ? 1 : hblank;
assign next_vblank = vreset ? 0 : vblankon ? 1 : vblank;
always @(posedge vclock) begin
  hcount <= hreset ? 0 : hcount + 1;
  hblank <= next_hblank;
  hsync <= hsyncon ? 0 : hsyncoff ? 1 : hsync;  // active low

  vcount <= hreset ? (vreset ? 0 : vcount + 1) : vcount;
  vblank <= next_vblank;
  vsync <= vsyncon ? 0 : vsyncoff ? 1 : vsync;  // active low

  blank <= next_vblank | (next_hblank & ~hreset);
end

endmodule
