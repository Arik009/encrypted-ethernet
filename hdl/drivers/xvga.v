`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 10/02/2015 02:05:19 AM; comments added 7/24/2018
// Design Name:
// Module Name: xvga
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Revision:
// Revision 1.0 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// xvga: Generate XVGA display signals (1024 x 768 @ 60Hz)
// vga: 640x480 verilog is also included by commented out
//
////////////////////////////////////////////////////////////////////////////////


module xvga(
	input vclock,
	output reg [clog2(VGA_H_TOT):0] hcount,
	output reg [clog2(VGA_V_TOT):0] vcount,
	output reg vsync, hsync,
	output reg blank);

`include "params.vh"

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
