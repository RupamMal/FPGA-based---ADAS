// (C) 2001-2017 Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License Subscription 
// Agreement, Intel MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Intel and sold by 
// Intel or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// (C) 2001-2013 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.

 
// $Id: //acds/rel/13.1/ip/.../avalon-st_data_format_adapter.sv.terp#1 $
// $Revision: #1 $
// $Date: 2013/09/21 $
// $Author: dmunday $


// --------------------------------------------------------------------------------
//| Avalon Streaming Data Adapter
// --------------------------------------------------------------------------------

`timescale 1ns / 100ps

// ------------------------------------------
// Generation parameters:
//   output_name:        Video_System_avalon_st_adapter_data_format_adapter_0
//   usePackets:         true
//   hasInEmpty:         false
//   inEmptyWidth:       0
//   hasOutEmpty:        true 
//   outEmptyWidth:      1
//   inDataWidth:        8
//   outDataWidth:       8
//   channelWidth:       0
//   inErrorWidth:       0
//   outErrorWidth:      0
//   inSymbolsPerBeat:   1
//   outSymbolsPerBeat:  1
//   maxState:           1
//   stateWidth:         1
//   maxChannel:         0
//   symbolWidth:        8
//   numMemSymbols:      1
//   symbolWidth:        8


// ------------------------------------------

 
module Video_System_avalon_st_adapter_data_format_adapter_0 (
 // Interface: in
 output reg         in_ready,
 input              in_valid,
 input [8-1 : 0]    in_data,
 input              in_startofpacket,
 input              in_endofpacket,
 // Interface: out
 input                out_ready,
 output reg           out_valid,
 output reg [8-1: 0]  out_data,
 output reg           out_startofpacket,
 output reg           out_endofpacket,
 output reg           out_empty,

  // Interface: clk
 input              clk,
 // Interface: reset
 input              reset_n

);


   always @* begin
      in_ready = out_ready; 
      out_valid = in_valid;
      out_data = in_data;
      out_startofpacket = in_startofpacket;
      out_endofpacket = in_endofpacket;
      out_empty = 0;
   end

endmodule

   

