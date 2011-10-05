//
// Copyright 2011 Horizon Analog, Inc.
// Base upon works Copyright 2011 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

module vita_rx_control_no_decimation
  #(parameter BASE=0,
    parameter WIDTH=32)
   (input clk, input reset, input clear,
    input set_stb, input [7:0] set_addr, input [31:0] set_data,
    
    input [63:0] vita_time,
    output overrun,

    // To vita_rx_framer
    output [5+64+WIDTH-1:0] sample_fifo_o,
    output sample_fifo_src_rdy_o,
    input sample_fifo_dst_rdy_i,
    
    // From DSP Core
    input [WIDTH-1:0] sample,
    output run,
    
	// External RAM
	input [35:0] RAM_D_pi,
	output [35:0] RAM_D_po,
	output RAM_D_poe,   
	output [20:0] RAM_A,
	output RAM_CE1n,
	output RAM_CENn,
	output RAM_WEn,
	output RAM_OEn,
	output RAM_LDn,
	
    output [31:0] debug_rx
    );

   localparam IBS_IDLE = 0;
   localparam IBS_WAITING = 1;
   localparam IBS_FILLING = 2;
   localparam IBS_DRAINING = 3;
   localparam IBS_OVERRUN = 4;			// Can't happen in this version.
   localparam IBS_BROKENCHAIN = 5;
   localparam IBS_LATECMD = 6;
   localparam IBS_ZEROLEN = 7;			// Can't happen in this version.
   
   wire [63:0] 	  	new_time;
   wire [31:0] 	  	new_command;
   wire 	  		sc_pre1;

   wire [63:0] 	  	rcvtime_pre;
   reg [63:0] 	  	rcvtime;
   wire [27:0] 	  	numlines_pre;
   wire 	  send_imm_pre, chain_pre, reload_pre, stop_pre;	// chain_pre, reload_pre, stop_pre are not used because chaining and continuous mode are not supported.
   reg 		  		send_imm;
   wire 	  		read_command_now, command_is_available, queue_command_now;
   reg 		  		sc_pre2;
   reg [27:0] 	  	lines_left;
   reg [2:0] 	  	ibs_state;
   wire 	  		now, early, late;
   wire 	  		sample_fifo_in_rdy;
   
   assign read_command_now = (ibs_state == IBS_IDLE) & command_is_available;
   
   setting_reg #(.my_addr(BASE)) sr_cmd
     (.clk(clk),.rst(reset),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out(new_command),.changed());

   setting_reg #(.my_addr(BASE+1)) sr_time_h
     (.clk(clk),.rst(reset),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out(new_time[63:32]),.changed());
   
   setting_reg #(.my_addr(BASE+2)) sr_time_l
     (.clk(clk),.rst(reset),.strobe(set_stb),.addr(set_addr),
      .in(set_data),.out(new_time[31:0]),.changed(sc_pre1));
   
   // FIFO to store commands sent from the settings bus
   always @(posedge clk)
     if(reset | clear)
       sc_pre2 <= 0;
     else
       sc_pre2 <= sc_pre1;
   
   assign      queue_command_now  = sc_pre1 & ~sc_pre2;

   wire [4:0]  command_queue_len;

   fifo_short #(.WIDTH(96)) commandfifo
     (.clk(clk),.reset(reset),.clear(clear),
      .datain({new_command,new_time}), .src_rdy_i(queue_command_now), .dst_rdy_o(),
      .dataout({send_imm_pre,chain_pre,reload_pre,stop_pre,numlines_pre,rcvtime_pre}),
      .src_rdy_o(command_is_available), .dst_rdy_i(read_command_now),
      .occupied(command_queue_len), .space() );
   
	// SRAM fifo to hold full-rate samples.
   
	wire [35:32]	unused_out;
	wire [WIDTH-1:0] sample_from_sram;
	reg [63:0] 		sample_time;
	wire			sram_read_complete;
	reg [17:0]		sram_address;
	wire			at_end_of_sram = &sram_address;
	wire			fill_sram = (ibs_state == IBS_FILLING);
	wire			drain_sram = (ibs_state == IBS_DRAINING);
	reg [1:0]		sram_pipeline_counter;	// Must count same as number of read stages in nobl_if_marc.
	wire			sram_pipeline_full = &sram_pipeline_counter;
	
	assign RAM_A[20:18] = 3'b000;
	
	nobl_if_marc  #(.WIDTH(36),.DEPTH(18)) nobl_if_marc
		(   
			.clk(clk),
			.rst(reset),
			
			.RAM_D_pi(RAM_D_pi),
			.RAM_D_po(RAM_D_po),
			.RAM_D_poe(RAM_D_poe),
			.RAM_A(RAM_A[17:0]),
			.RAM_WEn(RAM_WEn),
			.RAM_CENn(RAM_CENn),
			.RAM_LDn(RAM_LDn),
			.RAM_OEn(RAM_OEn),
			.RAM_CE1n(RAM_CE1n),
			
			.address(sram_address),
			
			.write_data({4'b0000, sample}),				// "out" to SRAM.
			
			.read_data({unused_out, sample_from_sram}),	// "in" from SRAM.
			.read_data_valid(sram_read_complete),
			
			.write(fill_sram),
			.read(drain_sram)
		);

   wire signal_cmd_done     = (lines_left == 1);
   wire signal_overrun 	    = (ibs_state == IBS_OVERRUN);
   wire signal_brokenchain  = (ibs_state == IBS_BROKENCHAIN);
   wire signal_latecmd 	    = (ibs_state == IBS_LATECMD);
   wire signal_zerolen 	    = (ibs_state == IBS_ZEROLEN);

   // Buffer of samples for while we're writing the packet headers
   wire [4:0] flags = {signal_zerolen,signal_overrun,signal_brokenchain,signal_latecmd,signal_cmd_done};

   wire       need_to_write	= (drain_sram & sram_read_complete & sram_pipeline_full) 
   				 			| signal_overrun | signal_brokenchain | signal_latecmd | signal_zerolen;
   
   fifo_short #(.WIDTH(5+64+WIDTH)) rx_sample_fifo
     (.clk(clk),.reset(reset),.clear(clear),
      .datain({flags,sample_time,sample_from_sram}), .src_rdy_i(need_to_write), .dst_rdy_o(sample_fifo_in_rdy),
      .dataout(sample_fifo_o), 
      .src_rdy_o(sample_fifo_src_rdy_o), .dst_rdy_i(sample_fifo_dst_rdy_i),
      .space(), .occupied() );
   
   time_compare time_compare (.time_now(vita_time), .trigger_time(rcvtime), .now(now), .early(early), .late(late));
   
   wire go_now 		    = now | send_imm;
   
   reg 	too_late;

   always @(posedge clk)
     if(reset | clear)
       too_late <= 0;
     else
       too_late <= late & ~send_imm;

   reg 	late_valid;
   
	always @(posedge clk)
	begin
		if (reset | clear) begin
			ibs_state 	   <= IBS_IDLE;
			lines_left 	   <= 0;
			rcvtime 	   <= 0;
			send_imm 	   <= 0;
			sram_address   <= 0;
			late_valid      <= 0;
		end
		else begin
			case (ibs_state)
				IBS_IDLE :
					begin
						if (command_is_available) begin
							lines_left <= numlines_pre;
							rcvtime <= rcvtime_pre;
							send_imm <= send_imm_pre;
							late_valid <= 0;
							ibs_state <= IBS_WAITING;
						end
					end
				IBS_WAITING :
					begin
						late_valid <= 1;
						if(late_valid)
							if (go_now) begin
								sample_time <= vita_time;
								ibs_state <= IBS_FILLING;
								sram_address <= 0;
							end
							else if (too_late)
								ibs_state <= IBS_LATECMD;
					end
				IBS_FILLING :
					begin
						if (at_end_of_sram) begin
							ibs_state <= IBS_DRAINING;
							sram_pipeline_counter <= 0;
						end
						sram_address <= sram_address + 1;	// Wraps around to zero at_end_of_sram.
					end
				IBS_DRAINING :
					// To prevent ambiguities when forwarding samples out of the SRAM, we
					// only advance to the next address after we've completely filled the
					// SRAM read pipeline and rx_sample_fifo is ready for more.
					// The samples "trickle" out at the same rate as decimation 4 because the
					// read pipeline has 4 stages.
					if (sram_pipeline_full) begin
						if (sram_read_complete & sample_fifo_in_rdy) begin
							sram_pipeline_counter <= 0;
							lines_left 	    <= lines_left - 1;
							sram_address	<= sram_address + 1;
							if ((lines_left == 1) | at_end_of_sram)
								ibs_state      <= IBS_IDLE;
							else if (command_is_available)
								ibs_state      <= IBS_BROKENCHAIN;
						end
					end
					else begin
						sram_pipeline_counter <= sram_pipeline_counter + 1;
					end
				IBS_OVERRUN :
					if(sample_fifo_in_rdy)
						ibs_state <= IBS_IDLE;
				IBS_LATECMD :
					if(sample_fifo_in_rdy)
						ibs_state <= IBS_IDLE;
				IBS_BROKENCHAIN :
					if(sample_fifo_in_rdy)
						ibs_state <= IBS_IDLE;
				IBS_ZEROLEN :
					if(sample_fifo_in_rdy)
						ibs_state <= IBS_IDLE;
			endcase // case(ibs_state)
		end
   end
   
   assign overrun 			= (ibs_state == IBS_OVERRUN);
   assign run 				= fill_sram;

   assign debug_rx = { { ibs_state[2:0], command_queue_len },
		       { 8'd0 },
		       { go_now, too_late, run, 1'b0, read_command_now, queue_command_now, 1'b0, ~command_is_available },
		       { 2'b0, overrun, chain_pre, sample_fifo_in_rdy, need_to_write, sample_fifo_src_rdy_o,sample_fifo_dst_rdy_i} };
   
endmodule // rx_control
