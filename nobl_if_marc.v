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

// [Original nobl_if was] Tested against an IDT 71v65603s150 in simulation and a Cypress 7C1356C in the real world.

module nobl_if_marc
	#(parameter WIDTH=18, DEPTH=19)  
	(
		input clk,
		input rst,
		
		input [WIDTH-1:0] RAM_D_pi,
		output reg [WIDTH-1:0] RAM_D_po,
		output reg RAM_D_poe,
		output reg [DEPTH-1:0] RAM_A,
		output reg RAM_WEn,
		output RAM_CENn,
		output RAM_LDn,
		output RAM_OEn,
		output reg RAM_CE1n,
		
		input [DEPTH-1:0] address,
		input [WIDTH-1:0] write_data,
		output reg [WIDTH-1:0] read_data,
		output reg read_data_valid,
		input write,
		input read
	);
   
	reg 	   			read_pipe1, read_pipe2, read_pipe3;
	reg 		   		write_pipe1, write_pipe2;
	reg [WIDTH-1:0] 	write_data_pipe1, write_data_pipe2;
	
	wire [DEPTH-1:0] 	address_gray;	// gray code the address to reduce EMI
	
	bin2gray #(.WIDTH(DEPTH)) bin2gray (.bin(address),.gray(address_gray));
   
	assign 				RAM_CENn = 1'b0;
	assign 	   			RAM_LDn = 0;
	assign 	   			RAM_OEn = 0;	// ZBT/NoBL RAM manages its own output enables.

	//
	// Pipeline stage 1.		Address is set here.
	//
	always @(posedge clk)
		if (rst) begin
			RAM_WEn <= 1;
			RAM_CE1n <= 1;
			RAM_A <= 0;
			read_pipe1 <= 0;
			write_pipe1 <= 0;
			write_data_pipe1  <= 0;
		end
		else begin
			RAM_CE1n <= ~(read ^ write);	// Can't do both at the same time. Does neither if attempted.
			RAM_WEn <= ~(write & ~read);
			RAM_A <= address_gray;
			read_pipe1 <= read & ~write;
			write_pipe1 <= write & ~read;
			write_data_pipe1 <= write_data;
		end
	
	//
	// Pipeline stage2.			Only delay happens here.
	//
	always @(posedge clk)
		if (rst) begin
			read_pipe2 <= 0;
			write_pipe2 <= 0;   
			write_data_pipe2 <= 0;
		end
		else begin
			read_pipe2 <= read_pipe1;
			write_pipe2 <= write_pipe1;
			write_data_pipe2 <= write_data_pipe1;
		end
	
	//
	// Pipeline stage3.		Writes happen here.
	//
	always @(posedge clk)
		if (rst) begin
			RAM_D_po <= 0;
			RAM_D_poe <= 1;	
			read_pipe3 <= 0;
		end
		else begin
			RAM_D_po <= write_data_pipe2;
			RAM_D_poe <= ~write_pipe2;
			read_pipe3 <= read_pipe2;
		end
	
	//
	// Pipeline stage4.		Reads happen here.
	//
	always @(posedge clk)
		if (rst) begin
			read_data_valid <= 0;
			read_data <= 0;
		end
		else begin
			read_data <= RAM_D_pi;
			read_data_valid <= read_pipe3;
		end
	
endmodule // nobl_if_marc
