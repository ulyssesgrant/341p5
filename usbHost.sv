`default_nettype none

// Write your usb host here.  Do not modify the port list.
module usbHost
  (input logic clk, rst_L, 
  usbWires wires);
 
  /* Tasks needed to be finished to run testbenches */

  task prelabRequest
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  (input bit  [15:0] data);

  logic [10:0] token = 11'b0010_1010000;
  logic [7:0] sync = 8'b0000_0001;
  logic [7:0] pid = 8'b0111_1000;

  ld_sync = 1;
  ld_pid = 1;
  ld_tok = 1;
  @(posedge clk);
  ld_sync = 0;
  ld_pid = 0;
  ld_tok = 0;
  sel_1 = 1;
  sel_2 = 0;

  //begin sending sync
  @(posedge clk);
  en_sync = 1;
  repeat (7) @(posedge clk);

  //begin sending pid_~pid
  en_sync = 0;
  sel_1 = 0;
  @(posedge clk);
  en_pid = 1;
  repeat (7) @(posedge clk);
  en_pid = 0;
  sel_2 = 1;

  //begin sending crc
  en_crc = 1;
  @(posedge clk);
  en_crc = 0;
  @(posedge clk);
  en_tok = 1;
  repeat (10) @(posedge clk);
  en_tok = 0;
  //5 more clock cycles for crc remainder
  repeat (5) @(posedge clk);

  //begin sending eop
  do_eop = 1;
  repeat (3) @(posedge clk);

  endtask: prelabRequest


  task readData
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  (input  bit [15:0]  mempage, // Page to write
   output bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: readData

  task writeData
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  (input  bit [15:0]  mempage, // Page to write
   input  bit [63:0] data, // array of bytes to write
   output bit        success);

  endtask: writeData

  // usbHost starts here!!
logic nrzi_in, nrzi_out,clear, wiresDP, wiresDM;
logic stuffer_in, stuffer_out, pause, crc_in, crc_out, en_crc, sync_out, pid_out, sync_pid_out;
logic ld_tok, en_tok, ld_sync, en_sync, ld_pid, en_pid;
logic [10:0] sr_in;


//implement enable_send as output of protocol_fsm
assign wires.DP = enable_send ? wiresDP : 1'bz;
assign wires.DM = enable_send ? wiresDM : 1'bz;

//CRC5 here!
sender crcSender(crc_in, (rst_L||en_crc), clk, pause, crc_out);
//for now: crcSender's output is tied to bitstuffer's input, but should implement mux with crc16's output later!
assign stuffer_in = crc_out;

//shift register to hold the token as it's sent to crc
shiftRegister #(11) shiftRegToken(clk, rst_L, ld_tok, en_tok, pause, token, crc_in);

//shift register to hold sync
shiftRegister #(8) shiftRegSync(clk, rst_L, ld_sync, en_sync, pause, sync, sync_out);

//shift register to hold pid
shiftRegister #(8) shiftRegPid(clk, rst_L, ld_pid, en_pid, pause, pid, pid_out);


///////////////////////////////////////////////////////////////
stuffer   bitstuff(stuffer_in, rst_l, stuffer_out, pause);  // stuff addr, endp,crc5,crc16, and DATA
///////////////////////////////////////////////////////////////
 //mux in sync, pid 

//mux for selecting between sync or pid
assign sync_pid_out = sel_1 ? sync_out : pid_out;

//mux for NRZI
assign nrzi_in = sel_2 ? stuffer_out : sync_pid_out;


  // %%%%%%%%%%%%%%%%%%%%%%%
  // need work %%%%%%%%%%%%%%%%%%
  //%%%%%%%%%%%%%%%%%%%%%%%%
  
////////////////////////////////////////////////////////////////
nrzi    flip(nrzi_in, start, rst_l, clear, nrzi_out);  
////////////////////////////////////////////////////////////////
  // small handler to send out DP and DM. use do_eop to control between NRZI output and EOP.
 logic [2:0] counter_dpdm; 
always_comb begin   // sending DP and DM
  if(do_eop) {wiresDP,wiresDM} = 2'b00;                             // deal with EOP
  else if (counter_dpdm == 3'd1) {wiresDP,wiresDM} = 2'b00;
  else if (counter_dpdm == 3'd2) {wiresDP,wiresDM} = 2'b10;
  else {wiresDP,wiresDM} = {nrzi_out,~nrzi_out};                 // go back to output from NRZI
end
always_ff @(posedge clk, negedge rst_l) begin
	if(~rst_l) counter_dpdm <= 3'd0;
	else if( do_eop ) counter_dpdm <= 3'd0;
	else counter_dpdm <= counter_dpdm +3'd1;
end
endmodule: usbHost


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       modified CRC 5 from hw2                                          //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module  sender(input logic bit_in, rst_l, clk, pause,  //pause for bit stuffing
						output logic send_bit);
	logic [4:0] Q;
	logic go;
	logic mux;
	logic out_bit;

	assign send_bit = (mux) ?out_bit : bit_in; // mux output between incoming bit and complement
	
	senderFSM sendit(clk,rst_l,pause,mux,go);
	crcCal calcIt(send_bit,clk,rst_l,pause, Q);
	complementMake make(rst_l, go, clk, pause,Q,out_bit);
	
endmodule: sender

module senderFSM( input logic clk, rst_l, pause,
							  output logic mux,go);

	logic [4:0] counter;
	enum logic [2:0]  { FIRST, DATA,COMP, DEAD} cs,ns;

always_comb begin
	go =1'b0;
	mux=1'b0; // mux =1 => shifting out COMP
	case(cs)
		FIRST: begin
			ns = DATA;
			end
		DATA: begin
			if(counter >= 5'd11) begin
				ns = COMP;
				go =1'b1;
				mux =1'b1;
				end
			else
				ns= DATA;
			end
		COMP:begin
		        if(counter>=5'd15) begin
					ns = DEAD;
					mux=1'b1;
					end
				else begin
					mux =1'b1;
					ns =COMP;
				end
			end
		DEAD: ns = DEAD;
	endcase
end

always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <= FIRST;
			counter <= 5'b0;
			end
		else if(pause)begin
			cs <= cs; // stall the process by one clock
			counter<= counter;
		end
		else begin
			cs<= ns;
			counter <= counter + 5'b00001;
			end
	end
endmodule:senderFSM

module  crcCal(input logic bit_in,clk,rst_l, pause, output logic [4:0] Q);

always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l)
			Q <= 5'b11111;
		else if(pause) Q <= Q; //stall for one clock.
		else begin
			Q[0] <= bit_in ^ Q[4];
			Q[1] <=	Q[0];
			Q[2] <= (bit_in ^ Q[4] ) ^ Q[1];
			Q[3] <= Q[2];
			Q[4] <= Q[3];
		end
end
endmodule: crcCal

module complementMake(input logic rst_l, go, clk, pause,
									  input logic [4:0] Q,
										output logic oneBit);
		logic [3:0] remainder;
		
		always_comb begin // the first clock just output inverted MSB value of remainder
			if(go) oneBit = ~Q[4];
			else oneBit = remainder[3]; // for subsequent clocks take output from shift register.
		end
		always_ff@(posedge clk, negedge rst_l) begin
			if(~rst_l)
				remainder <= 4'b0;
			else if(pause) remainder <= remainder; //stall for one clock.
			else if(go)
				remainder <= ~(Q[3:0]); // need to hold 4 bits since the first one is sent out on the clock
			else begin
				remainder[3:1]<= remainder[2:0];
				remainder[0] <= 1'b0;
				end
		end
endmodule: complementMake


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                           Bit Stuffing                                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module stuffer(input logic bit_in, rst_l,
					 output logic bit_out, pause);  // stuff addr, endp,crc5,crc16, and DATA

	logic [2:0] count;
always_comb begin
if(count ==3'd6)begin // found 6 ones, stuff a 0, pause for 1 clock
	pause =1'd1;
	bit_out =1'd0;
end
else begin
	pause =1'd0;
	bit_out = bit_in;
end
end
 
always_ff @(posedge clk, negedge rst_l)begin
	if(~rst_l) count <= 0;   //reset
	else if (clear) count<=0;  // count up to 6 or found a zero. clear
	else if(bit_in) count <= count +3'd1; // keep counting.
	else count <=0;   //  found a 0, clear.
	end
endmodule:stuffer

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                              NRZI                                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// assume first one is 1;

module nrzi(input logic bit_in, start, rst_L, clear,
				  output logic bit_out);  // everything except EOP
				  
logic past, clear;
always_comb begin

if(bit_in) bit_out = past;  // input is 1, don't invert
else bit_out =~past;      // input is 0 invert.

end
 
always_ff @(posedge clk, negedge rst_L) begin
	if(~rst_L) past <= 1'd1;
	else if(clear) past <= 1'd1;
	else past <= bit_out;
	end				  
endmodule

module shiftRegister
	#(parameter w = 11)
	(input logic clk, rst_L, ld, en, pause,
	 input logic [w-1:0] in,
	 output logic out);

	logic [w-1:0] val;

	always_ff @(posedge clk, negedge rst_L) begin
		if (rst)
			val <= w'd0;
			out <= 1'b0;
		else if (ld) begin
			val <= in;
			out <= val[w-1];
		end
		else if (en && !pause) begin
			val <= val << 1;
			out <= val[w-1];
		end
	end
endmodule: shiftRegister
