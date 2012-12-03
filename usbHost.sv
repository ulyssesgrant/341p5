`default_nettype none

//18-341 P5 Gun Charnmanee (gcharnma) Dylan Koenig (djkoenig)

// Write your usb host here.  Do not modify the port list.
module usbHost
  (input logic clk, rst_L, 
  usbWires wires);
 
  /* Tasks needed to be finished to run testbenches */

  task prelabRequest
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  (input bit  [15:0] data);

  usbHost.token = 11'b1010000_0010;
  usbHost.sync = 8'b0000_0001;
  usbHost.pid = 8'b1000_0111;
  usbHost.sel_3 <= 0; //Do CRC5

  usbHost.do_eop <= 0;
  usbHost.en_sync <= 0;
  usbHost.en_crc_L <= 1;
  usbHost.en_pid <= 0;
  usbHost.en_tok <= 0;
  usbHost.clear <= 1;

  usbHost.ld_sync <= 1;
  usbHost.ld_pid <= 1;
  usbHost.ld_tok <= 1;
  usbHost.sel_1 <= 1;
  usbHost.sel_2 <= 0;
  usbHost.en_crc_L <= 1; //turn off CRCs
  @(posedge clk);
  usbHost.enable_send <= 1;
  usbHost.clear <= 0;
  usbHost.en_sync <= 1;
  usbHost.ld_sync <= 0;
  usbHost.ld_pid <= 0;
  usbHost.ld_tok <= 0;

  @(posedge clk);
  //begin sending sync
  usbHost.en_sync <= 1;
  repeat (7) @(posedge clk);

  //begin sending pid_~pid
  usbHost.en_sync <= 0;
  usbHost.sel_1 <= 0;
  usbHost.en_pid <= 1;
  repeat (7) @(posedge clk);
  @(posedge clk);
  usbHost.en_pid <= 0;
  usbHost.sel_2 <= 1;

  //begin sending crc
  usbHost.en_crc_L <= 0; // turn on CRC
  usbHost.en_tok <= 1;
  @(posedge clk);
  //usbHost.en_crc_L <= 1;
  repeat (9) @(posedge clk);
  usbHost.en_tok <= 0;
  // off by one?
  //5 more clock cycles for crc remainder
  repeat (5) @(posedge clk);

  //begin sending eop
  @(posedge clk);
  usbHost.en_crc_L <= 1;
  usbHost.do_eop <= 1;
  repeat (2) @(posedge clk);
  usbHost.do_eop <= 0;
  @(posedge clk);
  
  usbHost.enable_send <= 0;

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
logic nrzi_in, nrzi_out, clear, start, wiresDP, wiresDM;
logic stuffer_in, stuffer_out, pause, crc_in, crc16_in, crc_out, crc16_out, en_crc_L, sync_out, pid_out, sync_pid_out;
logic ld_tok, en_tok, ld_sync, en_sync, ld_pid, en_pid, ld_data, en_data, enable_send, do_eop;
logic sel_1, sel_2, sel_3; //sel_1 for sync or pid, sel_2 for nrzi input, sel_3 for crc16 or crc
logic [10:0] sr_in, token;
logic [63:0] data;
logic [7:0] sync, pid;
logic clear_sender;

assign clear_sender = en_crc_L;
//implement enable_send as output of protocol_fsm
assign wires.DP = enable_send ? wiresDP : 1'bz;
assign wires.DM = enable_send ? wiresDM : 1'bz;

///////////////////////////////CRC machines
sender crcSender(crc_in, rst_L, clk, clear_sender ,pause, crc_out);
sender16 crcSender16(crc16_in, rst_L, clk, clear_sender ,pause, crc16_out);
//for now: crcSender's output is tied to bitstuffer's input, but should implement mux with crc16's output later!
assign stuffer_in = sel_3 ? crc16_out : crc_out;

//shift register to hold the token as it's sent to crc
shiftRegister #(11) shiftRegToken(clk, rst_L, ld_tok, en_tok, pause, token, crc_in);

//shift register to hold sync
shiftRegister #(8) shiftRegSync(clk, rst_L, ld_sync, en_sync, 1'd0, sync, sync_out);

//shift register to hold pid
shiftRegister #(8) shiftRegPid(clk, rst_L, ld_pid, en_pid, 1'd0, pid, pid_out);
//shift register to hold DATA
shiftRegister #(64) shiftRegData(clk, rst_L, ld_data, en_data, pause, data, crc16_in);


///////////////////////////////////////////////////////////////
stuffer   bitstuff(stuffer_in, clk, rst_L, clear, stuffer_out, pause);  // stuff addr, endp,crc5,crc16, and DATA
///////////////////////////////////////////////////////////////
 //mux in sync, pid 

//mux for selecting between sync or pid
assign sync_pid_out = sel_1 ? sync_out : pid_out;

//mux for NRZI
assign nrzi_in = sel_2 ? stuffer_out : sync_pid_out;
  
////////////////////////////////////////////////////////////////
nrzi    flip(nrzi_in, start, clk, rst_L, clear, nrzi_out);  
////////////////////////////////////////////////////////////////
  // small handler to send out DP and DM. use do_eop to control between NRZI output and EOP.
 logic [2:0] counter_dpdm; 
always_comb begin   // sending DP and DM
	if (do_eop) begin
		if (counter_dpdm == 3'd2) begin
			{wiresDP,wiresDM} = 2'b10;
		end
		else {wiresDP,wiresDM} = 2'b00;
	end
	else {wiresDP,wiresDM} = {nrzi_out,~nrzi_out};                 // go back to output from NRZI
end
always_ff @(posedge clk, negedge rst_L) begin
	if(~rst_L) counter_dpdm <= 3'd0;
	else if(do_eop) counter_dpdm <= counter_dpdm +  3'd1;
	else counter_dpdm <= 3'd0;
end
endmodule: usbHost


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       modified CRC 5 from hw2                                          //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module  sender(input logic bit_in, rst_l, clk, clear, pause,  //pause for bit stuffing
						output logic send_bit);
	logic [4:0] Q;
	logic go;
	logic mux;
	logic out_bit;

	assign send_bit = (mux) ?out_bit : bit_in; // mux output between incoming bit and complement
	
	senderFSM sendit(clk,rst_l,clear,pause,mux,go);
	crcCal calcIt(send_bit,clk,rst_l,clear,pause, Q);
	complementMake make(rst_l, go, clk,clear, pause,Q,out_bit);
	
endmodule: sender

module senderFSM( input logic clk, rst_l, clear, pause,
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
		else if(clear) begin
			cs<=FIRST;
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

module  crcCal(input logic bit_in,clk,rst_l, clear, pause, output logic [4:0] Q);

always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l)
			Q <= 5'b11111;
		else if(clear) Q<= 5'b11111;
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

module complementMake(input logic rst_l, go, clk, clear, pause,
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
			else if (clear ) remainder <=4'b0;
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
//                                                           CRC16                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module  sender16(input logic bit_in, rst_l, clk, clear, pause,  //pause for bit stuffing
						output logic send_bit);
	logic [15:0] Q;
	logic go;
	logic mux;
	logic out_bit;

	assign send_bit = (mux) ?out_bit : bit_in; // mux output between incoming bit and complement
	
	senderFSM16 sendit(clk,rst_l, clear, pause,mux,go);
	crcCal16 calcIt(send_bit,clk,rst_l, clear, pause,Q);
	complementMake16 make(rst_l,go,clk, clear, pause,Q,out_bit);
	
endmodule: sender16

module senderFSM16( input logic clk, rst_l, clear, pause,
							  output logic mux,go);

	logic [6:0] counter;
	enum logic [2:0]  {FIRST,DATA,COMP,DEAD} cs,ns;

always_comb begin
	go =1'b0;
	mux=1'b0; // mux =1 => shifting out COMP
	case(cs)
		FIRST: begin
			ns = DATA;
			end
		DATA: begin
			if(counter >= 7'd64) begin
				ns = COMP;
				go = 1'b1;
				mux = 1'b1;
			end
			else
				ns= DATA;
			end
		COMP:begin
		        if(counter >= 7'd80) begin
					ns = DEAD;
					mux = 1'b1;
				end
				else begin
					mux = 1'b1;
					ns = COMP;
				end
			end
		DEAD: ns = DEAD;
	endcase
end

always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <= FIRST;
			counter <= 7'd0;
			end
		else if( clear) begin
			cs<= FIRST;
			counter<= 7'd0;
		end
		else if(pause)begin
			cs <= cs; // stall the process by one clock
			counter<= counter;
		end
		else begin
			cs<= ns;
			counter <= counter + 7'd1;
			end
	end
endmodule:senderFSM16

module  crcCal16(input logic bit_in,clk,rst_l, clear, pause, output logic [15:0] Q);

always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l)
			Q <= 16'b1111_1111_1111_1111;
		else if(clear) Q<= 16'hFFFF;
		else if(pause) Q <= Q; //stall for one clock.
		else begin
			Q[0] <= bit_in^Q[15];
			Q[1] <= Q[0];
			Q[2] <= (bit_in^Q[15])^Q[1];
			Q[3] <= Q[2];
			Q[4] <= Q[3];
			Q[5] <= Q[4];
			Q[6] <= Q[5];
			Q[7] <= Q[6];
			Q[8] <= Q[7];
			Q[9] <= Q[8];
			Q[10] <= Q[9];
			Q[11] <= Q[10];
			Q[12] <= Q[11];
			Q[13] <= Q[12];
			Q[14] <= Q[13];
			Q[15] <= (bit_in^Q[15])^Q[14];
		end
end
endmodule: crcCal16

module complementMake16(input logic rst_l, go, clk, clear, pause,
									  input logic [15:0] Q,
										output logic oneBit);
		logic [14:0] remainder;
		
		always_comb begin // the first clock just output inverted MSB value of remainder
			if(go) oneBit = ~Q[15];
			else oneBit = remainder[14]; // for subsequent clocks take output from shift register.
		end
		always_ff@(posedge clk, negedge rst_l) begin
			if(~rst_l)
				remainder <= 15'd0;
			else if(clear) remainder <=15'd0;
			else if(pause) remainder <= remainder; //stall for one clock.
			else if(go)
				remainder <= ~(Q[14:0]); // need to hold 15 bits since the first one is sent out on the clock
			else begin
				remainder[14:1] <= remainder[13:0];
				remainder[0] <= 1'b0;
				end
		end
endmodule: complementMake16
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                           Bit Stuffing                                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module stuffer(input logic bit_in, clk, rst_l, clear,
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

module nrzi(input logic bit_in, start, clk, rst_L, clear,
				  output logic bit_out);  // everything except EOP
				  
logic past;
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
	assign out = val[w-1];

	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			val <= 'd0;
		end
		else if (ld) begin
			val <= in;
		end
		else if (en && !pause) begin
			val <= val << 1;
		end
	end
endmodule: shiftRegister

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            SEND TOKEN FSM                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module send_token(input logic clk, rst_l, start, pause,
							  input logic [10:0]  token_in,
							 input logic [15:0] in_reg,
							 output logic do_eop,en_sync,en_crc_L, en_pid, en_tok, clear, ld_sync, ld_pid,ld_tok, sel_1,sel_2,enable_send,
							 output logic [10:0] token_out,
							 output logic done); // done signal sends to above.

	logic [4:0] sync_count, pid_count,token_count, eop_count;
	logic [4:0] sync_add,pid_add, token_add, eop_add;
	enum logic [2:0]  {IDLE, SYNC, PID, TOKE, EOP} cs,ns;
	logic clear_counter;
assign token_out = token_in;
always_comb begin
	sync_add = 5'b0;
	pid_add =5'b0;
	token_add = 5'b0;
	eop_add = 5'b0;
	clear_counter=1'b0;
	do_eop = 1'b0;
	en_sync =1'b0;
	en_crc_L = 1'b1;// has CRC off.
	en_pid = 1'b0;
	en_tok = 1'b0;
	clear = 1'b1;
	ld_sync =1'b0;   //
	ld_pid = 1'b0;    //
	ld_tok = 1'b0;    //
	sel_1 =1'b0;      //
	sel_2 =1'b0;     //
	enable_send = 1'b0;
	case(cs)
		IDLE :begin
				if(start) begin
					ns = SYNC;
					ld_sync <= 1'b1;
					ld_pid <=1'b1;
					ld_tok <= 1'b1;
					sel_1 <=1'b1;
					sel_2<=1'b0;
					end
				else ns = IDLE;    // wait for start signal. 
		end
		SYNC: begin
			en_sync = 1'b1;
			sel_1 = 1'b1;
			sel_2 = 1'b0;
			if(sync_count < 5'b7)begin
				ns = SYNC;	
			end
			else ns = PID; 
		end
		PID: begin
			en_pid = 1'b1;
			sel_1 = 1'b0;
			se_2  = 1'b0;
			if(pid_count<5'b7) begin
				ns = PID;
			end
			else ns = TOKE;
		end
		TOKE:begin
			sel_1 = 1'b0;
			sel_2 = 1'b1;
			en_crc_L = 1'b0;
			if(token_count <5'10) en_tok =1'b1;
			else en_tok = 1'b0;
			if(token_count < 5'b15) begin
			ns = TOKE;
			end
			else ns = EOP;
		end
		EOP: begin
			en_crc_L =1'b1;
			do_eop  1'b1;
			if(eop_count <5'b2) begin
				ns = EOP;
			end
			else begin
				ns = IDLE;
				clear_counter =1'b1;
			end
		end
	endcase
end

	always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <=IDLE;
			sync_count <= 5'b0;
			pid_count <= 5'b0;
			token_count <= 5'b0;
			eop_count <= 5'b0;
		end
		else if(clear_counter) begin
			cs <=ns;
			sync_count <= 5'b0;
			pid_count <= 5'b0;
			token_count <= 5'b0;
			eop_count <= 5'b0;
		end
		else if(pause) begin
			cs <=cs;
			sync_count <= sync_count;
			pid_count <= pid_count;
			token_count <= token_count;
			eop_count <= eop_count;
		end
		else 
			cs <=ns;
			sync_count <= sync_count + sync_add;
			pid_count <= pid_count +pid_add;
			token_count <= token_count +token_add;
			eop_count <= eop_count + eop_add;
	end
endmodule: send_token
//////////////////////////////////////////////////////////////////////////////////////////////////////////
