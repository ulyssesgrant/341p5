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
  usbHost.data_out =64'hDEADBEEF1987CAFE;
  usbHost.sync = 8'b0000_0001;
  usbHost.mode = 2'b0;
  usbHost.pid = 8'b1000_0111;
  
  /*
  usbHost.mode = 2'd0;
  usbHost.start_send_token <=1'b1;
  wait(usbHost.done_send_token);
  usbHost.start_send_token<= 1'b0;
  */

   usbHost.mode = 2'd1;
  usbHost.start_send_data <=1'b1;
  wait(usbHost.done_send_data);
  usbHost.start_send_data<= 1'b0;
  
  /*
   usbHost.mode = 2'd2;
  usbHost.start_send_hand <=1'b1;
  wait(usbHost.done_send_hand);
  usbHost.start_send_hand<= 1'b0;
  */
  
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
logic ld_tok, en_tok, ld_sync, en_sync, ld_pid, en_pid, enable_send, do_eop;
logic sel_1, sel_2, sel_3; //sel_1 for sync or pid, sel_2 for nrzi input, sel_3 for crc16 or crc5
logic [10:0] token;
logic [63:0] data_out;
logic [7:0] sync, pid;
logic clear_sender;
logic done_send_token,start_send_token;
logic clear_stuffer;
logic start_send_data, start_send_hand;
logic start_top, read_write;
////////////////////////////////////////////////////////////////
logic [1:0] mode; // 0 = SEND_TOKEN, 1 = SEND_DATA, 2 = SEND_HAND
////////////////////////////////////////////////////////////////
////////////////////////////////////////////
logic do_eop_token,en_sync_token,en_crc_L_token, en_pid_token, en_tok_token, clear_token, ld_sync_token, ld_pid_token,
			ld_tok_token, sel_1_token,sel_2_token,enable_send_token,clear_stuffer_token;
logic do_eop_hand,en_sync_hand, en_pid_hand, clear_hand, ld_sync_hand, ld_pid_hand, sel_1_hand,sel_2_hand,enable_send_hand,
		 done_send_hand;
logic  do_eop_data, en_sync_data ,en_crc_L_data, en_pid_data, en_data_data, clear_data, ld_sync_data, ld_pid_data,
		 ld_data_data, sel_1_data,sel_2_data,enable_send_data, clear_stuffer_data,
		done_send_data;

send_token handle_token(clk, rst_L, start_send_token, pause,
							  do_eop_token,en_sync_token,en_crc_L_token, en_pid_token, en_tok_token, clear_token, ld_sync_token, ld_pid_token,
							  ld_tok_token, sel_1_token,sel_2_token,enable_send_token,clear_stuffer_token,
							  done_send_token); // done signal sends to above.
							  
send_data handle_data(clk, rst_L, start_send_data, pause,
							 do_eop_data, en_sync_data ,en_crc_L_data, en_pid_data, en_data_data, clear_data, ld_sync_data, ld_pid_data,
							 ld_data_data, sel_1_data,sel_2_data,enable_send_data, clear_stuffer_data,
							 done_send_data); // done signal sends to above.
							  
							  
send_ack_nak handle_hand( clk, rst_L, start_send_hand, pause,  // ack or nak is determined by the higher FSM
							  do_eop_hand,en_sync_hand, en_pid_hand, clear_hand, ld_sync_hand, ld_pid_hand, sel_1_hand,sel_2_hand,enable_send_hand,
							 done_send_hand); // done signal sends to above.  							  
							  

//////////   RECEIVES MAGIC FLOATING BLOCK OF CODE   //////////

logic data_in_valid;
 always_comb begin
 if( {wires.DP,wires.DM} == 2'b00) begin // if no info or doing eop
	data_in_valid = 1'b0;
 end
 else begin //activate nrzi 
	data_in_valid = 1'b0;
 end
 end
 // check eop
 logic eop_count, eop_valid;
 logic [2:0] counter_eop;
 //////////////////////////////////////////////////////////////////////////
 always_comb begin
 eop_valid =1'b0;
	if(({wires.DP,wires.DM} == 2'b00)&&(counter_eop<2'b2)) begin
		eop_count =1'b1; // count 2 consecutive XX
	end
	else if( counter_eop <2'b2) begin
		eop_count =1b'0;
	end
	else if ({wires.DP,wires.DM} == 2'b10) begin
		eop_count =1'b1; // see 2 consecutive XX follow by 1.
		eop_valid =1'b1;
	end
	else eop_count =1'b0;
 end
always_ff @(posedge clk, negedge rst_l) begin
	if(~rst_l) counter_eop <= 3'd0;
	else if(eop_count) counter_eop <= counter_eop +  3'd1;
	else counter_eop <= 3'd0;
end
 /////////////////////////////////////////////////////////////////////
 logic [63:0] msg_out;
 logic msg_ok, done, receive, receive_hand, r_acknak_fail, ack, nak;
 logic r_data_start, r_data_finish, r_data_fail, r_data_success;
 logic process_success, system_done;
 logic reverse_nrzi_in, reverse_nrzi_out, unstuff_out;
 assign reverse_nrzi_in = wires.DP; 
reverse_nrzi takein(reverse_nrzi_in, clk, rst_l, ~data_in_valid , reverse_nrzi_out); 
// if data not valid, clear.
logic clear_sync1, clear_sync2, valid_sync, pid_valid, clear_pid1, clear_pid2, clear_unstuff, clear_crc;
// check sync
check_sync checker(reverse_nrzi_out, clk, rst_l, (clear_sync1&&clear_sync2), valid_sync);
////////////////////////////
check_pid testpid(reverse_nrzi_out, clk, rst_l, (clear_pid1&&clear_pid2), pid_valid,pid_out);
////////////////////////////
 reverse_stuffer unstuff(reverse_nrzi_out, clk, rst_l, clear_unstuff, unstuff_out, pause_receive); 
receiver takecrc16(unstuff_out, rst_l, clk, pause_receive, clear_unstuff, clear_crc, msg_out, msg_ok,done);

//receive_data fsm instantiation
receive_data r_data_fsm(clk, rst_l, pause_receive, r_data_start, valid_sync, msg_ok, clear_sync1, r_data_finish, r_data_fail, r_data_success, clear_pid1, clear_crc, clear_unstuff);

//receive_acknak fsm instantiation
receive_acknak r_acknak_fsm(clk, rst_l, receive_hand, pid_out, r_acknak_fail, clear_pid2, clear_sync2, ack, nak, receive);

//////////   RECEIVES MAGIC BLOCK OF CODE ENDS HERE   //////////


/***top_fsm***/
top_fsm topFSM(clk, rst_L, start_top, read_write, done_send_token, done_send_data, done_send_hand, receive, ack, nak, r_acknak_fail, r_data_finish, r_data_fail, r_data_success, start_send_token, start_send_data, start_send_hand, mode, r_data_start, process_success, system_done, receive_hand);



mux4ways#(1)  mux1(mode, do_eop_token,do_eop_data,do_eop_hand, 1'b0 ,do_eop),
					   mux2(mode, en_sync_token, en_sync_data, en_sync_hand,1'b0, en_sync),
					   mux3(mode, en_crc_L_token, en_crc_L_data, 1'b1, 1'b0, en_crc_L),
					   mux4(mode, en_pid_token, en_pid_data, en_pid_hand, 1'b0, en_pid),
					   mux5(mode, en_tok_token, 1'b0, 1'b0, 1'b0, en_tok),
					   mux6(mode, clear_token, clear_data, clear_hand, 1'b0,clear),
					   mux7(mode,  ld_sync_token, ld_sync_data, ld_sync_hand, 1'b0,ld_sync),
					   mux8(mode, ld_pid_token, ld_pid_data, ld_pid_hand, 1'b0,ld_pid),
					   mux9(mode, ld_tok_token, 1'b0, 1'b0,1'b0, ld_tok),
					   mux10(mode, sel_1_token, sel_1_data, sel_1_hand,1'bz, sel_1),
					   mux11(mode, sel_2_token, sel_2_data, sel_2_hand, 1'bz,sel_2),
					   mux12(mode, enable_send_token, enable_send_data, enable_send_hand,1'bz,enable_send),
					   mux13(mode, clear_stuffer_token, clear_stuffer_data, 1'b1,1'bz,clear_stuffer);
always_comb begin
	case(mode)
		2'd0: sel_3 = 1'b0;
		2'd1: sel_3 = 1'b1;
		2'd2: sel_3 = 1'bz;
		2'd3: sel_3 = 1'bz;
	endcase
end
////////////////////////////////////////////
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
shiftRegister #(64) shiftRegData(clk, rst_L, ld_data_data, en_data_data, pause, data_out, crc16_in);


///////////////////////////////////////////////////////////////
stuffer   bitstuff(stuffer_in, clk, rst_L, clear_stuffer, stuffer_out, pause);  // stuff addr, endp,crc5,crc16, and DATA
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
		        if(counter >= 7'd79) begin
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

module mux4ways#(parameter w = 1) (input logic [1:0] sel,
														input logic [w-1:0] inA,inB,inC,inD,
														output logic [w-1:0] out);
always_comb begin
	case(sel)
		2'd0:out = inA;
		2'd1:out = inB;
		2'd2:out = inC;
		2'd3:out = inD;
	endcase
end
endmodule:mux4ways

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                            SEND TOKEN FSM                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module send_token(input logic clk, rst_l, start, pause,
							 output logic do_eop,en_sync,en_crc_L, en_pid, en_tok, clear, ld_sync, ld_pid,ld_tok, sel_1,sel_2,enable_send, clear_stuffer,
							 output logic done); // done signal sends to above.

	logic [4:0] sync_count, pid_count,token_count, eop_count;
	logic [4:0] sync_add,pid_add, token_add, eop_add;
	enum logic [2:0]  {IDLE, SYNC, PID, TOKE, EOP} cs,ns;
	logic clear_counter;
always_comb begin
	done = 1'b0;
	clear_stuffer = 1'b1;
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
			clear = 1'b0;
			en_sync = 1'b1;
			sel_1 = 1'b1;
			sel_2 = 1'b0;
			sync_add =1'b1;
			enable_send =1'b1;
			if(sync_count < 5'd7)begin
				ns = SYNC;	
			end
			else ns = PID; 
		end
		PID: begin
			clear = 1'b0;
			en_pid = 1'b1;
			sel_1 = 1'b0;
			sel_2  = 1'b0;
			pid_add = 1'b1;
			enable_send =1'b1;
			if(pid_count<5'd7) begin
				ns = PID;
			end
			else ns = TOKE;
		end
		TOKE:begin
			clear_stuffer = 1'b0;
			clear = 1'b0;
			sel_1 = 1'b0;
			sel_2 = 1'b1;
			en_crc_L = 1'b0;
			token_add =1'b1;
			enable_send =1'b1;
			if(token_count <5'd10) en_tok =1'b1;
			else en_tok = 1'b0;
			if(token_count < 5'd15) begin
			ns = TOKE;
			end
			else ns = EOP;
		end
		EOP: begin
			clear = 1'b0;
			en_crc_L =1'b1;
			do_eop  = 1'b1;
			eop_add =1'b1;
			enable_send =1'b1;
			if(eop_count <5'd2) begin
				ns = EOP;
			end
			else begin
				ns = IDLE;
				done = 1'b1;
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
		else begin
			cs <=ns;
			sync_count <= sync_count + sync_add;
			pid_count <= pid_count +pid_add;
			token_count <= token_count +token_add;
			eop_count <= eop_count + eop_add;
		end
	end
endmodule: send_token
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               SEND_HAND                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////
module send_ack_nak(input logic clk, rst_l, start, pause,  // ack or nak is determined by the higher FSM
							 output logic do_eop,en_sync, en_pid, clear, ld_sync, ld_pid, sel_1,sel_2,enable_send,
							 output logic done); // done signal sends to above.  

	logic [4:0] sync_count, pid_count,token_count, eop_count;
	logic [4:0] sync_add,pid_add, token_add, eop_add;
	enum logic [2:0]  {IDLE, SYNC, PID, EOP} cs,ns;
	logic clear_counter;
always_comb begin
	done = 1'b0;
	sync_add = 5'b0;
	pid_add =5'b0;
	eop_add = 5'b0;
	clear_counter=1'b0;
	do_eop = 1'b0;
	en_sync =1'b0;
	en_pid = 1'b0;
	clear = 1'b1;
	ld_sync =1'b0;   //
	ld_pid = 1'b0;    //
	sel_1 =1'b0;      //
	sel_2 =1'b0;     //
	enable_send = 1'b0;
	case(cs)
		IDLE :begin
			if(start) begin
				ns = SYNC;
				ld_sync <= 1'b1;
				ld_pid <=1'b1;
				sel_1 <=1'b1;
				sel_2<=1'b0;
				end
			else ns = IDLE;    // wait for start signal. 
		end
		SYNC: begin
			clear = 1'b0;
			en_sync = 1'b1;
			sel_1 = 1'b1;
			sel_2 = 1'b0;
			sync_add =1'b1;
			enable_send =1'b1;
			if(sync_count < 5'd7)begin
				ns = SYNC;	
			end
			else ns = PID; 
		end
		PID: begin
			clear = 1'b0;
			en_pid = 1'b1;
			sel_1 = 1'b0;
			sel_2  = 1'b0;
			pid_add = 1'b1;
			enable_send =1'b1;
			if(pid_count<5'd7) begin
				ns = PID;
			end
			else ns = EOP;
		end
		EOP: begin
			clear = 1'b0;
			do_eop  = 1'b1;
			eop_add =1'b1;
			enable_send =1'b1;
			if(eop_count <5'd2) begin
				ns = EOP;
			end
			else begin
				ns = IDLE;
				done = 1'b1;
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
			eop_count <= 5'b0;
		end
		else if(clear_counter) begin
			cs <=ns;
			sync_count <= 5'b0;
			pid_count <= 5'b0;
			eop_count <= 5'b0;
		end
		else if(pause) begin
			cs <=cs;
			sync_count <= sync_count;
			pid_count <= pid_count;
			eop_count <= eop_count;
		end
		else begin
			cs <=ns;
			sync_count <= sync_count + sync_add;
			pid_count <= pid_count +pid_add;
			eop_count <= eop_count + eop_add;
		end
	end
endmodule: send_ack_nak
//////////////////////////////////////////////////////////////////////////////////////////////////////////

module send_data(input logic clk, rst_l, start, pause,
							 output logic do_eop,en_sync,en_crc_L, en_pid, en_data, clear, ld_sync, ld_pid,ld_data, sel_1,sel_2,enable_send, clear_stuffer,
							 output logic done); // done signal sends to above.

	logic [4:0] sync_count, pid_count, eop_count;
	logic [6:0] data_count;
	logic [4:0] sync_add,pid_add, data_add, eop_add;
	enum logic [2:0]  {IDLE, SYNC, PID, DATA, EOP} cs,ns;
	logic clear_counter;
always_comb begin
	done = 1'b0;
	clear_stuffer = 1'b1;
	sync_add = 5'b0;
	pid_add =5'b0;
	data_add = 5'b0;
	eop_add = 5'b0;
	clear_counter=1'b0;
	do_eop = 1'b0;
	en_sync =1'b0;
	en_crc_L = 1'b1;// has CRC off.
	en_pid = 1'b0;
	en_data = 1'b0;
	clear = 1'b1;
	ld_sync =1'b0;   //
	ld_pid = 1'b0;    //
	ld_data = 1'b0;    //
	sel_1 =1'b0;      //
	sel_2 =1'b0;     //
	enable_send = 1'b0;
	case(cs)
		IDLE :begin
				if(start) begin
					ns = SYNC;
					ld_sync <= 1'b1;
					ld_pid <=1'b1;
					ld_data <= 1'b1;
					sel_1 <=1'b1;
					sel_2<=1'b0;
					end
				else ns = IDLE;    // wait for start signal. 
		end
		SYNC: begin
			clear = 1'b0;
			en_sync = 1'b1;
			sel_1 = 1'b1;
			sel_2 = 1'b0;
			sync_add =1'b1;
			enable_send =1'b1;
			if(sync_count < 5'd7)begin
				ns = SYNC;	
			end
			else ns = PID; 
		end
		PID: begin
			clear = 1'b0;
			en_pid = 1'b1;
			sel_1 = 1'b0;
			sel_2  = 1'b0;
			pid_add = 1'b1;
			enable_send =1'b1;
			if(pid_count<5'd7) begin
				ns = PID;
			end
			else ns = DATA;
		end
		DATA:begin
			clear_stuffer = 1'b0;
			clear = 1'b0;
			sel_1 = 1'b0;
			sel_2 = 1'b1;
			en_crc_L = 1'b0;
			data_add =1'b1;
			enable_send =1'b1;
			if(data_count <7'd63) en_data =1'b1;
			else en_data = 1'b0;
			if(data_count < 7'd79) begin
			ns = DATA;
			end
			else ns = EOP;
		end
		EOP: begin
			clear = 1'b0;
			en_crc_L =1'b1;
			do_eop  = 1'b1;
			eop_add =1'b1;
			enable_send =1'b1;
			if(eop_count <5'd2) begin
				ns = EOP;
			end
			else begin
				ns = IDLE;
				done = 1'b1;
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
			data_count <= 7'b0;
			eop_count <= 5'b0;
		end
		else if(clear_counter) begin
			cs <=ns;
			sync_count <= 5'b0;
			pid_count <= 5'b0;
			data_count <= 5'b0;
			eop_count <= 7'b0;
		end
		else if(pause) begin
			cs <=cs;
			sync_count <= sync_count;
			pid_count <= pid_count;
			data_count <= data_count;
			eop_count <= eop_count;
		end
		else begin
			cs <=ns;
			sync_count <= sync_count + sync_add;
			pid_count <= pid_count +pid_add;
			data_count <= data_count +data_add;
			eop_count <= eop_count + eop_add;
		end
	end
endmodule: send_data

//////////////////////////////////////////////////////////////////////////////////////////////////////////

/*****TOP_FSM*****/

module top_fsm (input logic clk, rst_l,
						input logic start, read_write,                          //read_write = 0 = read, =1 = write.
						input logic done_send_token, done_send_data, done_send_hand, receive, ack, nak, r_hand_fail, r_data_finish, r_data_fail, r_data_success,
						output logic start_send_token, start_send_data, start_send_hand,
						output logic [1:0] mode, // 0  = send_token 1 = send_data, 2 = send_hand
						output logic r_data_start, process_success, system_done, r_hand);

	enum logic [2:0]  {IDLE, SEND_TOKEN, SEND_DATA, RECEIVE_HAND, SEND_HAND,RECEIVE_DATA} cs,ns;

	logic clr_remember, ld_remember;
	logic [1:0] remember,remember_new;; //10 = ACK  01 = NAK
	logic [3:0] r_fail_count, r_hand_fail_count;
	logic r_fail_done, r_hand_fail_done;
	logic [7:0] pid_reg;
	assign r_fail_done = (r_fail_count == 4'd8);
	assign r_hand_fail_done = (r_hand_fail_count == 4'd8);

	always_comb begin
		start_send_token = 1'b0;
		start_send_data =1'b0;
		start_send_hand = 1'b0;
		pid_reg = 8'b0;
		fail_count_new = fail_count;
		r_data_start = 0;
		process_success = 0;
		system_done = 0;
		r_hand = 0;
		remember_new = 2'b0;
		clr_remember =1'b0;
		ld_remember = 1'b0;
		case(cs)
			IDLE :begin
				if(start) begin
					ns = SEND_TOKEN;
					start_send_token =1'b1; // start the first fsm.
					if(read_write)begin // write
						pid = 8'b1000_0111;
					end
					else begin// read, has different pid.
						pid = 8'b1001_0110;
					end
				end
				else
					ns = IDLE;
			end
			SEND_TOKEN:begin
				if(done_send_token) begin
					if(read_write) begin// write
						ns = SEND_DATA; // start sending data
						start_send_data =1'b1; //start send_data fsm
						pid = 8'b1100_0011;
					end
					else begin //read
						ns = RECEIVE_DATA; //start receiving data
						r_data_start = 1'b1;
					   //set some flags
					end
				end
				else ns = SEND_TOKEN; //stay here till done signal is seen.
			end
			SEND_DATA:begin
				if(done_send_data)begin 
					ns = RECEIVE_HAND; // finished sending, time to look for handshake
					r_hand = 1; // start handshake fsm
				end
				else begin
					ns = SEND_DATA;
				end
			end
			RECEIVE_HAND: begin
				if (r_hand_fail || nak) begin
					if (!r_hand_fail_done) begin
						ns = SEND_DATA;
						start_send_data =1'b1; //try again
						pid = 8'b1100_0011;
					end
					else begin
						//give up
						ns = IDLE;
						process_success = 0;
						system_done = 1;
						clr_remember =1'b1;
					end
				end
				else if (receive && ack) begin // received properly with ack, FINISH!!
					process_success = 1;
					system_done = 1;
					ns = IDLE;
					clr_remember =1'b1;
				end
			end
			SEND_HAND:begin
				if(done_send_hand)begin //send_hand is finished
					if(remember == 2'b10) begin //it's a ACK
						process_success = 1; // complete successfully.
						system_done = 1;
						ns = IDLE;
					end
					else if (remember ==2'b01) begin // it's a NAK, try again?
						ns = RECEIVE_DATA;
						r_data_start = 1'b1;
					end
					else begin // error handing sent. Nuclear option.
						ns=IDLE; //system failed went back to begining.
						process_success = 0;
						system_done = 1;
					end
				end
				else begin
					ns = SEND_HAND;
				end
			end
			RECEIVE_DATA: begin
				if (r_data_fail) begin
					if(!r_fail_done) begin
						ns = SEND_HAND;
						start_send_hand = 1'b1;
						pid_reg= 8'b0101_1010;
						ld_remember 1'b1;
						remember_new = 2'b01; //NAK
						//sendNAK!
					end
					else begin // give up
						ns = IDLE;
						process_success = 0;
						system_done = 1;
						clr_remember =1'b1;
					end
				end
				else if (r_data_finish && r_data_success) begin
					ns = SEND_HAND;
					start_send_hand =1'b1;
					pid_reg= 8'b0100_1011;
					ld_remember =1'b1;
					remember_new = 2'b10; //ACK
					//sendACK!
				end
			end
		endcase
	end
	
always_ff @(posedge clk, negedge rst_l)begin //mini- register
	if(~rst_l) remember <= 2'b0;
	else if( clr_remember) remember<=2'b0;
	else if( ld_remember) remember <= remember_new;
	else remember <= remember;
end
	
	always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <= IDLE;
			r_fail_count <= 4'd0;
			r_hand_fail_count <= 4'd0;
		end
		else begin
			cs <= ns;
			r_fail_count <= r_fail_count + r_data_fail;
			r_hand_fail_count <= r_hand_fail_count + (r_hand_fail || nak);
		end
	end

endmodule: top_fsm

/*****RECEIVE DATAPATH AND FSMS*****/
module reverse_stuffer(input logic bit_in, clk, rst_l, clear,
									output logic bit_out, pause);  // unstuff stuff
								//just let the other part of the system know which
								// value to ignore. bit_out doesn't matter.

	always_comb begin
if(count ==3'd6)begin // found 6 ones, stuff a 0, pause for 1 clock
	pause =1'd1;
	bit_out =bit_in;
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
					 
					 
endmodule: reverse_stuffer

module reverse_nrzi(logic input bit_in, clk, rst_L, clear,
								output logic bit_out);
								
logic past;

always_comb begin
if(bit_in == past) bit_out = 1'b1;  // output doesn't change, bit_out = 1 
else bit_out = 1'b0;
end
 
always_ff @(posedge clk, negedge rst_L) begin
	if(~rst_L) past <= 1'd1;
	else if(clear) past <= 1'd1; //initialize to 1.
	else past <= bit_in;
	end	
endmodule: reverse_nrzi

module check_sync(logic input bit_in, clk, rst_l, clear, 
								output logic valid);
// start looking after if clear is not asserted. If sees the right pattern, keep incrementing counter.					
logic [3:0]	counter, counter_new;				
always_comb begin
		counter_new = 4'b0;
		valid =1'b0;
	if((counter <= 4'b6)&&(bit_in ==1'b0)) begin
		counter_new = counter +4'b1; // sees 0,
	end
	else if ((counter ==4'b7)&&(bit_in == 1'b1)) begin
		counter_new = counter +4'b1; // sees 1,
	end
	else if (counter ==4'b8) begin // pattern found.
		valid =1'b0;
		counter_new = 4'b0; //reset counter
	end
	else	counter_new =4'b0; // reset counter if pattern doesn't match
end
always_ff @(posedge clk, negedge rst_l) begin
		if(!rst_l) counter <= 4'b0;
		else if( clear )  counter <= 4'b0;
		else counter <= counter_new;
end
endmodule: check_sync

module check_pid(input logic bit_in, clk, rst_l, clear,
							output logic pid_valid,
							output logic[3:0]  Q);// take in LSB pid, 
							// reverse order to MSB and output if the pid is correct.

	logic [7:0] holder;
always_comb begin
	if(holder[3:0] == ~(holder[7:4])) begin
			pid_valid = 1'b1;   
			Q[0] = holder[4];  // register = 1011_0100
			Q[1] = holder[5];  // send out 1101
			Q[2] = holder[6];
			Q[3] = holder[7];
	end
	else begin
		Q =0;
		pid_valid =1'b0;
	end
end
always_ff @(posedge clk, negedge rst_l) begin
	if(~rst_l) holder <= 8'b0;
	else if (clear ) holder<= 8'b0;
	else begin
		holder <= {holder[7:1],bit_in};
	end
end
endmodule: check_pid

///////////////////////////////////////////////////////////////////////////////

module receiver(input logic bit_in, rst_l, clk, pause, clear_data, clear_crc,
				output logic [63:0] msg_out, output logic msg_ok,done);
	
	logic [63:0] temp;
	logic [15:0] Q;
	logic shift_it, check_it;
	
	crcCal16 calcGot16(bit_in,clk,rst_l,clear_crc,pause,Q);
	receiverFSM get(clk,rst_l,pause,(clear_data&&clear_crc),shift_it,check_it);
	assign msg_out = temp;
	always_comb begin // check ok signal
		if(check_it) msg_ok = (Q == 16'h8005) ? 1'b1: 1'b0;
		else msg_ok =1'b0;
		done = check_it;
	end	
	always_ff @(posedge clk, negedge rst_l) begin //register to hold msg
		if(~rst_l)
			temp <= 64'b0;
		else if (clear_data&&clear_crc)
			temp <= 64'd0;
		else if (shift_it) begin //shift signal control for first 11 shift by FSM
			temp[63:1] <=temp[62:0];
			temp[0] <= bit_in;
			end
		else
			temp <= temp;
	end
endmodule: receiver

module receiverFSM(input logic clk, rst_l, pause, clear,
				output logic shift_it,check_it);

	logic [6:0] counter;
	enum logic [2:0]  { FIRST, DATA,COMP, DEAD,DOWN} cs,ns;

always_comb begin
	check_it =1'b0;
	shift_it =1'b0;
	case(cs)
		FIRST: begin
					ns = DATA;
					shift_it =1'b1;
					//load into register
				end
		DATA: begin
				if(counter >= 7'd64) ns = COMP;
				else begin
					ns = DATA;
					shift_it = 1'b1;
					//load into register
				end
			end
		COMP: begin
					ns = (counter>=5'd80)? DEAD: COMP;
					end
		DEAD: begin
					ns = DOWN;
					check_it =1'b1; //assert check
					end
		DOWN: ns = DOWN;
	endcase
end

always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <= FIRST;
			counter <= 7'b0;
			end
		else if (clear) begin
			cs<= FIRST;
			counter <= 7'b0;
		end
		else if(pause) begin
			cs<= cs;
			counter <=counter;
			end
		else begin
			cs<= ns;
			counter <= counter + 7'b0001;
			end
	end

endmodule:receiverFSM

/*****RECEIVE_DATA FSM*****/

module receive_data(input logic clk, rst_L, pause, r_data_start, valid_sync, correct,
					output logic en_sync_L, finish, fail, success, en_pid_L, en_crc_L, en_unstuff_L);

	//en_pid_L is clear_pid in datapath

	enum logic [3:0] {IDLE, WATCH, TIMEOUT, READPID, READDATA, CRCREC, WAITEOP1, WAITEOP2} cs, ns;

	logic [7:0] timeout_count;
	logic [2:0] pid_count;
	logic [5:0] data_count;
	logic [3:0] crc_count;
	logic [1:0] eop_count;
	logic timeout_add, timedOut, pid_add, pidDone, data_add, dataDone, en_crc_L, crc_add, crcDone, eop_add, eopDone;

	assign timedOut = timeout_count == 8'd255;
	assign pidDone = pid_count == 3'd7;
	assign dataDone = data_count == 6'd63;
	assign crcDone = crc_count == 4'd15;
	assign eopDone = eop_count == 2'd2;

	always_comb begin
		timeout_add = 0;
		en_sync_L = 1;
		pid_add = 0;
		fail = 0;
		success = 0;
		en_pid_L = 1;
		data_add = 0;
		en_crc_L = 1;
		crc_add = 0;
		eop_add = 0;
		en_unstuff_L = 1;
		case(cs)
			IDLE: begin
				ns = r_data_start ? WATCH : IDLE;
			end
			WATCH: begin
				timeout_add = 1;
				en_sync_L = 0;
				if (valid_sync && !timedOut)
					ns = READPID;
				else if (timedOut) begin
					ns = TIMEOUT;
					fail = 1;
				end
				else
					ns = WATCH;
			end
			TIMEOUT: begin
				ns = IDLE;
			end
			READPID: begin
				pid_add = 1;
				ns = pidDone ? READDATA : READPID;
				en_pid_L = 0;
			end
			READDATA: begin
				data_add = 1;
				ns = (dataDone && !pause) ? CRCREC : READDATA;
				en_crc_L = 0;
				en_unstuff_L = 0;
			end
			CRCREC: begin
				crc_add = 1;
				en_crc_L = 0;
				if (crcDone && !pause)  begin
					if (correct)
						ns = WAITEOP2;
					else
						ns = WAITEOP1;
				end
				else
					ns = CRCREC;
			end
			WAITEOP1: begin
				eop_add = 1;
				finish = eopDone ? 1 : 0;
				success = 0;
				fail = eopDone ? 1 : 0;
				ns = eopDone ? IDLE : WAITEOP1;
			end
			WAITEOP2: begin
				eop_add = 1;
				finish = eopDone ? 1 : 0;
				success = eopDone ? 1 : 0;
				fail = 0;
				ns = eopDone ? IDLE : WAITEOP2;
			end
		endcase
	end

	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			cs <= IDLE;
			timeout_count <= 8'd0;
			pid_count <= 3'd0;
			data_count <= 6'd0;
			crc_count <= 4'd0;
			eop_count <= 2'd0;
		end
		else if (pause) begin
			cs <= cs;
			timeout_count <= timeout_count;
			pid_count <= pid_count;
			data_count <= data_count;
			crc_count <= crc_count;
			eop_count <= eop_count;
		end
		else begin
			cs <= ns;
			timeout_count <= timeout_count + timeout_add;
			pid_count <= pid_count + pid_add;
			data_count <= data_count + data_add;
			crc_count <= crc_count + crc_add;
			eop_count <= eop_count + eop_add;
		end

endmodule: receive_data

/*****RECEIVE_ACKNAK FSM*****/

module receive_acknak(input logic clk, rst_L, receive_hand,
					input logic [3:0] pid,
					output logic fail, en_pid_L, en_sync_L, ack, nak, receive);

	//en_pid_L is clear_pid in datapath

	enum logic [2:0] {IDLE, WATCH, TIMEOUT, READPID, WAITEOP1, WAITEOP2} cs, ns;

	logic [7:0] timeout_count;
	logic [2:0] pid_count;
	logic [1:0] eop_count;
	logic timeout_add, timedOut, pid_add, pidDone, eop_add, eopDone;

	assign timedOut = timeout_count == 8'd255;
	assign pidDone = pid_count == 3'd7;
	assign eopDone = eop_count == 2'd2;

	always_comb begin
		timeout_add = 0;
		en_sync_L = 1;
		pid_add = 0;
		fail = 0;
		en_pid_L = 1;
		eop_add = 0;
		ack = 0;
		nak = 0;
		receive = 0;
		case(cs)
			IDLE: begin
				ns = receive_hand ? WATCH : IDLE;
			end
			WATCH: begin
				timeout_add = 1;
				en_sync_L = 0;
				if (valid_sync && !timedOut)
					ns = READPID;
				else if (timedOut) begin
					ns = TIMEOUT;
					fail = 1;
				end
				else
					ns = WATCH;
			end
			TIMEOUT: begin
				ns = IDLE;
			end
			READPID: begin
				pid_add = 1;
				if (pidDone) begin
					if (pid == 4'b0010) //ack
						ns = WAITEOP1;
					else if (pid == 4'b1010) //nak
						ns = WAITEOP2;
				end
				ns = pidDone ? READDATA : READPID;
				en_pid_L = 0;
			end
			WAITEOP1: begin //ack
				eop_add = 1;
				ack = eopDone ? 1 : 0;
				receive = eopDone ? 1 : 0
				ns = eopDone ? IDLE : WAITEOP1;
			end
			WAITEOP2: begin //nak
				eop_add = 1;
				nak = eopDone ? 1 : 0;
				receive = eopDone ? 1 : 0;
				ns = eopDone ? IDLE : WAITEOP2;
			end
		endcase
	end

	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			cs <= IDLE;
			timeout_count <= 8'd0;
			pid_count <= 3'd0;
			eop_count <= 2'd0;
		end
		else if (pause) begin
			cs <= cs;
			timeout_count <= timeout_count;
			pid_count <= pid_count;
			eop_count <= eop_count;
		end
		else begin
			cs <= ns;
			timeout_count <= timeout_count + timeout_add;
			pid_count <= pid_count + pid_add;
			eop_count <= eop_count + eop_add;
		end

endmodule: receive_acknak
