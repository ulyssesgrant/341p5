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
 logic msg_ok, done;
 logic reverse_nrzi_in, reverse_nrzi_out, unstuff_out;
 assign reverse_nrzi_in = wires.DP; 
reverse_nrzi takein(reverse_nrzi_in, clk, rst_l, ~data_in_valid , reverse_nrzi_out); 
// if data not valid, clear.
logic clear_sync, valid_sync,pid_valid;
// check sync
check_sync checker(reverse_nrzi_out, clk, rst_l, clear_sync, valid_sync);
////////////////////////////
check_pid testpid(reverse_nrzi_out, clk, rst_l, clear_pid, pid_valid,pid_out);
////////////////////////////
 reverse_stuffer unstuff(reverse_nrzi_out, clk, rst_l, clear_unstuff, unstuff_out, pause_receive); 
receiver takecrc16(unstuff_out, rst_l, clk, pause_receive, msg_out, msg_ok,done);
 
logic [2:0] counter_dpdm; 
always_comb begin   // sending DP and DM
	if (do_eop) begin
		if (counter_dpdm == 3'd3) begin
			{wiresDP,wiresDM} = 2'b10;
		end
		else {wiresDP,wiresDM} = 2'b00;
	end
	else {wiresDP,wiresDM} = {nrzi_out,~nrzi_out};                 // go back to output from NRZI
end


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

module receiver(input logic bit_in, rst_l, clk, pause
						 output logic [63:0] msg_out, output logic msg_ok,done);
	
	logic [63:0] temp;
	logic [15:0] Q;
	logic shift_it, check_it;
	
	crcCal16 calcGot16(bit_in,clk,rst_l,pause,Q);
	receiverFSM get(clk,rst_l, pause, shift_it,check_it);
	assign msg_out = temp;
	always_comb begin // check ok signal
		if(check_it) msg_ok = (Q == 16'h8005) ? 1'b1: 1'b0;
		else msg_ok =1'b0;
		done = check_it;
	end	
	always_ff @(posedge clk, negedge rst_l) begin //register to hold msg
		if(~rst_l)
			temp <= 64'b0;
		else if (shift_it) begin //shift signal control for first 11 shift by FSM
			temp[63:1] <=temp[62:0];
			temp[0] <= bit_in;
			end
		else
			temp <= temp;
	end
endmodule: receiver

module receiverFSM(input logic clk, rst_l, pause
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