module top_fsm(input logic clk, rst_l,
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
