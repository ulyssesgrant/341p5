module top_fsm(input logic clk, rst_l,
						input logic start, read_write,                          //read_write = 0 = read, =1 = write.
						input logic done_send_token, done_send_data, done_send_hand, r_fail, ack, nak, r_hand_fail, r_data_finish,
						output logic start_send_token, start_send_data, start_send_hand,
						output logic r_data_start, s_data_start, process_success, system_done, r_hand);

	enum logic [2:0]  {IDLE, SEND_TOKEN, SEND_DATA, RECEIVE_HAND, SEND_HAND,RECEIVE_DATA} cs,ns;

	logic [3:0] r_fail_count, r_hand_fail_count;
	logic r_fail_done, r_hand_fail_done;

	assign r_fail_done = r_fail_count == 4'd8;
	assign r_hand_fail_done = r_hand_fail_count == 4'd8;

	always_comb begin
		start_send_token = 1'b0;
		fail_count_new = fail_count;
		r_data_start = 0;
		s_data_start = 0;
		process_success = 0;
		system_done = 0;
		r_hand = 0;
		
		case(cs)
			IDLE :begin
				if(start) begin
					ns = SEND_TOKEN;
					start_send_token =1'b1;
				end
				else
					ns = IDLE;
					
			end
			SEND_TOKEN:begin
				if(done_send_token)
					if(read_write) begin// write
						ns = SEND_DATA;
						s_data_start = 1;
						//set some flags
					end
					else begin //read
						ns = RECEIVE_DATA;
						r_data_start = 1;
					   //set some flags
					end
			end
			SEND_DATA:begin
				if(done_send_data)begin
					ns = RECEIVE_HAND;
					r_hand = 1;
					//set some flags
				end
				else begin
					ns = SEND_DATA;
				end
			end
			RECEIVE_HAND: begin
				if (r_hand_fail || nak) begin
					if (!r_hand_fail_done) begin
						ns = SEND_DATA;
						s_data_start = 1;
					end
					else begin
						//give up
						ns = IDLE;
						process_success = 0;
						system_done = 1;
					end
				end
				else if (receive && ack) begin
					process_success = 1;
					system_done = 1;
					ns = IDLE;
				end
			end
			SEND_HAND:begin
				if( stuff is done)begin
				
				end
			end
			RECEIVE_DATA: begin
				if (fail) begin
					if(!r_fail_done) begin
						ns = SEND_HAND;
						//sendNAK!
					end
					else begin
						ns = IDLE;
						process_success = 0;
						system_done = 1
						//set some flags but give up
					end
				end
				else if (r_data_finish && success) begin
					ns = SEND_HAND;
					//sendACK!
					// set some flags send_ack
				end
			end
		endcase
	end

	always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <= IDLE;
			r_fail_count <= 4'd0;
			r_hand_fail_count <= 4'd0;
		end
		else begin
			cs <= ns;
			r_fail_count <= r_fail_count + r_fail;
			r_hand_fail_count <= r_hand_fail_count + (r_hand_fail || nak);
		end
	end
endmodule: send_data
//////////////////////////////////////////////////////////////////////////////////////////////////////////
