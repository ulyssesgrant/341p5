module top_fsm(input logic clk, rst_l,
						input logic start, read_write,                          //read_write = 0 = read, =1 = write.
						input logic done_send_token, done_send_data, done_send_hand,
						output logic start_send_token, start_send_data, start_send_hand,
						output logic);


						
	enum logic [2:0]  {IDLE, SEND_TOKEN, SEND_DATA, RECEIVE_HAND, SEND_HAND,RECEIVE_DATA} cs,ns;
	logic [3:0] fail_count, fail_count _new;
always_comb begin
	start_send_token =1'b0;
	fail_count_new = fail_count;
	
	
	
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
					//set some flags
				end
				else begin //read
				 ns = RECEIVE_DATA;
				   //set some flags
				end
		end
		SEND_DATA:begin
			if(done_send_data)begin
				ns = RECEIVE_HAND;
				//set some flags
			end
			else begin
				ns = SEND_DATA;
			end
		end
		RECEIVE_HAND:begin
			if(      something finish )begin
				if(fail and still have attempts)begin
					ns = SEND_DATA;
					fail_count_new = fail_count +1; //increment fail counter.
					end
				else if ( fail and out of attempts)begin
					ns = IDLE;
					success = 1'b0;
					system_done =1'b1;
				end
			end
		end
		SEND_HAND:begin
			if( stuff is done)begin
			
			end
		end
		RECEIVE_DATA:begin
			if( stuff is done done )begin
				if( failed with attempts left do) begin
					
				end
				else if (failed with no attempts) begin
					ns = IDLE;
					//set some flags but give up
				end
				else
					ns = SEND_HAND;
					// set some flags send_ack
				end
			end
		end
	endcase
end

	always_ff @(posedge clk, negedge rst_l) begin
		if(~rst_l) begin
			cs <=IDLE;
			fail_count <=4'b0;
		end
		else begin
			cs <=ns;
			fail_count <= fail_count_new;
		end
	end
endmodule: send_data
//////////////////////////////////////////////////////////////////////////////////////////////////////////