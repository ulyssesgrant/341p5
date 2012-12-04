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
