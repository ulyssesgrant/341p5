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
