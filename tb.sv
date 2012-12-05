// Write a tester here using the readData and writeData tasks

module test
  (output logic clk, rst_L);
  
  logic [7:0]   flash_addr;
  logic [63:0]  flash_data,flash_data_out;
  logic [63:0]  receivedMsg;
  logic         success;

  // NOTE: For prelab you do not have to do timeouts.  It just needs to send
  // packets
  
  initial 
  begin
    rst_L = 1'b1;
    clk   = 1'b1;
    @(posedge clk) rst_L = 1'b0;
    @(posedge clk) rst_L = 1'b1;
    
		$display("\n");
    $display("********************************************************************************");
    $display("*****************************WRITE AND READ TEST********************************");
    $display("********************************************************************************");
    
///////////////////////////////////////////////////////////////////////////////
// Host sends data to the device to be printed
///////////////////////////////////////////////////////////////////////////////
    flash_addr = 8'hAB;
    flash_data = 64'h10_10_10_10_10_10_10_10;
    
    // Read Data
    @(posedge clk);
    $display("");
    $display("Sending an OUT to endpoint 4");

    // Prelab data is purely for you to test different packets.  It is not
    // actually used in this prelab.
	
	
$monitor($stime,,
"DP=%b,bit_in=%b,sync_val=%h,do_eop=%b,receive_success =%b, top.cs =%s,receive_fsm =%s,process_success =%b correct=%b msg =%h, Q = %h counter=%d", top.wires.DP,top.host.reverse_nrzi_out,
usbHost.shiftRegSync.val,usbHost.do_eop,top.host.process_success,
top.host.topFSM.cs.name, top.host.r_data_fsm.cs.name,top.host.process_success,
top.host.r_data_fsm.correct,top.host.msg_out,
top.host.takecrc16.Q,top.host.takecrc16.get.counter);

/*  test receiving end.
		$monitor($stime,, "DP=%b,DM=%b,nrzi_in=%b sync_val=%h ,pause=%b,do_eop=%b, receive_success =%b, top.cs =%s, ack_nak =%s,pid_in =%b pid_count=%d process_success =%b", top.wires.DP,top.wires.DM,usbHost.nrzi_in,
usbHost.shiftRegSync.val,
usbHost.pause,usbHost.do_eop,top.host.process_success,
top.host.topFSM.cs.name, top.host.r_acknak_fsm.cs.name,
top.host.r_acknak_fsm.pid,
top.host.r_acknak_fsm.pid_count,top.host.process_success);
*/	
	
	/*
	$monitor($stime,, "DP=%b,DM=%b,nrzi_in=%b sync_val=%h sync_pid_out =%b pid_out=%b stuffer_out=%b,pause=%b,do_eop=%b ld_sync =%b receive_success =%b, top.cs =%s, ack_nak =%s",top.wires.DP,top.wires.DM,usbHost.nrzi_in,
usbHost.shiftRegSync.val, usbHost.sync_pid_out,usbHost.pid_out,usbHost.stuffer_out,usbHost.pause,usbHost.do_eop, usbHost.ld_sync,top.host.process_success, top.host.topFSM.cs.name, top.host.r_acknak_fsm.cs.name);
    */
	/*
	$monitor($stime,, "nrzi_in=%b sync_val=%b sync_pid_out =%b pid_out=%b stuffer_out=%b,pause=%b,do_eop=%b sync_count =%d pid_count=%d token_count =%d eop_count =%d",usbHost.nrzi_in,
usbHost.shiftRegSync.val, usbHost.sync_pid_out,usbHost.pid_out,usbHost.stuffer_out,usbHost.pause,usbHost.do_eop,top.host.handle_token.sync_count,top.host.handle_token.pid_count,
		top.host.handle_token.token_count,top.host.handle_token.eop_count);
		*/
	//$monitor($stime,, "sync_out=%b,pid_out=%b",usbHost.sync_out,usbHost.pid_out);
	host.writeData(flash_addr,flash_data,success);
	host.readData(flash_addr,flash_data_out,success);
    //host.prelabRequest(flash_data);
    @(posedge clk);
		$display("\n");
    $display("********************************************************************************");
    $display("*****************************END TEST*******************************************");
    $display("********************************************************************************");
    
     $finish();
  end

  always #1 clk = ~clk;

endmodule
