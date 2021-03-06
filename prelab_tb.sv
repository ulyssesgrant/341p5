module test
  (output logic clk, rst_L);
  
  logic [7:0]   flash_addr;
  logic [63:0]  flash_data;
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
    flash_data = 64'h0;
    
    // Read Data
    @(posedge clk);
    $display("");
    $display("Sending an OUT to endpoint 4");

    // Prelab data is purely for you to test different packets.  It is not
    // actually used in this prelab.
	
	$monitor($stime,, "DP=%b,DM=%b,nrzi_in=%b sync_val=%b sync_pid_out =%b pid_out=%b stuffer_out=%b,pause=%b,do_eop=%b ld_sync =%b",usbHost.wires.DP,usbHost.wires.DM,usbHost.nrzi_in,
usbHost.shiftRegSync.val, usbHost.sync_pid_out,usbHost.pid_out,usbHost.stuffer_out,usbHost.pause,usbHost.do_eop, usbHost.ld_sync);
    
	/*
	$monitor($stime,, "nrzi_in=%b sync_val=%b sync_pid_out =%b pid_out=%b stuffer_out=%b,pause=%b,do_eop=%b sync_count =%d pid_count=%d token_count =%d eop_count =%d",usbHost.nrzi_in,
usbHost.shiftRegSync.val, usbHost.sync_pid_out,usbHost.pid_out,usbHost.stuffer_out,usbHost.pause,usbHost.do_eop,top.host.handle_token.sync_count,top.host.handle_token.pid_count,
		top.host.handle_token.token_count,top.host.handle_token.eop_count);
		*/
	//$monitor($stime,, "sync_out=%b,pid_out=%b",usbHost.sync_out,usbHost.pid_out);
	host.writeData(flash_data,flash_data,success);
    //host.prelabRequest(flash_data);
    thumbDrive.outputData();
    @(posedge clk);
		$display("\n");
    $display("********************************************************************************");
    $display("*****************************END TEST*******************************************");
    $display("********************************************************************************");
    
     $finish();
  end

  always #1 clk = ~clk;

endmodule
