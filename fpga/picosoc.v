
/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Claire Xenia Wolf <claire@yosyshq.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */


 `ifndef PICORV32_REGS
 `ifdef PICORV32_V
 //`error "picosoc.v must be read before picorv32.v!"
 `endif
 
 `define PICORV32_REGS picosoc_regs
 `endif
 
 `ifndef PICOSOC_MEM
 `define PICOSOC_MEM picosoc_mem
 `endif
 
 // this macro can be used to check if the verilog files in your
 // design are read in the correct order.

 `define PICOSOC_V
//`define COMPRESSED_ISA
 module picosoc (
	 input clk,
	 input clka,
	 input resetn,
	 
     input [48:0] in_err,
     output [31:0] mem_imem_rdata,
     output [31:0] mem_imem_rdata1,
     input [63:0] new_ascii_instr,   
     output        err,
     output        trap, 
     output   ena,
     output [14:0]counter1,
     input [31:0]mem_la_addr,
	 output        iomem_valid,
	 input         iomem_ready,
	 output [ 3:0] iomem_wstrb,
	 output [31:0] iomem_addr,
	 output [31:0] iomem_wdata,
	 input  [31:0] iomem_rdata,
     input         mem_valid,
	 input         mem_instr,
	 output       mem_ready,
	 output mem_busy,
	input  [31:0] mem_addr,
	input  [31:0] mem_wdata,
	input  [ 3:0] mem_wstrb,
	input  [31:0] mem_rdata,
	input  [31:0] mem_rdata_latched,
	input compressed_instr,
    output trace_valid,
	output [35:0] trace_data 
	
 );
   
    parameter AXI_TEST = 0;
	parameter VERBOSE = 0;
	parameter [0:0] BARREL_SHIFTER = 0;	//MV removed
 	 parameter [0:0] ENABLE_MUL = 0; //MV removed
 	 parameter [0:0] ENABLE_DIV = 0; //MV removed
	 parameter [0:0] ENABLE_FAST_MUL = 0; //MV removed
	 parameter [0:0] ENABLE_COMPRESSED = 0; //MV removed
	 parameter [0:0] ENABLE_COUNTERS = 1; //MV removed
	 parameter [0:0] ENABLE_IRQ_QREGS = 1; //MV removed
 
	 parameter integer MEM_WORDS = 256;
	 parameter [31:0] STACKADDR = 32'h ffff_ffff;       // end of memory
	 parameter [31:0] PROGADDR_RESET = 32'h 0000_0000; // 1 MB into flash changed to 0 for direct sram access
	 parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010; //MV changed
 	 
	 reg [31:0] irq;
	 reg mem_ready_prev;
	 wire irq_stall = 0;
	 wire irq_uart = 0;
 
	/* always @* begin
		 irq = 0;
		 irq[3] = irq_stall;
		 irq[4] = irq_uart;
		//  irq[5] = irq_5;
		//  irq[6] = irq_6;
		//  irq[7] = irq_7;
	 end*/
 
     
	 wire spimem_ready;
	 wire [31:0] spimem_rdata;
 
	 reg ram_ready;
	 wire [31:0] ram_rdata;
     
    
	 assign iomem_valid = mem_valid && (mem_addr[31:24] > 8'h 01); //if upper 8 bits of mem_addr is TRUE && mem_valid is TRUE then iomem_valid is TRUE
	 assign iomem_wstrb = mem_wstrb;
	 assign iomem_addr = mem_addr;
	 assign iomem_wdata = mem_wdata;
 
	 
 
 	
	
	//reg [31:0] irq = 0;

	reg [15:0] count_cycle = 0;
	always @(posedge clk) count_cycle <= resetn ? count_cycle + 1 : 0;

	always @* begin
		irq = 0;
		irq[4] = &count_cycle[12:0];
		irq[5] = &count_cycle[15:0];
	end

	wire        mem_imem_awvalid;
	wire        mem_imem_awready;
	wire [31:0] mem_imem_awaddr;
	wire [ 2:0] mem_imem_awprot;

	wire        mem_imem_wvalid;
	wire        mem_imem_wready;
	wire [31:0] mem_imem_wdata;
	wire [ 3:0] mem_imem_wstrb;

	wire        mem_imem_bvalid;
	wire        mem_imem_bvalid1;
	wire        mem_imem_bready;

	wire        mem_imem_arvalid;
	wire        mem_imem_arready;
	wire [31:0] mem_imem_araddr;
	wire [ 2:0] mem_imem_arprot;

	wire        mem_imem_rvalid;
	wire        mem_imem_rvalid1;
	wire        mem_imem_rready;
	

	axi4_memory #(
		.AXI_TEST (AXI_TEST),
		.VERBOSE  (VERBOSE)
	) mem (
		.clk             (clk             ),
		.ena(ena),
		.counter1(counter1),
		 .mem_valid   (mem_valid  ),
		.clka          (clka),
		.resetn         (resetn         ),
		.mem_imem_awvalid (mem_imem_awvalid ),
		.mem_imem_awready (mem_imem_awready ),
		.mem_imem_awaddr  (mem_imem_awaddr  ),
		.mem_imem_awprot  (mem_imem_awprot  ),

		.mem_imem_wvalid  (mem_imem_wvalid  ),
		.mem_imem_wready  (mem_imem_wready  ),
		.mem_imem_wdata   (mem_imem_wdata   ),
		.mem_imem_wstrb   (mem_imem_wstrb   ),

		.mem_imem_bvalid  (mem_imem_bvalid  ),
		.mem_imem_bvalid1  (mem_imem_bvalid1  ),
		.mem_imem_bready  (mem_imem_bready  ),

		.mem_imem_arvalid (mem_imem_arvalid ),
		
		.mem_imem_arready (mem_imem_arready ),
		.mem_imem_araddr  (mem_imem_araddr  ),
		.mem_imem_arprot  (mem_imem_arprot  ),

		.mem_imem_rvalid  (mem_imem_rvalid  ),
		.mem_imem_rvalid1  (mem_imem_rvalid1  ),
		.mem_imem_rready  (mem_imem_rready  ),
		.mem_imem_rdata   (mem_imem_rdata   ),
		.mem_imem_rdata1   (mem_imem_rdata1   ),

		.tests_passed    (tests_passed    )
	);

`ifdef RISCV_FORMAL
	wire        rvfi_valid;
	wire [63:0] rvfi_order;
	wire [31:0] rvfi_insn;
	wire        rvfi_trap;
	wire        rvfi_halt;
	wire        rvfi_intr;
	wire [4:0]  rvfi_rs1_addr;
	wire [4:0]  rvfi_rs2_addr;
	wire [31:0] rvfi_rs1_rdata;
	wire [31:0] rvfi_rs2_rdata;
	wire [4:0]  rvfi_rd_addr;
	wire [31:0] rvfi_rd_wdata;
	wire [31:0] rvfi_pc_rdata;
	wire [31:0] rvfi_pc_wdata;
	wire [31:0] rvfi_mem_addr;
	wire [3:0]  rvfi_mem_rmask;
	wire [3:0]  rvfi_mem_wmask;
	wire [31:0] rvfi_mem_rdata;
	wire [31:0] rvfi_mem_wdata;
`endif

	picorv32_axi #(
`ifndef SYNTH_TEST
`ifdef SP_TEST
		.ENABLE_REGS_DUALPORT(0),
`endif
`ifdef COMPRESSED_ISA
		.COMPRESSED_ISA(1),
`endif
		.ENABLE_MUL(1),
		.ENABLE_DIV(1),
		.ENABLE_IRQ(1),
		.ENABLE_TRACE(1)
`endif
	) uut (
		.clk            (clk            ),
		.resetn         (resetn         ),
		.trap           (trap           ),
	    .in_err      (in_err     ),
		
		 .new_ascii_instr (new_ascii_instr),
		 .mem_valid   (mem_valid  ),
		 
		 .mem_ready   (mem_ready  ),
		 .mem_rdata (mem_rdata),
		 .mem_rdata_latched (mem_rdata_latched),
		  .compressed_instr(compressed_instr),
		 .mem_addr (mem_addr),
		 .mem_la_addr (mem_la_addr),
		 .err         (err        ),
		 .mem_busy    (mem_busy   ),
		.mem_axi_awvalid(mem_imem_awvalid),
		.mem_axi_awready(mem_imem_awready),
		.mem_axi_awaddr (mem_imem_awaddr ),
		.mem_axi_awprot (mem_imem_awprot ),
		.mem_axi_wvalid (mem_imem_wvalid ),
		.mem_axi_wready (mem_imem_wready ),
		.mem_axi_wdata  (mem_imem_wdata  ),
		.mem_axi_wstrb  (mem_imem_wstrb  ),
		.mem_axi_bvalid (mem_imem_bvalid ),
		.mem_axi_bvalid1 (mem_imem_bvalid1),
		.mem_axi_bready (mem_imem_bready ),
		.mem_axi_arvalid(mem_imem_arvalid),
		.mem_axi_arready(mem_imem_arready),
		.mem_axi_araddr (mem_imem_araddr ),
		.mem_axi_arprot (mem_imem_arprot ),
		.mem_axi_rvalid (mem_imem_rvalid ),
		.mem_axi_rvalid1 (mem_imem_rvalid1),
		.mem_axi_rready (mem_imem_rready ),
		.mem_axi_rdata  (mem_imem_rdata  ),
		.irq            (irq            ),
`ifdef RISCV_FORMAL
		.rvfi_valid     (rvfi_valid     ),
		.rvfi_order     (rvfi_order     ),
		.rvfi_insn      (rvfi_insn      ),
		.rvfi_trap      (rvfi_trap      ),
		.rvfi_halt      (rvfi_halt      ),
		.rvfi_intr      (rvfi_intr      ),
		.rvfi_rs1_addr  (rvfi_rs1_addr  ),
		.rvfi_rs2_addr  (rvfi_rs2_addr  ),
		.rvfi_rs1_rdata (rvfi_rs1_rdata ),
		.rvfi_rs2_rdata (rvfi_rs2_rdata ),
		.rvfi_rd_addr   (rvfi_rd_addr   ),
		.rvfi_rd_wdata  (rvfi_rd_wdata  ),
		.rvfi_pc_rdata  (rvfi_pc_rdata  ),
		.rvfi_pc_wdata  (rvfi_pc_wdata  ),
		.rvfi_mem_addr  (rvfi_mem_addr  ),
		.rvfi_mem_rmask (rvfi_mem_rmask ),
		.rvfi_mem_wmask (rvfi_mem_wmask ),
		.rvfi_mem_rdata (rvfi_mem_rdata ),
		.rvfi_mem_wdata (rvfi_mem_wdata ),
`endif
		.trace_valid    (trace_valid    ),
		.trace_data     (trace_data     )
	);

`ifdef RISCV_FORMAL
	picorv32_rvfimon rvfi_monitor (
		.clock          (clk           ),
		.reset          (!resetn       ),
		.rvfi_valid     (rvfi_valid    ),
		.rvfi_order     (rvfi_order    ),
		.rvfi_insn      (rvfi_insn     ),
		.rvfi_trap      (rvfi_trap     ),
		.rvfi_halt      (rvfi_halt     ),
		.rvfi_intr      (rvfi_intr     ),
		.rvfi_rs1_addr  (rvfi_rs1_addr ),
		.rvfi_rs2_addr  (rvfi_rs2_addr ),
		.rvfi_rs1_rdata (rvfi_rs1_rdata),
		.rvfi_rs2_rdata (rvfi_rs2_rdata),
		.rvfi_rd_addr   (rvfi_rd_addr  ),
		.rvfi_rd_wdata  (rvfi_rd_wdata ),
		.rvfi_pc_rdata  (rvfi_pc_rdata ),
		.rvfi_pc_wdata  (rvfi_pc_wdata ),
		.rvfi_mem_addr  (rvfi_mem_addr ),
		.rvfi_mem_rmask (rvfi_mem_rmask),
		.rvfi_mem_wmask (rvfi_mem_wmask),
		.rvfi_mem_rdata (rvfi_mem_rdata),
		.rvfi_mem_wdata (rvfi_mem_wdata)
	);
`endif

 	 
	 
	 always @(posedge clk)
		 ram_ready <= mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS;
 
	 `PICOSOC_MEM #(	
		 .WORDS(MEM_WORDS)
	 ) memory (
		 .clk(clk),
		 .wen((mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS) ? mem_wstrb : 4'b0),
		 .addr(mem_addr[23:2]),
		 .wdata(mem_wdata),
		 .rdata(ram_rdata)
	 );
endmodule


  module axi4_memory #(
	parameter AXI_TEST = 0,
	parameter VERBOSE = 0
) (
	/* verilator lint_off MULTIDRIVEN */

	input             clk,
    input   clka,
    input resetn,
    output mem_valid,
    output reg ena,
    output [14:0]counter1,
	input             mem_imem_awvalid,
	output reg        mem_imem_awready,
	input      [31:0] mem_imem_awaddr,
	input      [ 2:0] mem_imem_awprot,

	input             mem_imem_wvalid,
	output reg        mem_imem_wready,
	input      [31:0] mem_imem_wdata,
	input      [ 3:0] mem_imem_wstrb,

	output reg        mem_imem_bvalid,
	output reg        mem_imem_bvalid1,
	input             mem_imem_bready,

	input             mem_imem_arvalid,
	output reg        mem_imem_arready,
	input      [31:0] mem_imem_araddr,
	input      [ 2:0] mem_imem_arprot,

	output reg        mem_imem_rvalid,
	output reg        mem_imem_rvalid1,

	input             mem_imem_rready,
	output   reg   [31:0] mem_imem_rdata,
    output     [31:0] mem_imem_rdata1,
	output reg        tests_passed
);
	reg [31:0]   memory [0:128*1024/4-1] /* verilator public */;
    reg verbose;
	//initial verbose = $test$plusargs("verbose") || VERBOSE;
    //reg ena =0;
	reg axi_test;
	//initial axi_test = $test$plusargs("axi_test") || AXI_TEST;
   /* initial begin
		//mem_imem_awready = 0;
		//mem_imem_wready = 0;
		mem_imem_bvalid = 0;
		mem_imem_bvalid1 = 1;
		//mem_imem_arready = 0;
		mem_imem_rvalid = 0;
		mem_imem_rvalid1 = 1;
		//tests_passed = 0;
	end*/

/*	reg [63:0] xorshift64_state = 64'd88172645463325252;

	task xorshift64_next;
		begin
			// see page 4 of Marsaglia, George (July 2003). "Xorshift RNGs". Journal of Statistical Software 8 (14).
			xorshift64_state = xorshift64_state ^ (xorshift64_state << 13);
			xorshift64_state = xorshift64_state ^ (xorshift64_state >>  7);
			xorshift64_state = xorshift64_state ^ (xorshift64_state << 17);
		end
	endtask

	reg [2:0] fast_imem_transaction = ~0;
	reg [4:0] async_imem_transaction = ~0;
	reg [4:0] delay_imem_transaction = 0;

	always @(posedge clk) begin
		if (axi_test) begin
				xorshift64_next;
				{fast_imem_transaction, async_imem_transaction, delay_imem_transaction} <= xorshift64_state;
		end
	end*/

	reg latched_raddr_en = 0;
	reg latched_waddr_en = 0;
	reg latched_wdata_en = 0;
    reg rand;
	reg fast_raddr = 0;
	reg fast_waddr = 0;
	reg fast_wdata = 0;
	
	reg mem_imem_bvalid_next;
    reg mem_imem_rvalid_next;
    
    reg mem_imem_bvalid1_next;
    reg mem_imem_rvalid1_next;

	reg [31:0] latched_raddr;
	reg [31:0] latched_waddr;
	reg [31:0] latched_wdata;
	reg [ 3:0] latched_wstrb;
	reg        latched_rinsn;
	//wire  [31:0] mem_imem_rdata1;
//wire [14:0] counter1;
//reg ena;
reg wea = 1'b0;
wire [14:0] counter2;
reg dummy;
wire dummy1;
reg en_set=0;
reg [31:0]mem_imem_rdata2;
reg mem_rdata_req;

assign counter1 = (mem_imem_araddr >> 2);
assign counter2 = counter1;

assign dummy1 = mem_imem_rvalid;
//assign ena= ((dummy && !mem_imem_bvalid) || dummy1) ? 1 : 0 ;



/*
always @(negedge clk) begin
    if (!resetn) begin
        dummy <= 1'b0; // Set en to zero when resetn is low
        en_set <= 1'b0; // Reset en_set when resetn is low
    end else begin
        dummy <= 1'b1;
        if (mem_imem_bvalid  || (en_set && !dummy1)) begin
            dummy <= 1'b0; // Set en to zero only when mem_imem_bvalid goes high
            en_set <= 1'b1;
        end else if (dummy1) begin
            dummy <= 1'b1; // Set en to one when mem_imem_rvalid is high
            en_set <= 1'b0;
        end
    end
   
end
*/


always @(negedge clk) begin
    if (!resetn)
        ena <= 1'b0;
    else ena<= 1'b1;
   end




blk_mem_gen_0 your_instance_name (
    .clka(clka),      // input wire clka
    .ena(ena),         // input wire ena
    .wea(wea),         // input wire [0 : 0] wea
    .addra(counter1), // input wire [15 : 0] addra
    .douta(mem_imem_rdata1)  // output wire [31 : 0] douta
);


//assign mem_imem_rdata1 = mem_imem_rdata2;



//assign counter1 = (mem_imem_araddr >> 2)+1;
/*
always@(posedge mem_valid) begin


if(mem_valid)
 ena = 1'b1;
 
 end */
 
 always @(negedge clk) begin
  mem_imem_bvalid<= mem_imem_bvalid_next;
  mem_imem_rvalid<= mem_imem_rvalid_next;
  
  
  if(mem_rdata_req)
   mem_imem_rdata<= mem_imem_rdata1;
  
 end   
 
always @(posedge clk) begin
  mem_imem_bvalid1<= mem_imem_bvalid1_next;
  mem_imem_rvalid1<= mem_imem_rvalid1_next;
 end   


always @(negedge clk) begin
      
      mem_imem_bvalid_next<=0;
      mem_imem_rvalid_next<=0;
      mem_rdata_req <= 0;
      
      
		if (mem_imem_arvalid && !(latched_raddr_en || fast_raddr)) begin 
		mem_imem_arready <= 1;
		latched_raddr = mem_imem_araddr;
		latched_rinsn = mem_imem_arprot[2];
		latched_raddr_en = 1;
		//fast_raddr <= 1;
		end 
		
		if (mem_imem_awvalid && !(latched_waddr_en || fast_waddr) ) begin 
		mem_imem_awready <= 1;
		latched_waddr = mem_imem_awaddr;
		latched_waddr_en = 1;
		//fast_waddr <= 1;
		end 
		
		if (mem_imem_wvalid  && !(latched_wdata_en || fast_wdata) ) begin
		mem_imem_wready <= 1;
		latched_wdata = mem_imem_wdata;
		latched_wstrb = mem_imem_wstrb;
		latched_wdata_en = 1;
		//fast_wdata <= 1;
		end 
		
		if (!mem_imem_rvalid_next && latched_raddr_en) begin
		/*if (verbose)
			$display("RD: ADDR=%08x DATA=%08x%s", latched_raddr, memory[latched_raddr >> 2], latched_rinsn ? " INSN" : "");*/
		if (latched_raddr < 128*1024) begin
		mem_rdata_req<=1;
	     //mem_imem_rdata2 <= mem_imem_rdata1 ;
			//ena =1'b1;
			
			mem_imem_rvalid_next <= 1;
			latched_raddr_en = 0;
		end else begin
		     rand<=1'b0;
		   
			/*$display("OUT-OF-BOUNDS MEMORY READ FROM %08x", latched_raddr);
			$finish;*/
		end
	end 
	//else ena =1'b0;
		//mem_imem_rdata1 = mem_imem_rdata;
		if (!mem_imem_bvalid_next && latched_waddr_en && latched_wdata_en)  begin
		/*if (latched_waddr < 128*1024) begin
		     ena = 1'b0;
		     wea = 1'b0;
			if (latched_wstrb[0]) [ 7: 0] <= latched_wdata[ 7: 0];
			if (latched_wstrb[1]) memory[latched_waddr >> 2][15: 8] <= latched_wdata[15: 8];
			if (latched_wstrb[2]) memory[latched_waddr >> 2][23:16] <= latched_wdata[23:16];
			if (latched_wstrb[3]) memory[latched_waddr >> 2][31:24] <= latched_wdata[31:24];
		end 
		
		else*/
		if (latched_waddr == 32'h1000_0000) begin
			//rand<= latched_wdata[7:0];
				//$write("%c", latched_wdata[7:0]);
				rand <=1'b0;
         	end	
		
		mem_imem_bvalid_next <= 1;
		latched_waddr_en = 0;
		latched_wdata_en = 0;
	end
	
		
	end

	
	

	always @(posedge clk) begin
	
		fast_raddr <= 0;
		fast_waddr <= 0;
		fast_wdata <= 0;
     
        mem_imem_rvalid1_next<=1;
        mem_imem_bvalid1_next<=1;    
		if (mem_imem_rvalid_next && mem_imem_rready) begin
			mem_imem_rvalid1_next <= 0;
		end

		if (mem_imem_bvalid_next && mem_imem_bready) begin
			mem_imem_bvalid1_next <= 0;
		end

	end
	
endmodule
 
 
 // Implementation note:
 // Replace the following two modules with wrappers for your SRAM cells.
 
 //register file
 module picosoc_regs (
	 input clk, wen,
	 input [5:0] waddr,
	 input [5:0] raddr1,
	 input [5:0] raddr2,
	 input [31:0] wdata,
	 output [31:0] rdata1,
	 output [31:0] rdata2
 );
	 reg [31:0] regs [0:31];
 
	 always @(posedge clk)
		 if (wen) regs[waddr[4:0]] <= wdata;
 
	 assign rdata1 = regs[raddr1[4:0]];
	 assign rdata2 = regs[raddr2[4:0]];
 endmodule
 
 //sram
 module picosoc_mem #(
	 parameter integer WORDS = 256
 ) (
	 input clk,
	 input [3:0] wen,
	 input [21:0] addr,
	 input [31:0] wdata,
	 output reg [31:0] rdata
 );
	 reg [31:0] mem [0:WORDS-1];
 
	 always @(posedge clk) begin
		 rdata <= mem[addr];
		 if (wen[0]) mem[addr][ 7: 0] <= wdata[ 7: 0];
		 if (wen[1]) mem[addr][15: 8] <= wdata[15: 8];
		 if (wen[2]) mem[addr][23:16] <= wdata[23:16];
		 if (wen[3]) mem[addr][31:24] <= wdata[31:24];
	 end
 endmodule


 