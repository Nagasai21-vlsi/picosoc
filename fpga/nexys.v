 module nexys (
	input clk,
	
   input [6:0] in_err,
	// output ser_tx,
	// input ser_rx,
	output [6:0]seg_display,
	output wire [7:0] anodes,
    output resetn,
     
	//output led1,
	//output led2,
	//output  led3,
	output led4,
	output led5,
	output err,
	output trap,
	output mem_busy,
	output mem_valid,
	output mem_valid1,
	output mem_ready
   
    
	// output flash_csb,
	// output flash_clk,
	// inout  flash_io0,
	// inout  flash_io1,
	// inout  flash_io2,
	// inout  flash_io3,	
);
  //  reg led3 =1'b0;
  wire ena;
  wire [14:0]counter1;
  wire   [31:0] mem_rdata;
  wire   [31:0] mem_rdata_latched;
  wire   [31:0] mem_addr;
   wire [63:0] new_ascii_instr;
    wire [31:0]mem_la_addr;
    wire compressed_instr;
 // reg  [31:0] mem_rdata;
    wire [31:0]mem_imem_rdata;
     wire [31:0]mem_imem_rdata1;
    wire clka;
    reg [3:0] disp=0;
  // reg  [6:0] in_err=0;
    /*
    initial begin
    in_err =0;
    #5950 in_err= 7'b0001100;
    #200 in_err =7'b0000100;
     #500 in_err = 0;
    end*/
    
     parameter integer MEM_WORDS = 32768; // 32768/1024=32Kbytes of BRAM
     parameter ORIG_CLK_FREQ = 200_000_000; // Original clock frequency (100 MHz)
    parameter SLOWED_CLK_FREQ = 1; // Slowed clock frequency (100 Hz)
    parameter DIVIDER = ORIG_CLK_FREQ / (2 * SLOWED_CLK_FREQ); // Divide factor for slowed clock
    //power on reset
    reg [1:0] reset_cnt = 0;
	assign resetn = &reset_cnt;
	//reg resetn;
    wire [48:0] error;
    reg [39:0] count = 0; // N-bit counter for generating slowed clock
   reg slowed_clk = 0; // Output slowed clock signal
     
   assign clka = slowed_clk; 
   assign error = {31'b0, in_err, 11'b0};
   assign anodes = 8'b11111110; 
   assign mem_valid1 = !mem_valid; 
  





always @(posedge clk) begin
    if (count == DIVIDER - 1) begin
        count <= 0;
        slowed_clk <= ~slowed_clk; // Toggle the output slowed clock
    end else begin
        count <= count + 1;
    end
end

	always @(posedge clk) begin
		reset_cnt <= reset_cnt + !resetn;
	end 

	always @ (posedge clk) begin
    case (new_ascii_instr)
        64'h0077616974697271: disp <= 4'b0000; // waitirq
        64'h006D61736B697271: disp <= 4'b0001; // maskirq
        64'h00000000006A616C: disp <= 4'b0010; // jal
        64'h0000000061646469: disp <= 4'b0011; // addi
        64'h0000000000007377: disp <= 4'b0100; // sw
        64'h0000000000616464: disp <= 4'b0101; // add
        64'h0000000000737562: disp <= 4'b0110; // sub
        64'h0000000000006C77: disp <= 4'b0111; // lw
       // default: disp <= 4'bXXXX; // Default value if no pattern matches
    endcase
 end   
 /*
 always @(posedge slowed_clk) begin
    case(mem_rdata)
    32'h0600600B: disp <=4'b0110 ;
    endcase
    
end
*/	
	sevensegdisp display0(.data(disp), .seg(seg_display));

	
	wire ledr_n, ledg_n;
	wire [7:0] leds;


	//assign led3 = leds[3];
	assign led4 = leds[4];
	assign led5 = leds[5];

	assign ledr_n = !leds[6];
	assign ledg_n = !leds[7];
	

	// wire flash_io0_oe, flash_io0_do, flash_io0_di;
	// wire flash_io1_oe, flash_io1_do, flash_io1_di;
	// wire flash_io2_oe, flash_io2_do, flash_io2_di;
	// wire flash_io3_oe, flash_io3_do, flash_io3_di;

	// SB_IO #(
	// 	.PIN_TYPE(6'b 1010_01),
	// 	.PULLUP(1'b 0)
	// ) flash_io_buf [3:0] (
	// 	.PACKAGE_PIN({flash_io3, flash_io2, flash_io1, flash_io0}),
	// 	.OUTPUT_ENABLE({flash_io3_oe, flash_io2_oe, flash_io1_oe, flash_io0_oe}),
	// 	.D_OUT_0({flash_io3_do, flash_io2_do, flash_io1_do, flash_io0_do}),
	// 	.D_IN_0({flash_io3_di, flash_io2_di, flash_io1_di, flash_io0_di})
	// );

	wire        iomem_valid;
	reg         iomem_ready;
	wire [3:0]  iomem_wstrb;
	wire [31:0] iomem_addr;
	wire [31:0] iomem_wdata;
	reg  [31:0] iomem_rdata;

	reg [31:0] gpio;
	assign leds = gpio;

	always @(posedge clk) begin
		if (!resetn) begin
			gpio <= 0;
		end else begin
			iomem_ready <= 0;
			if (iomem_valid && !iomem_ready && iomem_addr[31:24] == 8'h 03) begin
				iomem_ready <= 1;
				iomem_rdata <= gpio;
				if (iomem_wstrb[0]) gpio[ 7: 0] <= iomem_wdata[ 7: 0];
				if (iomem_wstrb[1]) gpio[15: 8] <= iomem_wdata[15: 8];
				if (iomem_wstrb[2]) gpio[23:16] <= iomem_wdata[23:16];
				if (iomem_wstrb[3]) gpio[31:24] <= iomem_wdata[31:24];
			end
		end
	end

	picosoc soc (
		.clk          (clk       ),
		.clka          (clka        ),
		.resetn       (resetn      ),
		.in_err       (error       ),
		.new_ascii_instr(new_ascii_instr),
        .mem_rdata (mem_rdata),
        .mem_rdata_latched (mem_rdata_latched),
        .compressed_instr(compressed_instr),
        .mem_addr (mem_addr),
        .mem_la_addr(mem_la_addr),
		//.ser_tx       (ser_tx      ),
		//.ser_rx       (ser_rx      ),

		//.flash_csb    (flash_csb   ),
		//.flash_clk    (flash_clk   ),

		//.flash_io0_oe (flash_io0_oe),
		//.flash_io1_oe (flash_io1_oe),
		//.flash_io2_oe (flash_io2_oe),
		//.flash_io3_oe (flash_io3_oe),

		//.flash_io0_do (flash_io0_do),
		//.flash_io1_do (flash_io1_do),
		//.flash_io2_do (flash_io2_do),
		//.flash_io3_do (flash_io3_do),

		//.flash_io0_di (flash_io0_di),
		//.flash_io1_di (flash_io1_di),
		//.flash_io2_di (flash_io2_di),
		//.flash_io3_di (flash_io3_di),

		//.irq_5        (1'b0        ),
		//.irq_6        (1'b0        ),
		//.irq_7        (1'b0        ),
		.mem_busy      (mem_busy   ),
		.ena (ena),
		.counter1(counter1),
		.mem_imem_rdata(mem_imem_rdata),
		.mem_imem_rdata1(mem_imem_rdata1),
		.trap          (trap       ),
		.err           (err        ),
		.mem_ready     (mem_ready  ),
		.mem_valid     (mem_valid  ),
		.iomem_valid  (iomem_valid ),
		.iomem_ready  (iomem_ready ),
		.iomem_wstrb  (iomem_wstrb ),
		.iomem_addr   (iomem_addr  ),
		.iomem_wdata  (iomem_wdata ),
		.iomem_rdata  (iomem_rdata )
	);
endmodule
