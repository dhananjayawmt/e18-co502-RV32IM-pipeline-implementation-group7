`include "dcache.v"

module memory_tb;

    // Test bench inputs
    reg CLK;
    reg RESET;
    reg [3:0] READ;
    reg [2:0] WRITE;
    reg [31:0] ADDRESS;
    reg [31:0] WRITE_DATA;
    reg [127:0] MEM_READ_DATA;
    reg MEM_BUSY_WAIT;

    // Test bench outputs
    wire [127:0] MEM_WRITE_DATA;
    wire [31:0] READ_DATA;
    wire [27:0] MEM_ADDRESS;
    wire BUSY_WAIT;
    wire MAIN_MEM_READ;
    wire MAIN_MEM_WRITE;

    // Instantiate the module under test
    dcache dut (
        .CLK(CLK),
        .RESET(RESET),
        .READ(READ),
        .WRITE(WRITE),
        .ADDRESS(ADDRESS),
        .WRITE_DATA(WRITE_DATA),
        .MEM_READ_DATA(MEM_READ_DATA),
        .MEM_BUSY_WAIT(MEM_BUSY_WAIT),
        .MEM_WRITE_DATA(MEM_WRITE_DATA),
        .READ_DATA(READ_DATA),
        .MEM_ADDRESS(MEM_ADDRESS),
        .BUSY_WAIT(BUSY_WAIT),
        .MAIN_MEM_READ(MAIN_MEM_READ),
        .MAIN_MEM_WRITE(MAIN_MEM_WRITE)
    );

    // Clock generation
    always #5 CLK = ~CLK;

    // Initial stimulus
    initial begin

        $dumpfile("simulation.vcd");
        $dumpvars(0, memory_tb);

        CLK = 0;
        RESET = 1;
        READ = 0;
        WRITE = 0;
        ADDRESS = 0;
        WRITE_DATA = 0;
        MEM_READ_DATA = 0;
        MEM_BUSY_WAIT = 0;
        
        #7 RESET = 0;  // Deassert RESET
        
        ADDRESS = 32'b00000000_00000000_00000000_00010000;
        #5 WRITE = 0;
        
        // Add more test cases here
        
        #10 $finish;  // End simulation
    end

    // Display outputs
    always @(posedge CLK) begin
        $display("READ_DATA = %h, MEM_ADDRESS = %h, BUSY_WAIT = %b, MAIN_MEM_READ = %b, MAIN_MEM_WRITE = %b",
            READ_DATA, MEM_ADDRESS, BUSY_WAIT, MAIN_MEM_READ, MAIN_MEM_WRITE);
    end

endmodule
