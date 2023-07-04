`include "cache_controller.v"
`include "data_memory.v"

module dcache (
  input wire clk,
  input wire reset,
  input wire [31:0] address,
  input wire [31:0] write_data,
  input wire write_enable,
  output wire [31:0] read_data
);

  parameter CACHE_SIZE = 512; // Cache size in bytes
  parameter WORD_SIZE = 4; // Word size in bytes
  parameter LINE_SIZE = 4; // Line size in words
  parameter ASSOCIATIVITY = 4; // Set associativity

  localparam NUM_LINES = CACHE_SIZE / (WORD_SIZE * LINE_SIZE);
  localparam NUM_SETS = NUM_LINES / ASSOCIATIVITY;
  localparam OFFSET_BITS = $clog2(LINE_SIZE);
  localparam SET_INDEX_BITS = $clog2(NUM_SETS);
  localparam TAG_BITS = 32 - OFFSET_BITS - SET_INDEX_BITS;

  reg [TAG_BITS-1:0] cache_tags [0:NUM_SETS-1][0:ASSOCIATIVITY-1];
  reg [31:0] cache_data [0:NUM_SETS-1][0:ASSOCIATIVITY-1][LINE_SIZE-1:0];
  reg [ASSOCIATIVITY-1:0] cache_lru [0:NUM_SETS-1]; // LRU counter for each set

  // Cache controller instance  
cache_controller controller (
    .CLK(CLK),
    .RESET(RESET),
    .MEM_BUSY_WAIT(controller_MEM_BUSY_WAIT),
    .MEM_READ_DATA(controller_MEM_READ_DATA),
    .CACHE_WRITE_DATA(controller_CACHE_WRITE_DATA),
    .CACHE_ADDRESS(controller_CACHE_ADDRESS),
    .MAIN_MEM_READ(controller_MAIN_MEM_READ),
    .MAIN_MEM_WRITE(controller_MAIN_MEM_WRITE),
    .READ_CACHE(controller_READ_CACHE),
    .WRITE_CACHE(controller_WRITE_CACHE)
  );

  // Main memory instance
  data_memory memory (
    .CLK(clk),
    .RESET(reset),
    .READ(controller.read_miss),
    .WRITE(controller.write_miss),
    .ADDRESS(controller.miss_address),
    .WRITEDATA(controller.miss_write_data),
    .READDATA(controller.miss_read_data),
    .BUSYWAIT(controller.miss_busy_wait)
  );

  // Cache read and write operations
  always @(posedge clk or posedge reset) begin
    if (reset) begin
      // Reset cache and LRU counters
      for (integer i = 0; i < NUM_SETS; i = i + 1) begin
        for (integer j = 0; j < ASSOCIATIVITY; j = j + 1) begin
          cache_tags[i][j] <= 0;
          for (integer k = 0; k < LINE_SIZE; k = k + 1)
            cache_data[i][j][k] <= 0;
        end
        cache_lru[i] <= 0;
      end
    end else begin
      // Cache read and write operations
      integer set_index = address[31:OFFSET_BITS+TAG_BITS];
      integer tag = address[31:OFFSET_BITS];
      integer word_offset = address[OFFSET_BITS-1:0];
      integer lru_counter;

      // Cache read operation
      if (~write_enable) begin
        lru_counter = cache_lru[set_index]; // Store the current LRU counter value

        // Check if the requested data is in the cache
        for (integer i = 0; i < ASSOCIATIVITY; i = i + 1) begin
          if (cache_tags[set_index][i] == tag) begin
            read_data <= cache_data[set_index][i][word_offset];

            // Update LRU counter based on LRU algorithm
            for (integer j = 0; j < ASSOCIATIVITY; j = j + 1) begin
              if (cache_lru[set_index][j] < lru_counter) // Increase counters for lower values
                cache_lru[set_index][j] = cache_lru[set_index][j] + 1;
            end
            cache_lru[set_index][i] = 0; // Reset counter for accessed line

            return;
          end
        end

        // Cache miss
        // Read data from main memory and update cache
        controller.read_miss <= 1; // Signal cache controller for memory read
        controller.miss_address <= address; // Set miss address

        // Wait for memory read completion
        if (~controller.miss_busy_wait) begin
          // Update cache with the fetched data
          for (integer i = 0; i < ASSOCIATIVITY; i = i + 1) begin
            if (cache_lru[set_index][i] == ASSOCIATIVITY - 1) begin
              // Replace the LRU cache line with the fetched data
              cache_tags[set_index][i] <= tag;
              for (integer j = 0; j < LINE_SIZE; j = j + 1)
                cache_data[set_index][i][j] <= controller.miss_read_data[j * WORD_SIZE +: WORD_SIZE];
            end
          end
          // Update LRU counter based on LRU algorithm
          for (integer i = 0; i < ASSOCIATIVITY; i = i + 1) begin
            if (cache_lru[set_index][i] < lru_counter) // Increase counters for lower values
              cache_lru[set_index][i] = cache_lru[set_index][i] + 1;
          end
          cache_lru[set_index][ASSOCIATIVITY - 1] = 0; // Reset counter for newly fetched line

          read_data <= controller.miss_read_data; // Set read data from memory
        end

        return;
      end
      // Cache write operation
      else begin
        lru_counter = cache_lru[set_index]; // Store the current LRU counter value

        // Check if the requested data is in the cache
        for (integer i = 0; i < ASSOCIATIVITY; i = i + 1) begin
          if (cache_tags[set_index][i] == tag) begin
            cache_data[set_index][i][word_offset] <= write_data;
            // Update main memory (You need to implement the memory write operation here)

            // Update LRU counter based on LRU algorithm
            for (integer j = 0; j < ASSOCIATIVITY; j = j + 1) begin
              if (cache_lru[set_index][j] < lru_counter) // Increase counters for lower values
                cache_lru[set_index][j] = cache_lru[set_index][j] + 1;
            end
            cache_lru[set_index][i] = 0; // Reset counter for accessed line

            return;
          end
        end

        // Cache miss
        // Read data from main memory and update cache
        controller.write_miss <= 1; // Signal cache controller for memory write
        controller.miss_address <= address; // Set miss address
        controller.miss_write_data <= write_data; // Set write data

        // Wait for memory write completion
        if (~controller.miss_busy_wait) begin
          // Update cache with the fetched data
          for (integer i = 0; i < ASSOCIATIVITY; i = i + 1) begin
            if (cache_lru[set_index][i] == ASSOCIATIVITY - 1) begin
              // Replace the LRU cache line with the fetched data
              cache_tags[set_index][i] <= tag;
              for (integer j = 0; j < LINE_SIZE; j = j + 1)
                cache_data[set_index][i][j] <= controller.miss_write_data[j * WORD_SIZE +: WORD_SIZE];
            end
          end
          // Update LRU counter based on LRU algorithm
          for (integer i = 0; i < ASSOCIATIVITY; i = i + 1) begin
            if (cache_lru[set_index][i] < lru_counter) // Increase counters for lower values
              cache_lru[set_index][i] = cache_lru[set_index][i] + 1;
          end
          cache_lru[set_index][ASSOCIATIVITY - 1] = 0; // Reset counter for newly fetched line
        end

        // Update main memory (You need to implement the memory write operation here)
        MEM_ARRAY[address] <= `READ_WRITE_DELAY write_data[7:0];
        MEM_ARRAY[address + 1] <= `READ_WRITE_DELAY write_data[15:8];
        MEM_ARRAY[address + 2] <= `READ_WRITE_DELAY write_data[23:16];
        MEM_ARRAY[address + 3] <= `READ_WRITE_DELAY write_data[31:24];


        return;
      end
    end
  end
endmodule
