module cache_controller (
  input wire CLK,
  input wire RESET,
  input wire MEM_BUSY_WAIT,
  input wire [31:0] MEM_READ_DATA,
  input wire [31:0] CACHE_WRITE_DATA,
  input wire [31:0] CACHE_ADDRESS,
  output wire MAIN_MEM_READ,
  output wire MAIN_MEM_WRITE,
  output wire READ_CACHE,
  output wire WRITE_CACHE
);
  // Define constants
  parameter TAG_WIDTH = 16;
  parameter SET_INDEX_WIDTH = 6;
  parameter WORD_OFFSET_WIDTH = 2;
  parameter ASSOCIATIVITY = 4;
  parameter NUM_SETS = 64;
  parameter CACHE_SIZE = 4096;

  // Declare cache data structures
  reg [31:0] cache_tags [NUM_SETS-1:0][ASSOCIATIVITY-1:0];
  reg [31:0] cache_data [NUM_SETS-1:0][ASSOCIATIVITY-1:0][3:0];
  reg [1:0] cache_dirty_and_valid [NUM_SETS-1:0][ASSOCIATIVITY-1:0];
  reg [1:0] lru_counters [NUM_SETS-1:0][ASSOCIATIVITY-1:0];

  // Declare internal signals
  reg [15:0] tag;
  reg [5:0] set_index;
  reg [1:0] word_offset;
  reg [1:0] state, next_state;
  reg [3:0] i;
  reg valid, dirty;
  reg hit;
  reg [3:0] hit_line;
  reg [3:0] prefetch_line;

  // Extract address components
  always @* begin
    tag = CACHE_ADDRESS[31:16];
    set_index = CACHE_ADDRESS[15:10];
    word_offset = CACHE_ADDRESS[1:0];
  end

  // Cache initialization
  initial begin
    // Initialize cache tags and data
    for (i = 0; i < NUM_SETS; i = i + 1) begin
      for (j = 0; j < ASSOCIATIVITY; j = j + 1) begin
        cache_tags[i][j] = 32'h0;
        cache_data[i][j] = 32'h0;
        cache_dirty_and_valid[i][j] = 2'b00;
        lru_counters[i][j] = j;
      end
    end
  end

  // Cache hit detection
  always @* begin
    hit = 1'b0;
    hit_line = 4'b1111;
    for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
      if (valid && (cache_tags[set_index][i] == tag)) begin
        hit = 1'b1;
        hit_line = i;
        $display("Cache Hit");
        valid = 1'b1;
        dirty = cache_dirty_and_valid[set_index][i][0];
        break;
      end
    end
    prefetch_line = hit_line + 1; // Spatial prefetching: Prefetch next cache line
  end

  // Cache controller state transition
  always @(posedge CLK) begin
    if (RESET) begin
      state <= 2'b00;
      next_state <= 2'b00;
    end else begin
      state <= next_state;
    end
  end

  // Cache controller next state logic
  always @* begin
    case (state)
      2'b00: begin
        if (!valid && (MEM_BUSY_WAIT || CACHE_WRITE_DATA != 32'h0)) begin
          next_state = 2'b01;
        end else if (valid && !hit && (MEM_BUSY_WAIT || CACHE_WRITE_DATA != 32'h0)) begin
          next_state = 2'b10;
        end else begin
          next_state = 2'b00;
        end
      end

      2'b01: begin
        next_state = 2'b00;
      end

      2'b10: begin
        next_state = 2'b00;
      end

      default: begin
        next_state = 2'b00;
      end
    endcase
  end

  // Cache controller output generation
  always @* begin
    MAIN_MEM_READ = 1'b0;
    MAIN_MEM_WRITE = 1'b0;
    READ_CACHE = 1'b0;
    WRITE_CACHE = 1'b0;

    case (state)
      2'b00: begin
        if (!valid && (MEM_BUSY_WAIT || CACHE_WRITE_DATA != 32'h0)) begin
          MAIN_MEM_READ = 1'b1;
          READ_CACHE = 1'b1;
        end else if (valid && !hit && (MEM_BUSY_WAIT || CACHE_WRITE_DATA != 32'h0)) begin
          if (cache_dirty_and_valid[set_index][lru_counters[set_index][ASSOCIATIVITY-1]][1]) begin
            MAIN_MEM_WRITE = 1'b1;
            WRITE_CACHE = 1'b1;
          end else begin
            READ_CACHE = 1'b1;
          end
          // Spatial prefetching: Prefetch next cache line
          if (prefetch_line >= ASSOCIATIVITY) begin
            prefetch_line = 0;
          end
          if (!cache_dirty_and_valid[set_index][prefetch_line][1]) begin
            MAIN_MEM_READ = 1'b1;
          end
        end
      end

      2'b01: begin
        cache_tags[set_index][lru_counters[set_index][ASSOCIATIVITY-1]] = tag;
        cache_data[set_index][lru_counters[set_index][ASSOCIATIVITY-1]] = CACHE_WRITE_DATA;
        cache_dirty_and_valid[set_index][lru_counters[set_index][ASSOCIATIVITY-1]] = 2'b10;
        for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
          if (lru_counters[set_index][i] < lru_counters[set_index][ASSOCIATIVITY-1]) begin
            lru_counters[set_index][i] = lru_counters[set_index][i] + 1;
          end else begin
            lru_counters[set_index][i] = 0;
          end
        end
        dirty = 1'b0;
        valid = 1'b1;
        WRITE_CACHE = 1'b0;
      end

      2'b10: begin
        for (i = 0; i < ASSOCIATIVITY; i = i + 1) begin
          if (lru_counters[set_index][i] < lru_counters[set_index][hit_line]) begin
            lru_counters[set_index][i] = lru_counters[set_index][i] + 1;
          end
        end
        lru_counters[set_index][hit_line] = 0;
        dirty = 1'b0;
        valid = 1'b1;
        READ_CACHE = 1'b0;
      end
    endcase
  end
endmodule
