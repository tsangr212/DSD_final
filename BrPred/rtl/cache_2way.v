module D_cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input              clk;
    // processor interface
    input              proc_reset;
    input              proc_read, proc_write;
    input      [29:0]  proc_addr;
    input      [31:0]  proc_wdata;
    output             proc_stall;
    output reg [31:0]  proc_rdata;
    // memory interface
    input      [127:0] mem_rdata;
    input              mem_ready;
    output             mem_read;
    output             mem_write;
    output reg  [27:0]  mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    localparam STATE_IDLE        = 2'b00;
    localparam STATE_WRITEBACK   = 2'b01;
    localparam STATE_ALLOCATE    = 2'b10;
    localparam STATE_READY       = 2'b11;

    wire [127:0] mem_wdata_r;
    wire [27:0]  mem_addr_r;
    reg mem_read_r;
    reg [127:0] mem_rdata_r;
    reg mem_read_out;

    wire proc_access;
    wire [155:0] sram_rdata;
    wire [25:0]  sram_tag;
    wire [25:0]  proc_addr_tag;
    wire [1:0]   proc_addr_index;
    wire [127:0] sram_data;
    wire sram_hit;
    wire sram_dirty;
    wire [1:0] proc_addr_offset;
    reg  [155:0] sram_wdata;
    reg sram_write;
    reg [2:0] state;
    reg [2:0] state_nxt;
    reg [27:0] buffer_addr_w, buffer_addr_r;
    reg buffer_wen;
    reg [27:0] read_addr;
    wire [127:0] buffer_wdata;
    wire [27:0] buf_addr;
    wire stall_in;
    wire stall_out;
    wire proc_stall_r;

// internal info
    reg [155:0] sram[0:3][0:1];
    reg lru[0:3]; // lru[index]: the next way to be write back
    reg way;
    reg [155:0] entry[0:1];
    reg valid[0:1];
    reg hit[0:1];
    reg [25:0] tag[0:1];
    wire lru_nxt;
    reg mem_ready_r;
    wire mem_ready_wire;

    integer i;

//==== combinational circuit ==============================
    assign proc_access = proc_read || proc_write;
    assign stall_in = proc_access && !sram_hit;
    assign proc_stall = ((stall_out && proc_write) || proc_read || proc_write) && !sram_hit;
    assign buffer_wdata = sram_data;

    buffer wb1 ( // Write buffer
	.clk(clk),
    .reset(proc_reset),
    .ready(mem_ready),
	.w_en(buffer_wen),
	.addr_in(buffer_addr_r),
	.data_in(buffer_wdata),
    .mem_read(mem_read_r),
    .stall_out(stall_out),
    .mem_wen(mem_write),
    .addr_out(buf_addr),
    .data_out(mem_wdata_r)
	);

    always @(*) begin
        for (i=0; i<32; i=i+1) begin
            proc_rdata[i] = sram_data[(proc_addr_offset<<5)+i];
        end
    end

    assign sram_dirty = sram_rdata[154];
    assign sram_tag = sram_rdata[153:128];
    assign sram_data = sram_rdata[127:0];

    assign proc_addr_tag = proc_addr[29:4];
    assign proc_addr_index = proc_addr[3:2];
    assign proc_addr_offset = proc_addr[1:0];

    always @(*) begin
        sram_wdata = {2'b11, proc_addr_tag, sram_data};

        if (state == STATE_READY) begin
            // allocate
            sram_wdata[154] = 0; // not dirty
            sram_wdata[127:0] = mem_rdata;
        end

        else begin
            // write hit
            sram_wdata[154] = 1; // dirty
            for (i=0; i<32; i=i+1) begin
                sram_wdata[(proc_addr_offset<<5)+i] = proc_wdata[i];
            end
        end
    end

//// Combinational Logic ////
    assign sram_rdata = entry[way];
    assign sram_hit = hit[0] || hit[1];
    assign lru_nxt = ~way;

    always @(*) begin
        for (i=0; i<2; i=i+1) begin
            entry[i] = sram[proc_addr_index][i];
            valid[i] = entry[i][155];
            tag[i] = entry[i][153:128];
            hit[i] = valid[i] && tag[i] == proc_addr_tag;
        end
    end

    always @(*) begin
        if (hit[0]) begin
            way = 0;
        end
        else if (hit[1]) begin
            way = 1;
        end
        else begin
            way = lru[proc_addr_index];
        end
    end

//// Finite-State Machine ////
// Next State Logic
    always @(*) begin
        state_nxt = state;

        case (state)
            STATE_IDLE: begin
                    if (proc_stall && !stall_out) begin
                        if (sram_dirty) begin
                            state_nxt = STATE_WRITEBACK;
                        end
                        
                        else begin
                            state_nxt = STATE_ALLOCATE;
                        end
                    end
                // end
            end

            STATE_WRITEBACK: begin
                state_nxt = STATE_ALLOCATE;
            end

            STATE_ALLOCATE: begin
                if (mem_ready) begin
                    state_nxt = STATE_READY;
                end
            end

            STATE_READY: begin
                state_nxt = STATE_IDLE;
            end
        endcase
    end

// State Register
    always @(posedge clk) begin
        if (proc_reset) begin
            state <= STATE_IDLE;
        end

        else begin
            state <= state_nxt;
        end
    end

assign mem_addr_r = (state == STATE_ALLOCATE) ? read_addr : buf_addr;
assign mem_read = (state == STATE_READY) ? 0 : mem_read_out;
// Output Logic
    always @(*) begin
        mem_read_r = 0;
        buffer_wen = 0;
        buffer_addr_w = buffer_addr_r;
        sram_write = 0;
        read_addr = proc_addr[29:2];
        case (state)
            STATE_IDLE: begin
                sram_write = proc_write && sram_hit;
            end

            STATE_WRITEBACK: begin
                buffer_wen = 1;
                buffer_addr_w = {sram_tag, proc_addr_index};
            end

            STATE_ALLOCATE: begin
                mem_read_r = 1;
                read_addr = proc_addr[29:2];
            end

            STATE_READY: begin
                sram_write = 1;
            end
        endcase
    end
//==== sequential circuit =================================
    always @(posedge clk) begin
        if (proc_reset) begin
            for (i=0; i<8; i=i+1) begin
                sram[i/2][i%2] <= 0;
            end
        end

        else if (sram_write) begin
            sram[proc_addr_index][way] <= sram_wdata;
        end
    end

    always @(posedge clk) begin
        if (proc_reset) begin
            for (i=0; i<4; i=i+1) begin
                lru[i] <= 0;
            end
        end
        else if (sram_write) begin
            lru[proc_addr_index] <= lru_nxt;
        end
    end
    always @(posedge clk) begin
        if (proc_reset) begin
            buffer_addr_r <= 0;
            mem_addr <= 0;
            mem_wdata <= 0; 
            mem_read_out <= 0;
            // mem_write <= 0;
            // proc_stall <= 0;
        end
        else begin 
            buffer_addr_r <= buffer_addr_w;
            mem_addr <= mem_addr_r;
            mem_wdata <= mem_wdata_r;
            mem_read_out <= mem_read_r;
            // mem_write <= mem_write_r;
            // proc_stall <= proc_stall_r;
        end
    end
endmodule

module buffer(clk, reset, ready, w_en, addr_in, data_in, mem_read, stall_out, mem_wen, addr_out, data_out);
    input clk;
    input reset;
    input ready;
    input w_en;
    input [27:0] addr_in;
    input [127:0] data_in;
    input mem_read;
    output reg stall_out;
    output reg mem_wen;
    output reg [27:0] addr_out;
    output [127:0] data_out;
    reg [127:0] dataout_w, dataout_r;
	// reg flag;
	// reg [1:0] write_ptr;
    localparam WRITE  = 0;
    localparam WAIT_READY  = 1;
    reg state_w, state_r;
    assign data_out = dataout_r;
    always @(*) begin
        state_w = state_r;
        dataout_w = dataout_r;
        case (state_r)
            WRITE: begin
                if (w_en) begin
                    state_w = WAIT_READY;
                    dataout_w = data_in;
                end
            end

            WAIT_READY: begin
                if (mem_wen && ready) state_w = WRITE;
            end
        endcase
    end
	always @(*) begin
        case (state_r)
            
            WRITE: begin
                mem_wen = 0;
                addr_out = addr_in;
                stall_out = 0;
            end

            WAIT_READY: begin
                if(mem_read) mem_wen = 0;
                else mem_wen = 1;
                addr_out = addr_in;
                stall_out = 1;
            end
        endcase
	end

    always @(posedge clk) begin
        if (reset) begin
            state_r <= WRITE;
            dataout_r <= 0;
        end

        else begin
            state_r <= state_w;
            dataout_r <= dataout_w;
        end
    end
endmodule