module I_cache(
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
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    // internal
    reg         valid     [0:7];
    reg         valid_nxt [0:7];
    // reg         dirty     [0:7];
    // reg         dirty_nxt [0:7];
    reg [24:0]  tag       [0:7];
    reg [24:0]  tag_nxt   [0:7];
    reg [127:0] data      [0:7];
    reg [127:0] data_nxt  [0:7];
    reg hit;
    // output
    reg proc_stall_c;
    reg [31:0] proc_rdata_c;
    reg mem_read_c;
    // reg mem_write_c;
    reg [27:0] mem_addr_c;
    // reg [127:0] mem_wdata_c;
    reg test;

    wire askToDo;
    assign askToDo = proc_read || proc_write;
    
    integer i;

//==== combinational circuit ==============================
    //assign
    assign proc_stall = proc_stall_c;
    assign proc_rdata = proc_rdata_c;
    assign mem_read = mem_read_c;
    assign mem_write = 0;
    assign mem_addr = mem_addr_c;
    assign mem_wdata = 0;
    
    always @(*) begin
        hit = ( valid[proc_addr[4:2]] && ( proc_addr[29:5] == tag[proc_addr[4:2]] ) );
        //revise
        if((~askToDo) || hit) begin
            proc_stall_c = 0;
        end 
        else begin
            proc_stall_c = 1;
        end   
    end

    always @(*) begin //output

        for(i=0;i<8;i=i+1) begin
            valid_nxt[i] = valid[i];
            // dirty_nxt[i] = dirty[i];
            tag_nxt[i]   = tag[i];
            data_nxt[i]  = data[i];
        end
        proc_rdata_c = 0;
        mem_read_c = 0;
        // mem_write_c = 0;
        mem_addr_c = 0;
        // mem_wdata_c = 0;

        if(hit) begin
            if(proc_read) begin
                case (proc_addr[1:0])
                    2'd0: proc_rdata_c = data[proc_addr[4:2]][31:0];
                    2'd1: proc_rdata_c = data[proc_addr[4:2]][63:32];
                    2'd2: proc_rdata_c = data[proc_addr[4:2]][95:64];
                    2'd3: proc_rdata_c = data[proc_addr[4:2]][127:96];
                    default: proc_rdata_c = 32'd0;
                endcase
                mem_read_c = 0;
                // mem_write_c = 0;
            end
            // if(proc_write) begin
            //     case (proc_addr[1:0])
            //         2'd0: data_nxt[proc_addr[4:2]] = {data[proc_addr[4:2]][127:32], proc_wdata};
            //         2'd1: data_nxt[proc_addr[4:2]] = {data[proc_addr[4:2]][127:64], proc_wdata, data[proc_addr[4:2]][31:0]};
            //         2'd2: data_nxt[proc_addr[4:2]] = {data[proc_addr[4:2]][127:96], proc_wdata, data[proc_addr[4:2]][63:0]};
            //         2'd3: data_nxt[proc_addr[4:2]] = {proc_wdata, data[proc_addr[4:2]][95:0]};
            //         default: data_nxt[proc_addr[4:2]] = data[proc_addr[4:2]];
            //     endcase
            //     // dirty_nxt[proc_addr[4:2]] = 1;
            // end
        end
        else begin //miss
            if(mem_ready) begin
                if(askToDo) begin
                    // if(dirty[proc_addr[4:2]]) begin
                    //     mem_read_c = 0;
                    //     mem_write_c = 0;
                    //     dirty_nxt[proc_addr[4:2]] = 0;
                    // end
                    begin
                        mem_read_c = 0;
                        // mem_write_c = 0;
                        valid_nxt[proc_addr[4:2]] = 1;
                        // dirty_nxt[proc_addr[4:2]] = 0;
                        tag_nxt[proc_addr[4:2]] = proc_addr[29:5];
                        data_nxt[proc_addr[4:2]] = mem_rdata;
                    end
                end
            end
            else begin
                if(askToDo) begin
                    // if(dirty[proc_addr[4:2]]) begin
                    //     proc_rdata_c = 0;
                    //     mem_read_c = 0;
                    //     // mem_write_c = 1;
                    //     mem_addr_c = {tag[proc_addr[4:2]], proc_addr[4:2]};
                    //     // mem_wdata_c = data[proc_addr[4:2]];
                    // end
                    begin
                        mem_read_c = 1;
                        // mem_write_c = 0;
                        mem_addr_c = proc_addr[29:2];
                    end
                end
            end
        end       
    end

    

//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for(i=0;i<8;i=i+1) begin
                valid[i] <= 0;
                // dirty[i] <= 0; 
            end
        end
        else begin
            for(i=0;i<8;i=i+1) begin
                valid[i] <= valid_nxt[i];
                // dirty[i] <= dirty_nxt[i];
            end
        end
    end

    always @( posedge clk ) begin
        for(i=0;i<8;i=i+1) begin
            tag[i] <= tag_nxt[i];
            data[i] <= data_nxt[i];
        end
    end

endmodule