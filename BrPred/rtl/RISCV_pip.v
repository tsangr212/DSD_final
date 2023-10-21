module RISCV_Pipeline(
		// control interface
		clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata,
//--------------PC-----------------
		PC
	);

////////// wire/reg def //////////
input clk,rst_n;
input ICACHE_stall,DCACHE_stall;
input [31:0] ICACHE_rdata,DCACHE_rdata;
output ICACHE_wen,ICACHE_ren,DCACHE_wen,DCACHE_ren;
output [29:0] ICACHE_addr,DCACHE_addr;
output [31:0] ICACHE_wdata,DCACHE_wdata;
output [31:0] PC;


// IF/ID
reg [31:0] ins_ID_w,ins_ID_r;
reg [31:0] PC_ID_w,PC_ID_r,PCnxt_ID_r;
//reg branch_pred_ID_r;
reg [31:0] addr_untaken_ID;

// ID/EX
reg [3:0] ALU_op_EX_r;
reg jalr_EX_r,jal_EX_r,branch_EX_r,MemRead_EX_r,MemtoReg_EX_r,MemWrite_EX_r,ALUSrc_EX_r,RegWrite_EX_r;
reg [31:0] r_data1_EX_r,r_data2_EX_r,imm_EX_r;
reg [4:0] rd_EX_r,rs1_EX_r,rs2_EX_r;
reg [31:0] PC_EX_r,PCnxt_EX_r;
//reg branch_pred_EX_r;
reg [31:0] addr_untaken_EX_r;



// EX/MEM
reg [31:0] r_data2_MEM_r;
//reg [31:0] branch_addr_MEM_r;
reg [4:0] rd_MEM_r;
reg [31:0] ALU_out_MEM_r;
reg MemRead_MEM_r,MemtoReg_MEM_r,MemWrite_MEM_r,RegWrite_MEM_r,jal_MEM_r,jalr_MEM_r,branch_MEM_r;

//MEM
reg PC_src;
reg [31:0] branch_addr_final;

// MEM/WB
reg [31:0] ALU_out_WB_r;
reg [31:0] m_data_WB_r;
reg MemtoReg_WB_r,RegWrite_WB_r;
reg [4:0] rd_WB_r;

// other
wire [31:0] data_WB;
reg [31:0] jal_addr_ID;
reg [31:0] jalr_addr_EX;

//control signal (ID)
reg jalr,jal,branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;

//----- data hazard -----//
reg data_hazard_1_EX,data_hazard_2_EX,data_hazard_1_MEM,data_hazard_2_MEM;
reg load_hazard;
reg branch_miss;
reg branch_result_EX;

////////// combinatial part //////////




// IF stage //
reg [31:0] PC_r,PC_w;
reg [31:0] PC4_IF,PC2_IF,PC0_IF,PCnxt_IF;
wire flush_branch,flush_jal;
wire i_state;
wire stall;
wire [31:0] addr_pred; // address predicted, address not predicted
wire [31:0] instruction_raw,instruction_IF;
wire [1:0] PC_jump2;

assign PC = PC_r;
assign flush_branch = (branch_miss || jalr_EX_r);
assign flush_jal = jal; //ID stage jal

assign ICACHE_ren = 1;
assign ICACHE_wen = 0; //no writing in I memory
assign ICACHE_addr = (i_state)? PC2_IF[31:2] : PC[31:2];
assign ICACHE_wdata = 0;

assign stall = ICACHE_stall || DCACHE_stall;
assign instruction_raw = {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};

/*
branch_prediction bp1 ( // Prediction
	.clk(clk),
	.rst_n(rst_n),
	.PC(PC_r),
	.PC_nxt(PCnxt_IF),
	.instruction(instruction_IF),
	.addr_pred(addr_pred),
	.addr_untaken(addr_untaken_IF),
	.branch_pred(branch_pred_IF),
	.branch_EX(branch_EX_r),
	.branch_IF(branch_IF),
	.branch_result(branch_result_EX)
	);
*/

decompression dp1(
	.clk(clk),
	.rst_n(rst_n),
	.instruction_raw(instruction_raw),
	.PC(PC_r[1]),
	.stall1(ICACHE_stall),
	.stall2(DCACHE_stall),
	.flush1(flush_branch),
	.flush2(flush_jal),
	.PC_jump2(PC_jump2),
	.instruction_IF(instruction_IF),
	.i_state(i_state)
	);

always @(*) begin // PC
	PC4_IF = PC_r + 4;
	PC2_IF = PC_r + 2;
	PC0_IF = PC_r;
	if (PC_jump2 == 2'b01) PCnxt_IF = PC2_IF;
	else if (PC_jump2 == 2'b10) PCnxt_IF = PC0_IF;
	else PCnxt_IF = PC4_IF;		

	if (stall || load_hazard) PC_w = PC_r;
	else if (branch_miss) PC_w = addr_untaken_EX_r; // from EX stage
	else if (jalr_EX_r) PC_w = jalr_addr_EX; // from EX stage
	else if (jal) 		PC_w = jal_addr_ID; //from ID stage
	//else if (branch_IF) PC_w = addr_pred;
	else PC_w = PCnxt_IF; //always not taken
end

always @(*) begin // data pass to next stage
	ins_ID_w = instruction_IF; //TODO: compression
	PC_ID_w = PC_r;
end


// ID stage //
reg [4:0] rs_1,rs_2,rd;
reg [6:0] opcode,func_7;
reg [2:0] func_3;

always @(*) begin //decode
	opcode = ins_ID_r[6:0];
	rs_1 = ins_ID_r[19:15];
	rs_2 = ins_ID_r[24:20];
	rd = ins_ID_r[11:7];
	func_7 = ins_ID_r[31:25];
	func_3 = ins_ID_r[14:12];
end

reg [31:0] r_data1,r_data2;
wire [31:0] data1,data2;
reg_file reg_file1(
	.clk(clk),
 	.rst_n(rst_n),
 	// write : WB stage
 	.wen(RegWrite_WB_r),  
 	.aw(rd_WB_r), 
 	.d(data_WB), 
 	// read : ID stage
 	.a1(rs_1), 
 	.a2(rs_2),
 	.q1(data1),
 	.q2(data2)
	);

// to prevent reg input/output hazard
// WB write register,but ID use this register at the same time.

always @(*) begin 
	if ((rd_WB_r == rs_1) && RegWrite_WB_r && (rs_1 != 0)) r_data1 = data_WB;
	else r_data1 = data1;
	
	if ((rd_WB_r == rs_2) && RegWrite_WB_r && (rs_2 != 0)) r_data2 = data_WB;
	else r_data2 = data2;
	
end


reg [31:0] imm;
always @(*) begin //imm generation
    case (opcode)
        // I-type
        7'b0000011, 7'b1100111, 7'b0010011: begin
	        if(func_3 == 3'b101) begin // SRL/SRA
	        	imm = {{27'b0}, ins_ID_r[24:20]}; 
	        end else begin
	         	imm = {{20{ins_ID_r[31]}}, ins_ID_r[31:20]}; 
	    	end
	    end
        // S-type
        7'b0100011 : imm = {{20{ins_ID_r[31]}}, ins_ID_r[31:25], ins_ID_r[11:7]};
        // Branch
        7'b1100011 : imm = {{20{ins_ID_r[31]}}, ins_ID_r[7], ins_ID_r[30:25], ins_ID_r[11:8], 1'b0};
        // J-type
        7'b1101111 : imm = {{12{ins_ID_r[31]}}, ins_ID_r[19:12], ins_ID_r[20], ins_ID_r[30:21], 1'b0};
        default    : imm = {{20{ins_ID_r[31]}}, ins_ID_r[31:20]};
    endcase
end

reg [31:0] destination;
always @(*) begin // jal address computation
	destination = PC_ID_r + imm;
	jal_addr_ID = destination;
	addr_untaken_ID = destination;
end

always @(*) begin // branch address computation
	
end


always @(*) begin  //main control
    case (opcode)
        // R-type
        7'b0110011 :
            begin
                jalr     = 0;
                jal      = 0;
                branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 1;
            end

        // I-type
        7'b0010011 :
            begin
                jalr     = 0;
                jal      = 0;
                branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
            end

        // sw
        7'b0100011 :
            begin
                jalr     = 0;
                jal      = 0;
                branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 1;
                ALUSrc   = 1;
                RegWrite = 0;
            end

        // lw
        7'b0000011 :
            begin
                jalr     = 0;
                jal      = 0;
                branch   = 0;
                MemRead  = 1;
                MemtoReg = 1;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
            end

        // beq
        7'b1100011 :
            begin
                jalr     = 0;
                jal      = 0;
                branch   = 1;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
            end

        // jal
        7'b1101111 :
            begin
                jalr     = 0;
                jal      = 1;
                branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 1;
            end

        // jalr
        7'b1100111 :
            begin
                jalr     = 1;
                jal      = 0;
                branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
            end

        default :
            begin
                jalr     = 0;
                jal      = 0;
                branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
            end
    endcase
end

reg [3:0] ALU_op;
parameter ADD = 0;
parameter SUB = 1;
parameter AND = 2;
parameter OR  = 3;
parameter SLT = 4;
parameter SRA = 5;
parameter SRL = 6;
parameter XOR = 7;
parameter BEQ = 8;
parameter BNE = 9;
parameter SLL = 10;

always @(*) begin //ALU control
	if(jal || jalr) begin //JAL/JALR
		ALU_op = ADD;
	end else begin
		case (func_3)
			3'b000:
			begin
				if (opcode == 7'b0110011)//R type
				begin 
					if(func_7[5]) ALU_op = SUB;
					else ALU_op = ADD; 
				end 
				else if (opcode == 7'b0010011) // Itype
				begin
					ALU_op = ADD;
				end
				else ALU_op = BEQ;
			end
			3'b001:
			begin
				if (opcode == 7'b0110011 || opcode == 7'b0010011)//R type / I type
				begin 
					ALU_op = SLL; 
				end 
				else ALU_op = BNE;
			end
			3'b010:
			begin
				if (opcode == 7'b0110011 || opcode == 7'b0010011)
				ALU_op = SLT;
				else
				ALU_op = ADD; // SW/LW
			end
			3'b100:
			begin
				ALU_op = XOR;
			end
			3'b101:
			begin
				if(func_7[5]) ALU_op = SRA;
				else ALU_op = SRL;
			end
			3'b110:
			begin
				ALU_op = OR;
			end
			3'b111:
			begin
				ALU_op = AND;
			end
			default: ALU_op = ADD;

		endcase
	end
end


// EX stage //

reg [31:0] ALU_in1,ALU_in2,ALU_in1_for,ALU_in2_for;
//reg [31:0] r_data2_final_EX; // for sw

always @(*) begin // forward control
	if 		(data_hazard_1_EX) 	ALU_in1_for = ALU_out_MEM_r;
	else if (data_hazard_1_MEM) ALU_in1_for = data_WB;
	else ALU_in1_for = r_data1_EX_r;
	
	if 		(data_hazard_2_EX) 	ALU_in2_for = ALU_out_MEM_r;
	else if (data_hazard_2_MEM) ALU_in2_for = data_WB;
	else ALU_in2_for = r_data2_EX_r;

end

always @(*) begin //select rs2 or imm
	ALU_in1 = ALU_in1_for;
	ALU_in2 = (ALUSrc_EX_r) ? imm_EX_r : ALU_in2_for; // I type / R type
end


reg equal;
always @(*) begin // branch verification
	equal = (ALU_in1 == ALU_in2);
	branch_result_EX = (ALU_op_EX_r == BEQ) ? (equal) : (!equal);
	branch_miss = (branch_EX_r && branch_result_EX); 
end

//reg ALU_zero; // whether ALU output zero
reg [31:0] ALU_out,ADD_result; 
reg overflow;

// ALU
reg signed [32:0] add_in1,add_in2; // extend 1 bit to prevent overflow
always @(*) begin //ALU
	add_in1 = {ALU_in1[31],ALU_in1};
	if (ALU_op_EX_r == ADD)
		add_in2 = {ALU_in2[31],ALU_in2}; //ADD
	else
		add_in2 = ~{ALU_in2[31],ALU_in2} + 1; //SUB/BEQ/BNE/SLT

	{overflow,ADD_result} = add_in1 + add_in2; // 32 bit to 31 bit


	case (ALU_op_EX_r)
		AND: ALU_out = ALU_in1 & ALU_in2;
		OR:  ALU_out = ALU_in1 | ALU_in2;
		XOR: ALU_out = ALU_in1 ^ ALU_in2;
		
		SLL: ALU_out = ALU_in1 << ALU_in2[4:0];
		SRL: ALU_out = ALU_in1 >> ALU_in2[4:0];
		SRA: ALU_out = $signed(ALU_in1) >>> ALU_in2[4:0]; 
		
		SLT: ALU_out = overflow;

		default: ALU_out = ADD_result; //ADD/SUB/BEQ/BNE/JALR
	endcase

	//ALU_zero = (ALU_op_EX_r == BEQ) ? (ALU_out == 0) : (ALU_out != 0); //If condition is true, output 1

end


reg [31:0] ALU_out_final;
always @(*) begin  // store JAL/JALR next address
	if (jal_EX_r || jalr_EX_r) begin
		ALU_out_final = PCnxt_EX_r;
	end 
	else ALU_out_final = ALU_out;
end


always @(*) begin // branch address
	jalr_addr_EX = ALU_out; //JALR
end

// MEM stage //

// memory function
assign DCACHE_ren = MemRead_MEM_r;
assign DCACHE_wen = MemWrite_MEM_r;
assign DCACHE_addr = ALU_out_MEM_r[31:2]; // delete byte offset
assign DCACHE_wdata = {r_data2_MEM_r[7:0],r_data2_MEM_r[15:8],r_data2_MEM_r[23:16],r_data2_MEM_r[31:24]};

reg [31:0] m_data_MEM;
always @(*) begin
	m_data_MEM =  {DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]};
end

// branch

/*
always @(*) begin
	branch_addr_final = branch_addr_MEM_r; //JAL/branch use PC + imm, JALR use rs1 + imm
	
	if (jal_MEM_r || jalr_MEM_r) begin
		PC_src = 1;
	end 
	else if (branch_MEM_r && ALU_zero_MEM_r) begin //BEQ/BNE
		PC_src = 1;
	end 
	else begin
		PC_src = 0;
	end

end
*/

// WB stage //
assign data_WB = (MemtoReg_WB_r) ? m_data_WB_r : ALU_out_WB_r;


//----- data hazard -----//
always @(*) begin // forwarding unit
	data_hazard_1_EX = (rs1_EX_r == rd_MEM_r) && (rd_MEM_r != 0) && (RegWrite_MEM_r);
	data_hazard_2_EX = (rs2_EX_r == rd_MEM_r) && (rd_MEM_r != 0) && (RegWrite_MEM_r);
	data_hazard_1_MEM = (rs1_EX_r == rd_WB_r) && (rd_WB_r != 0) && (RegWrite_WB_r) && (rs1_EX_r != rd_MEM_r);
	data_hazard_2_MEM = (rs2_EX_r == rd_WB_r) && (rd_WB_r != 0) && (RegWrite_WB_r) && (rs2_EX_r != rd_MEM_r);

	load_hazard = MemRead_EX_r && ((rs_1 == rd_EX_r) || (rs_2 == rd_EX_r)); //load use hazard
	
end




////////// sequential part //////////
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		PC_r 			<= 0;
		ins_ID_r 		<= 0;
		PC_ID_r 		<= 0;
		jal_EX_r 		<= 0;
		jalr_EX_r		<= 0;
		branch_EX_r 	<= 0;
		MemRead_EX_r	<= 0;
		MemtoReg_EX_r	<= 0;
		MemWrite_EX_r	<= 0;
		ALUSrc_EX_r		<= 0;
		RegWrite_EX_r	<= 0;
		ALU_op_EX_r		<= 0;
		branch_EX_r		<= 0;
		r_data1_EX_r	<= 0;
		r_data2_EX_r	<= 0;
		imm_EX_r		<= 0;
		rd_EX_r 		<= 0;
		rs1_EX_r		<= 0;
		rs2_EX_r		<= 0;
		PC_EX_r			<= 0;
		MemRead_MEM_r	<= 0;
		MemtoReg_MEM_r  <= 0;
		MemWrite_MEM_r	<= 0;
		RegWrite_MEM_r	<= 0;
		jal_MEM_r 		<= 0;
		jalr_MEM_r		<= 0;
		branch_MEM_r 	<= 0;
		r_data2_MEM_r	<= 0;
		rd_MEM_r 		<= 0;
		ALU_out_MEM_r	<= 0;
		MemtoReg_WB_r	<= 0;
		RegWrite_WB_r 	<= 0;
		ALU_out_WB_r	<= 0;
		m_data_WB_r		<= 0;
		rd_WB_r			<= 0;
		PCnxt_EX_r		<= 0;
		PCnxt_ID_r		<= 0;
		addr_untaken_EX_r<= 0;

	end else begin
		//IF stage
		PC_r 			<= PC_w;
		
		//IF/ID
		if(stall || load_hazard) begin
			ins_ID_r 	<= ins_ID_r;
			PC_ID_r 	<= PC_ID_r;
			PCnxt_ID_r	<= PCnxt_ID_r;

		end else if (flush_jal || flush_branch) begin
			ins_ID_r 	<= 0;
			PC_ID_r 	<= 0;
			PCnxt_ID_r	<= 0;

		end else begin
			ins_ID_r 	<= ins_ID_w;
			PC_ID_r 	<= PC_ID_w;
			PCnxt_ID_r	<= PCnxt_IF;
			
		end
		

		//ID/EX
		if (stall) begin
			jal_EX_r 		<= jal_EX_r;
			jalr_EX_r		<= jalr_EX_r;
			branch_EX_r 	<= branch_EX_r;
			MemRead_EX_r	<= MemRead_EX_r;
			MemtoReg_EX_r	<= MemtoReg_EX_r;
			MemWrite_EX_r	<= MemWrite_EX_r;
			ALUSrc_EX_r		<= ALUSrc_EX_r;
			RegWrite_EX_r	<= RegWrite_EX_r;
			ALU_op_EX_r		<= ALU_op_EX_r;
			r_data1_EX_r	<= r_data1_EX_r;
			r_data2_EX_r	<= r_data2_EX_r;
			imm_EX_r		<= imm_EX_r;
			rd_EX_r 		<= rd_EX_r;
			PC_EX_r			<= PC_EX_r;
			PCnxt_EX_r		<= PCnxt_EX_r;
			rs1_EX_r		<= rs1_EX_r;
			rs2_EX_r		<= rs2_EX_r;
			addr_untaken_EX_r<= addr_untaken_EX_r;

		end else if (flush_branch || load_hazard) begin
			jal_EX_r 		<= 0;
			jalr_EX_r		<= 0;
			branch_EX_r 	<= 0;
			MemRead_EX_r	<= 0;
			MemtoReg_EX_r	<= 0;
			MemWrite_EX_r	<= 0;
			ALUSrc_EX_r		<= 0;
			RegWrite_EX_r	<= 0;
			ALU_op_EX_r		<= 0;
			branch_EX_r		<= 0;
			r_data1_EX_r	<= 0;
			r_data2_EX_r	<= 0;
			imm_EX_r		<= 0;
			rd_EX_r 		<= 0;
			PC_EX_r			<= 0;
			PCnxt_EX_r		<= 0;
			rs1_EX_r		<= 0;
			rs2_EX_r		<= 0;
			addr_untaken_EX_r<= 0;
			
		end else begin
			jal_EX_r 		<= jal;
			jalr_EX_r		<= jalr;
			branch_EX_r 	<= branch;
			MemRead_EX_r	<= MemRead;
			MemtoReg_EX_r	<= MemtoReg;
			MemWrite_EX_r	<= MemWrite;
			ALUSrc_EX_r		<= ALUSrc;
			RegWrite_EX_r	<= RegWrite;
			ALU_op_EX_r		<= ALU_op;
			r_data1_EX_r	<= r_data1;
			r_data2_EX_r	<= r_data2;
			imm_EX_r		<= imm;
			rd_EX_r 		<= rd;
			PC_EX_r			<= PC_ID_r;
			PCnxt_EX_r		<= PCnxt_ID_r;
			rs1_EX_r		<= rs_1;
			rs2_EX_r		<= rs_2;
			addr_untaken_EX_r<= addr_untaken_ID;
			
		end
		
		//EX/MEM
		if (stall) begin
			MemRead_MEM_r	<= MemRead_MEM_r;
			MemtoReg_MEM_r  <= MemtoReg_MEM_r;
			MemWrite_MEM_r	<= MemWrite_MEM_r;
			RegWrite_MEM_r	<= RegWrite_MEM_r;
			jal_MEM_r 		<= jal_MEM_r;
			jalr_MEM_r		<= jalr_MEM_r;
			branch_MEM_r 	<= branch_MEM_r;
			r_data2_MEM_r	<= r_data2_MEM_r;
			rd_MEM_r 		<= rd_MEM_r;
			ALU_out_MEM_r	<= ALU_out_MEM_r;
			/*
		end else if (flush) begin
			MemRead_MEM_r	<= 0;
			MemtoReg_MEM_r  <= 0;
			MemWrite_MEM_r	<= 0;
			RegWrite_MEM_r	<= 0;
			jal_MEM_r 		<= 0;
			jalr_MEM_r		<= 0;
			branch_MEM_r 	<= 0;
			r_data2_MEM_r	<= 0;
			rd_MEM_r 		<= 0;
			ALU_out_MEM_r	<= 0;
			ALU_zero_MEM_r	<= 0;
			*/
		end else begin

			MemRead_MEM_r	<= MemRead_EX_r;
			MemtoReg_MEM_r  <= MemtoReg_EX_r;
			MemWrite_MEM_r	<= MemWrite_EX_r;
			RegWrite_MEM_r	<= RegWrite_EX_r;
			jal_MEM_r 		<= jal_EX_r;
			jalr_MEM_r		<= jalr_EX_r;
			branch_MEM_r 	<= branch_EX_r;
			r_data2_MEM_r	<= ALU_in2_for;			// warning: save instruction also needs to consider hazard
			rd_MEM_r 		<= rd_EX_r;
			ALU_out_MEM_r	<= ALU_out_final;
		end
		
		//MEM/WB
		if (stall) begin
			MemtoReg_WB_r	<= MemtoReg_WB_r;
			RegWrite_WB_r 	<= RegWrite_WB_r;
			ALU_out_WB_r	<= ALU_out_WB_r;
			m_data_WB_r		<= m_data_WB_r;
			rd_WB_r			<= rd_WB_r;
		end else begin
			MemtoReg_WB_r	<= MemtoReg_MEM_r;
			RegWrite_WB_r 	<= RegWrite_MEM_r;
			ALU_out_WB_r	<= ALU_out_MEM_r;
			m_data_WB_r		<= m_data_MEM;
			rd_WB_r			<= rd_MEM_r;
		end

	end
end

endmodule


module branch_prediction(clk,rst_n,PC,PC_nxt,instruction,addr_pred,addr_untaken,branch_pred,branch_IF,branch_EX,branch_result);
    input clk, rst_n;
	input [31:0] PC, PC_nxt, instruction;
    input branch_EX, branch_result; // branch result in reality from EX stage

    output [31:0] addr_pred, addr_untaken; // address of taken/ not taken
    output branch_pred; // Prediction: 1: branch, 0: not branch
    output branch_IF; // whether there is a branch instruction

    wire [31:0] imm,addr_jump;

    reg [1:0] pred_state, next_state; // pred_state: 11: strongly taken, 10: weakly taken, 01: weakly untaken, 00: strongly untaken
    //reg next_branch_pred;

    ////////// TODO //////////
    //
	/*
    always @(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			pred_state <= 2'b0;
        	//branch_pred <= 1;
		end

		else begin
			pred_state <= next_state;
        	//branch_pred <= next_branch_pred;
		end
    end

    always @(*) begin
        if (branch_EX == 1) begin
            if (branch_result == 1) begin
                case (pred_state)
                    2'b11: begin
                        next_state = 2'b11;
                        //next_branch_pred = 1;
                    end

                    2'b10: begin
                        next_state = 2'b11;
                        //next_branch_pred = 1;
                    end

                    2'b01: begin
                        next_state = 2'b10;
                        //next_branch_pred = 0;
                    end

                    2'b00: begin
                        next_state = 2'b01;
                        //next_branch_pred = 0;
                    end

                    default: begin
                        next_state = 2'b11;
                        //next_branch_pred = 1;
                    end
                endcase
            end

            else begin
                case (pred_state)
                    2'b11: begin
                        next_state = 2'b10;
                        //next_branch_pred = 1;
                    end

                    2'b10: begin
                        next_state = 2'b01;
                        //next_branch_pred = 1;
                    end

                    2'b01: begin
                        next_state = 2'b00;
                        //next_branch_pred = 0;
                    end

                    2'b00: begin
                        next_state = 2'b00;
                        //next_branch_pred = 0;
                    end

                    default: begin
                        next_state = 2'b10;
                        //next_branch_pred = 1;
                    end
                endcase
            end
        end

        else begin
            next_state = pred_state;
            //next_branch_pred = branch_pred;
        end
    end
*/
    //
    //assign branch_pred = pred_state[1];
	assign branch_pred = 0;
    assign imm = {{20{ instruction[31]}}, instruction[7],  instruction[30:25],  instruction[11:8], 1'b0}; //branch
    assign branch_IF = (instruction[6:0] == 7'b1100011) ? 1 : 0;
    assign addr_jump = PC + imm;
    assign {addr_pred,addr_untaken} = (branch_pred) ? {addr_jump,PC_nxt} : {PC_nxt,addr_jump};

endmodule

module decompression(clk,rst_n,instruction_raw,PC,stall1,stall2,flush1,flush2,PC_jump2,instruction_IF,i_state);
	input [31:0] instruction_raw;
	input PC;
	input clk,rst_n;
	input flush1, flush2, stall1, stall2;
	output reg [31:0] instruction_IF; // decoded instruction
	output [1:0] PC_jump2; //PC + 2
	output i_state;

	reg [15:0] buffer_r, buffer_w, compressed_Ins;
	reg instruction_state_r, instruction_state_w;
	reg [1:0] pct;
	reg [2:0] cas;
	// reg [31:0] stall_w, stall_r;

	always @(*) begin
        if(!PC) begin //PC+4
            if(instruction_raw[1:0] == 2'b11) begin //not compressed
                instruction_state_w = 1'b0;
                compressed_Ins    = instruction_raw[15:0];
                buffer_w        = buffer_r;
				pct 			= 0;
				cas = 1;
            end
			else begin //compressed
				instruction_state_w = 1'b0;
				compressed_Ins    = instruction_raw[15:0];
				buffer_w        = buffer_r;
				pct		  		= 1;
				cas = 3;
			end
		end
        else begin //PC+2
            if(instruction_raw[17:16] == 2'b11 && instruction_state_r == 1'b0) begin //not compressed
                instruction_state_w = 1'b1;
                compressed_Ins    = 16'b0;
                buffer_w        = instruction_raw[31:16];
				pct		  		= 2;
				cas = 4;
            end
            else if(instruction_state_r == 1'b1) begin
                instruction_state_w = 1'b0;
                compressed_Ins    = buffer_r;
                buffer_w          = buffer_r;
				pct		  		  = 0;
				cas = 5;
            end
            else begin //compressed
                instruction_state_w = 1'b0;
                compressed_Ins    = instruction_raw[31:16];
                buffer_w          = buffer_r;
				pct		  		  = 1;
				cas = 6;
            end
        end
	end

	always @(*) begin
		if (compressed_Ins[1:0] == 2'b11) begin //uncompressed
            if (instruction_state_r == 1'b1 && instruction_raw != 32'd0) begin
			 	instruction_IF = {instruction_raw[15:0],buffer_r};
			end
			else begin
				instruction_IF = instruction_raw;
			end
		end
		else begin
			case ({compressed_Ins[15:13], compressed_Ins[1:0]})
			5'b01000 : instruction_IF = {5'b00000, compressed_Ins[5], compressed_Ins[12:10], compressed_Ins[6], 2'b00, 2'b01, compressed_Ins[9:7], 3'b010, 2'b01, compressed_Ins[4:2], 7'b0000011};
					/* c.sw ---------------------------------------------------------------------------*/
				5'b11000 : instruction_IF = {5'b00000, compressed_Ins[5], compressed_Ins[12], 2'b01, compressed_Ins[4:2], 2'b01, compressed_Ins[9:7], 3'b010, compressed_Ins[11:10], compressed_Ins[6], 2'b00, 7'b0100011};
				5'b00001 : begin
					/* c.nop ----------------------------------------------------------------------*/
					if (compressed_Ins[12:2] == 11'b0)
						instruction_IF = {25'b0, 7'b0010011};
					/* c.addi --------------------------------------------------------------------*/
					else instruction_IF = {{7{compressed_Ins[12]}}, compressed_Ins[6:2], compressed_Ins[11:7], 3'b000, compressed_Ins[11:7], 7'b0010011};
				end
				/* c.jal --------------------------------------------------------------------------*/
				5'b00101 : instruction_IF = {compressed_Ins[12], compressed_Ins[8], compressed_Ins[10:9], compressed_Ins[6], compressed_Ins[7], compressed_Ins[2], compressed_Ins[11], compressed_Ins[5:3], compressed_Ins[12], {8{compressed_Ins[12]}}, 5'd1, 7'b1101111};
				5'b10001 : begin
					/* c.andi --------------------------------------------------------------------*/
					if (compressed_Ins[11:10] == 2'b10)
						instruction_IF = {{7{compressed_Ins[12]}}, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b111, 2'b01, compressed_Ins[9:7], 7'b0010011};
					/* Skip instruction ---------------------------------------------------------*/
					else if (compressed_Ins[12] == 1'b0 && compressed_Ins[6:2] == 5'b0)
						instruction_IF = 32'b0;
					/* c.srli --------------------------------------------------------------------*/
					else if (compressed_Ins[11:10] == 2'b00)
						instruction_IF = {7'b0000000, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b101, 2'b01, compressed_Ins[9:7], 7'b0010011};
					/* c.srai --------------------------------------------------------------------*/
					else
						instruction_IF = {7'b0100000, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b101, 2'b01, compressed_Ins[9:7], 7'b0010011};
				end
				/* c.j -----------------------------------------------------------------------*/
				5'b10101 : instruction_IF = {compressed_Ins[12], compressed_Ins[8], compressed_Ins[10:9], compressed_Ins[6], compressed_Ins[7], compressed_Ins[2], compressed_Ins[11], compressed_Ins[5:3], compressed_Ins[12], {8{compressed_Ins[12]}}, 5'd0, 7'b1101111};
				/* c.beqz --------------------------------------------------------------------*/
				5'b11001 : instruction_IF = {{4{compressed_Ins[12]}}, compressed_Ins[6], compressed_Ins[5], compressed_Ins[2], 5'd0, 2'b01, compressed_Ins[9:7], 3'b000, compressed_Ins[11], compressed_Ins[10], compressed_Ins[4], compressed_Ins[3], compressed_Ins[12], 7'b1100011};
				/* c.bnez --------------------------------------------------------------------*/
				5'b11101 : instruction_IF = {{4{compressed_Ins[12]}}, compressed_Ins[6], compressed_Ins[5], compressed_Ins[2], 5'd0, 2'b01, compressed_Ins[9:7], 3'b001, compressed_Ins[11], compressed_Ins[10], compressed_Ins[4], compressed_Ins[3], compressed_Ins[12], 7'b1100011};
				/* c.slli --------------------------------------------------------------------*/
				5'b00010 : instruction_IF = {7'b0000000, compressed_Ins[6:2], compressed_Ins[11:7], 3'b001, compressed_Ins[11:7], 7'b0010011};
				5'b10010 : begin
					if (compressed_Ins[6:2] == 5'd0) begin
						/* c.jalr --------------------------------------------------------------------*/
						if (compressed_Ins[12] && compressed_Ins[11:7] != 5'b0)
							instruction_IF = {12'b0, compressed_Ins[11:7], 3'b000, 5'd1, 7'b1100111};
						/* c.jr --------------------------------------------------------------------*/
						else instruction_IF = {12'b0, compressed_Ins[11:7], 3'b000, 5'd0, 7'b1100111};
					end
					else begin
						/* c.mv --------------------------------------------------------------------*/
						if (compressed_Ins[12] == 1'b0)
							instruction_IF = {7'b0000000, compressed_Ins[6:2], 5'd0, 3'b000, compressed_Ins[11:7], 7'b0110011};
						/* c.add --------------------------------------------------------------------*/
						else instruction_IF = {7'b0000000, compressed_Ins[6:2], compressed_Ins[11:7], 3'b000, compressed_Ins[11:7], 7'b0110011};
					end
				end
				default : instruction_IF = 32'b0;
			endcase
		end
	end

	// assign instruction_IF = (compressed_Ins[1:0] == 2'b11)? //not compressed
	// 			((instruction_state_r == 1'b1 && instruction_raw != 32'd0)? {instruction_raw[15:0],buffer_r} : instruction_raw) : 
	// 		(compressed_Ins == 16'b0)?
	// 			32'd0 :
	// 		(compressed_Ins[1:0] == 2'b00 && compressed_Ins[15:13] == 3'b010)? //c.lw
	// 			{5'b0, compressed_Ins[5], compressed_Ins[12:10], compressed_Ins[6], 2'b0, 2'b01, compressed_Ins[9:7], 3'b010, 2'b01, compressed_Ins[4:2], 7'b0000011} :
	// 		(compressed_Ins[1:0] == 2'b00 && compressed_Ins[15:13] == 3'b110)? //c.sw
	// 			{5'b0, compressed_Ins[5], compressed_Ins[12], 2'b01, compressed_Ins[4:2], 2'b01, compressed_Ins[9:7], 3'b010, compressed_Ins[11:10], compressed_Ins[6], 2'b0, 7'b0100011} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b000)? //c.addi, c.nop
	// 			{{7{compressed_Ins[12]}}, compressed_Ins[6:2], compressed_Ins[11:7], 3'b000, compressed_Ins[11:7], 7'b0010011} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b001)? //c.jal
	// 			{compressed_Ins[12], compressed_Ins[8], compressed_Ins[10:9], compressed_Ins[6], compressed_Ins[7], compressed_Ins[2], compressed_Ins[11], compressed_Ins[5:3], {9{compressed_Ins[12]}}, 5'b1, 7'b1101111} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[11:10] == 2'b10)? //c.andi
	// 			{{7{compressed_Ins[12]}}, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b111, 2'b01, compressed_Ins[9:7], 7'b0010011} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[11:10] == 2'b00)? //c.srli
	// 			{7'b0000000, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b101, 2'b01, compressed_Ins[9:7], 7'b0010011} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[11:10] == 2'b01)? //c.srai
	// 			{7'b0100000, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b101, 2'b01, compressed_Ins[9:7], 7'b0010011} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b101)? //c.j
	// 			{compressed_Ins[12], compressed_Ins[8], compressed_Ins[10:9], compressed_Ins[6], compressed_Ins[7], compressed_Ins[2], compressed_Ins[11], compressed_Ins[5:3], {9{compressed_Ins[12]}}, 5'b0, 7'b1101111} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b110)? //c.beqz
	// 			{{4{compressed_Ins[12]}}, compressed_Ins[6:5], compressed_Ins[2], 5'b0, 2'b01, compressed_Ins[9:7], 3'b000, compressed_Ins[11:10], compressed_Ins[4:3], compressed_Ins[12], 7'b1100011} :
	// 		(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b111)? //c.bnez
	// 			{{4{compressed_Ins[12]}}, compressed_Ins[6:5], compressed_Ins[2], 5'b0, 2'b01, compressed_Ins[9:7], 3'b001, compressed_Ins[11:10], compressed_Ins[4:3], compressed_Ins[12], 7'b1100011} :
	// 		(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b000)? //c.slli
	// 			{7'b0, compressed_Ins[6:2], compressed_Ins[11:7], 3'b001, compressed_Ins[11:7], 7'b0010011} :
	// 		(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[6:2] == 5'b00000 && compressed_Ins[12] == 1'b0)? //c.jr
	// 			{12'b0, compressed_Ins[11:7], 3'b0, 5'b00000, 7'b1100111} :
	// 		(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[6:2] == 5'b00000 && compressed_Ins[12] == 1'b1)? //c.jalr
	// 			{12'b0, compressed_Ins[11:7], 3'b0, 5'b00001, 7'b1100111} :
	// 		(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[12] == 1'b0)? //c.mv
	// 			{7'b0, compressed_Ins[6:2], 5'b0, 3'b0, compressed_Ins[11:7], 7'b0110011} :
	// 		(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[12] == 1'b1)? //c.add
	// 			{7'b0, compressed_Ins[6:2], compressed_Ins[11:7], 3'b0, compressed_Ins[11:7], 7'b0110011} :	{25'b0, 7'b0010011};
	// TODO //

	// assign instruction_IF = instruction_raw;
	assign PC_jump2 = pct;
	assign i_state = instruction_state_r;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			buffer_r <= 0;
			instruction_state_r <= 0;
			// stall_r <= 0;
		end
		else begin
			if (flush1 || flush2) begin
				buffer_r <= 0;
				instruction_state_r <= 0;
			end
			else if (!stall1 && !stall2) begin
				buffer_r <= buffer_w;
				instruction_state_r <= instruction_state_w;
			end
			else begin 
				buffer_r <= buffer_r;
				instruction_state_r <= instruction_state_r;
			end
			// stall_r <= stall_w;
			// pct_r <= pct_w;
		end
	end


endmodule

//read: instant   write: clock
module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw; //address 1/2/write

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;
    assign q1 = mem[a1];
    assign q2 = mem[a2];
    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                mem[i] <= 32'h0;
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule