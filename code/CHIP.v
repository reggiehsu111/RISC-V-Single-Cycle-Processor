// Your code

module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    reg [31:0] PC_nxt_w;
    assign PC_nxt = PC_nxt_w;

    
    // Todo: other wire/reg
    // wires
    wire jump;  // to determine whether PC should jump
    wire [31:0] Sum ; // Sum to calculate PC if jumped
    wire [31:0] Instruction; // Instruction fetched from text memory
    wire [31:0] Add1; // added value (PC or rd1), depending on JALR
    wire AUIPC, JALR, JAL, Branch, MemRead, MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite, ALU_Zero;
    wire [63:0] ImmGen_out, ALU_result;
    wire [3:0] ALU_Ctrl_out;
    wire m1, m2;

    assign rs1 = Instruction[19:15];
    assign rs2 = Instruction[24:20];
    assign rd = Instruction[11:7];

    assign Add1 = (JALR)?rs1_data:PC;
    assign Sum = Add1 + (ImmGen_out << 1);

    // fetch
    assign mem_addr_I = PC;
    assign Instruction = mem_rdata_I;

    // assign wires to data memory
    assign mem_wen_D = (!MemRead && MemWrite); //write when condition is true, else read
    assign mem_addr_D = ALU_result;
    assign mem_wdata_D = rs2_data;

    // where does the data to be stored in register come from?
    assign m1 = (MemToReg)? mem_rdata_D:ALU_result; // from alu result or data memory
    assign m2 = (AUIPC)? Sum:m1;
    assign rd_data = (JAL || JALR)? PC+4:m2;

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit(
    Control control0(
        .clk(clk),
        .rst_n(rst_n),
        .inst(Instruction[6:0]),
        .AUIPC(AUIPC),
        .JALR(JALR),
        .JAL(JAL),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemToReg(MemToReg),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite),
        .ALUOp(ALUOp));

    ALU_Ctrl alu_ctrl0(
        .clk(clk), 
        .rst_n(rst_n), 
        .ALUOp(ALUOp), 
        .inst_30(Instruction[30]), 
        .inst_14_12(Instruction[14:12]), 
        .ALU_Ctrl_out(ALU_Ctrl_out));

    ImmGen immgen0(
        .clk(clk),
        .rst_n(rst_n),
        .inst(Instruction),
        .imm(ImmGen_out));

    ALU alu0(
    .clk(clk), 
    .rst_n(rst_n), 
    .ALUSrc(ALUSrc),
    .read_data_1(rs1_data),
    .imm_gen_output(ImmGen_out),
    .read_data_2(rs2_data),
    .ALU_control(ALU_Ctrl_out),
    .zero(ALU_Zero), 
    .ALU_result(ALU_result));


    // Combinational circuit
    always @(*) begin
        if (JAL || JALR || (Branch && ALU_Zero)) begin
            PC_nxt_w = Sum;
        end
        else begin
            PC_nxt_w = PC + 4;
        end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt_w;
            
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

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
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module multDiv(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid, mode; // mode: 0: multu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 2'b00;
    parameter MULT = 2'b01;
    parameter DIV  = 2'b10;
    parameter OUT  = 2'b11;

    // Todo: Wire and reg
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo 5: wire assignments
    wire [31:0] temp_shreg;
    reg ready_reg;

    assign out = shreg;
    assign temp_shreg = shreg[63:32] - alu_in;
    assign ready = ready_reg;
    
    // Combinational always block
    // Todo 1: State machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (!valid) state_nxt = IDLE;
                else begin
                    if (mode) state_nxt = DIV;
                    else state_nxt = MULT;
                end 
            end
            MULT: begin
                if (counter != 5'b11111) state_nxt = MULT;
                else state_nxt = OUT;
            end
            DIV : begin
                if (counter != 5'b11111) state_nxt = DIV;
                else state_nxt = OUT;
            end
            OUT : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MULT: counter_nxt = counter + 1;
            DIV: counter_nxt = counter + 1;
            default: counter_nxt = 0;
        endcase
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MULT: begin
                if (shreg[0]) begin
                    alu_out = shreg[63:32] + alu_in;
                end
                else begin
                    alu_out[32] = 1'b0;
                    alu_out[31:0] = shreg[63:32];
                end
            end
            DIV : begin
                alu_out = shreg[62:31] - alu_in;
            end
            default: alu_out = 32'b0;
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) begin
                    shreg_nxt[63:32] = 32'b0;
                    shreg_nxt[31:0] = in_A;
                end
                else shreg_nxt = 0;
            end
            MULT: begin
            // shift right
                shreg_nxt[63:31] = alu_out[32:0];
                shreg_nxt[30:0] = shreg[31:1];
            end
            DIV: begin
                if (alu_out[32]) begin
                    shreg_nxt[63:1] = shreg[62:0];
                    shreg_nxt[0] = 1'b0;
                end
                else begin
                    shreg_nxt[63:32] = alu_out[31:0];
                    shreg_nxt[31:1] = shreg[30:0];
                    shreg_nxt[0] = 1'b1;
                end
            end
            default: shreg_nxt = 0;
        endcase
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            shreg <= 64'b0;
            counter <= 5'b0;
            alu_in <= 32'b0;
            ready_reg <= 1'b0;
        end
        else begin
            state <= state_nxt;
            shreg <= shreg_nxt;
            counter <= counter_nxt;
            alu_in <= alu_in_nxt;
            if (counter == 5'b11111) begin
                ready_reg <= 1'b1;
            end
            else ready_reg <= 1'b0;
        end
    end

endmodule

module ImmGen(clk, rst_n, inst, imm);
    input         clk, rst_n;
    input  [31:0] inst;
    output [63:0] imm;

    reg [63:0] imm_w;
    assign imm = imm_w;

    always @(*) begin
        case(inst[6:0])
            // I
            7'b1100111: // jalr
                begin
                    imm_w = { {52{inst[31]}}, inst[31:20] };
                end
            
            // S
            7'b0100011: // sw
                begin
                    imm_w = { {52{inst[31]}}, inst[31:25], inst[11:7] };
                end
            
            // SB
            7'b1100011: // beq
                begin
                    imm_w = { {53{inst[31]}}, inst[7], inst[30:25], inst[11:8] };
                end

            // U
            7'b0010111: // auipc
                begin
                    imm_w = { {32{inst[31]}}, inst[31:12], 12'b0 };
                end

            // UJ
            7'b1101111: // jal
                begin
                    imm_w = { {32{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0 };
                end

            default:
                begin
                    imm_w = 0;
                end
        endcase
    end
endmodule

module Control(clk, rst_n, inst, AUIPC, JALR, JAL, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite, ALUOp);
    input clk, rst_n;
    input [6:0] inst;
    output AUIPC, JALR, JAL, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;
    output [1:0] ALUOp;

    reg AUIPC_w, JALR_w, JAL_w, Branch_w, MemRead_w, MemToReg_w, MemWrite_w, ALUSrc_w, RegWrite_w;
    reg [1:0] ALUOp_w;

    assign AUIPC = AUIPC_w;
    assign JALR = JALR_w;
    assign JAL = JAL_w;
    assign Branch = Branch_w;
    assign MemRead = MemRead_w;
    assign MemWrite = MemWrite_w;
    assign MemToReg = MemToReg_w;
    assign ALUSrc = ALUSrc_w;
    assign RegWrite = RegWrite_w;
    assign ALUOp = ALUOp_w;

    always @(*) begin
        case(inst)
            // R add, sub
            7'b0110011: begin
                ALUSrc_w = 0;
                MemToReg_w = 0;
                RegWrite_w = 1;
                MemRead_w = 0;
                MemWrite_w = 0;
                Branch_w = 0;
                ALUOp_w = 2'b10;
                AUIPC_w = 0;
                JALR_w = 0;
                JAL_w = 0;

            end
            7'b0010011: begin // addi slti
                ALUSrc_w = 1;
                MemToReg_w = 0;
                RegWrite_w = 1;
                MemRead_w = 0;
                MemWrite_w = 0;
                Branch_w = 0;
                ALUOp_w = 2'b10;
                AUIPC_w = 0;
                JALR_w = 0;
                JAL_w = 0;
            end
            7'b0000011: begin// lw
                ALUSrc_w = 1;
                MemToReg_w = 1;
                RegWrite_w = 1;
                MemRead_w = 1;
                MemWrite_w = 0;
                Branch_w = 0;
                ALUOp_w = 2'b00;
                AUIPC_w = 0;
                JALR_w = 0;
                JAL_w = 0;
            end
            7'b1100111: begin// jalr
                ALUSrc_w = 0;
                MemToReg_w = 0;
                RegWrite_w = 1;
                MemRead_w = 0;
                MemWrite_w = 0;
                Branch_w = 0;
                ALUOp_w = 2'b00;
                AUIPC_w = 0;
                JALR_w = 1;
                JAL_w = 0;
            end
            
            // S
            7'b0100011: begin //sw
                ALUSrc_w = 1;
                MemToReg_w = 0;
                RegWrite_w = 0;
                MemRead_w = 0;
                MemWrite_w = 1;
                Branch_w = 0;
                ALUOp_w = 2'b00;
                AUIPC_w = 0;
                JALR_w = 0;
                JAL_w = 0;
            end
            
            // SB
            7'b1100011: begin // beq
                ALUSrc_w = 0;
                MemToReg_w = 0;
                RegWrite_w = 0;
                MemRead_w = 0;
                MemWrite_w = 0;
                Branch_w = 1;
                ALUOp_w = 2'b01;
                AUIPC_w = 0;
                JALR_w = 0;
                JAL_w = 0;
            end

            // U
            7'b0010111: begin // auipc
                ALUSrc_w = 1;
                MemToReg_w = 0;
                RegWrite_w = 1;
                MemRead_w = 0;
                MemWrite_w = 0;
                Branch_w = 0;
                ALUOp_w = 2'b00;
                AUIPC_w = 1;
                JALR_w = 0;
                JAL_w = 0;
            end

            // UJ
            7'b1101111: begin// jal
                ALUSrc_w = 0;
                MemToReg_w = 0;
                RegWrite_w = 1;
                MemRead_w = 0;
                MemWrite_w = 0;
                Branch_w = 0;
                ALUOp_w = 2'b00;
                AUIPC_w = 0;
                JALR_w = 0;
                JAL_w = 1;
            end

            default:
                begin
                    ALUSrc_w = 0;
                    MemToReg_w = 0;
                    RegWrite_w = 0;
                    MemRead_w = 0;
                    MemWrite_w = 0;
                    Branch_w = 0;
                    ALUOp_w = 2'b00;
                    AUIPC_w = 0;
                    JALR_w = 0;
                    JAL_w = 0;
                end
        endcase
    end

endmodule

module ALU_Ctrl(clk, rst_n, ALUOp, inst_30, inst_14_12, ALU_Ctrl_out);
    input clk, rst_n;
    input [1:0] ALUOp;
    input inst_30;
    input [2:0] inst_14_12; 
    output [3:0] ALU_Ctrl_out;

    reg [3:0] ALU_Ctrl_out_w;
    assign ALU_Ctrl_out = ALU_Ctrl_out_w;


    always @(*) begin
        case(ALUOp)
            2'b00: begin
                ALU_Ctrl_out_w = 4'b0010;
            end
            2'b01: begin
                ALU_Ctrl_out_w = 4'b0110;
            end
            2'b10: begin
                if (inst_14_12[1] || inst_30) begin
                    // subtract
                    ALU_Ctrl_out_w = 4'b0110;
                end
                else begin
                    ALU_Ctrl_out_w = 4'b0010;
                end
            end
            default: begin
                ALU_Ctrl_out_w = 0;
            end
        endcase
    end

endmodule
module ALU(
    clk, 
    rst_n, 
    ALUSrc,
    read_data_1,
    imm_gen_output,
    read_data_2,
    ALU_control,
    zero, 
    ALU_result
);

    input         clk, rst_n, ALUSrc;
    input  [63:0] imm_gen_output, read_data_2, read_data_1;
    input   [3:0] ALU_control;
    output zero;
    output [63:0] ALU_result;



    wire [63:0] mux_output;
    reg [63:0] ALU_result_w;
    assign ALU_result = ALU_result_w;
    
    parameter AND = 4'b0000;
    parameter OR  = 4'b0001;
    parameter ADD = 4'b0010;
    parameter SUB = 4'b0110;

    assign mux_output = (ALUSrc)?imm_gen_output:read_data_2;
    assign zero = (read_data_2-read_data_1==0);

    always @(*) begin
        case(ALU_control)
            AND:     ALU_result_w = read_data_1 & mux_output;
            OR:      ALU_result_w = read_data_1 | mux_output;
            ADD:     ALU_result_w = read_data_1 + mux_output;
            SUB:     ALU_result_w = read_data_1 - mux_output;
            default: ALU_result_w = 0;
        endcase
    end

endmodule
