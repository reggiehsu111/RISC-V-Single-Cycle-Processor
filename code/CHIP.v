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

    assign rs1 = Instruction[19:15];
    assign rs2 = Instruction[24:20];
    assign rd = Instruction[11:7];
    
    // Todo: other wire/reg
    // wires
    wire jump;  // to determine whether PC should jump
    wire [31:0] Sum ; // Sum to calculate PC if jumped
    wire [31:0] Instruction; // Instruction fetched from text memory
    wire [31:0] Add1; // added value (PC or rd1), depending on JALR
    wire AUIPC, JALR, JAL, Branch, MemRead, MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite;
    wire [63:0] ImmGen_out;

    assign Add1 = (JALR)?rs1_data:PC;
    assign Sum = Add1 + (ImmGen_out << 1);

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

    ImmGen immgen0(
        .clk(clk),
        .rst_n(rst_n),
        .inst(Instruction),
        .imm(ImmGen_out))
    // Combinational circuit
    always @(*) begin
        if (JAL || JALR || (Branch && ALU_Zero)) begin
            PC_nxt = Sum;
        end
        else begin
            PC_nxt = PC + 4;
        end
    end

    // Fetch
    always @(*) begin
        mem_addr_I = PC;
        Instruction = mem_rdata_I;

    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
            
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

module multDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW3

endmodule