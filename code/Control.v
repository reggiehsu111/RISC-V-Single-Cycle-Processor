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