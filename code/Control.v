module Control(clk, rst_n, inst, AUIPC, JALR, JAL, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite, AlUOp);
    input clk, rst_n;
    input [6:0] inst;
    output AUIPC, JALR, JAL, Branch, MemRead, MemToReg, MemWrite, ALUSrc, RegWrite;
    output [1:0] ALUOp;

    reg    AUIPC_nxt, JALR_nxt, JAL_nxt, Branch_nxt, MemRead_nxt, MemToReg_nxt, MemWrite_nxt, ALUSrc_nxt, RegWrite_nxt;
    reg [1:0] ALUOp_nxt;

    always @(*) begin
        case(inst)
            // R add, sub
            7'b0110011: begin
                ALUSrc_nxt = 0;
                MemToReg_nxt = 0;
                RegWrite_nxt = 1;
                MemRead_nxt = 0;
                MemWrite_nxt = 0;
                Branch = 0;
                ALUOp_nxt = 2'b10;
                AUIPC_nxt = 0;
                JALR_nxt = 0;
                JAL_nxt = 0;

            end
            7'b0010011: begin // addi slti
                ALUSrc_nxt = 1;
                MemToReg_nxt = 0;
                RegWrite_nxt = 1;
                MemRead_nxt = 0;
                MemWrite_nxt = 0;
                Branch = 0;
                ALUOp_nxt = 2'b10;
                AUIPC_nxt = 0;
                JALR_nxt = 0;
                JAL_nxt = 0;
            end
            7'b0000011: begin// lw
                ALUSrc_nxt = 1;
                MemToReg_nxt = 1;
                RegWrite_nxt = 1;
                MemRead_nxt = 1;
                MemWrite_nxt = 0;
                Branch = 0;
                ALUOp_nxt = 2'b00;
                AUIPC_nxt = 0;
                JALR_nxt = 0;
                JAL_nxt = 0;
            end
            7'b1100111: begin// jalr
                ALUSrc_nxt = 0;
                MemToReg_nxt = 0;
                RegWrite_nxt = 1;
                MemRead_nxt = 0;
                MemWrite_nxt = 0;
                Branch = 0;
                ALUOp_nxt = 2'b00;
                AUIPC_nxt = 0;
                JALR_nxt = 1;
                JAL_nxt = 0;
            end
            
            // S
            7'b0100011: begin //sw
                ALUSrc_nxt = 1;
                MemToReg_nxt = 0;
                RegWrite_nxt = 0;
                MemRead_nxt = 0;
                MemWrite_nxt = 1;
                Branch = 0;
                ALUOp_nxt = 2'b00;
                AUIPC_nxt = 0;
                JALR_nxt = 0;
                JAL_nxt = 0;
            end
            
            // SB
            7'b1100011: begin // beq
                ALUSrc_nxt = 0;
                MemToReg_nxt = 0;
                RegWrite_nxt = 0;
                MemRead_nxt = 0;
                MemWrite_nxt = 0;
                Branch = 1;
                ALUOp_nxt = 2'b01;
                AUIPC_nxt = 0;
                JALR_nxt = 0;
                JAL_nxt = 0;
            end

            // U
            7'b0010111: begin // auipc
                ALUSrc_nxt = 1;
                MemToReg_nxt = 0;
                RegWrite_nxt = 1;
                MemRead_nxt = 0;
                MemWrite_nxt = 0;
                Branch = 0;
                ALUOp_nxt = 2'b00;
                AUIPC_nxt = 1;
                JALR_nxt = 0;
                JAL_nxt = 0;
            end

            // UJ
            7'b1101111: begin// jal
                ALUSrc_nxt = 0;
                MemToReg_nxt = 0;
                RegWrite_nxt = 1;
                MemRead_nxt = 0;
                MemWrite_nxt = 0;
                Branch = 0;
                ALUOp_nxt = 2'b00;
                AUIPC_nxt = 0;
                JALR_nxt = 0;
                JAL_nxt = 1;
            end

            default:
                begin
                    ALUSrc_nxt = 0;
                    MemToReg_nxt = 0;
                    RegWrite_nxt = 0;
                    MemRead_nxt = 0;
                    MemWrite_nxt = 0;
                    Branch = 0;
                    ALUOp_nxt = 2'b00;
                    AUIPC_nxt = 0;
                    JALR_nxt = 0;
                    JAL_nxt = 0;
                end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ALUSrc <= 0;
            MemToReg <= 0;
            RegWrite <= 0;
            MemRead <= 0;
            MemWrite <= 0;
            Branch <= 0;
            ALUOp <= 0;
            AUIPC <= 0;
            JALR <= 0;
            JAL <= 0;
        end
        else begin
            ALUSrc <= ALUSrc_nxt;
            MemToReg <= MemToReg_nxt;
            RegWrite <= RegWrite_nxt;
            MemRead <= MemRead_nxt;
            MemWrite <= MemWrite_nxt;
            Branch <= Branch_nxt;
            ALUOp <= ALUOp_nxt;
            AUIPC <= AUIPC_nxt;
            JALR <= JALR_nxt;
            JAL <= JAL_nxt;
        end       
    end
endmodule