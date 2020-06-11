module ImmGen(clk, rst_n, inst, imm);
    input         clk, rst_n;
    input  [31:0] inst;
    output [63:0] imm;

    reg    [63:0] imm_nxt;

    always @(*) begin
        case(inst[6:0])
            // I
            7'b0000011: // lw
            7'b0010011: // addi slti
            7'b1100111: // jalr
                begin
                    imm_nxt = { 52{inst[31]}, inst[31:20] }
                end
            
            // S
            7'b0100011: // sw
                begin
                    imm_nxt = { 52{inst[31]}, inst[31:25], inst[11:7] }
                end
            
            // SB
            7'b1100011: // beq
                begin
                    imm_nxt = { 53{inst[31]}, inst[7], inst[30:25], inst[11:8] }
                end

            // U
            7'b0010111: // auipc
                begin
                    imm_nxt = { 32'b{inst[31]}, inst[31:12], 12'b0 }
                end

            // UJ
            7'b1101111: // jal
                begin
                    imm_nxt = { 32'b{inst[31]}, inst[19:12], inst[20], inst[30:21], 1'b0 }
                end

            default:
                begin
                    imm_nxt = 1'b0;
                end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imm <= 0;
        end
        else begin
            imm <= imm_nxt;
        end       
    end
endmodule