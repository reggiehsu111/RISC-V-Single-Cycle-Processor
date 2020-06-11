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