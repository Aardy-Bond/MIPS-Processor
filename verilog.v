module full_adder (
    input wire a,
    input wire b,
    input wire cin,
    output wire sum,
    output wire carry
);
    assign sum   = a ^ b ^ cin;                      // XOR chain
    assign carry = (a & b) | (b & cin) | (a & cin);  // Carry logic
endmodule

module rca_32bit (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire sub,             
    output wire [31:0] sum,
    output wire cout
);
    wire [31:0] b_xor;
    wire [31:0] carry;

    assign b_xor = b ^ {32{sub}};
    
    full_adder fa0 (
        .a(a[0]),
        .b(b_xor[0]),
        .cin(sub),
        .sum(sum[0]),
        .carry(carry[0])
    );

    genvar i;
    generate
        for (i = 1; i < 32; i = i + 1) begin : fa_chain
            full_adder fa (
                .a(a[i]),
                .b(b_xor[i]),
                .cin(carry[i - 1]),
                .sum(sum[i]),
                .carry(carry[i])
            );
        end
    endgenerate

    assign cout = carry[31];

endmodule

module and_32bit (
    input wire [31:0] a,
    input wire [31:0] b,
    output wire [31:0] result
);
    // Instantiate 32 1-bit AND gates
    and and0 (result[0], a[0], b[0]);
    and and1 (result[1], a[1], b[1]);
    and and2 (result[2], a[2], b[2]);
    and and3 (result[3], a[3], b[3]);
    and and4 (result[4], a[4], b[4]);
    and and5 (result[5], a[5], b[5]);
    and and6 (result[6], a[6], b[6]);
    and and7 (result[7], a[7], b[7]);
    and and8 (result[8], a[8], b[8]);
    and and9 (result[9], a[9], b[9]);
    and and10 (result[10], a[10], b[10]);
    and and11 (result[11], a[11], b[11]);
    and and12 (result[12], a[12], b[12]);
    and and13 (result[13], a[13], b[13]);
    and and14 (result[14], a[14], b[14]);
    and and15 (result[15], a[15], b[15]);
    and and16 (result[16], a[16], b[16]);
    and and17 (result[17], a[17], b[17]);
    and and18 (result[18], a[18], b[18]);
    and and19 (result[19], a[19], b[19]);
    and and20 (result[20], a[20], b[20]);
    and and21 (result[21], a[21], b[21]);
    and and22 (result[22], a[22], b[22]);
    and and23 (result[23], a[23], b[23]);
    and and24 (result[24], a[24], b[24]);
    and and25 (result[25], a[25], b[25]);
    and and26 (result[26], a[26], b[26]);
    and and27 (result[27], a[27], b[27]);
    and and28 (result[28], a[28], b[28]);
    and and29 (result[29], a[29], b[29]);
    and and30 (result[30], a[30], b[30]);
    and and31 (result[31], a[31], b[31]);

endmodule

module or_32bit (
    input wire [31:0] a,
    input wire [31:0] b,
    output wire [31:0] result
);
    // Instantiate 32 1-bit OR gates
    or or0 (result[0], a[0], b[0]);
    or or1 (result[1], a[1], b[1]);
    or or2 (result[2], a[2], b[2]);
    or or3 (result[3], a[3], b[3]);
    or or4 (result[4], a[4], b[4]);
    or or5 (result[5], a[5], b[5]);
    or or6 (result[6], a[6], b[6]);
    or or7 (result[7], a[7], b[7]);
    or or8 (result[8], a[8], b[8]);
    or or9 (result[9], a[9], b[9]);
    or or10 (result[10], a[10], b[10]);
    or or11 (result[11], a[11], b[11]);
    or or12 (result[12], a[12], b[12]);
    or or13 (result[13], a[13], b[13]);
    or or14 (result[14], a[14], b[14]);
    or or15 (result[15], a[15], b[15]);
    or or16 (result[16], a[16], b[16]);
    or or17 (result[17], a[17], b[17]);
    or or18 (result[18], a[18], b[18]);
    or or19 (result[19], a[19], b[19]);
    or or20 (result[20], a[20], b[20]);
    or or21 (result[21], a[21], b[21]);
    or or22 (result[22], a[22], b[22]);
    or or23 (result[23], a[23], b[23]);
    or or24 (result[24], a[24], b[24]);
    or or25 (result[25], a[25], b[25]);
    or or26 (result[26], a[26], b[26]);
    or or27 (result[27], a[27], b[27]);
    or or28 (result[28], a[28], b[28]);
    or or29 (result[29], a[29], b[29]);
    or or30 (result[30], a[30], b[30]);
    or or31 (result[31], a[31], b[31]);

endmodule

module mux_32bit(
    input [31:0] a,
    input [31:0] b,
    input sel,
    output [31:0] o
);
    assign o = (sel==0)?a:b;
endmodule

module mux_5bit(
    input [4:0] a,
    input [4:0] b,
    input sel,
    output [4:0] o
);
    assign o = (sel==0)?a:b;
endmodule

module sign_extend(
    input [15:0] a,
    output [31:0] b
);
    assign b = {{16{a[15]}},a};
endmodule

module pc(
    input clk,
    input rst,
    input [31:0] next_pc,
    output reg [31:0] curr_pc
);
    always@ (posedge clk or posedge rst)begin
        if(rst) begin
            curr_pc<=0;
        end
        else begin
            curr_pc<=next_pc;
        end
    end
endmodule

module reg_file(
    input clk,
    input regWrite,
    input [4:0] ra,//readregister1
    input [4:0] rb,//readregister2
    input [4:0] rc,//writeregister
    input [31:0] wd,//writedata
    output [31:0] od1,//outdata1
    output [31:0] od2 //outdata2
);
    reg [31:0] memory [31:0];
    
    integer i;

    initial begin
        for( i=0; i<32;i=i+1)
            memory[i]=i;
    end
    
    assign od1=memory[ra];
    assign od2=memory[rb];

    always @ (posedge clk)begin
        if(regWrite==1'b1 && rc!=0) begin
            memory[rc]<=wd;
        end
    end
endmodule

module data_mem(
    input clk,
    input memRead,memWrite,
    input [31:0] addr,write_data,
    output reg [31:0] read_data,
    output [15:0] test_value
);
    reg [31:0] memory [255:0];
    
    //assign read_data=32'b0;
    integer i;
    initial begin
        read_data=32'b0;
        for(i =  0; i < 256; i = i + 1) begin
         memory[i] = 32'b0; 
         end
    end
    always @(posedge clk)begin
        if(memWrite) begin
            memory [addr[9:2]] <=write_data;
        end
    end
    
    always@(*) begin
        if(memRead) begin
            read_data <= memory[addr[9:2]];
        end
        else begin
            read_data<=read_data;
        end
    end
    assign test_value=read_data[15:0];
endmodule

module instruct_mem(
    input [31:0] address,
    output reg [31:0] inst
);
    reg [31:0] memory [255:0];
    initial begin 
    inst =32'b0;
    $readmemh("instructions.mem", memory);
    end
    always @(*)begin
        inst = memory[address[9:2]];
    end
endmodule

module control(
    input [5:0]opcode, 
    output reg RegDst, 
    output reg jal,
    output reg jump,
    output reg ALUSrc, 
    output reg MemtoReg, 
    output reg RegWrite, 
    output reg MemRead, 
    output reg MemWrite, 
    output reg Branch, 
    output reg [1:0] AluOP
);

always @(opcode) begin
	case (opcode)
		6'b000000:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'b10000110010; //r
		6'b100011:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'b00011110000; //lw
		6'b101011:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'bx001x001000; //sw
		6'b000100:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'bx000x010101; //beq
        6'b000010:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'b01000010000;//j
        6'b000011:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'bx11xx100xxx;//jal
        6'b001000:{RegDst, jump, jal, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, AluOP}=11'b00010110000;//addi
		default:
	{RegDst,jump,jal,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,AluOP}=11'bxxxxx_xxx_x_xx;
	endcase
end
endmodule

module aluControl(
    input [1:0] AluOP,
    input [5:0] FnField,
    output reg [3:0] AluCtrl
);
    always @(*)begin
        case(AluOP)
            2'b00: AluCtrl=4'b0010;
            2'b01: AluCtrl=4'b0110;
            2'b10: begin
                case(FnField)
                    6'b100000: AluCtrl=4'b0010;//add
                    6'b100010: AluCtrl=4'b0110;//sub
                    6'b100100: AluCtrl=4'b0000;//and
                    6'b100101: AluCtrl=4'b0001;//or
                    6'b101010: AluCtrl=4'b0111;//slt
                    default: AluCtrl=4'b0000;
                endcase
            end
            default: AluCtrl=4'b0000;
        endcase    
    end
endmodule

module Alu(
    input [31:0] op1,op2,
    input [3:0] AluCtrl,
    output reg [31:0] result,
    output reg zero
);

    wire [31:0]  add,sub,orout,andout;
    wire c1,c2;
    
    rca_32bit r1(.a(op1),.b(op2),.sub(1'b0),.sum(add),.cout(c1));
    rca_32bit r2(.a(op1),.b(op2),.sub(1'b1),.sum(sub),.cout(c2));
    and_32bit an1(.a(op1),.b(op2),.result(andout));
    or_32bit o1(.a(op1),.b(op2),.result(orout));
    
    always @(*) begin
        case(AluCtrl)
            4'b0000: result=andout;//and
            4'b0001: result=orout;//or
            4'b0010: result=add;//add
            4'b0110: result=sub;//sub
            4'b0111: result = ($signed(op1) < $signed(op2)) ? 1 : 0;//slt
        endcase
        zero = (result==0)? 1:0;
    end
endmodule

module datapath(
    input clk,
    input rst,
    output [15:0] test_value
);
    wire [31:0] next_pc,curr_pc,inst,A1out,A2out,m4out;
    wire [31:0] seout,m2out,Aout;
    wire [31:0] read_data;
    wire zero;
    wire [31:0] jadd; 
    wire RegDst,jump,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch;
    wire [1:0] AluOP;
    wire [4:0] m1out;
    wire [3:0] AluCtrl;
    wire [31:0] wd,od1,od2;
    wire [31:0] m3out;
    wire [4:0] m7out;
    wire jal;
    wire ca1,ca2;

    pc pc1(.clk(clk),.rst(rst),.next_pc(next_pc),.curr_pc(curr_pc));

    instruct_mem imem1(.address(curr_pc),.inst(inst));
    rca_32bit A1(.a(curr_pc),.b(32'b00000000000000000000000000000100),.sub(1'b0),.sum(A1out),.cout(ca1));
    
    assign jadd={A1out[31:28],inst[25:0],2'b00};
    rca_32bit A2 (.a(A1out),.b({seout[29:0],2'b00}),.sub(1'b0),.sum(A2out),.cout(ca2));

    mux_32bit m4(.a(A1out),.b(A2out),.sel(Branch&zero),.o(m4out));
    mux_32bit m5(.a(m4out),.b(jadd),.sel(jump),.o(next_pc));
    mux_32bit m6(.a(m3out),.b(A1out),.sel(jal),.o(wd));
    
    mux_5bit m7(.a(m1out),.b(5'b11111),.sel(jal),.o(m7out));
    
    control c1(.opcode(inst[31:26]),.RegDst(RegDst),.jal(jal),.jump(jump),.ALUSrc(ALUSrc),.MemtoReg(MemtoReg),.RegWrite(RegWrite),.MemRead(MemRead),.MemWrite(MemWrite),.Branch(Branch),.AluOP(AluOP));

    mux_5bit m1(.a(inst[20:16]),.b(inst[15:11]),.sel(RegDst),.o(m1out));
    reg_file rf(.clk(clk),.regWrite(RegWrite),.ra(inst[25:21]),.rb(inst[20:16]),.rc(m7out),.wd(wd),.od1(od1),.od2(od2));

    sign_extend se(.a(inst[15:0]),.b(seout));
    mux_32bit m2(.a(od2),.b(seout),.sel(ALUSrc),.o(m2out));

    aluControl c2(.AluOP(AluOP),.FnField(inst[5:0]),.AluCtrl(AluCtrl));
    Alu A(.op1(od1),.op2(m2out),.AluCtrl(AluCtrl),.result(Aout),.zero(zero));

    data_mem dmem1(.clk(clk),.memRead(MemRead),.memWrite(MemWrite),.addr(Aout),.write_data(od2),.read_data(read_data),.test_value(test_value));
    mux_32bit m3(.a(Aout),.b(read_data),.sel(MemtoReg),.o(m3out));
    
    always@(*)begin
        $display("time : %t | curr_inst: %h | od2 : %d | read_data: %d | memwrite: %d | memread : %d | addr : %d | op1: %d | op2; %d",
                    $time,inst,od2,read_data,MemWrite,MemRead,Aout[9:2],od1,m2out);
    end
endmodule