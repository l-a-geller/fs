module root(
    input clk_i,
    input rst_i,
    input start_i,
    input [7:0] x_bi,    
    output busy_o,
    output [7:0] y_bo
);
    
localparam IDLE = 4'b0000;
localparam STATE1 = 4'b0001;
localparam STATE2 = 4'b0010;
localparam STATE3 = 4'b0011;
localparam STATE4 = 4'b0100;
localparam STATE5 = 4'b0101;
localparam STATE6 = 4'b0110;
localparam STATE7 = 4'b0111;
localparam STATE8 = 4'b1000;

reg[3:0] state, state_next;
reg[7:0] res;
reg[7:0] b;
reg[7:0] x;
reg signed[7:0] s;
reg[7:0] y;

reg [7:0] mult_a;
reg [7:0] mult_b;
wire [15:0] mult_out;
reg mult_start;
wire mult_busy;
    
mul mul(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .a_bi(mult_a),
    .b_bi(mult_b),
    .start_i(mult_start),
    .busy_o(mult_busy),
    .y_bo(mult_out)
);
    
assign busy_o = start_i || (state > 0);
assign y_bo = y;
    
always @(posedge clk_i) 
    if (rst_i) begin
        state <= IDLE;
    end else begin
        state <= state_next;
end
    
always @* begin
    case(state)
        IDLE:   state_next = (start_i) ? STATE1 : IDLE;
        STATE1: state_next = (s > -3) ? STATE2 : STATE8;
        STATE2: state_next = STATE3;
        STATE3: state_next = mult_busy ? STATE3 : STATE4;
        STATE4: state_next = mult_busy ? STATE4 : STATE5;
        STATE5: state_next = STATE6;
        STATE6: state_next = STATE7;
        STATE7: state_next = STATE1;
        STATE8: state_next = IDLE;
        default: state_next = IDLE;
    endcase
end
    
    always @(posedge clk_i) begin
        if(start_i) begin
            s <= 6;
            res <= 0;
            y <= 0;
            x <= x_bi;
        end else begin
            case (state)
                IDLE:
                   begin
                        x <= x_bi;
                        b <= 0;
                        res <= 0;
                    end
                STATE1:
                    begin
                        if (s > -3) begin
                            res = res << 1;
                        end
                    end
                STATE2:
                    begin
                        mult_start <= 1;
                        mult_a <= res + 1;
                        mult_b <= res;
                    end
                STATE3:
                    begin
                        if (mult_busy) begin
                            mult_start <= 0;
                        end else begin
                            mult_start <= 1;
                            mult_a <= mult_out;
                            mult_b <= 3;
                        end 
                    end
                STATE4:
                    begin
                        if (mult_busy) begin
                            mult_start <= 0;
                        end else begin
                            b <= mult_out + 1;
                    end end
                STATE5: b = b << s;
                STATE6: 
                    begin
                        if (x >= b) begin
                            x <= x - b;   
                            res <= res + 1;
                        end 
                    end
                STATE7:
                    begin
                        s = s - 3;
                    end
                STATE8:
                    begin
                        y = res;
                    end
            endcase
    end end
endmodule