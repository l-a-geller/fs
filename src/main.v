module main(
    input rst_i,
    input clk_i,
    input [7:0] a_in,
    input [7:0] b_in,
    input start_i,
    output busy_o,
    output reg [15:0] y_bo
);

reg [7:0] a;
reg [7:0] b;
reg [3:0] state, state_next;

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

reg [7:0] root_x;
wire [7:0] root_out;
wire root_busy;
reg root_start;

root root(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .start_i(root_start),
    .x_bi(root_x),    
    .busy_o(root_busy),
    .y_bo(root_out)
);

localparam IDLE = 2'b00;
localparam STATE1 = 2'b01;
localparam STATE2 = 2'b10;
localparam STATE3 = 2'b11;


assign busy_o = (state > 0 );


always @(posedge clk_i)
  if (!rst_i) begin
      state <= state_next;
  end else begin 
    state <= IDLE;
  end

always @(posedge clk_i) begin
    if(!rst_i) begin
        case (state)
            IDLE:
                begin
                    mult_start <= 0;
                    root_start <= 0;
                end
            STATE1:
                begin
                    a = a_in;
                    b = b_in;
                    root_start = 1;
                    root_x = b;
                end
            STATE2:
                begin
                    if (root_busy) begin
                        root_start <= 0;
                    end else begin
                        mult_start <= 1;
                        mult_a <= a;
                        mult_b <= root_out;
                    end
                end
            STATE3:
                begin
                    if (mult_busy) begin
                        mult_start <= 0;
                    end
                    y_bo <= mult_out;
                end
        endcase
    end else begin
        a <= a_in;
        b <= b_in;
        mult_start <= 0;
        root_start <= 0;
    end
end

always @(*) begin
  case(state)
      IDLE:   state_next = (start_i) ? STATE1 : IDLE;
      STATE1: state_next = STATE2;
      STATE2: state_next = (root_busy) ? STATE2 : STATE3;
      STATE3: state_next = (mult_busy) ? STATE3 : IDLE;
      default: state_next = IDLE;
  endcase
end

endmodule

