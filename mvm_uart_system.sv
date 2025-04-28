/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module project (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
); 

  localparam 
    CLOCKS_PER_PULSE = 50_000_000/19200,
    BITS_PER_WORD    = 8                ,
    PACKET_SIZE_TX   = BITS_PER_WORD + 5,
    W_Y_OUT          = 8                ,
    R                = 2                ,
    C                = 2                ,
    W_X              = 4                ,
    W_K              = 2                ;

  mvm_uart_system #(
    .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
    .BITS_PER_WORD    (BITS_PER_WORD   ),
    .PACKET_SIZE_TX   (PACKET_SIZE_TX  ),
    .W_Y_OUT          (W_Y_OUT         ),
    .R                (R               ), 
    .C                (C               ), 
    .W_X              (W_X             ), 
    .W_K              (W_K             )
  ) MVM_UART_SYSTEM (
    .clk (clk), 
    .rstn(rst_n), 
    .rx  (ui_in [0]),
    .tx  (uo_out[0])
  );

  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out[7:1] = 0;
  assign uio_out = 0;
  assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
  wire _unused = &{uio_in, ui_in[7:1], ena, clk, rst_n, 1'b0};

endmodule

module axis_matvec_mul #(
    parameter R=2, C=2, W_X=2, W_K=2,
              LATENCY = $clog2(C)+1,
              W_Y = W_X + W_K + $clog2(C)
  )(
    input  clk, rstn,
    output s_axis_kx_tready, 
    input  s_axis_kx_tvalid, 
    input  [R*C*W_K + C*W_X -1:0] s_axis_kx_tdata,
    input  m_axis_y_tready,
    output m_axis_y_tvalid,
    output [R*W_Y           -1:0] m_axis_y_tdata
  );
    wire [R*C*W_K-1:0] k;
    wire [C*W_X  -1:0] x;
    assign {k, x} = s_axis_kx_tdata;

    wire [R*W_Y-1:0] i_data;
    wire i_ready;

    matvec_mul #(
      .R(R), .C(C), .W_X(W_X), .W_K(W_K)
    ) MATVEC (  
      .clk(clk    ),
      .cen(i_ready),
      .kf (k      ),
      .xf (x      ), 
      .yf (i_data )
    );

    reg [LATENCY-2:0] shift;
    reg  i_valid;

    always @(posedge clk or negedge rstn)
      if     (!rstn)   {i_valid, shift} <= 0;
      else if(i_ready) {i_valid, shift} <= {shift, s_axis_kx_tvalid};

    skid_buffer #(.WIDTH(R*W_Y)) SKID (
      .clk     (clk    ), 
      .rstn    (rstn   ), 
      .s_ready (i_ready),
      .s_valid (i_valid), 
      .s_data  (i_data ),
      .m_ready (m_axis_y_tready),
      .m_valid (m_axis_y_tvalid), 
      .m_data  (m_axis_y_tdata )
    );

    assign s_axis_kx_tready = i_ready;

endmodule

module matvec_mul #(
    parameter R=8, C=8, W_X=8, W_K=8,
    localparam DEPTH = $clog2(C),
               W_M = W_X + W_K,
               W_Y = W_M + DEPTH
  )(  
    input  wire  clk, cen,
    input  wire  [R*C*W_K-1:0] kf,
    input  wire  [C*W_X  -1:0] xf, 
    output wire  [R*W_Y  -1:0] yf       
  );

  // Padding
  localparam C_PAD = 2**$clog2(C);
  reg [W_Y-1:0] tree [R][DEPTH+1][C_PAD]; // adder tree

  wire signed [W_X-1:0] x_pad        [C_PAD-1:0];
  wire signed [W_K-1:0] k_pad [R-1:0][C_PAD-1:0];

  genvar r, c, d, a;
  generate

    for (c=0; c<C_PAD; c=c+1) begin
      assign x_pad[c] = (c < C) ? xf[(c+1)*W_X-1 : c*W_X] : 0;
      for (r=0; r<R; r=r+1)
        assign k_pad[r][c] = (c < C) ? kf[((r*C + c) + 1)*W_K-1 : (r*C + c)*W_K] : 0;
    end

    for (r=0; r<R; r=r+1) begin
      for (c=0; c<C_PAD; c=c+1)
        always @(posedge clk)  // register after each mul
          if (cen) tree[r][0][c] <= $signed(k_pad[r][c]) * $signed(x_pad[c]);

      for (d=0; d<DEPTH; d=d+1)
        for (a=0; a<C_PAD/2**(d+1); a=a+1)
          always @(posedge clk)  // register after each add
            if (cen) tree [r][d+1][a] <= tree [r][d][2*a] + tree [r][d][2*a+1];

      assign yf[(r+1)*W_Y-1: r*W_Y] = tree [r][DEPTH][0];
    end
  endgenerate
endmodule


module mvm_uart_system #(
  parameter CLOCKS_PER_PULSE = 200_000_000/9600, //200_000_000/9600
            BITS_PER_WORD    = 8,
            PACKET_SIZE_TX   = BITS_PER_WORD + 5,
            W_Y_OUT          = 32,
            R=8, C=8, W_X=8, W_K=8
)(
  input  clk, rstn, rx,
  output tx
);

  localparam  W_BUS_KX = R*C*W_K + C*W_X,
              W_BUS_Y  = R*W_Y_OUT,
              W_Y      = W_X + W_K + $clog2(C);

  wire s_valid, m_valid, m_ready;
  wire [W_BUS_KX-1:0] s_data_kx;

  uart_rx #(
    .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
    .BITS_PER_WORD (BITS_PER_WORD), 
    .W_OUT (W_BUS_KX)
  ) UART_RX (
    .clk    (clk), 
    .rstn   (rstn), 
    .rx     (rx),
    .m_valid(s_valid),
    .m_data (s_data_kx)
  );

  wire [R*W_Y  -1:0] m_data_y;
  wire s_ready;
  wire _unused = s_ready;
  axis_matvec_mul #(
    .R(R), .C(C), .W_X(W_X), .W_K(W_K)
  ) AXIS_MVM (
    .clk    (clk      ), 
    .rstn   (rstn     ),
    .s_axis_kx_tready(s_ready), // assume always valid
    .s_axis_kx_tvalid(s_valid  ), 
    .s_axis_kx_tdata (s_data_kx),
    .m_axis_y_tready (m_ready  ),
    .m_axis_y_tvalid (m_valid  ),
    .m_axis_y_tdata  (m_data_y )
  );

  // Padding to 32 bits to be read in computer
  wire [W_Y      -1:0] y_up     [R-1:0];
  wire [W_Y_OUT  -1:0] o_up   [R-1:0];
  wire [R*W_Y_OUT-1:0] o_flat;

  genvar r;
  generate
  for (r=0; r<R; r=r+1) begin
    assign y_up  [r] = m_data_y[W_Y*(r+1)-1 : W_Y*r];
    assign o_up  [r] = W_Y_OUT'($signed(y_up[r])); // sign extend to 32b
    assign o_flat[W_Y_OUT*(r+1)-1 : W_Y_OUT*r] = o_up[r];
    // assign o_flat[W_Y_OUT*(r+1)-1 : W_Y_OUT*r] = $signed(m_data_y[W_Y*(r+1)-1 : W_Y*r]);
  end
  endgenerate

  uart_tx #(
   .CLOCKS_PER_PULSE (CLOCKS_PER_PULSE),
   .BITS_PER_WORD    (BITS_PER_WORD),
   .PACKET_SIZE      (PACKET_SIZE_TX),
   .W_OUT            (W_BUS_Y)
  ) UART_TX (
   .clk     (clk      ), 
   .rstn    (rstn     ), 
   .s_ready (m_ready  ),
   .s_valid (m_valid  ), 
   .s_data_f(o_flat   ),
   .tx      (tx       )
  );  

endmodule

module skid_buffer #(parameter WIDTH = 8)(
  input  wire clk, rstn, s_valid, m_ready,
  input  wire [WIDTH-1:0] s_data,
  output reg  [WIDTH-1:0] m_data,
  output reg  m_valid, s_ready
);
  localparam EMPTY=0, PARTIAL=1, FULL=2;
  reg [1:0] state, state_next;

  always @* begin
    state_next = state;
    unique case (state)
      EMPTY  : if      ( s_valid)             state_next = PARTIAL;
      PARTIAL: if      (!m_ready &&  s_valid) state_next = FULL;
               else if ( m_ready && !s_valid) state_next = EMPTY;
      FULL   : if      ( m_ready)             state_next = PARTIAL;
    endcase
  end
  always @(posedge clk or negedge rstn)
    if (!rstn) state <= EMPTY;
    else       state <= state_next;

  reg [WIDTH-1:0] buffer;
  always @(posedge clk or negedge rstn)
    if (!rstn) {m_valid, s_ready, buffer, m_data} <= 0;
    else begin
      
      m_valid <= state_next != EMPTY;
      s_ready <= state_next != FULL;

      if (state == PARTIAL && state_next == FULL)
        buffer <= s_data;

      unique case (state)
        EMPTY  : if (state_next == PARTIAL) m_data <= s_data;
        PARTIAL: if (m_ready && s_valid)    m_data <= s_data;
        FULL   : if (m_ready)               m_data <= buffer;
      endcase
    end
endmodule

module uart_rx #(
  parameter CLOCKS_PER_PULSE = 4, //200_000_000/9600
            BITS_PER_WORD    = 8,
            W_OUT = 24 //R*C*W_K + C*W_X,
)(
  input  wire clk, rstn, rx,
  output reg  m_valid,
  output reg  [W_OUT-1:0] m_data
);
  localparam  NUM_WORDS = W_OUT/BITS_PER_WORD;

  // Counters

  localparam W_CCLOCKS = $clog2(CLOCKS_PER_PULSE),
             W_CBITS   = $clog2(BITS_PER_WORD),
             W_CWORDS  = $clog2(NUM_WORDS);

  reg [W_CCLOCKS-1:0] c_clocks;
  reg [W_CBITS  -1:0] c_bits  ;
  reg [W_CWORDS -1:0] c_words ;

  // State Machine


  localparam IDLE=0, START=1, DATA=2, END=3;
  reg [1:0] state;

  always @(posedge clk or negedge rstn) begin
    
    if (!rstn) begin
      {c_words, c_bits, c_clocks, m_valid, m_data} <= '0;
      state <= IDLE;
    end else begin
      m_valid <= 0;
      
      case (state)

        IDLE :  if (rx == 0)                      
                  state <= START;

        START:  if (c_clocks == W_CCLOCKS'(CLOCKS_PER_PULSE/2-1)) begin
                  state <= DATA;
                  c_clocks <= 0;
                end else
                  c_clocks <= c_clocks + 1;

        DATA :  if (c_clocks == W_CCLOCKS'(CLOCKS_PER_PULSE-1)) begin
                  c_clocks <= 0;
                  m_data   <= {rx, m_data[W_OUT-1:1]};

                  if (c_bits == W_CBITS'(BITS_PER_WORD-1)) begin
                    state  <= END;
                    c_bits <= 0;

                    if (c_words == W_CWORDS'(NUM_WORDS-1)) begin
                      m_valid <= 1;
                      c_words <= 0;

                    end else c_words <= c_words + 1;
                  end else c_bits <= c_bits + 1;
                end else c_clocks <= c_clocks + 1;

        END  :  if (c_clocks == W_CCLOCKS'(CLOCKS_PER_PULSE-1)) begin
                  state <= IDLE;
                  c_clocks <= 0;
                end else
                  c_clocks <= c_clocks + 1;
      endcase
      end
  end
endmodule

module uart_tx #(
  parameter   CLOCKS_PER_PULSE = 4, //200_000_000/9600
              BITS_PER_WORD    = 8,
              PACKET_SIZE      = BITS_PER_WORD+5,
              W_OUT = 24, //R*C*W_K + C*W_X
  
  localparam  NUM_WORDS   = W_OUT/BITS_PER_WORD
)(
  input  wire clk, rstn, s_valid, 
  input  wire [NUM_WORDS*BITS_PER_WORD-1:0] s_data_f,
  output reg  tx, s_ready
);  
  
  genvar n;
  wire [BITS_PER_WORD-1:0] s_data [NUM_WORDS-1:0];

  generate
    for (n=0; n<NUM_WORDS; n=n+1)
      assign s_data[n] = s_data_f[BITS_PER_WORD*(n+1)-1 : BITS_PER_WORD*n];
  endgenerate

  localparam END_BITS = PACKET_SIZE-BITS_PER_WORD-1;
  reg [NUM_WORDS*PACKET_SIZE-1:0] s_packets;
  reg [NUM_WORDS*PACKET_SIZE-1:0] m_packets;

  generate
    for (n=0; n<NUM_WORDS; n=n+1)
      assign s_packets[PACKET_SIZE*(n+1)-1:PACKET_SIZE*n] = { ~(END_BITS'(0)), s_data[n], 1'b0};
  endgenerate
    
  assign tx = m_packets[0];

  // Counters
  localparam W_CPULSES = $clog2(NUM_WORDS*PACKET_SIZE),
             W_CCLOCKS = $clog2(CLOCKS_PER_PULSE);

  reg [W_CPULSES-1:0] c_pulses;
  reg [W_CCLOCKS-1:0] c_clocks;

  // State Machine

  localparam IDLE=0, SEND=1;
  reg state;

  always @(posedge clk or negedge rstn) begin

    if (!rstn) begin
      state     <= IDLE;
      m_packets <= -1;
      {c_pulses, c_clocks} <= 0;
    end else
      case (state)
        IDLE :  if (s_valid) begin                  
                  state      <= SEND;
                  m_packets  <= s_packets;
                end

        SEND :  if (c_clocks == W_CCLOCKS'(CLOCKS_PER_PULSE-1)) begin
                  c_clocks <= 0;

                  if (c_pulses == W_CPULSES'(NUM_WORDS*PACKET_SIZE-1)) begin
                    c_pulses  <= 0;
                    m_packets <= -1;
                    state     <= IDLE;

                  end else begin
                    c_pulses <= c_pulses + 1;
                    m_packets <= (m_packets >> 1);
                  end

                end else c_clocks <= W_CCLOCKS'(32'(c_clocks) + 1);
      endcase
  end

  assign s_ready = (state == IDLE);

endmodule


