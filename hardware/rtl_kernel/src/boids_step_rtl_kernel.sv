`timescale 1ns/1ns

module boids_step #(
    parameter integer C_S_AXI_CONTROL_ADDR_WIDTH = 6,
    parameter integer C_S_AXI_CONTROL_DATA_WIDTH = 32,
    parameter integer C_M_AXI_ADDR_WIDTH         = 64,
    parameter integer C_M_AXI_DATA_WIDTH         = 32,
    parameter integer C_M_AXI_ID_WIDTH           = 1,
    parameter integer C_M_AXI_USER_WIDTH         = 1,
    parameter integer MAX_BOIDS                  = 512
)(
    input  logic ap_clk,
    input  logic ap_rst_n,

    output logic                                m_axi_gmem0_AWVALID,
    input  logic                                m_axi_gmem0_AWREADY,
    output logic [C_M_AXI_ADDR_WIDTH-1:0]       m_axi_gmem0_AWADDR,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem0_AWID,
    output logic [7:0]                          m_axi_gmem0_AWLEN,
    output logic [2:0]                          m_axi_gmem0_AWSIZE,
    output logic [1:0]                          m_axi_gmem0_AWBURST,
    output logic [1:0]                          m_axi_gmem0_AWLOCK,
    output logic [3:0]                          m_axi_gmem0_AWCACHE,
    output logic [2:0]                          m_axi_gmem0_AWPROT,
    output logic [3:0]                          m_axi_gmem0_AWQOS,
    output logic [3:0]                          m_axi_gmem0_AWREGION,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem0_AWUSER,
    output logic                                m_axi_gmem0_WVALID,
    input  logic                                m_axi_gmem0_WREADY,
    output logic [C_M_AXI_DATA_WIDTH-1:0]       m_axi_gmem0_WDATA,
    output logic [(C_M_AXI_DATA_WIDTH/8)-1:0]   m_axi_gmem0_WSTRB,
    output logic                                m_axi_gmem0_WLAST,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem0_WID,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem0_WUSER,
    output logic                                m_axi_gmem0_ARVALID,
    input  logic                                m_axi_gmem0_ARREADY,
    output logic [C_M_AXI_ADDR_WIDTH-1:0]       m_axi_gmem0_ARADDR,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem0_ARID,
    output logic [7:0]                          m_axi_gmem0_ARLEN,
    output logic [2:0]                          m_axi_gmem0_ARSIZE,
    output logic [1:0]                          m_axi_gmem0_ARBURST,
    output logic [1:0]                          m_axi_gmem0_ARLOCK,
    output logic [3:0]                          m_axi_gmem0_ARCACHE,
    output logic [2:0]                          m_axi_gmem0_ARPROT,
    output logic [3:0]                          m_axi_gmem0_ARQOS,
    output logic [3:0]                          m_axi_gmem0_ARREGION,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem0_ARUSER,
    input  logic                                m_axi_gmem0_RVALID,
    output logic                                m_axi_gmem0_RREADY,
    input  logic [C_M_AXI_DATA_WIDTH-1:0]       m_axi_gmem0_RDATA,
    input  logic                                m_axi_gmem0_RLAST,
    input  logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem0_RID,
    input  logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem0_RUSER,
    input  logic [1:0]                          m_axi_gmem0_RRESP,
    input  logic                                m_axi_gmem0_BVALID,
    output logic                                m_axi_gmem0_BREADY,
    input  logic [1:0]                          m_axi_gmem0_BRESP,
    input  logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem0_BID,
    input  logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem0_BUSER,

    output logic                                m_axi_gmem1_AWVALID,
    input  logic                                m_axi_gmem1_AWREADY,
    output logic [C_M_AXI_ADDR_WIDTH-1:0]       m_axi_gmem1_AWADDR,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem1_AWID,
    output logic [7:0]                          m_axi_gmem1_AWLEN,
    output logic [2:0]                          m_axi_gmem1_AWSIZE,
    output logic [1:0]                          m_axi_gmem1_AWBURST,
    output logic [1:0]                          m_axi_gmem1_AWLOCK,
    output logic [3:0]                          m_axi_gmem1_AWCACHE,
    output logic [2:0]                          m_axi_gmem1_AWPROT,
    output logic [3:0]                          m_axi_gmem1_AWQOS,
    output logic [3:0]                          m_axi_gmem1_AWREGION,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem1_AWUSER,
    output logic                                m_axi_gmem1_WVALID,
    input  logic                                m_axi_gmem1_WREADY,
    output logic [C_M_AXI_DATA_WIDTH-1:0]       m_axi_gmem1_WDATA,
    output logic [(C_M_AXI_DATA_WIDTH/8)-1:0]   m_axi_gmem1_WSTRB,
    output logic                                m_axi_gmem1_WLAST,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem1_WID,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem1_WUSER,
    output logic                                m_axi_gmem1_ARVALID,
    input  logic                                m_axi_gmem1_ARREADY,
    output logic [C_M_AXI_ADDR_WIDTH-1:0]       m_axi_gmem1_ARADDR,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem1_ARID,
    output logic [7:0]                          m_axi_gmem1_ARLEN,
    output logic [2:0]                          m_axi_gmem1_ARSIZE,
    output logic [1:0]                          m_axi_gmem1_ARBURST,
    output logic [1:0]                          m_axi_gmem1_ARLOCK,
    output logic [3:0]                          m_axi_gmem1_ARCACHE,
    output logic [2:0]                          m_axi_gmem1_ARPROT,
    output logic [3:0]                          m_axi_gmem1_ARQOS,
    output logic [3:0]                          m_axi_gmem1_ARREGION,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem1_ARUSER,
    input  logic                                m_axi_gmem1_RVALID,
    output logic                                m_axi_gmem1_RREADY,
    input  logic [C_M_AXI_DATA_WIDTH-1:0]       m_axi_gmem1_RDATA,
    input  logic                                m_axi_gmem1_RLAST,
    input  logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem1_RID,
    input  logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem1_RUSER,
    input  logic [1:0]                          m_axi_gmem1_RRESP,
    input  logic                                m_axi_gmem1_BVALID,
    output logic                                m_axi_gmem1_BREADY,
    input  logic [1:0]                          m_axi_gmem1_BRESP,
    input  logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem1_BID,
    input  logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem1_BUSER,

    output logic                                m_axi_gmem2_AWVALID,
    input  logic                                m_axi_gmem2_AWREADY,
    output logic [C_M_AXI_ADDR_WIDTH-1:0]       m_axi_gmem2_AWADDR,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem2_AWID,
    output logic [7:0]                          m_axi_gmem2_AWLEN,
    output logic [2:0]                          m_axi_gmem2_AWSIZE,
    output logic [1:0]                          m_axi_gmem2_AWBURST,
    output logic [1:0]                          m_axi_gmem2_AWLOCK,
    output logic [3:0]                          m_axi_gmem2_AWCACHE,
    output logic [2:0]                          m_axi_gmem2_AWPROT,
    output logic [3:0]                          m_axi_gmem2_AWQOS,
    output logic [3:0]                          m_axi_gmem2_AWREGION,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem2_AWUSER,
    output logic                                m_axi_gmem2_WVALID,
    input  logic                                m_axi_gmem2_WREADY,
    output logic [C_M_AXI_DATA_WIDTH-1:0]       m_axi_gmem2_WDATA,
    output logic [(C_M_AXI_DATA_WIDTH/8)-1:0]   m_axi_gmem2_WSTRB,
    output logic                                m_axi_gmem2_WLAST,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem2_WID,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem2_WUSER,
    output logic                                m_axi_gmem2_ARVALID,
    input  logic                                m_axi_gmem2_ARREADY,
    output logic [C_M_AXI_ADDR_WIDTH-1:0]       m_axi_gmem2_ARADDR,
    output logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem2_ARID,
    output logic [7:0]                          m_axi_gmem2_ARLEN,
    output logic [2:0]                          m_axi_gmem2_ARSIZE,
    output logic [1:0]                          m_axi_gmem2_ARBURST,
    output logic [1:0]                          m_axi_gmem2_ARLOCK,
    output logic [3:0]                          m_axi_gmem2_ARCACHE,
    output logic [2:0]                          m_axi_gmem2_ARPROT,
    output logic [3:0]                          m_axi_gmem2_ARQOS,
    output logic [3:0]                          m_axi_gmem2_ARREGION,
    output logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem2_ARUSER,
    input  logic                                m_axi_gmem2_RVALID,
    output logic                                m_axi_gmem2_RREADY,
    input  logic [C_M_AXI_DATA_WIDTH-1:0]       m_axi_gmem2_RDATA,
    input  logic                                m_axi_gmem2_RLAST,
    input  logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem2_RID,
    input  logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem2_RUSER,
    input  logic [1:0]                          m_axi_gmem2_RRESP,
    input  logic                                m_axi_gmem2_BVALID,
    output logic                                m_axi_gmem2_BREADY,
    input  logic [1:0]                          m_axi_gmem2_BRESP,
    input  logic [C_M_AXI_ID_WIDTH-1:0]         m_axi_gmem2_BID,
    input  logic [C_M_AXI_USER_WIDTH-1:0]       m_axi_gmem2_BUSER,

    input  logic                                s_axi_control_AWVALID,
    output logic                                s_axi_control_AWREADY,
    input  logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] s_axi_control_AWADDR,
    input  logic                                s_axi_control_WVALID,
    output logic                                s_axi_control_WREADY,
    input  logic [C_S_AXI_CONTROL_DATA_WIDTH-1:0] s_axi_control_WDATA,
    input  logic [(C_S_AXI_CONTROL_DATA_WIDTH/8)-1:0] s_axi_control_WSTRB,
    input  logic                                s_axi_control_ARVALID,
    output logic                                s_axi_control_ARREADY,
    input  logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] s_axi_control_ARADDR,
    output logic                                s_axi_control_RVALID,
    input  logic                                s_axi_control_RREADY,
    output logic [C_S_AXI_CONTROL_DATA_WIDTH-1:0] s_axi_control_RDATA,
    output logic [1:0]                          s_axi_control_RRESP,
    output logic                                s_axi_control_BVALID,
    input  logic                                s_axi_control_BREADY,
    output logic [1:0]                          s_axi_control_BRESP,
    output logic                                interrupt
);

    localparam integer STRB_W = C_S_AXI_CONTROL_DATA_WIDTH / 8;
    localparam integer IDX_W = (MAX_BOIDS <= 1) ? 1 : $clog2(MAX_BOIDS);
    localparam integer ACTIVE_W = (MAX_BOIDS <= 1) ? 1 : $clog2(MAX_BOIDS + 1);

    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_AP_CTRL = 6'h00;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_GIE     = 6'h04;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_IER     = 6'h08;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_ISR     = 6'h0c;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_STATE_0 = 6'h10;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_STATE_1 = 6'h14;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_VEL_0   = 6'h1c;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_VEL_1   = 6'h20;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_NEXT_0  = 6'h28;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_NEXT_1  = 6'h2c;
    localparam logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] ADDR_N       = 6'h34;

    typedef enum logic [3:0] {
        KS_IDLE          = 4'd0,
        KS_LOAD_STATE_AR = 4'd1,
        KS_LOAD_STATE_R  = 4'd2,
        KS_LOAD_VEL_AR   = 4'd3,
        KS_LOAD_VEL_R    = 4'd4,
        KS_CORE_RELEASE  = 4'd5,
        KS_CORE_START    = 4'd6,
        KS_CORE_WAIT     = 4'd7,
        KS_STORE_AW      = 4'd8,
        KS_STORE_W       = 4'd9,
        KS_STORE_B       = 4'd10,
        KS_DONE          = 4'd11
    } kernel_state_t;

    logic rst;
    logic [63:0] reg_state_mem;
    logic [63:0] reg_vel_mem;
    logic [63:0] reg_next_state;
    logic [31:0] reg_n;
    logic gie_reg;
    logic [0:0] ier_reg;
    logic [0:0] isr_reg;

    logic ap_start_reg;
    logic auto_restart_reg;
    logic ap_done_reg;
    logic ap_idle_reg;
    logic ap_ready_reg;

    logic aw_captured;
    logic w_captured;
    logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] wr_addr;
    logic [C_S_AXI_CONTROL_DATA_WIDTH-1:0] wr_data;
    logic [STRB_W-1:0] wr_strb;

    logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] rd_addr;

    logic [MAX_BOIDS-1:0][26:0] core_in_pos_x;
    logic [MAX_BOIDS-1:0][26:0] core_in_pos_y;
    logic [MAX_BOIDS-1:0][26:0] core_in_vel_x;
    logic [MAX_BOIDS-1:0][26:0] core_in_vel_y;
    logic [MAX_BOIDS-1:0][26:0] core_out_pos_x;
    logic [MAX_BOIDS-1:0][26:0] core_out_pos_y;
    logic [MAX_BOIDS-1:0][26:0] core_out_vel_x;
    logic [MAX_BOIDS-1:0][26:0] core_out_vel_y;
    logic [12:0] core_active_boids;
    logic core_reset;
    logic core_start;
    logic core_done;

    kernel_state_t kernel_state;
    logic [IDX_W-1:0] idx;
    logic [ACTIVE_W-1:0] active_count;
    logic [31:0] rd_state_word;

    logic m0_arvalid;
    logic [C_M_AXI_ADDR_WIDTH-1:0] m0_araddr;
    logic m0_rready;

    logic m1_arvalid;
    logic [C_M_AXI_ADDR_WIDTH-1:0] m1_araddr;
    logic m1_rready;

    logic m2_awvalid;
    logic [C_M_AXI_ADDR_WIDTH-1:0] m2_awaddr;
    logic m2_wvalid;
    logic [C_M_AXI_DATA_WIDTH-1:0] m2_wdata;
    logic m2_bready;

    function automatic logic [C_S_AXI_CONTROL_DATA_WIDTH-1:0] apply_wstrb(
        input logic [C_S_AXI_CONTROL_DATA_WIDTH-1:0] prior,
        input logic [C_S_AXI_CONTROL_DATA_WIDTH-1:0] incoming,
        input logic [STRB_W-1:0] strobe
    );
        integer i;
        begin
            apply_wstrb = prior;
            for (i = 0; i < STRB_W; i = i + 1) begin
                if (strobe[i]) begin
                    apply_wstrb[(8*i) +: 8] = incoming[(8*i) +: 8];
                end
            end
        end
    endfunction

    function automatic logic [ACTIVE_W-1:0] clamp_n(input logic [31:0] n_in);
        logic [31:0] max_u32;
        begin
            max_u32 = MAX_BOIDS;
            if (n_in == 32'd0) begin
                clamp_n = '0;
            end
            else if (n_in > max_u32) begin
                clamp_n = logic'(MAX_BOIDS[ACTIVE_W-1:0]);
            end
            else begin
                clamp_n = n_in[ACTIVE_W-1:0];
            end
        end
    endfunction

    function automatic logic signed [26:0] int16_to_fix(input logic [15:0] in_word);
        logic signed [26:0] tmp;
        begin
            tmp = $signed({{11{in_word[15]}}, in_word});
            int16_to_fix = tmp <<< 15;
        end
    endfunction

    function automatic logic [15:0] fix_to_int16(input logic signed [26:0] fix_word);
        logic signed [15:0] q;
        begin
            q = fix_word[26:15];
            fix_to_int16 = q;
        end
    endfunction

    function automatic logic [31:0] read_reg(input logic [C_S_AXI_CONTROL_ADDR_WIDTH-1:0] addr);
        begin
            case (addr)
                ADDR_AP_CTRL: read_reg = {24'd0, auto_restart_reg, 1'b0, ap_ready_reg, ap_idle_reg, ap_done_reg, ap_start_reg};
                ADDR_GIE:     read_reg = {31'd0, gie_reg};
                ADDR_IER:     read_reg = {31'd0, ier_reg[0]};
                ADDR_ISR:     read_reg = {31'd0, isr_reg[0]};
                ADDR_STATE_0: read_reg = reg_state_mem[31:0];
                ADDR_STATE_1: read_reg = reg_state_mem[63:32];
                ADDR_VEL_0:   read_reg = reg_vel_mem[31:0];
                ADDR_VEL_1:   read_reg = reg_vel_mem[63:32];
                ADDR_NEXT_0:  read_reg = reg_next_state[31:0];
                ADDR_NEXT_1:  read_reg = reg_next_state[63:32];
                ADDR_N:       read_reg = reg_n;
                default:      read_reg = 32'd0;
            endcase
        end
    endfunction

    assign rst = ~ap_rst_n;

    assign s_axi_control_BRESP = 2'b00;
    assign s_axi_control_RRESP = 2'b00;

    assign interrupt = gie_reg & isr_reg[0];

    assign s_axi_control_AWREADY = (~aw_captured) & (~s_axi_control_BVALID);
    assign s_axi_control_WREADY  = (~w_captured) & (~s_axi_control_BVALID);
    assign s_axi_control_ARREADY = ~s_axi_control_RVALID;

    assign m_axi_gmem0_AWVALID = 1'b0;
    assign m_axi_gmem0_AWADDR  = '0;
    assign m_axi_gmem0_AWID    = '0;
    assign m_axi_gmem0_AWLEN   = 8'd0;
    assign m_axi_gmem0_AWSIZE  = 3'd2;
    assign m_axi_gmem0_AWBURST = 2'b01;
    assign m_axi_gmem0_AWLOCK  = 2'b00;
    assign m_axi_gmem0_AWCACHE = 4'b0011;
    assign m_axi_gmem0_AWPROT  = 3'b000;
    assign m_axi_gmem0_AWQOS   = 4'b0000;
    assign m_axi_gmem0_AWREGION = 4'b0000;
    assign m_axi_gmem0_AWUSER   = '0;
    assign m_axi_gmem0_WVALID = 1'b0;
    assign m_axi_gmem0_WDATA  = '0;
    assign m_axi_gmem0_WSTRB  = {(C_M_AXI_DATA_WIDTH/8){1'b1}};
    assign m_axi_gmem0_WLAST  = 1'b1;
    assign m_axi_gmem0_WID    = '0;
    assign m_axi_gmem0_WUSER  = '0;
    assign m_axi_gmem0_ARVALID = m0_arvalid;
    assign m_axi_gmem0_ARADDR  = m0_araddr;
    assign m_axi_gmem0_ARID    = '0;
    assign m_axi_gmem0_ARLEN   = 8'd0;
    assign m_axi_gmem0_ARSIZE  = 3'd2;
    assign m_axi_gmem0_ARBURST = 2'b01;
    assign m_axi_gmem0_ARLOCK  = 2'b00;
    assign m_axi_gmem0_ARCACHE = 4'b0011;
    assign m_axi_gmem0_ARPROT  = 3'b000;
    assign m_axi_gmem0_ARQOS   = 4'b0000;
    assign m_axi_gmem0_ARREGION = 4'b0000;
    assign m_axi_gmem0_ARUSER   = '0;
    assign m_axi_gmem0_RREADY = m0_rready;
    assign m_axi_gmem0_BREADY = 1'b0;

    assign m_axi_gmem1_AWVALID = 1'b0;
    assign m_axi_gmem1_AWADDR  = '0;
    assign m_axi_gmem1_AWID    = '0;
    assign m_axi_gmem1_AWLEN   = 8'd0;
    assign m_axi_gmem1_AWSIZE  = 3'd2;
    assign m_axi_gmem1_AWBURST = 2'b01;
    assign m_axi_gmem1_AWLOCK  = 2'b00;
    assign m_axi_gmem1_AWCACHE = 4'b0011;
    assign m_axi_gmem1_AWPROT  = 3'b000;
    assign m_axi_gmem1_AWQOS   = 4'b0000;
    assign m_axi_gmem1_AWREGION = 4'b0000;
    assign m_axi_gmem1_AWUSER   = '0;
    assign m_axi_gmem1_WVALID = 1'b0;
    assign m_axi_gmem1_WDATA  = '0;
    assign m_axi_gmem1_WSTRB  = {(C_M_AXI_DATA_WIDTH/8){1'b1}};
    assign m_axi_gmem1_WLAST  = 1'b1;
    assign m_axi_gmem1_WID    = '0;
    assign m_axi_gmem1_WUSER  = '0;
    assign m_axi_gmem1_ARVALID = m1_arvalid;
    assign m_axi_gmem1_ARADDR  = m1_araddr;
    assign m_axi_gmem1_ARID    = '0;
    assign m_axi_gmem1_ARLEN   = 8'd0;
    assign m_axi_gmem1_ARSIZE  = 3'd2;
    assign m_axi_gmem1_ARBURST = 2'b01;
    assign m_axi_gmem1_ARLOCK  = 2'b00;
    assign m_axi_gmem1_ARCACHE = 4'b0011;
    assign m_axi_gmem1_ARPROT  = 3'b000;
    assign m_axi_gmem1_ARQOS   = 4'b0000;
    assign m_axi_gmem1_ARREGION = 4'b0000;
    assign m_axi_gmem1_ARUSER   = '0;
    assign m_axi_gmem1_RREADY = m1_rready;
    assign m_axi_gmem1_BREADY = 1'b0;

    assign m_axi_gmem2_AWVALID = m2_awvalid;
    assign m_axi_gmem2_AWADDR  = m2_awaddr;
    assign m_axi_gmem2_AWID    = '0;
    assign m_axi_gmem2_AWLEN   = 8'd0;
    assign m_axi_gmem2_AWSIZE  = 3'd2;
    assign m_axi_gmem2_AWBURST = 2'b01;
    assign m_axi_gmem2_AWLOCK  = 2'b00;
    assign m_axi_gmem2_AWCACHE = 4'b0011;
    assign m_axi_gmem2_AWPROT  = 3'b000;
    assign m_axi_gmem2_AWQOS   = 4'b0000;
    assign m_axi_gmem2_AWREGION = 4'b0000;
    assign m_axi_gmem2_AWUSER   = '0;
    assign m_axi_gmem2_WVALID = m2_wvalid;
    assign m_axi_gmem2_WDATA  = m2_wdata;
    assign m_axi_gmem2_WSTRB  = {(C_M_AXI_DATA_WIDTH/8){1'b1}};
    assign m_axi_gmem2_WLAST  = 1'b1;
    assign m_axi_gmem2_WID    = '0;
    assign m_axi_gmem2_WUSER  = '0;
    assign m_axi_gmem2_ARVALID = 1'b0;
    assign m_axi_gmem2_ARADDR  = '0;
    assign m_axi_gmem2_ARID    = '0;
    assign m_axi_gmem2_ARLEN   = 8'd0;
    assign m_axi_gmem2_ARSIZE  = 3'd2;
    assign m_axi_gmem2_ARBURST = 2'b01;
    assign m_axi_gmem2_ARLOCK  = 2'b00;
    assign m_axi_gmem2_ARCACHE = 4'b0011;
    assign m_axi_gmem2_ARPROT  = 3'b000;
    assign m_axi_gmem2_ARQOS   = 4'b0000;
    assign m_axi_gmem2_ARREGION = 4'b0000;
    assign m_axi_gmem2_ARUSER   = '0;
    assign m_axi_gmem2_RREADY = 1'b0;
    assign m_axi_gmem2_BREADY = m2_bready;

    boids_step_soa_dual #(
        .NUM_OF_BOIDS(MAX_BOIDS),
        .CELL_SHIFT(5)
    ) boids_core (
        .clk(ap_clk),
        .reset(core_reset),
        .start(core_start),
        .active_boids(core_active_boids),
        .in_pos_x(core_in_pos_x),
        .in_pos_y(core_in_pos_y),
        .in_vel_x(core_in_vel_x),
        .in_vel_y(core_in_vel_y),
        .done(core_done),
        .out_pos_x(core_out_pos_x),
        .out_pos_y(core_out_pos_y),
        .out_vel_x(core_out_vel_x),
        .out_vel_y(core_out_vel_y)
    );

    always_ff @(posedge ap_clk) begin
        if (rst) begin
            reg_state_mem <= '0;
            reg_vel_mem <= '0;
            reg_next_state <= '0;
            reg_n <= '0;
            gie_reg <= 1'b0;
            ier_reg <= '0;
            isr_reg <= '0;
            ap_start_reg <= 1'b0;
            auto_restart_reg <= 1'b0;
            ap_done_reg <= 1'b0;
            ap_idle_reg <= 1'b1;
            ap_ready_reg <= 1'b1;
            aw_captured <= 1'b0;
            w_captured <= 1'b0;
            wr_addr <= '0;
            wr_data <= '0;
            wr_strb <= '0;
            rd_addr <= '0;
            s_axi_control_BVALID <= 1'b0;
            s_axi_control_RVALID <= 1'b0;
            s_axi_control_RDATA <= '0;
            kernel_state <= KS_IDLE;
            idx <= '0;
            active_count <= '0;
            rd_state_word <= '0;
            m0_arvalid <= 1'b0;
            m0_araddr <= '0;
            m0_rready <= 1'b0;
            m1_arvalid <= 1'b0;
            m1_araddr <= '0;
            m1_rready <= 1'b0;
            m2_awvalid <= 1'b0;
            m2_awaddr <= '0;
            m2_wvalid <= 1'b0;
            m2_wdata <= '0;
            m2_bready <= 1'b0;
            core_reset <= 1'b1;
            core_start <= 1'b0;
            core_active_boids <= '0;
        end
        else begin
            core_start <= 1'b0;

            if (!aw_captured && s_axi_control_AWVALID && s_axi_control_AWREADY) begin
                aw_captured <= 1'b1;
                wr_addr <= s_axi_control_AWADDR;
            end

            if (!w_captured && s_axi_control_WVALID && s_axi_control_WREADY) begin
                w_captured <= 1'b1;
                wr_data <= s_axi_control_WDATA;
                wr_strb <= s_axi_control_WSTRB;
            end

            if (aw_captured && w_captured && !s_axi_control_BVALID) begin
                case (wr_addr)
                    ADDR_AP_CTRL: begin
                        if (wr_strb[0]) begin
                            if (wr_data[0]) begin
                                ap_start_reg <= 1'b1;
                            end
                            auto_restart_reg <= wr_data[7];
                        end
                    end
                    ADDR_GIE: begin
                        if (wr_strb[0]) begin
                            gie_reg <= wr_data[0];
                        end
                    end
                    ADDR_IER: begin
                        if (wr_strb[0]) begin
                            ier_reg[0] <= wr_data[0];
                        end
                    end
                    ADDR_ISR: begin
                        if (wr_strb[0] && wr_data[0]) begin
                            isr_reg[0] <= 1'b0;
                        end
                    end
                    ADDR_STATE_0: begin
                        reg_state_mem[31:0] <= apply_wstrb(reg_state_mem[31:0], wr_data, wr_strb);
                    end
                    ADDR_STATE_1: begin
                        reg_state_mem[63:32] <= apply_wstrb(reg_state_mem[63:32], wr_data, wr_strb);
                    end
                    ADDR_VEL_0: begin
                        reg_vel_mem[31:0] <= apply_wstrb(reg_vel_mem[31:0], wr_data, wr_strb);
                    end
                    ADDR_VEL_1: begin
                        reg_vel_mem[63:32] <= apply_wstrb(reg_vel_mem[63:32], wr_data, wr_strb);
                    end
                    ADDR_NEXT_0: begin
                        reg_next_state[31:0] <= apply_wstrb(reg_next_state[31:0], wr_data, wr_strb);
                    end
                    ADDR_NEXT_1: begin
                        reg_next_state[63:32] <= apply_wstrb(reg_next_state[63:32], wr_data, wr_strb);
                    end
                    ADDR_N: begin
                        reg_n <= apply_wstrb(reg_n, wr_data, wr_strb);
                    end
                    default: begin
                    end
                endcase

                aw_captured <= 1'b0;
                w_captured <= 1'b0;
                s_axi_control_BVALID <= 1'b1;
            end
            else if (s_axi_control_BVALID && s_axi_control_BREADY) begin
                s_axi_control_BVALID <= 1'b0;
            end

            if (!s_axi_control_RVALID && s_axi_control_ARVALID && s_axi_control_ARREADY) begin
                rd_addr <= s_axi_control_ARADDR;
                s_axi_control_RDATA <= read_reg(s_axi_control_ARADDR);
                s_axi_control_RVALID <= 1'b1;
                if (s_axi_control_ARADDR == ADDR_AP_CTRL) begin
                    ap_done_reg <= 1'b0;
                end
            end
            else if (s_axi_control_RVALID && s_axi_control_RREADY) begin
                s_axi_control_RVALID <= 1'b0;
            end

            case (kernel_state)
                KS_IDLE: begin
                    core_reset <= 1'b1;
                    m0_arvalid <= 1'b0;
                    m0_rready <= 1'b0;
                    m1_arvalid <= 1'b0;
                    m1_rready <= 1'b0;
                    m2_awvalid <= 1'b0;
                    m2_wvalid <= 1'b0;
                    m2_bready <= 1'b0;
                    idx <= '0;

                    if (ap_start_reg) begin
                        active_count <= clamp_n(reg_n);
                        core_active_boids <= clamp_n(reg_n);
                        ap_idle_reg <= 1'b0;
                        ap_ready_reg <= 1'b0;
                        ap_done_reg <= 1'b0;
                        if (!auto_restart_reg) begin
                            ap_start_reg <= 1'b0;
                        end

                        if (clamp_n(reg_n) == '0) begin
                            kernel_state <= KS_DONE;
                        end
                        else begin
                            kernel_state <= KS_LOAD_STATE_AR;
                        end
                    end
                end

                KS_LOAD_STATE_AR: begin
                    if (!m0_arvalid) begin
                        m0_araddr <= reg_state_mem + {{(C_M_AXI_ADDR_WIDTH-IDX_W-2){1'b0}}, idx, 2'b00};
                        m0_arvalid <= 1'b1;
                    end
                    if (m0_arvalid && m_axi_gmem0_ARREADY) begin
                        m0_arvalid <= 1'b0;
                        m0_rready <= 1'b1;
                        kernel_state <= KS_LOAD_STATE_R;
                    end
                end

                KS_LOAD_STATE_R: begin
                    if (m_axi_gmem0_RVALID && m0_rready) begin
                        rd_state_word <= m_axi_gmem0_RDATA;
                        m0_rready <= 1'b0;
                        kernel_state <= KS_LOAD_VEL_AR;
                    end
                end

                KS_LOAD_VEL_AR: begin
                    if (!m1_arvalid) begin
                        m1_araddr <= reg_vel_mem + {{(C_M_AXI_ADDR_WIDTH-IDX_W-2){1'b0}}, idx, 2'b00};
                        m1_arvalid <= 1'b1;
                    end
                    if (m1_arvalid && m_axi_gmem1_ARREADY) begin
                        m1_arvalid <= 1'b0;
                        m1_rready <= 1'b1;
                        kernel_state <= KS_LOAD_VEL_R;
                    end
                end

                KS_LOAD_VEL_R: begin
                    if (m_axi_gmem1_RVALID && m1_rready) begin
                        m1_rready <= 1'b0;
                        core_in_pos_x[idx] <= int16_to_fix(rd_state_word[31:16]);
                        core_in_pos_y[idx] <= int16_to_fix(rd_state_word[15:0]);
                        core_in_vel_x[idx] <= int16_to_fix(m_axi_gmem1_RDATA[31:16]);
                        core_in_vel_y[idx] <= int16_to_fix(m_axi_gmem1_RDATA[15:0]);

                        if ((idx + 1'b1) < active_count) begin
                            idx <= idx + 1'b1;
                            kernel_state <= KS_LOAD_STATE_AR;
                        end
                        else begin
                            idx <= '0;
                            kernel_state <= KS_CORE_RELEASE;
                        end
                    end
                end

                KS_CORE_RELEASE: begin
                    core_reset <= 1'b0;
                    kernel_state <= KS_CORE_START;
                end

                KS_CORE_START: begin
                    core_start <= 1'b1;
                    kernel_state <= KS_CORE_WAIT;
                end

                KS_CORE_WAIT: begin
                    if (core_done) begin
                        idx <= '0;
                        kernel_state <= KS_STORE_AW;
                    end
                end

                KS_STORE_AW: begin
                    if (!m2_awvalid) begin
                        m2_awaddr <= reg_next_state + {{(C_M_AXI_ADDR_WIDTH-IDX_W-2){1'b0}}, idx, 2'b00};
                        m2_awvalid <= 1'b1;
                    end
                    if (m2_awvalid && m_axi_gmem2_AWREADY) begin
                        m2_awvalid <= 1'b0;
                        m2_wdata <= {
                            fix_to_int16($signed(core_out_pos_x[idx])),
                            fix_to_int16($signed(core_out_pos_y[idx]))
                        };
                        m2_wvalid <= 1'b1;
                        kernel_state <= KS_STORE_W;
                    end
                end

                KS_STORE_W: begin
                    if (m2_wvalid && m_axi_gmem2_WREADY) begin
                        m2_wvalid <= 1'b0;
                        m2_bready <= 1'b1;
                        kernel_state <= KS_STORE_B;
                    end
                end

                KS_STORE_B: begin
                    if (m_axi_gmem2_BVALID && m2_bready) begin
                        m2_bready <= 1'b0;
                        if ((idx + 1'b1) < active_count) begin
                            idx <= idx + 1'b1;
                            kernel_state <= KS_STORE_AW;
                        end
                        else begin
                            kernel_state <= KS_DONE;
                        end
                    end
                end

                KS_DONE: begin
                    core_reset <= 1'b1;
                    ap_done_reg <= 1'b1;
                    ap_idle_reg <= 1'b1;
                    ap_ready_reg <= 1'b1;
                    isr_reg[0] <= ier_reg[0];

                    if (auto_restart_reg && (active_count != '0)) begin
                        idx <= '0;
                        ap_done_reg <= 1'b0;
                        ap_idle_reg <= 1'b0;
                        ap_ready_reg <= 1'b0;
                        kernel_state <= KS_LOAD_STATE_AR;
                    end
                    else begin
                        kernel_state <= KS_IDLE;
                    end
                end

                default: begin
                    kernel_state <= KS_IDLE;
                end
            endcase
        end
    end

endmodule
