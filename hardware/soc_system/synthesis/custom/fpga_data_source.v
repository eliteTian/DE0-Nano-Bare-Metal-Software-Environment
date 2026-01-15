`timescale 1ns / 1ps
module fpga_data_source # (
    // Width of address bus in bits
    parameter ADDR_WIDTH = 10,
    // Width of input (slave) interface data bus in bits
    parameter S_DATA_WIDTH = 32,
    // Width of input (slave) interface wstrb (width of data bus in words)
    parameter S_STRB_WIDTH = 4,
    // Width of ID signal
    parameter ID_WIDTH = 14,
    // Propagate awuser signal
    parameter AWUSER_ENABLE = 0,
    // Width of awuser signal
    parameter AWUSER_WIDTH = 1,
    // Propagate wuser signal
    parameter WUSER_ENABLE = 0,
    // Width of wuser signal
    parameter WUSER_WIDTH = 1,
    // Propagate buser signal
    parameter BUSER_ENABLE = 0,
    // Width of buser signal
    parameter BUSER_WIDTH = 1)
(
  input             clk,
  input             reset_n,

  //avalon slave interface
  output  [ 31: 0]  avs_readdata,
  input   [  1: 0]  avs_address,
  input             avs_chipselect,
  input             avs_write_n,
  input   [ 31: 0]  avs_writedata,

  output [7:0]      axis4_m_tdata,
  output            axis4_m_tvalid,
  output            axis4_m_tlast,
  input             axis4_m_tready,

  input             dma_ack,
  output            dma_req,
  output            dma_single, 

  input  wire [ID_WIDTH-1:0]      s_axi_awid,
  input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
  input  wire [3:0]               s_axi_awlen,
  input  wire [2:0]               s_axi_awsize,
  input  wire [1:0]               s_axi_awburst,
  input  wire [3:0]               s_axi_awcache,
  input  wire [2:0]               s_axi_awprot,
  input  wire [AWUSER_WIDTH-1:0]  s_axi_awuser,
  input  wire                     s_axi_awvalid,
  output wire                     s_axi_awready,
  input  wire [1:0]               s_axi_awlock,
  
  input  wire [S_DATA_WIDTH-1:0]  s_axi_wdata,
  input  wire [S_STRB_WIDTH-1:0]  s_axi_wstrb,
  input  wire                     s_axi_wlast,
  input  wire                     s_axi_wvalid,
  output wire                     s_axi_wready,
  input  wire [ID_WIDTH-1:0]      s_axi_wid,

  output wire [ID_WIDTH-1:0]      s_axi_bid,
  output wire [1:0]               s_axi_bresp,
  output wire                     s_axi_bvalid,
  input  wire                     s_axi_bready,

  input  wire [ID_WIDTH-1:0]      s_axi_arid,
  input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
  input  wire [3:0]               s_axi_arlen,
  input  wire [2:0]               s_axi_arsize,
  input  wire [1:0]               s_axi_arburst,
  input  wire [3:0]               s_axi_arcache,
  input  wire [2:0]               s_axi_arprot,
  input  wire                     s_axi_arvalid,
  output wire                     s_axi_arready,
  input  wire [1:0]               s_axi_arlock,

  output wire [ID_WIDTH-1:0]      s_axi_rid,
  output wire [S_DATA_WIDTH-1:0]  s_axi_rdata,
  output wire [1:0]               s_axi_rresp,
  output wire                     s_axi_rlast,
  output wire                     s_axi_rvalid,
  input  wire                     s_axi_rready
  
);

wire              clk_en;
reg [31:0]          CTRL, STAT, DMA_CTRL;
wire[31:0]          dbg_reg;


wire        cmd_valid = CTRL[0]; // 1 means a pending cmd
wire[1:0]   cmd_type = CTRL[2:1]; //wr = 1, rd = 0; 2'b01=dump; 2'b11 = rsvd 
wire[11:0]  cmd_addr = CTRL[15:4]; //4K bytes
wire[7:0]   cmd_data = CTRL[23:16];
wire        cmd_clear_cnt = CTRL[31];

wire        pend    = STAT[0];


always @(posedge clk or negedge  reset_n) begin
    if (reset_n == 0) begin
        CTRL <= 0;
        DMA_CTRL <= 0;
    end else begin
        if (avs_chipselect && ~avs_write_n) begin
          case( avs_address)
            2'b00:
              CTRL <= avs_writedata;
            2'b10:
              DMA_CTRL <= avs_writedata;
            default: ;
          endcase
        end else begin
            if(clear_cmd) begin // clear cmd valid by HW.
                CTRL <= CTRL & 32'hFFFFFFFE;
            end
            if(cmd_clear_cnt) begin //clear bit 31, cmd_clear_cnt by HW
                CTRL <= CTRL & 32'hEFFFFFFF;
            end
        end
    end

  end

assign  avs_readdata =  avs_address==2'b00 ? CTRL: 
                        avs_address==2'b01 ? STAT:
                        avs_address==2'b10 ? DMA_CTRL: 
                        avs_address==2'b11 ? dbg_reg : 32'hFFFFFFFF;


reg[7:0]  mem[0:4095];
reg[11:0] addr;
reg[7:0]  as_rdata;
wire[7:0] as_rdata_w;
reg       as_rvalid;

reg [7:0]  axis4_m_tdata_r;
reg        axis4_m_tvalid_r;
reg        axis4_m_tlast_r;
reg        axis4_m_tready_r;
reg        rd_en;
reg        wr_en;
reg        clear_cmd;

always@(posedge clk) begin
    if(wr_en) begin
        mem[addr] <= cmd_data;
        as_rvalid <= 1'b0;
    end else begin
        if(rd_en) begin
            as_rdata <= mem[addr];
            as_rvalid <= 1'b1;
        end else begin
            as_rvalid <= 1'b0;
        end
    end
end

assign as_rdata_w = mem[addr];

reg[1:0] state;
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        state <= 2'b00;
        STAT <= 0;
        axis4_m_tdata_r <= 0;
        axis4_m_tvalid_r <= 0;
        axis4_m_tlast_r <= 1'b1;    
        rd_en <= 1'b0;
        wr_en <= 1'b0;
        clear_cmd <= 1'b0;
        axis4_m_tlast_r <= 1'b0;

    end else begin
        case(state)
            2'b00: begin //IDLE state, awaiting command, wr cmd can be done in a single cycle, rd command has to wait for completion
                rd_en <= 1'b0;
                wr_en <= 1'b0;
                clear_cmd <= 1'b0;
                axis4_m_tlast_r <= 1'b0;
                
                if(cmd_valid) begin
                    STAT <= 32'b1;
                    addr <= cmd_addr;
                    clear_cmd <= 1'b1;
                    if(cmd_type == 2'b01) begin // write to ram
                        wr_en <= 1'b1;
                        state <= 2'b00;
                    end else if(cmd_type == 2'b00) begin //read from ram
                        rd_en <= 1'b1;
                        state <= 2'b01; //wait for as_rvalid
                    end else if(cmd_type == 2'b10) begin //dump block ram from addr 0 via axi4 st if
                        addr <= 0;
                        state <= 2'b10;
                        axis4_m_tvalid_r <= 1'b1;
                    end
                end

            end
            2'b01: begin
                clear_cmd <= 1'b0;
                rd_en <= 1'b0;
                if(as_rvalid==1'b1) begin
                    STAT[0] <= 1'b0;
                    STAT[15:8] <= as_rdata;
                    state <= 2'b00;
                end
            end

            2'b10: begin //to be studied
                if(~&addr) begin //addr is not all fs.
                    if(axis4_m_tready) begin
                        addr <= addr + 1;
                    end
                end else begin
                    STAT[0] <= 1'b0;
                    state <= 2'b00;
                    axis4_m_tvalid_r <= 1'b0;
                    axis4_m_tlast_r <= 1'b1;
                end
            end
            
        endcase
    end
end
                    

assign axis4_m_tdata = as_rdata_w;
assign axis4_m_tvalid = axis4_m_tvalid_r;
assign axis4_m_tlast = axis4_m_tlast_r;

reg[15:0] cnt;
 
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        cnt <= 0;
    end else begin
        if(cmd_clear_cnt) begin
            cnt <= 0;
        end else begin
            cnt <= axis4_m_tvalid_r&axis4_m_tready? cnt+1: cnt;
        end
    end
end

assign dbg_reg[15:0] = cnt;
assign dbg_reg[27:16] = addr;
assign dbg_reg[29:28] = state;

reg[1:0]    ar_state;
reg[1:0]    aw_state;
reg[1:0]    w_state;
reg[1:0]    b_state;
reg[1:0]    r_state;

//AXI slave device
//Two relationships that must be maintained are:
//• read data must always follow the address to which the data relates
//• a write response must always follow the last write transfer in the write transaction
//to which the write response relates.
//In any transaction:
//• the VALID signal of one AXI component must not be dependent on the READY
//signal of the other component in the transaction
//• the READY signal can wait for assertion of the VALID signal.
//While it is acceptable to wait for VALID to be asserted before asserting READY, it is
//also acceptable to assert READY by default prior to the assertion of VALID and this
//can result in a more efficient design.
reg [ID_WIDTH-1:0]      awid;
reg [ADDR_WIDTH-1:0]    awaddr;
reg [3:0]               awlen;
reg [2:0]               awsize;
reg [1:0]               awburst;
reg [3:0]               awcache;
reg [2:0]               awprot;
reg [AWUSER_WIDTH-1:0]  awuser;
reg [1:0]               awlock;
reg                     awready;
reg                     awinfo_acquired;
wire [31:0]             awinfo = {awsize,awaddr,awlen,awburst};
wire                    awvalid = s_axi_awvalid;

//only when in b_state==2'b11, this state, wr_phase is possible to be deasserted.
reg[1:0]                wr_phase; 

always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        wr_phase <= 2'b00;
    end else begin
        if(awvalid && awready) begin //awvalid always asserts first
            wr_phase <= 2'b01; //commensing
        end else if (wlast) begin //wrapping up
            wr_phase <= 2'b10;
        end else if(bready&bvalid) begin
            wr_phase <= 2'b00; //idle
        end
    end
end

reg[1:0]                rd_phase; 

always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        rd_phase <= 2'b00;
    end else begin
        if(arvalid & arready) begin //awvalid always asserts first
            rd_phase <= 2'b01; //commensing
        end else if (rlast) begin //wrapping up
            rd_phase <= 2'b10;
        end 
    end
end



//create a 4K space for AXI slave. Either fixed burst or incr burst can
//access this region. Incremental burst just fills up the buffer. so a
//last written address is needed. Either way, wchannel computes axi_addr;
//AW state machine
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        aw_state <= 2'b00;
        awready <= 1'b1;
        awaddr  <= 0;
        awlen   <= 0;
        awsize  <= 0;
        awburst <= 0;
        awid   <= 0;
    end else begin
        case(aw_state)
            2'b00: begin // idle state, awaiting awvalid
                awready <= 1'b1;
                if(awvalid && awready) begin //awvalid always asserts first
                    awready <= 1'b0;
                    awaddr  <= s_axi_awaddr;
                    awlen   <= s_axi_awlen;
                    awsize  <= s_axi_awsize;
                    awburst <= s_axi_awburst;
                    awid   <= s_axi_awid;
                    aw_state <= 2'b01; //wait for write completion, deassert awready.
                end
            end
            2'b01: begin
                awready <= 1'b0;
                if(wr_phase == 2'b10 ) begin
                    aw_state <= 2'b00;
                    awaddr  <= 0;
                    awlen   <= 0;
                    awsize  <= 0;
                    awburst <= 0;
                    awid   <= 0;
                    awinfo_acquired <= 1'b0;     
                end
            end
        endcase
    end
end

reg [S_DATA_WIDTH-1:0]  wdata;
reg [S_STRB_WIDTH-1:0]  wstrb;
reg                     wlast;
reg                     wvalid;
reg                     wready;
reg [ID_WIDTH-1:0]      wid;
reg[9:0]                axi_addr_reg;

//W state machine
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        w_state <= 2'b00;
        wready <= 1'b1;
        wid   <= 0;
        axi_addr_reg <= 0;
    end else begin
        case(w_state)
            2'b00: begin //awaiting registered info from AW channel.
                wready <= 1'b0;                
                if(wr_phase==2'b01) begin
                    if(awlen==0 && awburst==0) begin //only support single, fixed burst for now
                        w_state <= 2'b01;
                        axi_addr_reg <= awaddr;
                    end
                end
            end
            2'b01: begin //writing in operation
                wready <= 1'b1;
                if(wvalid && wready) begin
                    axi_addr_reg <= axi_addr;
                    if(wlast) begin //awvalid always asserts first
                        wready     <= 1'b0;
                        w_state    <= 2'b00; //wait for write completion, deassert awready.
                    end 
                end
            end
        endcase
    end
end


reg [ID_WIDTH-1:0]      bid;
reg [1:0]               bresp;
reg                     bvalid;
wire                    bready = s_axi_bready;

//B state machine
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        bid    <= 0;
        bvalid <= 1'b0;
        bresp  <= 0;

    end else begin
        case(b_state)
            2'b00: begin //awaiting last beat.       
                if(wr_phase==2'b10) begin
                    b_state <= 2'b01;
                    bvalid <= 1'b1;
                    bresp <= 0;
                    bid <= awid;
                end
            end
            2'b01: begin //asserting valid and let master know the response
                if(bready&bvalid) begin
                    b_state <= 2'b00;
                    bvalid <= 1'b0;
                end
            end
        endcase
    end
end

reg [ID_WIDTH-1:0]      arid;
reg [ADDR_WIDTH-1:0]    araddr;
reg [3:0]               arlen;
reg [2:0]               arsize;
reg [1:0]               arburst;
reg [3:0]               arcache;
reg [2:0]               arprot;
reg [AWUSER_WIDTH-1:0]  aruser;
reg [1:0]               arlock;
reg                     arready;
wire                    arvalid = s_axi_arvalid;

//AR state machine
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        ar_state <= 2'b00;
        arready <= 1'b1;
        araddr  <= 0;
        arlen   <= 0;
        arsize  <= 0;
        arburst <= 0;
        arid   <= 0;
    end else begin
        case(ar_state)
            2'b00: begin // idle state, awaiting awvalid
                arready <= 1'b1;
                if(arvalid && arready) begin //arvalid always asserts first
                    arready <= 1'b0;
                    araddr  <= s_axi_araddr;
                    arlen   <= s_axi_arlen;
                    arsize  <= s_axi_arsize;
                    arburst <= s_axi_arburst;
                    arid   <= s_axi_arid;
                    ar_state <= 2'b01; //wait for write completion, deassert awready.
                end
            end
            2'b01: begin             
                arready <= 1'b0;
                if(rd_phase==2'b01) begin
                    ar_state <= 2'b00;
                    araddr  <= 0;
                    arlen   <= 0;
                    arsize  <= 0;
                    arburst <= 0;
                    arid   <= 0;
                end
            end
        endcase
    end
end



//storage space
reg[31:0]  axi_mem[0:1024];
wire[11:0] axi_addr;

assign axi_addr = awaddr;

wire axi_wr_en = wready&wvalid; //so far only burst size 0

always@(posedge clk) begin
    if(axi_wr_en) begin
        axi_mem[axi_addr[11:2]] <= s_axi_wdata ;
    end 
end

assign rdata = rvalid? axi_mem[araddr]:0;

wire [S_DATA_WIDTH-1:0]  rdata;
reg [S_STRB_WIDTH-1:0]  rstrb;
reg                     rlast;
reg                     rvalid;
reg                     rready;
reg [ID_WIDTH-1:0]      rid;
reg [1:0]               rresp;


//R state machine
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        r_state <= 2'b00;
        rready <= 1'b1;
        rid   <= 0;
        rresp <= 0;
    end else begin
        case(r_state)
            2'b00: begin //awaiting registered info from AW channel.
                rready <= 1'b0;                
                if(rd_phase==2'b01) begin
                    if(arlen==0 && arburst==0) begin //only support single, fixed burst read for now
                        r_state <= 2'b01;
                        rvalid <= 1'b1;
                        rlast <= 1'b1;
                        rid <= arid;
                    end
                end
            end
            2'b01: begin //waiting for handshake from master
                if(rready & rvalid) begin
                    rresp <= 2'b00;
                    r_state <= 2'b00;
                end
            end
        endcase
    end
end

assign s_axi_awready    = awready;
assign s_axi_wready     = wready;
assign s_axi_arready    = arready;
assign s_axi_bid        = bid;
assign s_axi_bresp      = bresp;
assign s_axi_bvalid     = bvalid;
assign s_axi_rid        = rid;
assign s_axi_rdata      = rdata;
assign s_axi_rresp      = rresp;
assign s_axi_rlast      = rlast;
assign s_axi_rvalid     = rvalid;
endmodule
