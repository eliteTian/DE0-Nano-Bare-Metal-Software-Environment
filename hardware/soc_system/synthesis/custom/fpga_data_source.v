`timescale 1ns / 1ps
module fpga_data_source # (
    // Width of address bus in bits
    parameter ADDR_WIDTH = 12,
    // Width of input (slave) interface data bus in bits
    parameter DATA_WIDTH = 32,
    // Width of input (slave) interface wstrb (width of data bus in words)
    parameter STRB_WIDTH = 4,
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
  input             rst,

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

  input  wire [ID_WIDTH-1:0]      axi_awid, //
  input  wire [ADDR_WIDTH-1:0]    axi_awaddr, //
  input  wire [3:0]               axi_awlen, //
  input  wire [2:0]               axi_awsize, //
  input  wire [1:0]               axi_awburst,
  input  wire [3:0]               axi_awcache,
  input  wire [2:0]               axi_awprot,
  input  wire [AWUSER_WIDTH-1:0]  axi_awuser,
  input  wire                     axi_awvalid,//
  output wire                     axi_awready,
  input  wire [1:0]               axi_awlock,
  
  input  wire [DATA_WIDTH-1:0]  axi_wdata,
  input  wire [STRB_WIDTH-1:0]  axi_wstrb,
  input  wire                     axi_wlast,
  input  wire                     axi_wvalid,
  output wire                     axi_wready,
  input  wire [ID_WIDTH-1:0]      axi_wid,

  output wire [ID_WIDTH-1:0]      axi_bid,
  output wire [1:0]               axi_bresp,
  output wire                     axi_bvalid,
  input  wire                     axi_bready,

  input  wire [ID_WIDTH-1:0]      axi_arid,
  input  wire [ADDR_WIDTH-1:0]    axi_araddr,
  input  wire [3:0]               axi_arlen,
  input  wire [2:0]               axi_arsize,
  input  wire [1:0]               axi_arburst,
  input  wire [3:0]               axi_arcache,
  input  wire [2:0]               axi_arprot,
  input  wire                     axi_arvalid,
  output wire                     axi_arready,
  input  wire [1:0]               axi_arlock,

  output wire [ID_WIDTH-1:0]      axi_rid,
  output wire [DATA_WIDTH-1:0]  axi_rdata,
  output wire [1:0]               axi_rresp,
  output wire                     axi_rlast,
  output wire                     axi_rvalid,
  input  wire                     axi_rready
  
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


always @(posedge clk or posedge rst) begin
    if (rst) begin
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
always@(posedge clk, posedge rst) begin
    if(rst) begin
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
 
always@(posedge clk, posedge rst) begin
    if(rst) begin
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
wire                    awvalid = axi_awvalid;




//create a 4K space for AXI slave. Either fixed burst or incr burst can
//access this region. Incremental burst just fills up the buffer. so a
//last written address is needed. Either way, wchannel computes axi_addr;
//AW state machine
//
//
always@(posedge clk, posedge rst) begin
    if(rst) begin
        awready <= 1'b0;
        awaddr  <= 0;
        awlen   <= 0;
        awsize  <= 0;
        awburst <= 0;
        awid   <= 0;
    end else begin
        if(awvalid && wvalid && ~awready && ~wready) begin //awvalid always asserts first
            awaddr  <= axi_awaddr;
            awlen   <= axi_awlen;
            awsize  <= axi_awsize;
            awburst <= axi_awburst;
            awid    <= axi_awid;
            awready <= 1'b1;
        end else begin
            awready <= 1'b0;
        end
    end
end
wire [DATA_WIDTH-1:0]  wdata = axi_wdata;
wire [STRB_WIDTH-1:0]  wstrb =axi_wstrb;
wire                    wlast = axi_wlast;
wire                    wvalid = axi_wvalid ;
reg                     wready;
reg [ID_WIDTH-1:0]      wid;

reg[ADDR_WIDTH -1:0] wr_addr;

wire[31:0] wr_mask;
genvar i;
generate
    for(i = 0; i!= STRB_WIDTH; i=i+1) begin : STROBE
        assign wr_mask[8*(i+1)-1:8*i] = wstrb[i]? 8'hFF: 0;
    end
endgenerate

wire[ADDR_WIDTH-3:0] ram_index = wr_addr[ADDR_WIDTH-1:2];
//W state machine
reg write_pending;
always@(posedge clk, posedge rst) begin
    if(rst) begin
        w_state <= 2'b00;
        wready <= 1'b0;
        wid   <= 0;
        wr_addr <= 0;
        write_pending <= 1'b0;
    end else begin
        if(awvalid && wvalid && ~awready && ~write_pending) begin //new AW addr update cycle.next cycle awready asserts
            wr_addr <= axi_awaddr; 
            wready <= 1'b1;
            write_pending <= 1'b1;
        end else if(wvalid & write_pending) begin // If not aw addr update cycle, take data when wvalid.
            if(wlast) begin
                axi_mem[ram_index] <= wdata & wr_mask;
                wready <= 1'b0;
                write_pending <= 1'b0;
            end else begin
                axi_mem[ram_index] <= wdata & wr_mask ;            
                wr_addr <= wr_addr + 4;
            end
        end
        
    end
end




reg [ID_WIDTH-1:0]      bid;
reg [1:0]               bresp;
reg                     bvalid;
wire                    bready = axi_bready;

//B state machine
always@(posedge clk, posedge rst) begin
    if(rst) begin
        bid    <= 0;
        bvalid <= 1'b0;
        bresp  <= 0;
        b_state <= 0;
    end else begin
        if(wlast&wready&wvalid) begin // finish condition
            if(bready&&bvalid) begin //when handshake happens, can deassert bvalid
                bvalid <= 1'b0; //assert bvalid regardless of bready
            end else begin //assert first and check handshake
                bvalid <= 1'b1;
            end
            bresp <= 0;
            bid <= awid;
        end else begin 
            bvalid <= 1'b0;
        end

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
wire                    arready;
wire                    arvalid = axi_arvalid;


wire [ID_WIDTH-1:0]     bottom_arid;
wire [ADDR_WIDTH-1:0]   bottom_araddr;
wire [3:0]              bottom_arlen;
wire [2:0]              bottom_arsize;
wire [1:0]              bottom_arburst;
wire                    bottom_arvalid;
//ADDR_WIDTH == 12
wire [31-ADDR_WIDTH:0]  rsv_addr_lhsv;
wire [31-ADDR_WIDTH:0]  rsv_addr_rhsv;

//AR state machine
assign arready = ~ar_slot0[0]; //as long as the last slot is not occupied, can accept cmd

reg[63:0]   ar_slot0;
reg[63:0]   ar_slot1;
reg[63:0]   ar_slot2;
reg[63:0]   ar_slot3;

wire[63:0] ar_cmd = { axi_araddr, rsv_addr_rhsv, //32bit
                      axi_arlen, 1'b0, axi_arsize,      //8bit
                      axi_arburst,6'h0, //2bit
                      axi_arid, 1'b0, axi_arvalid};  //16bit

wire ar_push = arvalid & arready;
wire ar_cmd_avail = ar_slot0[0] | ar_slot1[0] | ar_slot2[0] | ar_slot3[0];

always@(posedge clk, posedge rst) begin
    if(rst) begin
        ar_slot0 <=0; 
        ar_slot1 <=0;
        ar_slot2 <=0;
        ar_slot3 <=0;
    end else begin
        if(ar_pull) begin
            if(ar_slot0[0]) begin
                ar_slot0 <= 0;
            end else if(ar_slot1[0]) begin
                ar_slot1 <= 0;
            end else if(ar_slot2[0]) begin
                ar_slot2 <= 0;
            end else if(ar_slot3[0]) begin
                ar_slot3 <= 0;
            end
        end
        
        if(ar_push) begin
            ar_slot0 <= ar_slot1;
            ar_slot1 <= ar_slot2;
            ar_slot2 <= ar_slot3;
            ar_slot3 <= ar_cmd;
        end
        
    end
end

wire[63:0] ar_cmd_to_process = ar_slot0[0]? ar_slot0:
                               ar_slot1[0]? ar_slot1:
                               ar_slot2[0]? ar_slot2:
                               ar_slot3[0]? ar_slot3: 0;
wire [4:0] rsv5;
wire [5:0] rsv6;
wire       rsv1;
wire       rsv1_0;

//wire[63:0] ar_cmd = { axi_araddr, rsv_addr_rhsv //32bit
//                      axi_arlen, 1'b0, axi_arsize      //8bit
//                      axi_arburst,6'h0, //2bit
//                      axi_arid, 1'b0, axi_arvalid};  //16bit

assign  {bottom_araddr, rsv_addr_lhsv,                     //32bit
         bottom_arlen,rsv1_0, bottom_arsize,               //8bit
         bottom_arburst, rsv6,                             //2bit
         bottom_arid, rsv1, bottom_arvalid} = ar_cmd_to_process;

//storage space
reg[31:0]  axi_mem[0:2**(ADDR_WIDTH-3)-1]; //each is a word, not a bite.


wire[ADDR_WIDTH-3:0] rd_ram_index = araddr[ADDR_WIDTH-1:2];
assign rdata = rvalid? axi_mem[rd_ram_index]:0;

wire [DATA_WIDTH-1:0]   rdata;
reg [STRB_WIDTH-1:0]    rstrb;
reg                     rlast;
reg                     rvalid;
reg [ID_WIDTH-1:0]      rid;
reg [1:0]               rresp;
wire                    rready = axi_rready;
reg                     read_pending;
reg                     ar_pull;

//R state machine
always@(posedge clk, posedge rst) begin
    if(rst) begin
        rid          <= 0;
        rresp        <= 0;
        rlast        <= 1'b0;
        rvalid       <= 1'b0;
        read_pending <= 1'b0;
        arlen        <= 0;        
        araddr       <= 0;

    end else begin
        if(~read_pending) begin
            rlast <= 1'b0;
            rvalid <= 1'b0;
            ar_pull <= 1'b0;            
            if(ar_cmd_avail && ~ar_pull) begin //update araddr/ arlen
                ar_pull <= 1'b1;
                araddr  <= bottom_araddr;
                rid     <= bottom_arid;
                if(bottom_arlen==0) begin
                    rvalid <= 1'b1;
                    rlast  <= 1'b1;
                end else begin
                    read_pending <= 1'b1;
                    rvalid <= 1'b1;
                    arlen <= bottom_arlen-1;

                end
            end
        end else begin
            ar_pull <= 1'b0;            
            if(arlen!=0) begin
                if(rready) begin
                    arlen <= arlen - 1;
                    araddr <= araddr + 4;
                end
            end else begin //last beat
                rlast <= 1'b1;
                if(rready) begin
                    araddr <= araddr + 4;
                    read_pending <= 1'b0;
                end
            end
        end

    end
end

assign axi_awready    = awready;
assign axi_wready     = wready;
assign axi_arready    = arready;
assign axi_bid        = bid;
assign axi_bresp      = bresp;
assign axi_bvalid     = bvalid;
assign axi_rid        = rid;
assign axi_rdata      = rdata;
assign axi_rresp      = rresp;
assign axi_rlast      = rlast;
assign axi_rvalid     = rvalid;

endmodule
