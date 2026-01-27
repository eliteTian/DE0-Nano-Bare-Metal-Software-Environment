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
  output wire [DATA_WIDTH-1:0]    axi_rdata,
  output wire [1:0]               axi_rresp,
  output wire                     axi_rlast,
  output wire                     axi_rvalid,
  input  wire                     axi_rready
  
);

parameter  FIFO_DEPTH = 128;
parameter  PTR_WIDTH = $clog2(FIFO_DEPTH);


wire              clk_en;
reg [31:0]          CTRL, STAT, DMA_CTRL;
wire[31:0]          dbg_reg;


wire        cmd_valid = CTRL[0]; // 1 means a pending cmd
wire[1:0]   cmd_type = CTRL[2:1]; //wr = 1, rd = 0; 2'b01=dump; 2'b11 = rsvd 
wire[11:0]  cmd_addr = CTRL[15:4]; //4K bytes
wire[7:0]   cmd_data = CTRL[23:16];
wire        cmd_clear_cnt = CTRL[31];

wire        pend    = STAT[0];

assign    dma_single = 1'b1; 
reg       dma_req_reg;
      
assign    dma_req = DMA_CTRL[0];


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
            if(dma_ack) begin
                DMA_CTRL[0] <= 1'b0;
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
wire                    awready;
reg                     awinfo_acquired;
wire [31:0]             awinfo = {awsize,awaddr,awlen,awburst};
wire                    awvalid = axi_awvalid;




//create a 4K space for AXI slave. Either fixed burst or incr burst can
//access this region. Incremental burst just fills up the buffer. so a
//last written address is needed. Either way, wchannel computes axi_addr;
//AW state machine
assign awready = ~aw_slot0[0]; //as long as the last slot is not occupied, can accept cmd

reg[63:0]   aw_slot0;
reg[63:0]   aw_slot1;
reg[63:0]   aw_slot2;
reg[63:0]   aw_slot3;
wire [31-ADDR_WIDTH:0]  rsv_addr_lhsv_aw;
wire [31-ADDR_WIDTH:0]  rsv_addr_rhsv_aw;

wire[63:0] aw_cmd = { axi_awaddr, rsv_addr_rhsv_aw, //32bit
                      axi_awlen, 1'b0, axi_awsize,      //8bit
                      axi_awburst,6'h0, //2bit
                      axi_awid, 1'b0, axi_awvalid};  //16bit
                  
                  

wire aw_push = awvalid & awready;
wire aw_cmd_avail = aw_slot0[0] | aw_slot1[0] | aw_slot2[0] | aw_slot3[0];
reg  aw_pull;
always@(posedge clk, posedge rst) begin
    if(rst) begin
        aw_slot0 <=0; 
        aw_slot1 <=0;
        aw_slot2 <=0;
        aw_slot3 <=0;
    end else begin
        if(aw_pull) begin
            if(aw_slot0[0]) begin
                aw_slot0 <= 0;
            end else if(aw_slot1[0]) begin
                aw_slot1 <= 0;
            end else if(aw_slot2[0]) begin
                aw_slot2 <= 0;
            end else if(aw_slot3[0]) begin
                aw_slot3 <= 0;
            end
        end
        if(aw_push) begin
            aw_slot0 <= aw_slot1;
            aw_slot1 <= aw_slot2;
            aw_slot2 <= aw_slot3;
            aw_slot3 <= aw_cmd;
        end 
    end
end

wire[63:0] aw_cmd_to_process = aw_slot0[0]? aw_slot0:
                               aw_slot1[0]? aw_slot1:
                               aw_slot2[0]? aw_slot2:
                               aw_slot3[0]? aw_slot3: 0;
wire [4:0] rsv5_aw;
wire [5:0] rsv6_aw;
wire       rsv1_aw;
wire       rsv1_aw_0;

wire [ID_WIDTH-1:0]     bottom_awid;
wire [ADDR_WIDTH-1:0]   bottom_awaddr;
wire [3:0]              bottom_awlen;
wire [2:0]              bottom_awsize;
wire [1:0]              bottom_awburst;
wire                    bottom_awvalid;



assign  {bottom_awaddr, rsv_addr_lhsv_aw,                     //32bit
         bottom_awlen,rsv1_aw, bottom_awsize,               //8bit
         bottom_awburst, rsv6_aw,                             //2bit
         bottom_awid, rsv1_aw_0, bottom_arvalid} = aw_cmd_to_process;





wire [DATA_WIDTH-1:0]   wdata = axi_wdata;
wire [STRB_WIDTH-1:0]   wstrb =axi_wstrb;
wire                    wlast = axi_wlast;
wire                    wvalid = axi_wvalid ;
wire                    wready;
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
reg[DATA_WIDTH-1:0]     fifo[0:FIFO_DEPTH-1];
reg[PTR_WIDTH:0]        wrptr, rdptr; //one extra bit to show status
wire[PTR_WIDTH:0]       wrptr_next = wrptr+1;
wire                     wr_fifo;
reg                      rd_fifo;

reg[2:0]                frame_num; //how many frames are in fifo
always@(posedge clk) begin
    if(wr_fifo) begin
        fifo[wrptr[PTR_WIDTH-1:0]] <= wdata_fifo;
    end
end

always@(posedge clk, posedge rst) begin
    if(rst) begin
        wrptr <= 0;
        rdptr <= 0;
        frame_num <= 0;
    end else begin
        wrptr       <= wrptr+wr_fifo;
        rdptr       <= rdptr+rd_fifo;
        frame_num   <= wlast? frame_num + 1 : frame_num;
    end
end
wire afull = wrptr_next[PTR_WIDTH-1:0]==rdptr[PTR_WIDTH-1:0] && wrptr_next[PTR_WIDTH]^rdptr[PTR_WIDTH];
wire full  = wrptr[PTR_WIDTH-1:0]==rdptr[PTR_WIDTH-1:0] && wrptr[PTR_WIDTH]^rdptr[PTR_WIDTH];
wire empty = wrptr == rdptr;

assign wready       = ~afull;
assign wr_fifo      = wvalid && wready;
wire[31:0] wdata_fifo   = wdata & wr_mask;
wire[31:0] rdata_fifo   = fifo[rdptr[PTR_WIDTH-1:0]];
//Write machine
reg         write_done;

always@(posedge clk, posedge rst) begin
    if(rst) begin
        wid           <= 0;
        write_pending <= 1'b0;
        aw_pull       <= 1'b0;
        awlen         <= 0;
        rd_fifo       <= 0;
        wr_addr       <= 0;
        awid          <= 0;
        write_done    <= 1'b0;
    end else begin
        if(~write_pending) begin
            rd_fifo <= 1'b0;
            aw_pull <= 1'b0;
            write_done <= 1'b0;
            if(aw_cmd_avail && ~aw_pull) begin //update awaddr/ awlen
                aw_pull <= 1'b1;
                wr_addr  <= bottom_awaddr;
                wid      <= bottom_awid; //axi3 needs
                awlen    <= bottom_awlen;
                awid     <= bottom_awid;
                write_pending <= 1'b1;
                rd_fifo <= ~empty; // only read when fifo not empty, rd_fifo is clearing old data
            end
        end else begin
            aw_pull <= 1'b0;
            if(awlen!=0) begin //multiple burst
                if(rd_fifo) begin //rdata available 
                    axi_mem[ram_index] <= rdata_fifo ;
                    rd_fifo <= ~empty; //as long as fifo not empty, read so that first data fall through.
                    wr_addr <= wr_addr + 4;
                    awlen <= awlen - 1;     
                end
            end else begin //single burst//end burst
                if(rd_fifo) begin //rdata available 
                    axi_mem[ram_index] <= rdata_fifo ;
                    rd_fifo <= ~empty; //as long as fifo not empty, read so that first data fall through.
                    write_pending <= 0;
                    rd_fifo <= 1'b0;
                    write_done <= 1'b1;
                end
            end
       end
    end
end






reg [ID_WIDTH-1:0]      bid;
reg [1:0]               bresp;
reg                     bvalid;
wire                    bready = axi_bready;
reg                     wait_handshake;
//B state machine
always@(posedge clk, posedge rst) begin
    if(rst) begin
        bid    <= 0;
        bvalid <= 1'b0;
        bresp  <= 0;
        b_state <= 0;
        wait_handshake <= 0;
    end else begin
        //if(wlast&wready&wvalid) begin // finish condition
        if(~wait_handshake) begin
            if(write_done) begin
                wait_handshake <= 1'b1;
                bvalid <= 1'b1;
                bresp <= 0;
                bid <= awid;
            end
        end else begin
            if(bready) begin
                wait_handshake <= 1'b0;
                bvalid <= 1'b0;
            end
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
reg[31:0]  axi_mem[0:2**(ADDR_WIDTH-2)-1]; //each is a word, not a bite.


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
