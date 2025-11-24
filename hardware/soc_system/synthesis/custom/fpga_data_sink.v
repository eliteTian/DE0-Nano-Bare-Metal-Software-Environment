module fpga_data_sink (
  input            clk,
  input            reset_n,
  //avalon slave interface
  output  [ 31: 0] avs_readdata,
  input   [  1: 0] avs_address,
  input            avs_chipselect,
  input            avs_write_n,
  input   [ 31: 0] avs_writedata,

  input [7:0]      axis4_s_tdata,
  input            axis4_s_tvalid,
  input            axis4_s_tlast,
  output           axis4_s_tready
  
);

wire              clk_en;
reg [31:0]   CTRL, STAT, reg2;
wire[31:0]   dbg_reg;
wire         cmd_valid = CTRL[0]; // 1 means a pending cmd
wire[1:0]    cmd_type = CTRL[2:1]; //wr = 1, rd = 0; 2'b01=dump; 2'b11 = rsvd 
wire[11:0]   cmd_addr = CTRL[15:4];
wire[7:0]    cmd_data = CTRL[23:16];
wire         cmd_clear_cnt = CTRL[31];

wire        pend    = STAT[0];


  always @(posedge clk or negedge  reset_n) begin
        if (reset_n == 0) begin
          CTRL <= 0;
          reg2 <= 0;
        end else begin
          if (avs_chipselect && ~avs_write_n) begin
            case( avs_address)
              2'b00:
                CTRL <= avs_writedata;
              2'b10:
                reg2 <= avs_writedata;
              default: ;
            endcase
        end else if(clear_cmd) begin // clear cmd valid by HW.
              CTRL <= CTRL & 32'hFFFFFFFE;
        end else if(cmd_clear_cnt) begin
              CTRL <= CTRL & 32'hEFFFFFFF;
        end

      end

    end


  assign  avs_readdata =  avs_address==0? CTRL: 
                          avs_address==2'b01 ? STAT:
                          avs_address==2'b10 ? reg2:
                          avs_address==2'b11 ? dbg_reg: 32'hFFFFFFFF;

reg[7:0]    mem[0:4095];
reg[11:0]   addr;
reg[7:0]    rdata;
reg         rvalid;


reg        axis4_s_tready_r;
reg[7:0]   axis4_s_tdata_r;

reg        rd_en;
reg        wr_en;
reg        clear_cmd;

always@(posedge clk) begin
    if(wr_en) begin
        if(state==2'b10) begin
            mem[addr] <= axis4_s_tdata_r;
        end else begin
            mem[addr] <= cmd_data;
        end
    end else begin
        if(rd_en) begin
            rdata <= mem[addr];
            rvalid <= 1'b1;
        end else begin
            rvalid <= 1'b0;
        end
    end
end

reg[1:0] state;
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        state <= 2'b00;
        STAT <= 0;
        rd_en <= 1'b0;
        wr_en <= 1'b0;
        clear_cmd <= 1'b0;
        axis4_s_tdata_r <= 0;
    end else begin
        case(state)
            2'b00: begin //IDLE state, awaiting command, wr cmd can be done in a single cycle, rd command has to wait for completion
                rd_en <= 1'b0;
                wr_en <= 1'b0;
                clear_cmd <= 1'b0;
                if(cmd_valid) begin
                    STAT <= 32'b1;
                    state <= 2'b01;
                    addr <= cmd_addr;
                    clear_cmd <= 1'b1;
                    if(cmd_type == 2'b01) begin // write to ram
                        wr_en <= 1'b1;
                        state <= 2'b00;
                    end 
                    if(cmd_type == 2'b00) begin //read from ram
                        rd_en <= 1'b1;
                        state <= 2'b01; //wait for rvalid
                    end
                    if(cmd_type == 2'b10) begin //receive dump block ram from addr 0 via axi4 st if
                        addr <= 0;
                        state <= 2'b10;
                    end
                end
            end
            2'b01: begin
                clear_cmd <= 1'b0;
                rd_en <= 1'b0;
                if(rvalid==1'b1) begin
                    STAT[0] <= 1'b0;
                    STAT[15:8] <= rdata;
                    state <= 2'b00;
                end
            end

            2'b10: begin //getting data from axis4 master.
                clear_cmd <= 1'b0;
                if(~&addr) begin //when addr is all 1's
                    if(axis4_s_tvalid & axis4_s_tready) begin
                        wr_en <= 1'b1;
                        addr <= wr_en? addr + 1:addr;
                        axis4_s_tdata_r <= axis4_s_tdata;
                    end else begin
                        wr_en <= 1'b0;
                    end
                end else begin
                    wr_en <= 1'b0;
                    state <= 2'b00;
                    axis4_s_tready_r <= 1'b0;
                    STAT[0] <= 1'b0;
                end


            end
        endcase
    end
end
                    
assign axis4_s_tready = 1'b1; 

reg[15:0] cnt;
 
always@(posedge clk, negedge reset_n) begin
    if(!reset_n) begin
        cnt <= 0;
    end else begin
        if(cmd_clear_cnt) begin
            cnt <= 0;
        end else begin
            cnt <= axis4_s_tvalid&axis4_s_tready? cnt+1: cnt;
        end
    end
end

assign dbg_reg[15:0] = cnt;
assign dbg_reg[27:16] = addr;
assign dbg_reg[29:28] = state;

endmodule
