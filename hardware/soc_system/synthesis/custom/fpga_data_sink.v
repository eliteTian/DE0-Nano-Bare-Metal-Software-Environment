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
reg     [  31: 0] CTRL, STAT, reg2, reg3;

wire        cmd_valid = CTRL[0]; // 1 means a pending cmd
wire        cmd_type = CTRL[2:1]; //wr = 1, rd = 0; 2'b01=dump; 2'b11 = rsvd 
wire[4:0]   cmd_addr = CTRL[12:8];
wire[7:0]   cmd_data = CTRL[23:16];

wire        pend    = STAT[0];


  assign clk_en = 1;
  always @(posedge clk or negedge  reset_n) begin
        if (reset_n == 0) begin
          CTRL <= 0;
          reg2 <= 0;
          reg3 <= 0;
        end else begin
          if (avs_chipselect && ~avs_write_n) begin
            case( avs_address)
              2'b00:
                CTRL <= avs_writedata;
              2'b10:
                reg2 <= avs_writedata;
              2'b11:
                reg3 <= avs_writedata;
              default: ;
            endcase
          end
          if(clear_cmd) begin // clear cmd valid by HW.
              CTRL <= CTRL & 32'hFFFFFFFE;
          end
      end

    end


  assign  avs_readdata =  avs_address==0? CTRL: 
                     avs_address==2'b01 ? STAT:
                     avs_address==2'b10 ? reg2: reg3;


reg[7:0] mem[0:31];
reg[4:0] addr;
reg[7:0] rdata;
reg      rvalid;


reg        axis4_s_tready_r;
reg        rd_en;
reg        wr_en;
reg        clear_cmd;

always@(posedge clk) begin
    if(wr_en) begin
        mem[addr] <= cmd_data;
        rvalid <= 1'b0;
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
        axis4_s_tready_r <= 1'b1;    
        rd_en <= 1'b0;
        wr_en <= 1'b0;
        clear_cmd <= 1'b0;
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
                    if(cmd_type == 2'b10) begin //dump block ram from addr 0 via axi4 st if
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

            2'b10: begin //to be studied
                axis4_s_tready_r <= 1'b1;
            end
        endcase
    end
end
                    
assign axis4_s_tready = axis4_s_tready_r; 

endmodule
