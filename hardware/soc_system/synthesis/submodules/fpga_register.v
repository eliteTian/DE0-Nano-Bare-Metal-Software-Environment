module fpga_registers (
                            // inputs:
                             avs_address,
                             avs_chipselect,
                             clk,
                             reset_n,
                             avs_write_n,
                             avs_writedata,

                            // outputs:
                             avs_readdata
                          )
;

  output  [ 31: 0] avs_readdata;
  input   [  1: 0] avs_address;
  input            avs_chipselect;
  input            clk;
  input            reset_n;
  input            avs_write_n;
  input   [ 31: 0] avs_writedata;


wire              clk_en;
reg     [  31: 0] reg0, reg1, reg2, reg3;

wire    [  31: 0]  out_port;
wire    [  31: 0]  read_mux_out;
wire    [  31: 0]  avs_readdata;

  assign clk_en = 1;
  always @(posedge clk or negedge  reset_n) begin
        if (reset_n == 0) begin
          reg0<= 0;
          reg1 <= 0;
          reg2 <= 0;
          reg3 <= 0;
        end else begin
          if (avs_chipselect && ~avs_write_n) begin
            case( avs_address)
              2'b00:
                reg0 <= avs_writedata;
              2'b01:
                reg1 <= avs_writedata;
              2'b10:
                reg2 <= avs_writedata;
              2'b11:
                reg3 <= avs_writedata;
            endcase
          end
      end

    end


  assign  avs_readdata =  avs_address==0? reg0: 
                     avs_address==2'b01 ? reg1:
                     avs_address==2'b10 ? reg2: reg3;
                

endmodule
