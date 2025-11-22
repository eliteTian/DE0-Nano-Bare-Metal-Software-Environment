module fpga_dsp(
    input           clk,
    input           rstn,
    
    input [7:0]     axis4_s_tdata,
    output          axis4_s_tready,
    input           axis4_s_tvalid,
    input           axis4_s_tlast,

    
    output[7:0]     axis4_m_tdata,
    input           axis4_m_tready,
    output          axis4_m_tvalid,
    output          axis4_m_tlast,
    
    
    input[3:0]      apb_slave_paddr,
    input           apb_slave_penable,
    output[31:0]    apb_slave_prdata, 
    input[31:0]     apb_slave_pwdata, 
    input           apb_slave_pwrite, 
    input           apb_slave_psel,
    output          apb_slave_pready

);

reg[31:0] reg0, reg1, reg2, reg3;
reg[31:0] rdata;
assign apb_slave_pready = 1'b1;
reg[31:0] apb_slave_prdata_r;

always@(posedge clk, negedge rstn) begin
    if(!rstn) begin
       // rdata <= 0;
        reg0 <=0;
        reg1 <=0;
        reg2 <=0;
        reg3 <=0;
    end else begin
        if(apb_slave_psel&apb_slave_penable&apb_slave_pready) begin
            if(apb_slave_pwrite) begin
               case(apb_slave_paddr[3:2])
                   2'b00:
                       reg0 <= apb_slave_pwdata;
                   2'b01:
                       reg1 <= apb_slave_pwdata;
                   2'b10:
                       reg2 <= apb_slave_pwdata;
                   2'b11:
                       reg3 <= apb_slave_pwdata;
               endcase
            end 
        end
    end
end

always@* begin
    if(apb_slave_psel&apb_slave_penable&apb_slave_pready&~apb_slave_pwrite) begin
        case(apb_slave_paddr[3:2])
            2'b00: apb_slave_prdata_r = reg0;
            2'b01: apb_slave_prdata_r = reg1;
            2'b10: apb_slave_prdata_r = reg2;
            2'b11: apb_slave_prdata_r = reg3;
        endcase
    end else begin
        apb_slave_prdata_r = 32'hFFFFFFFF;
    end     
end
assign apb_slave_prdata = apb_slave_prdata_r;
/*assign apb_slave_prdata = apb_slave_psel&apb_slave_enable&apb_slave_pready&~apb_slave_write  ? (apb_slave_paddr[3:2]==2'b00? reg0:
                                                                                                apb_slave_paddr[3:2]==2'b01? reg1:
                                                                                                apb_slave_paddr[3:2]==2'b10? reg2:
                                                                                                apb_slave_paddr[3:2]==2'b11? reg3: 32'hFFFFFFFF ):32'hFFFFFFFF; */
                          

assign axis4_m_tdata = axis4_s_tdata;
assign axis4_m_tvalid = axis4_s_tvalid;
assign axis4_m_tlast = axis4_s_tlast;
assign axis4_s_tready = axis4_m_tready;


endmodule
