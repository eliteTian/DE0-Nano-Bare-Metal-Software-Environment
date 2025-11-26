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
wire [1:0] sel = reg0[1:0];

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
                          
integer i, j;
//9 taps of singed integer

wire signed [7:0] coeff [0:8];
assign coeff[0] = reg2[7:0];
assign coeff[8] = reg2[7:0];

assign coeff[1] = reg2[15:8];
assign coeff[7] = reg2[15:8];

assign coeff[2] = reg2[23:16];
assign coeff[6] = reg2[23:16];

assign coeff[3] = reg2[31:24];
assign coeff[5] = reg2[31:24];

assign coeff[4] = reg3[7:0];

//    coeff[0] = 8'sd28; 
//    coeff[1] = 8'sd63;
//    coeff[2] = 8'sd95;
//    coeff[3] = 8'sd119;
//    coeff[4] = 8'sd127;
//    coeff[5] = 8'sd119;
//    coeff[6] = 8'sd95;
//    coeff[7] = 8'sd63;
//    coeff[8] = 8'sd28;

    


reg signed [7:0]  delayed_signal[0:8];
reg signed [15:0] prod[0:8];
reg signed [16:0] sum0[0:4];
reg signed [17:0] sum1[0:2];
reg signed [18:0] sum2[0:1];
reg signed [19:0] sum3;


//not ready if downstream not ready
assign axis4_s_tready = axis4_m_tready;

//shift register.
always@(posedge clk, negedge rstn) begin
    if(!rstn) begin
        for(i=0; i<9; i=i+1) begin
            delayed_signal[i] <= 0;
        end
    end else if(axis4_s_tvalid & axis4_m_tready) begin
        delayed_signal[0] <= axis4_s_tdata;
        for(i=1; i<=8; i=i+1) begin
            delayed_signal[i] <= delayed_signal[i-1];
        end
    end
end

//always@(posedge clk, negedge rstn) begin// unnecessary pipeline stages
//always@* begin 
always@(posedge clk, negedge rstn) begin
    if(!rstn) begin
        for(j=0; j<9; j=j+1) begin
            prod[j] <= 0;
        end
    end else if(axis4_s_tvalid & axis4_m_tready) begin
        for(j=0; j<9; j=j+1) begin
            prod[j] <= delayed_signal[j]*coeff[j];
        end
    end
end

always@* begin 
    sum0[0] = prod[0]+prod[1];
    sum0[1] = prod[2]+prod[3];
    sum0[2] = prod[4]+prod[5];
    sum0[3] = prod[6]+prod[7];
    sum0[4] = prod[8];
end

always@(posedge clk, negedge rstn) begin
    if(!rstn) begin
        sum1[0] <= 0;
        sum1[1] <= 0;
        sum1[2] <= 0;
    end else if(axis4_s_tvalid & axis4_m_tready) begin
        sum1[0] <= sum0[1]+sum0[0];
        sum1[1] <= sum0[2]+sum0[3];
        sum1[2] <= sum0[4];
    end
end

always@* begin
    sum2[0] = sum1[0] + sum1[1];
    sum2[1] = sum1[2];
end

always@(posedge clk) begin
    if(!rstn) begin
        sum3 <= 0;

    end else if(axis4_s_tvalid & axis4_m_tready) begin
        sum3 <= sum2[0] + sum2[1];
    end
end
reg axis4_m_tvalid_r;
always@(posedge clk) begin
    if(!rstn) begin
        axis4_m_tvalid_r <= 1'b0;
    end else begin
        axis4_m_tvalid_r <= axis4_s_tvalid;
    end
end




//arithmetic shift, preserving signs. register controlled gain.
wire signed [7:0] clean_sig_msb_shift_0 = sum3>>>12;
wire signed [7:0] clean_sig_msb_shift_1 = sum3>>>11;
wire signed [7:0] clean_sig_msb_shift_2 = sum3>>>10;
wire signed [7:0] clean_sig_msb_shift_3 = sum3>>>9;
wire signed [7:0] clean_sig;

assign clean_sig = sel==2'b00 ? clean_sig_msb_shift_0 :
                   sel==2'b01 ? clean_sig_msb_shift_1 :
                   sel==2'b10 ? clean_sig_msb_shift_2 :
                   sel==2'b11 ? clean_sig_msb_shift_3 : clean_sig_msb_shift_0;

assign axis4_m_tvalid = axis4_m_tvalid_r;
assign axis4_m_tdata = clean_sig;




//assign axis4_m_tdata = axis4_s_tdata;
//assign axis4_m_tvalid = axis4_s_tvalid;
assign axis4_m_tlast = axis4_s_tlast;

endmodule
