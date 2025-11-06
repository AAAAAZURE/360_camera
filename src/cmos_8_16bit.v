module cmos_8_16bit #(
    parameter RESET_ON_BLANK = 1'b0, // 1: 每行空白时强制相位复位
    parameter SWAP_BYTES     = 1'b0  // 1: 输出为 {prev, curr}，0: {curr, prev}
)(
	input              rst,
	input              pclk,
	input [7:0]        pdata_i,
	input              de_i,
	output reg[15:0]   pdata_o,
	output reg	       hblank,
	output reg         de_o
);

	reg de_d1;//,de_d2;
	reg x_cnt;
	
	always@(posedge pclk)begin 
		de_d1 <= de_i;
		//de_d2 <= de_d1;
	 	hblank <= de_d1;
	end 

	always@(posedge pclk)
	begin
		if (RESET_ON_BLANK) begin
			if (!de_i)
				x_cnt <= 1'b0;           // 行空白阶段强制复位，保证每行起始字节一致
			else
				x_cnt <= ~x_cnt;         // 有效数据内交替翻转
		end else begin
			if (de_i & !de_d1)       // 行开始：从第一个字节开始
				x_cnt <= 1'b0;           // 0表示第一个字节（高字节），1表示第二个字节（低字节）
			else if (de_i)           // 只有在数据有效时才切换
				x_cnt <= ~x_cnt ;
			// 如果de_i无效，保持x_cnt不变
		end
	end

	always@(posedge pclk or posedge rst)
	begin
		if(rst)
			de_o <= 1'b0;
		else if(de_i && x_cnt)  // 只在完整16位数据准备好时输出有效信号
			de_o <= 1'b1;
		else
			de_o <= 1'b0;
	end

	reg[7:0] pdata_i_d0;
	always@(posedge pclk)	//Latch
	begin
		pdata_i_d0 <= pdata_i;
	end

	always@(posedge pclk or posedge rst)
	begin
		if(rst)
			pdata_o <= 16'd0;
		else if(de_i && x_cnt) begin  // 第二个字节时组装16位数据
		pdata_o <= SWAP_BYTES ? {pdata_i_d0, pdata_i} : {pdata_i, pdata_i_d0};
	end else if(!de_i) begin // 数据无效时清零输出，确保边界清晰
		pdata_o <= 16'd0;
	end
		// 其他情况保持数据稳定
	end

endmodule