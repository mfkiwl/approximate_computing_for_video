module processing_fsm(clock, reset, start_signal, size, done, 
							buf_0_write_en, buf_1_write_en, error_buf_write_en,
							buf_0_w_data, buf_1_w_data, error_mem_w_data,
							buf_0_r_data, buf_1_r_data, error_mem_r_data,
							buf_0_addr, buf_1_addr, error_mem_addr, buf_sel);
    input clock;
    input reset;
    input start_signal;
    input size;
    output reg done;
    
    output reg buf_0_write_en;
    output reg buf_1_write_en;
    output reg error_buf_write_en; 

    output reg [7:0] buf_0_w_data;
    output reg [7:0] buf_1_w_data;
    output [15:0] error_mem_w_data; 

    input  [7:0] buf_0_r_data;
    input  [7:0] buf_1_r_data;
    input  [15:0] error_mem_r_data; 
    
    output reg [15:0] buf_0_addr;
    output reg [15:0] buf_1_addr;
    output reg [15:0] error_mem_addr; 
    
    output reg buf_sel;
    
    parameter PIXEL_OFFSET = 1'b1, LINE_OFFSET = 9'd256, LINE_MAX = 8'd239, PIXELS_PER_LINE = 9'd256;
	
	parameter MAX_ADDRESS = 18'd61439;
    
    // State Assignments
    parameter
        WAIT              = 5'b00000,
        LOAD_MID_ERR      = 5'b00001,
        LOAD_BTM_ERR      = 5'b00010,
        CHECK_PACKET_ERR  = 5'b00011,
        NW_PIXEL          = 5'b00100,
        NORTH_PIXEL       = 5'b00101,
        NE_PIXEL          = 5'b00110,
        WEST_PIXEL        = 5'b00111,
        EAST_PIXEL        = 5'b01000,
        SW_PIXEL          = 5'b01001,
        SOUTH_PIXEL       = 5'b01010,
        SE_PIXEL          = 5'b01011,
        MEM_WAIT_1        = 5'b01100,
        MEM_WAIT_2        = 5'b01101,
        MEM_WAIT_3        = 5'b01110,
        SUM               = 5'b01111,
        WRITE_RESULT      = 5'b10000,
        DONE              = 5'b10001;

    // Pixel registers
	reg [7:0] nw_pixel;
    reg [7:0] n_pixel;
    reg [7:0] ne_pixel;
    reg [7:0] w_pixel;
    reg [7:0] e_pixel;
    reg [7:0] sw_pixel;
    reg [7:0] s_pixel;
    reg [7:0] se_pixel;

    // Initialization state flag
    reg init;
    
    // Address for pixel memory access
    reg [15:0] pixel_address;

    // Sum and average of pixel values
    reg [10:0] sum; 
     
    // Buffers to contain packet error flags
    reg [15:0] err_buf_top;
    reg [15:0] err_buf_mid;
    reg [15:0] err_buf_btm;

    // Various counters to track location in the frame
    reg [1:0] channel_count;
    reg [5:0] pixel_count;
    reg [3:0] packet_count;
    reg [7:0] line_count;

    // Configurable maximum values
    wire [5:0] pixel_max;
    wire [3:0] packet_max;

    // Corner case indicator flags
    reg at_top;
    reg at_bottom;
    reg at_left;
    reg at_right;

    // Offsets for when we are at packet edge
    reg error_line_offset_w;
    reg error_line_offset_e;
    
    // Current state
    reg [4:0] state;

    // Previous state logic
    reg from_buf_ne;
    reg from_buf_n;
    reg from_buf_nw;
    reg from_buf_w;
    reg from_buf_e;
    reg from_buf_sw;
    reg from_buf_s;
    reg from_buf_se;
    
    // Logic for max value assignments
	assign pixel_max = 	(size == 2'b10) ? 63 :
						(size == 2'b01) ? 31 :
										  15 ;
	
	assign packet_max =	(size == 2'b10) ? 3 :
						(size == 2'b01) ? 7 :
									     15 ;
	
	// Logic that determines if we are the the top, bottom, left, 
    // or right edges of the frame.
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			at_top = 0;
			at_bottom = 0;
			at_left = 0;
			at_right = 0;
		end
		else begin
			case(state)
                CHECK_PACKET_ERR: begin
                    // If we are at the top of the frame
                    if (line_count == 0) begin
                        at_top <= 1'b1;
                        at_bottom <= 1'b0;
                    end
                    // If we are at the bottom of the frame
                    else if (line_count == 239) begin
                        at_top <= 1'b0;
                        at_bottom <= 1'b1;
                    end
                    // If we are not at the top or bottom
                    else begin 
                        at_top <= 1'b0;
                        at_bottom <= 1'b0;
                    end
                    
                    // If we are at the left side of the frame
                    if (pixel_count == 0 && packet_count == 0) begin
                        at_left <= 1'b1; 
                        at_right <= 1'b0;
                    end 
                    // If we are at the right side of the frame
                    else if (pixel_count == pixel_max && packet_count == packet_max) begin
                        at_left <= 1'b0; 
                        at_right <= 1'b1;
                    end
                    // If we are not at the left or right side of the frame
                    else begin
                        at_left <= 1'b0; 
                        at_right <= 1'b0;
                    end
                end
                // Otherwise follow
                default: begin
                    at_top = at_top;
                    at_bottom = at_bottom;
                    at_left = at_left;
                    at_right = at_right;
                end
			endcase
		end	
	end

	// Check to see if we are at the edge of a packet.
    // If we are at the edge of a packet we will need
    // to check error flags for adjacent packets.
    always @(posedge clock, negedge reset) begin
		if(!reset) begin
			error_line_offset_w <= 1'b0;
			error_line_offset_e <= 1'b0;
		end
		else begin
			case(state)
				CHECK_PACKET_ERR: begin
                    // Check to see if current packet is in error. If it is we 
                    // need to check error bit of adjacent pixels, if it isn't
                    // then we doent need to do any of this.
                    if (err_buf_mid[15-packet_count] == 1) begin
                        // If we are at the leftmost side of a packet
                        if (pixel_count == 0) begin
                            error_line_offset_w <= 1'b1;
                            error_line_offset_e <= 1'b0;
                        end
                        // If we are at the rightmost side of a packet.
                        else if (pixel_count == pixel_max) begin
                            error_line_offset_e <= 1'b1;
                            error_line_offset_w <= 1'b0;
                        end
                        // If we are not at the leftmost or rightmost side.
                        else begin
                            error_line_offset_w <= 1'b0;
                            error_line_offset_e <= 1'b0;
                        end
                    end
                    // If the current packet is not in error.
                    else begin
                        error_line_offset_w <= 1'b0;
                        error_line_offset_e <= 1'b0;
                    end
                end
				default: begin
                    error_line_offset_w <= error_line_offset_w;
                    error_line_offset_e <= error_line_offset_e;
				end
			endcase
		end
	end

	// Logic that determines the address to access for the current pixel
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			pixel_address <= 0;
		end
		else begin
			case(state)
                // If in the wait state just reset to 0
                WAIT: begin
                    pixel_address <= 0;
                end
                // When performing error correction calculate the
                // address for the current pixel
                CHECK_PACKET_ERR: begin
                    pixel_address <= line_count * PIXELS_PER_LINE + 
                                     packet_count * (pixel_max + 1) + 
                                     pixel_count;
                end
                default: begin
                    pixel_address <= pixel_address;
                end
			endcase
		end	
	end
	
    // Logic to read from various pixels.
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			nw_pixel <= 0;
			n_pixel <= 0;
			ne_pixel <= 0;
			w_pixel <= 0;
			e_pixel <= 0;
			sw_pixel <= 0;
			s_pixel <= 0;
			se_pixel <= 0;
		end
		else begin
			case(state)
				WEST_PIXEL:	begin
                    if (from_buf_nw) begin
                        nw_pixel <= buf_1_r_data;
                    end
                    else begin
                        nw_pixel <= buf_0_r_data;
                    end
                end
				EAST_PIXEL:	begin
                    if (from_buf_n) begin
                        n_pixel <= buf_1_r_data;
                    end
                    else begin
                        n_pixel <= buf_0_r_data;
                    end
                end
				SW_PIXEL: begin
                    if (from_buf_ne) begin
                        ne_pixel <= buf_1_r_data;
                    end
                    else begin
                        ne_pixel <= buf_0_r_data;
                    end
                end
				SOUTH_PIXEL: begin
                    if (from_buf_w) begin
                        w_pixel <= buf_1_r_data;
                    end
                    else begin
                        w_pixel <= buf_0_r_data;
                    end
                end
				SE_PIXEL: begin
                    if (from_buf_e) begin
                        e_pixel <= buf_1_r_data;
                    end
                    else begin
                        e_pixel <= buf_0_r_data;
                    end
                end
				MEM_WAIT_1: begin
                    if (from_buf_sw) begin
                        sw_pixel <= buf_1_r_data;
                    end
                    else begin
                        sw_pixel <= buf_0_r_data;
                    end
                end
                MEM_WAIT_2: begin
                    if (from_buf_s) begin
                        s_pixel <= buf_1_r_data;
                    end
                    else begin
                        s_pixel <= buf_0_r_data;
                    end
                end
                MEM_WAIT_3: begin
                    if (from_buf_se) begin
                        se_pixel <= buf_1_r_data;
                    end
                    else begin
                        se_pixel <= buf_0_r_data;
                    end
                end
				default: begin
                    nw_pixel <= nw_pixel;
                    n_pixel <= n_pixel;
                    ne_pixel <= ne_pixel;
                    w_pixel <= w_pixel;
                    e_pixel <= e_pixel;
                    sw_pixel <= sw_pixel;
                    s_pixel <= s_pixel;
                    se_pixel <= se_pixel;
                end
			endcase
		end
	end
	
    // Logic to create the sum and average from the read in pixel values.
    // Takes into consideration corner cases where some pixels may not be
    // available to average (i.e. At the frame edge or corner).
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			sum <= 0;
		end
		else begin
			case(state)
				SUM: begin
                    if (at_top && at_left) begin
                        sum <= (e_pixel + e_pixel + se_pixel + se_pixel + s_pixel + s_pixel + e_pixel + s_pixel) >> 3;
                    end
                    else if (at_top && at_right) begin
                        sum <= (w_pixel + w_pixel + sw_pixel + sw_pixel + s_pixel + s_pixel + w_pixel + s_pixel) >> 3;
                    end
                    else if (at_top) begin
                        sum <= (w_pixel + w_pixel + sw_pixel + s_pixel + s_pixel + se_pixel + e_pixel + e_pixel) >> 3;
                    end
                    else if (at_bottom && at_left) begin
                        sum <= (n_pixel + n_pixel + ne_pixel + ne_pixel + e_pixel + e_pixel + n_pixel + e_pixel) >> 3;
                    end
                    else if (at_bottom && at_right) begin
                        sum <= (w_pixel + w_pixel + nw_pixel + nw_pixel + n_pixel + n_pixel + w_pixel + n_pixel) >> 3;
                    end
                    else if (at_left) begin
                        sum <= (n_pixel + n_pixel + ne_pixel + e_pixel + e_pixel + se_pixel + s_pixel + s_pixel) >> 3;
                    end
                    else if (at_right) begin
                        sum <= (n_pixel + n_pixel + nw_pixel + w_pixel + w_pixel + sw_pixel + s_pixel + s_pixel) >> 3;
                    end
                    else if (at_bottom) begin
                        sum <= (w_pixel + nw_pixel + n_pixel + ne_pixel + e_pixel + w_pixel + n_pixel + e_pixel) >> 3; 
                    end
                    else begin
                        sum <= (nw_pixel + n_pixel + ne_pixel + w_pixel + e_pixel + sw_pixel + s_pixel + se_pixel) >> 3; 
                    end
                end
				default: begin
                    sum <= sum;
                end
			endcase
		end
	end
	
    // Logic to keep track of the pixel, packet, and line counters. 
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			line_count <= 0;
			packet_count <= 0;
			pixel_count <= 0;
		end
		else begin
			case(state)
				WAIT: begin
                    // Reset counters
                    pixel_count <= 0;
                    packet_count <= 0;
                    line_count <= 0;
                end
				CHECK_PACKET_ERR: begin
                    if(err_buf_mid[15-packet_count] == 1) begin
                        pixel_count <= pixel_count;
                        packet_count <= packet_count;
                        line_count <= line_count;
                    end
                    else begin
                        if (packet_count == packet_max) begin
                            if (line_count == LINE_MAX) begin
                                pixel_count <= 0;
                                packet_count <= 0;
                                line_count <= 0;
                            end
                            else begin
                                pixel_count <= 0;
                                packet_count <= 0;
                                line_count <= line_count + 1;
                            end
                        end
                        else begin
                            pixel_count <= 0;
                            packet_count <= packet_count + 1;
                            line_count <= line_count;
                        end
                    end
                end
				WRITE_RESULT: begin
                        if (pixel_count == pixel_max) begin
                            pixel_count <= 0;
                            if (packet_count == packet_max) begin
                                packet_count <= 0;
                                if(line_count == LINE_MAX) begin
                                    line_count <= 0;
                                end
                                else begin
                                    line_count <= line_count + 1;
                                end
                            end
                            else begin
                                packet_count <= packet_count + 1;
                                line_count <= line_count;
                            end
                        end
                        else begin
                            pixel_count <= pixel_count + 1;
                            packet_count <= packet_count;
                            line_count <= line_count;
                        end
                    end
				DONE: begin
                    line_count <= 0;
                    packet_count <= 0;
                    pixel_count <= 0;
                end
				default: begin
                    line_count <= line_count;
                    packet_count <= packet_count;
                    pixel_count <= pixel_count;
                end
			endcase
		end
	end
	
    // Logic to write result to relevent pixel buffers.
	always @(posedge clock, negedge reset)
	begin
		if(!reset) begin
			buf_0_w_data <= 0;
			buf_0_write_en <= 0;
			buf_1_w_data <= 0;
			buf_1_write_en <= 0;
		end
		else begin
			case(state)
				WRITE_RESULT: begin
                    // Write the result of sum into the relevant buffers.
                    if (buf_sel) begin
                        buf_1_w_data <= sum;
                        buf_1_write_en <= 1;
                    end
                    else begin
                        buf_0_w_data <= sum;
                        buf_0_write_en <= 1;
                    end
                end
				default: begin
                    buf_0_w_data <= buf_0_w_data;
                    buf_0_write_en <= 0;
                    buf_1_w_data <= buf_1_w_data;
                    buf_1_write_en <= 0;
                end
			endcase
		end
	end
	
	// Logic to set done signals high, indicating we are
    // done processing the frame.
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			buf_sel <= 0;
			done <= 0;
		end
		else begin
			case(state)
				DONE: begin
                    buf_sel <= ~buf_sel;
                    done <= 1;
                end
				default: begin
                    buf_sel <= buf_sel;
                    done <= 0;
                end
			endcase
		end
	end
 
    // Logic that determines the addresses of the buffers we are addressing
    // for pixel value reads.
	always @(posedge clock, negedge reset) begin
		if(!reset) begin
			buf_0_addr <= 16'b0;
			buf_1_addr <= 16'b0;
			from_buf_nw <= 1'b0;
			from_buf_n <= 1'b0;
			from_buf_ne <= 1'b0;
			from_buf_w <= 1'b0;
			from_buf_e <= 1'b0;
			from_buf_sw <= 1'b0;
			from_buf_s <= 1'b0;
			from_buf_se <= 1'b0;
		end
		else begin
			case(state)
				NW_PIXEL: begin
                    if (buf_sel == 0) begin
                        if (err_buf_top[15-(packet_count - error_line_offset_w)]) begin
                            buf_0_addr <= buf_0_addr;
                            buf_1_addr <= pixel_address - PIXEL_OFFSET - LINE_OFFSET;
                            from_buf_nw <= 1'b1;
                        end 
                        else begin
                            buf_0_addr <= pixel_address - PIXEL_OFFSET - LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_nw <= 1'b0;
                        end
                    end
                    else begin
                        if (err_buf_top[15-(packet_count - error_line_offset_w)]) begin
                            buf_0_addr <= pixel_address - PIXEL_OFFSET - LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_nw <= 1'b0;
                        end 
                        else begin
                            buf_1_addr <= pixel_address - PIXEL_OFFSET - LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_nw <= 1'b1;
                        end
                    end
                end
				NORTH_PIXEL: begin
                    if (buf_sel == 0) begin
                        if (err_buf_top[15-packet_count]) begin
                            buf_1_addr <= pixel_address - LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_n <= 1'b1;
                        end 
                        else begin
                            buf_0_addr <= pixel_address - LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_n <= 1'b0;
                        end
                    end
                    else begin
                        if (err_buf_top[15-packet_count]) begin
                            buf_0_addr <= pixel_address - LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_n <= 1'b0;
                        end 
                        else begin
                            buf_1_addr <= pixel_address - LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_n <= 1'b1;
                        end
                    end
                end
				NE_PIXEL: begin
                    if (buf_sel == 0) begin
                        if (err_buf_top[15-(packet_count - error_line_offset_w)]) begin
                            buf_1_addr <= pixel_address + PIXEL_OFFSET - LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_ne <= 1'b1;
                        end 
                        else begin
                            buf_0_addr <= pixel_address + PIXEL_OFFSET - LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_ne <= 1'b0;
                        end
                    end
                    else begin
                        if (err_buf_top[15-(packet_count - error_line_offset_w)]) begin
                            buf_0_addr <= pixel_address + PIXEL_OFFSET - LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_ne <= 1'b0;
                        end 
                        else begin
                            buf_1_addr <= pixel_address + PIXEL_OFFSET - LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_ne <= 1'b1;
                        end
                    end
                end
				WEST_PIXEL: begin
                    if (buf_sel == 0) begin
                        buf_1_addr <= pixel_address - PIXEL_OFFSET;
                        buf_0_addr <= buf_0_addr;
                        from_buf_w <= 1'b1;
                    end
                    else begin
                        buf_0_addr <= pixel_address - PIXEL_OFFSET;
                        buf_1_addr <= buf_1_addr;
                        from_buf_w <= 1'b0;
                    end
                end
				EAST_PIXEL: begin
                    if (buf_sel == 0) begin
                        buf_1_addr <= pixel_address + PIXEL_OFFSET;
                        buf_0_addr <= buf_1_addr;
                        from_buf_e <= 1'b1;
                    end
                    else begin
                        buf_0_addr <= pixel_address + PIXEL_OFFSET;
                        buf_1_addr <= buf_1_addr;
                        from_buf_e <= 1'b0;
                    end
                end
				SW_PIXEL: begin
                    if (buf_sel == 0) begin
                        if (err_buf_btm[15 - (packet_count - error_line_offset_w)]) begin
                            buf_1_addr <= pixel_address - PIXEL_OFFSET + LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_sw <= 1'b1;
                        end 
                        else begin
                            buf_0_addr <= pixel_address - PIXEL_OFFSET + LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_sw <= 1'b0;
                        end
                    end
                    else begin
                        if (err_buf_btm[15- (packet_count - error_line_offset_w)]) begin
                            buf_0_addr <= pixel_address - PIXEL_OFFSET + LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_sw <= 1'b0;
                        end 
                        else begin
                            buf_1_addr <= pixel_address - PIXEL_OFFSET + LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_sw <= 1'b1;
                        end
                    end
                end
				SOUTH_PIXEL: begin
                    if (buf_sel == 0) begin
                        if (err_buf_btm[15-packet_count]) begin
                            buf_1_addr <= pixel_address + LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_s <= 1'b1;
                        end 
                        else begin
                            buf_0_addr <= pixel_address + LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_s <= 1'b0;
                        end
                    end
                    else begin
                        if (err_buf_btm[15-packet_count]) begin
                            buf_0_addr <= pixel_address + LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_s <= 1'b0;
                        end 
                        else begin
                            buf_1_addr <= pixel_address + LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_s <= 1'b1;
                        end
                    end
                end
				SE_PIXEL: begin
                    if (buf_sel == 0) begin
                        if (err_buf_btm[15-(packet_count + error_line_offset_e)]) begin
                            buf_1_addr <= pixel_address + PIXEL_OFFSET + LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_se <= 1'b1;
                        end 
                        else begin
                            buf_0_addr <= pixel_address + PIXEL_OFFSET + LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_se <= 1'b0;
                        end
                    end
                    else begin
                        if (err_buf_btm[15-(packet_count + error_line_offset_e)]) begin
                            buf_0_addr <= pixel_address + PIXEL_OFFSET + LINE_OFFSET;
                            buf_1_addr <= buf_1_addr;
                            from_buf_se <= 1'b0;
                        end 
                        else begin
                            buf_1_addr <= pixel_address + PIXEL_OFFSET + LINE_OFFSET;
                            buf_0_addr <= buf_0_addr;
                            from_buf_se <= 1'b1;
                        end
                    end
                end
				default: begin
                    buf_0_addr <= buf_0_addr;
                    buf_1_addr <= buf_1_addr;
                    from_buf_nw <= from_buf_nw;
                    from_buf_n <= from_buf_n;
                    from_buf_ne <= from_buf_ne;
                    from_buf_w <= from_buf_w;
                    from_buf_e <= from_buf_e;
                    from_buf_sw <= from_buf_sw;
                    from_buf_s <= from_buf_s;
                    from_buf_se <= from_buf_se;
                end
			endcase
		end
	end
    
    // Logic that loads the error buffers. 
    always @(posedge clock, negedge reset)
    begin
        if(!reset) begin
            init <= 1'b1;
            err_buf_top <= 16'b0;
            err_buf_mid <= 16'b0;
            err_buf_btm <= 16'b0;
            error_mem_addr <= 16'b0;
        end
        else begin
            case(state)
                WAIT: begin
                    init <= 1'b1;
                    err_buf_top <= 16'b0;
                    err_buf_mid <= error_mem_r_data;
                    err_buf_btm <= 16'b0;
                    if (start_signal) begin
                        error_mem_addr <= 16'b1;
                    end
                    else begin
                        error_mem_addr <= 16'b0;
                    end
                end

                LOAD_MID_ERR: begin
                    init <= 1'b1;
                    err_buf_top <= err_buf_top;
                    err_buf_mid <= err_buf_mid;
                    err_buf_btm <= err_buf_btm;
                    error_mem_addr <= 16'b01;
                end

                LOAD_BTM_ERR: begin
                    init <= 1'b0;
                    if (init == 1'b1) begin
                        err_buf_top <= err_buf_top;
                        err_buf_mid <= err_buf_mid;
                        err_buf_btm <= err_buf_btm;
                        error_mem_addr <= error_mem_addr;
                    end
                    else begin
                        err_buf_top <= err_buf_mid;
                        err_buf_mid <= err_buf_btm;
                        err_buf_btm <= err_buf_btm;
                        error_mem_addr <= error_mem_addr;
                    end
                end

                CHECK_PACKET_ERR: begin
                    err_buf_top <= err_buf_top;
                    err_buf_mid <= err_buf_mid;
                    err_buf_btm <= error_mem_r_data;
                    error_mem_addr <= line_count + 1'b1;
                end

                default: begin
                    err_buf_top <= err_buf_top;
                    err_buf_mid <= err_buf_mid;
                    err_buf_btm <= err_buf_btm;
                    error_mem_addr <= error_mem_addr;
                end
            endcase
        end
    end

    // Next state logic. See documentation for state transitions and 
    // descriptions.
	always @ (posedge clock, negedge reset) begin
        if(!reset) begin
            state <= WAIT;
        end
        else begin
            case(state)
                WAIT: begin
                    if(start_signal) begin
                        state <= LOAD_MID_ERR;
                    end
                    else begin
                        state <= WAIT;
                    end
                end

                LOAD_MID_ERR: begin
                    state <= LOAD_BTM_ERR;
                end

                LOAD_BTM_ERR: begin
                    state <= CHECK_PACKET_ERR;
                end
                
                CHECK_PACKET_ERR: begin
                    // If error bit is high we need to do error correction.
                    if (err_buf_mid[15-packet_count] == 1'b1) begin
                        state <= NW_PIXEL;
                    end
                    // No errors. Proceed to next packet.
                    else begin
                        if (packet_count == packet_max) begin
                            if (line_count == LINE_MAX) begin
                                state <= DONE;
                            end
                            else begin
                                state <= LOAD_BTM_ERR;
                            end
                        end
                        else begin
                            state <= CHECK_PACKET_ERR;
                        end
                    end
                end

                NW_PIXEL: begin
                    state <= NORTH_PIXEL;
                end
                
                NORTH_PIXEL: begin
                    state <= NE_PIXEL;
                end

                NE_PIXEL: begin
                    state <= WEST_PIXEL;
                end


                WEST_PIXEL: begin
                    state <= EAST_PIXEL;
                end 

                EAST_PIXEL: begin
                    state <= SW_PIXEL;
                end
                
                SW_PIXEL: begin
                    state <= SOUTH_PIXEL;
                end
                
                SOUTH_PIXEL: begin
                    state <= SE_PIXEL;
                end
                
                SE_PIXEL: begin
                    state <= MEM_WAIT_1;
                end
                     
                MEM_WAIT_1: begin
                    state <= MEM_WAIT_2;
                end

                MEM_WAIT_2: begin
                    state <= MEM_WAIT_3;
                end
                MEM_WAIT_3: begin
                    state <= SUM;
                end
                SUM: begin
                    state <= WRITE_RESULT;
                end
                     
                WRITE_RESULT: begin
                    if (pixel_count == pixel_max) begin
                        if (packet_count == packet_max) begin
                            if(line_count == LINE_MAX) begin
                                state <= DONE;
                            end
                            else begin
                                state <= LOAD_BTM_ERR;
                            end
                        end
                        else begin
                            state <= CHECK_PACKET_ERR;
                        end
                    end
                    else begin
                        state <= CHECK_PACKET_ERR;
                    end
                end

                DONE: begin
                    state <= WAIT;
                end 
            endcase
        end
    end

endmodule
