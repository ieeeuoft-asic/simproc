module simproc_system #(
    parameter CLK_BITS = 10
) (
    input  logic                        clk,
    input  logic                        rst,

    input  logic    [CLK_BITS - 1 : 0]  clk_per_bit,
    input  logic                        uart_rx,

    output logic                        uart_tx,

    // Debug outputs
    output logic                        halt,
    output logic                        done
    // output logic    [7:0]               cmd_byte,
    // output logic    [7:0]               addr_byte,
    // output logic    [7:0]               data_byte,
    // output logic    [7:0]               tx_reg
);

    logic [7:0] cmd_byte;
    logic [7:0] addr_byte;
    logic [7:0] data_byte;
    logic [7:0] tx_reg;

    // Internal Signals
    // DP Ram
    logic [7:0] mem_din;
    logic [7:0] mem_dout;
    logic [7:0] mem_addr;
    logic       mem_we;

    // External Communication
    logic [7:0] din_b;
    logic [7:0] addr_b;
    logic       we_b;
    logic [7:0] dout_b;

    // UART
    logic [7:0] rx_data;
    logic       rx_done;
    logic [7:0] tx_data;
    logic       tx_en;
    logic       tx_busy;
    logic       tx_done;

    // SimProc
    logic [7:0] pc_set_val;
    logic       pc_set_wr;
    logic       run;
    logic [7:0] pc_val;

    // Module instantiations
    // DP Ram
    dp_ram #(
        .DATA_WIDTH(8),
        .MEM_DEPTH(64)
    ) RAM1 (
        .clk(clk),

        // Port A
        .din_a(mem_din),
        .addr_a(mem_addr),
        .we_a(mem_we),
        .dout_a(mem_dout),

        // Port B
        .din_b(din_b),
        .addr_b(addr_b),
        .we_b(we_b),
        .dout_b(dout_b)
    );

    // SimProc
    simproc CPU1 (
        .clk(clk),
        .rst(rst),

        // Memory interface
        .mem_din(mem_din),
        .mem_addr(mem_addr),
        .mem_we(mem_we),
        .mem_dout(mem_dout),

        // Debug interface
        .pc_set_val(pc_set_val),
        .pc_set_wr(pc_set_wr),
        .run(run),
        .pc_val(pc_val),
        .halt(halt),
        .done(done)
    );

    // UART
    UART_wrapper #(
        .CLK_BITS(CLK_BITS),
        .DATA_WIDTH(8),
        .PARITY_BITS(0),
        .STOP_BITS(1)
    ) UART1 (
        .clk(clk),
        .rst(rst),

        // Inputs
        .clk_per_bit(clk_per_bit),
        .TX_dataIn(tx_data),
        .TX_en(tx_en),

        .RX_dataIn(uart_rx),
        
        // Outputs
        .TX_out(uart_tx),

        .TX_done(tx_done),
        .TX_busy(tx_busy),
        .RX_dataOut(rx_data),
        .RX_done(rx_done),
        .RX_parityError()
    );

    // UART Command FSM
    typedef enum logic [2:0] { 
        UART_IDLE,
        WAIT_CMD,
        WAIT_ADDR,
        WAIT_DATA,
        EXEC,
        SEND_RESP,
        SEND_WAIT
    } uart_state_t;

    typedef enum logic [7:0] {
        CMD_READ    = 8'hA3,
        CMD_WRITE   = 8'h5C,
        CMD_PING    = 8'hF0,
        CMD_RUN     = 8'h3F,
        CMD_HALT    = 8'hC7,
        CMD_STEP    = 8'h6D,
        CMD_SET_PC  = 8'h99,
        CMD_GET_PC  = 8'hB2 
    } cmd_t;

    uart_state_t uart_curr_state, uart_next_state;

    // Handshake for TX
    logic       tx_start;
    logic [7:0] tx_buffer;
    


    // Run Logic
    logic       run_reg;
    logic       step_pulse;

    // State logic (state table)
    always_comb begin
        // Default next state
        uart_next_state = uart_curr_state;

        // Default TX
        tx_data         = 8'h00;
        tx_en           = 0;
        tx_buffer       = 8'b0;

        // Default SimProc controls
        run             = step_pulse | run_reg;
        pc_set_val      = 8'b0;
        pc_set_wr       = 0;

        // Default RAM Port B 
        din_b           = 8'b0;
        addr_b          = 8'b0;
        we_b            = 0;

        case(uart_curr_state)
            UART_IDLE: begin
                uart_next_state = WAIT_CMD;
            end

            WAIT_CMD: begin
                if (rx_done) begin
                    uart_next_state = WAIT_ADDR;
                end
            end

            WAIT_ADDR: begin
                if (rx_done) begin
                    uart_next_state = WAIT_DATA;
                end
            end

            WAIT_DATA: begin
                if (rx_done) begin
                    uart_next_state = EXEC;
                end
            end 

            EXEC: begin
                // Default in case I miss a branch
                uart_next_state = WAIT_CMD;
                case (cmd_byte)
                    CMD_READ: begin
                        addr_b          = addr_byte;
                        tx_buffer       = dout_b;

                        uart_next_state = SEND_RESP;
                    end

                    CMD_WRITE: begin
                        addr_b          = addr_byte;
                        din_b           = data_byte;
                        we_b            = 1;

                        uart_next_state = WAIT_CMD;
                    end

                    CMD_PING: begin
                        tx_buffer       = 8'hAA;

                        uart_next_state = SEND_RESP;
                    end

                    CMD_RUN: begin
                        // Handled in synchronous (ff) block
                        uart_next_state = WAIT_CMD;
                    end

                    CMD_HALT: begin
                        // Handled in synchronous (ff) block
                        uart_next_state = WAIT_CMD;
                    end

                    CMD_STEP: begin
                        // Handled in synchronous (ff) block
                        uart_next_state = WAIT_CMD;
                    end

                    CMD_SET_PC: begin
                        pc_set_val      = addr_byte;
                        pc_set_wr       = 1; 

                        uart_next_state = WAIT_CMD;
                    end

                    CMD_GET_PC: begin
                        tx_buffer       = pc_val;

                        uart_next_state = SEND_RESP;
                    end

                    default: begin
                        uart_next_state = WAIT_CMD;
                    end
                endcase
            end

            SEND_RESP: begin
                tx_data = tx_reg;
                tx_en   = 1;

                uart_next_state = SEND_WAIT;
            end

            SEND_WAIT: begin
                tx_data = tx_reg;
                if (tx_done) begin
                    uart_next_state = WAIT_CMD;
                end
                else begin
                    uart_next_state = SEND_WAIT;
                end
            end

            default: uart_next_state = UART_IDLE;
        endcase
    end

    // Next state registers
    always_ff @(posedge clk) begin
        if (rst) begin
            uart_curr_state <= UART_IDLE;
            cmd_byte        <= 8'b0;
            addr_byte       <= 8'b0;
            data_byte       <= 8'b0;
            tx_reg          <= 8'b0;

            // Run logic
            run_reg         <= 0;
            step_pulse      <= 0;
        end
        else begin
            uart_curr_state <= uart_next_state;

            // Get RX bytes
            if (rx_done) begin
                case (uart_curr_state)
                    WAIT_CMD:   cmd_byte    <= rx_data;
                    WAIT_ADDR:  addr_byte   <= rx_data;
                    WAIT_DATA:  data_byte   <= rx_data;
                endcase
            end

            // Latch tx_buffer when entering SEND_RESP
            if (uart_curr_state == EXEC && uart_next_state == SEND_RESP) begin
                tx_reg <= tx_buffer;
            end

            // CMD_RUN: latch run to 1
            if (uart_curr_state == EXEC && cmd_byte == CMD_RUN) begin
                run_reg <= 1;
            end
            // CMD_HALT: set run to 0
            else if (uart_curr_state == EXEC && cmd_byte == CMD_HALT) begin
                run_reg <= 0;
            end
    
            // CMD_STEP: pulse run once
            if (uart_curr_state == EXEC && cmd_byte == CMD_STEP) begin
                step_pulse <= 1;
            end
            else begin
                step_pulse <= 0;
            end
        end
    end
endmodule

// SimProc Modules

module simproc (
    input  logic            clk,
    input  logic            rst,

    // Memory interface
    output logic    [7:0]   mem_din,
    output logic    [7:0]   mem_addr,
    output logic            mem_we,
    input  logic    [7:0]   mem_dout,

    // Debug interface
    input  logic    [7:0]   pc_set_val,
    input  logic            pc_set_wr,
    input  logic            run,
    output logic    [7:0]   pc_val,
    output logic            halt,
    output logic            done
);

    // ALU wires
    logic [2:0] alu_op;
    logic [7:0] alu_in_a;
    logic [7:0] alu_in_b;
    logic       alu_n, alu_z;
    logic [7:0] alu_out;

    // Register File wires
    logic       rf_write;
    logic [1:0] rf_reg_a_in;
    logic [1:0] rf_reg_b_in;
    logic [1:0] rf_reg_w_in;
    logic [7:0] rf_data_w_in;
    logic [7:0] rf_data_a_out;
    logic [7:0] rf_data_b_out;

    // Program Counter wires
    logic [7:0] pc_in;
    logic       pc_wr;
    logic [7:0] pc_out;

    // Module instantiation
    ALU ALU1 (
        // Inputs
        .ALUop(alu_op),
        .A(alu_in_a),
        .B(alu_in_b),
        // Outputs
        .N(alu_n),
        .Z(alu_z),
        .ALUout(alu_out)
    );

    register_file RF1 (
        .clk(clk),
        .rst(rst),
        // Inputs
        .RFWrite(rf_write),
        .regA(rf_reg_a_in),
        .regB(rf_reg_b_in),
        .regW(rf_reg_w_in),
        .dataW(rf_data_w_in),
        // Outputs
        .dataA(rf_data_a_out),
        .dataB(rf_data_b_out)
    );

    program_counter PC1 (
        .clk(clk),
        .rst(rst),
        // Inputs
        .pc_in(pc_in),
        .pc_wr(pc_wr),
        // Outputs
        .pc_out(pc_out)
    );

    // Data Path wires
    logic [7:0] instr_reg_in;
    logic [7:0] instr_reg_out;

    logic [7:0] mdr_in;
    logic [7:0] mdr_out;

    logic [7:0] reg_a_in;
    logic [7:0] reg_a_out;

    logic [7:0] reg_b_in;
    logic [7:0] reg_b_out;

    logic [7:0] alu_reg_in;
    logic [7:0] alu_reg_out;

    logic       n_flag_in, z_flag_in;
    logic       n_flag_out, z_flag_out;

    // Load wires
    logic       ir_load;
    logic       mdr_load;
    logic       ab_load;
    logic       alu_out_load;
    logic       flag_wr;

    // Assign wires
    always_comb begin
        n_flag_in       = alu_n;
        z_flag_in       = alu_z;
        alu_reg_in      = alu_out;
        reg_a_in        = rf_data_a_out;
        reg_b_in        = rf_data_b_out;
        mdr_in          = mem_dout;
        instr_reg_in    = mem_dout;
        pc_val          = pc_out;
    end

    // FSM states
    typedef enum logic[2:0] { 
        IDLE, CYCLE_1, CYCLE_2, CYCLE_3, CYCLE_4, CYCLE_5
    } state_t;

    // ALU operation codes
    typedef enum logic [2:0] {
        ALU_ADD  = 3'b000,
        ALU_SUB  = 3'b001,
        ALU_OR   = 3'b010,
        ALU_NAND = 3'b011,
        ALU_SHL  = 3'b100,
        ALU_SHR  = 3'b101
    } aluop_t;
    
    // Instruction opcodes
    typedef enum logic [3:0] {
        OP_ADD   = 4'b0100,
        OP_SUB   = 4'b0110,
        OP_NAND  = 4'b1000,
        OP_ORI   = 4'b0111,  // Only check 3 LSB
        OP_LOAD  = 4'b0000,
        OP_STORE = 4'b0010,
        OP_BNZ   = 4'b1001,
        OP_BPZ   = 4'b0101,
        OP_BZ    = 4'b1010,
        OP_SHIFT = 4'b0011,  // Only check 3 LSB
        OP_JUMP  = 4'b0001   // Custom instruction
    } opcode_t;

    // Shift instructions
    typedef enum logic {
        SHIFT_R  = 1'b0,
        SHIFT_L  = 1'b1
    } shift_t;

    state_t curr_state, next_state;

    always_comb begin
        // Default values
        next_state      = curr_state;
        done            = 0;
        halt            = 0;

        pc_in           = 8'b0;
        pc_wr           = 0;

        alu_in_a        = 8'b0;
        alu_in_b        = 8'b0;
        mem_addr        = 8'b0;
        mem_we          = 0;
        mem_din         = 8'b0;
        ir_load         = 0;
        rf_write        = 0;
        rf_data_w_in    = 8'b0;
        alu_op          = ALU_ADD;
        alu_out_load    = 0;
        flag_wr         = 0;

        rf_reg_a_in     = 2'b0;
        rf_reg_b_in     = 2'b0;
        ab_load         = 0;

        mdr_load        = 0;

        rf_reg_w_in     = 2'b0;

        // State case statement (state table)
        case (curr_state)
            IDLE: begin
                halt = 1;
                pc_in = pc_set_val;
                pc_wr = pc_set_wr;

                if (run) begin
                    next_state = CYCLE_1;
                end
                else begin
                    next_state = IDLE;
                end
            end

            CYCLE_1: begin
                // IR <- mem[PC]
                mem_addr        = pc_out;
                ir_load         = 1;

                // PC <- PC + 1
                alu_in_a        = pc_out;
                alu_in_b        = 8'h01;
                alu_op          = ALU_ADD;
                pc_in           = alu_out;
                pc_wr           = 1;

                next_state      = CYCLE_2;
            end

            CYCLE_2: begin
                // Preload regA and regB data in registers
                rf_reg_a_in     = instr_reg_out[7:6];
                rf_reg_b_in     = instr_reg_out[5:4];
                ab_load         = 1;

                next_state      = CYCLE_3;
            end

            CYCLE_3: begin
                next_state = IDLE;
                if (instr_reg_out[3:0] == OP_ADD || instr_reg_out[3:0] == OP_SUB ||
                    instr_reg_out[3:0] == OP_NAND) begin
                    // Select operation
                    case (instr_reg_out[3:0])
                        OP_ADD:  alu_op = ALU_ADD;
                        OP_SUB:  alu_op = ALU_SUB;
                        OP_NAND: alu_op = ALU_NAND;
                        default: alu_op = ALU_ADD;
                    endcase
                    // select registers A and B
                    alu_in_a        = reg_a_out;
                    alu_in_b        = reg_b_out;
                    // load result in register and set flags
                    alu_out_load    = 1;
                    flag_wr         = 1;

                    next_state = CYCLE_4;
                end

                else if (instr_reg_out[2:0] == OP_SHIFT[2:0]) begin
                    // Select shift operation
                    if (instr_reg_out[5] == 1)
                        alu_op = ALU_SHL;
                    else
                        alu_op = ALU_SHR;
                    // select registers A and B
                    alu_in_a        = reg_a_out;
                    alu_in_b        = {6'b0, instr_reg_out[4:3]};
                    // load result in register and set flags
                    alu_out_load    = 1;
                    flag_wr         = 1;

                    next_state      = CYCLE_4;
                end

                else if (instr_reg_out[3:0] == OP_LOAD) begin
                    // MDR <- mem[rB]
                    mem_addr        = reg_b_out;
                    mdr_load        = 1;

                    next_state      = CYCLE_4;
                end

                else if (instr_reg_out[3:0] == OP_STORE) begin
                    // mem[rB] = rA
                    mem_addr        = reg_b_out;
                    mem_din         = reg_a_out;
                    mem_we          = 1;

                    // Check for run
                    done            = 1;
                    if (run)
                        next_state = CYCLE_1;
                    else
                        next_state = IDLE;

                end

                else if (instr_reg_out[3:0] == OP_BNZ || instr_reg_out[3:0] == OP_BPZ ||
                    instr_reg_out[3:0] == OP_BZ || instr_reg_out[3:0] == OP_JUMP) begin
                    // Select operation
                    case (instr_reg_out[3:0])
                        OP_BNZ: if (!z_flag_out) pc_wr = 1;
                        OP_BPZ: if (!n_flag_out) pc_wr = 1;
                        OP_BZ:  if (z_flag_out)  pc_wr = 1;
                        OP_JUMP:                 pc_wr = 1;
                    endcase
                    // A <- PC, B <- SE(instr[7:4])
                    alu_in_a        = pc_out;
                    alu_in_b        = {{4{instr_reg_out[7]}}, instr_reg_out[7:4]};
                    alu_op          = ALU_ADD;
                    pc_in           = alu_out;

                    // Check for run
                    done            = 1;
                    if (run)
                        next_state = CYCLE_1;
                    else
                        next_state = IDLE;
                end

                else if (instr_reg_out[2:0] == OP_ORI[2:0]) begin
                    // rA <- RF[0]
                    rf_reg_a_in     = 2'b0; // change this to 1, to match SimProc
                    ab_load         = 1;

                    next_state      = CYCLE_4;
                end
            end

            CYCLE_4: begin
                next_state = IDLE;
                if (instr_reg_out[3:0] == OP_ADD || instr_reg_out[3:0] == OP_SUB ||
                    instr_reg_out[3:0] == OP_NAND || instr_reg_out[2:0] == OP_SHIFT[2:0]) 
                    begin
                    // RF[instr[7:6]] <- ALU out
                    rf_reg_w_in     = instr_reg_out[7:6];
                    rf_data_w_in    = alu_reg_out;
                    rf_write        = 1;

                    // Check for run
                    done            = 1;
                    if (run)
                        next_state = CYCLE_1;
                    else
                        next_state = IDLE;
                end

                else if (instr_reg_out[3:0] == OP_LOAD) begin
                    // RF[instr[7:6]] <- MDR
                    rf_reg_w_in     = instr_reg_out[7:6];
                    rf_data_w_in    = mdr_out;
                    rf_write        = 1;

                    // Check for run
                    done            = 1;
                    if (run)
                        next_state = CYCLE_1;
                    else
                        next_state = IDLE;
                end

                else if (instr_reg_out[2:0] == OP_ORI[2:0]) begin
                    // A <- rA, B <- ZE(instr[7:3])
                    alu_in_a        = reg_a_out;
                    alu_in_b        = {3'b0, instr_reg_out[7:3]};
                    alu_op          = ALU_OR;
                    alu_out_load    = 1;
                    flag_wr         = 1;

                    next_state      = CYCLE_5;
                end
            end

            CYCLE_5: begin
                // only for ORI
                // RF[0] <- ALU out (change later to be SimProc accurate)
                    rf_reg_w_in     = 2'b0; // change this to 1, to match SimProc
                    rf_data_w_in    = alu_reg_out;
                    rf_write        = 1;

                    // Check for run
                    done            = 1;
                    if (run)
                        next_state = CYCLE_1;
                    else
                        next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            curr_state      <= IDLE;

            instr_reg_out   <= 8'b0;
            mdr_out         <= 8'b0;
            reg_a_out       <= 8'b0;
            reg_b_out       <= 8'b0;
            alu_reg_out     <= 8'b0;
            n_flag_out      <= 0;
            z_flag_out      <= 0;
        end
        else begin
            curr_state <= next_state;

            if (ir_load) begin
                instr_reg_out   <= instr_reg_in;
            end
            if (mdr_load) begin
                mdr_out         <= mdr_in;
            end
            if (ab_load) begin
                reg_a_out       <= reg_a_in;
                reg_b_out       <= reg_b_in;
            end
            if (alu_out_load) begin
                alu_reg_out     <= alu_reg_in;
            end
            if (flag_wr) begin
                n_flag_out      <= n_flag_in;
                z_flag_out      <= z_flag_in;
            end
        end
    end
endmodule

module register_file (
	input  logic            clk,
    input  logic            rst,
	input  logic            RFWrite,
	input  logic    [1:0]   regA,
	input  logic    [1:0]   regB,
	input  logic    [1:0]   regW,
	input  logic    [7:0]   dataW,

	output logic    [7:0]   dataA,
	output logic    [7:0]   dataB
);

    logic [7:0] rf[0:3];

	always_ff @ (posedge clk) begin
        if (rst) begin
            rf[0] <= 8'b0;
            rf[1] <= 8'b0;
            rf[2] <= 8'b0;
            rf[3] <= 8'b0;
        end
		else if (RFWrite) begin
            rf[regW] <= dataW;
		end
	end
	always_comb begin
        dataA = rf[regA];
        dataB = rf[regB];
    end
endmodule

module program_counter (
    input  logic            clk,
    input  logic            rst,
    input  logic    [7:0]   pc_in,
    input  logic            pc_wr,
    output logic    [7:0]   pc_out
); 
    always_ff @ (posedge clk) begin
        if (rst) begin
            pc_out <= 8'b0;
        end
        else if (pc_wr) begin
            pc_out <= pc_in;
        end
    end
endmodule

module ALU (
    input  logic    [2:0]   ALUop,
    input  logic    [7:0]   A,
    input  logic    [7:0]   B,
    output logic            N,
    output logic            Z,
    output logic    [7:0]   ALUout
);

    typedef enum logic [2:0] {
        ADD  = 3'b000,
        SUB  = 3'b001,
        OR   = 3'b010,
        NAND = 3'b011,
        SHL  = 3'b100,
        SHR  = 3'b101
    } aluop_t;

    always @ (*) begin
        case(ALUop) 
            ADD:        ALUout = A + B;
            SUB:        ALUout = A - B;
            OR:         ALUout = A | B;
            NAND:       ALUout = ~(A & B);
            SHL:        ALUout = A << B[1:0];
            SHR:        ALUout = A >> B[1:0];

            default:    ALUout = 8'h00; 
        endcase
    end

    assign N = ALUout[7];
    assign Z = ~(|ALUout);
endmodule

// Dual Port Ram Module

module dp_ram #(
    parameter DATA_WIDTH = 8,
    parameter MEM_DEPTH = 32
) (
    input  logic            clk,

	input  logic    [7:0]   din_a,
    input  logic    [7:0]   addr_a,
    input  logic            we_a,
    output logic    [7:0]   dout_a,

    input  logic    [7:0]   din_b,
    input  logic    [7:0]   addr_b,
    input  logic            we_b,
    output logic    [7:0]   dout_b
);

    logic [7:0] mem [0:MEM_DEPTH-1];

    always_ff @ (posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= din_a;
        end
        if (we_b) begin
            mem[addr_b] <= din_b;
        end
    end

    always_comb begin
        dout_a = mem[addr_a];
        dout_b = mem[addr_b];
    end    
endmodule

// UART Modules

module UART_wrapper #(
    parameter CLK_BITS     = 8, // bits for adjustable BAUD rate, min BAUD = F_CLK / (2^CLK_BITS)
    parameter DATA_WIDTH   = 8,
    parameter PARITY_BITS  = 0,
    parameter STOP_BITS    = 1
) (
    input  logic                    clk,
    input  logic                    rst,

    input  logic   [CLK_BITS-1:0]   clk_per_bit,

    input  logic   [DATA_WIDTH-1:0] TX_dataIn,
    input  logic                    TX_en,

    input  logic                    RX_dataIn,

    output logic                    TX_out,
    output logic                    TX_done,
    output logic                    TX_busy,

    output logic   [DATA_WIDTH-1:0] RX_dataOut,
    output logic                    RX_done,
    output logic                    RX_parityError
);
    // UART Transmitter Module
    UART_TX #(
        .CLK_BITS(CLK_BITS),
        .DATA_WIDTH(DATA_WIDTH),
        .PARITY_BITS(PARITY_BITS),
        .STOP_BITS(STOP_BITS)
        ) 
        UART_TX1 ( 
        .clk(clk),
        .rst(rst),

        .clk_per_bit(clk_per_bit),
        .dataIn(TX_dataIn),
        .TXen(TX_en),

        .TXout(TX_out),
        .TXdone(TX_done),
        .busy(TX_busy)
    );

    // UART Receiver Module
    UART_RX #(
        .CLK_BITS(CLK_BITS),
        .DATA_WIDTH(DATA_WIDTH),
        .PARITY_BITS(PARITY_BITS),
        .STOP_BITS(STOP_BITS)
        )
        UART_RX1 (
        .clk(clk),
        .rst(rst),

        .clk_per_bit(clk_per_bit),
        .dataIn(RX_dataIn),

        .RXout(RX_dataOut),
        .RXdone(RX_done),
        .parityError(RX_parityError)
    );
endmodule

module UART_RX #(
    parameter CLK_BITS = 8,   // bits for adjustable BAUD rate, min BAUD = F_CLK / (2^CLK_BITS)
    parameter DATA_WIDTH = 8,
    parameter STOP_BITS = 2,  // either 1 or 2 stop bits
    parameter PARITY_BITS = 1,
    parameter PACKET_SIZE = DATA_WIDTH + STOP_BITS + PARITY_BITS + 1
    // Total Packet Size = DATA_WIDTH + STOP_BITS + 1 Start Bit + 1 Parity Bit
) ( 
    input  logic                                clk,
    input  logic                                rst,

    input  logic    [CLK_BITS - 1 : 0]          clk_per_bit,
    input  logic                                dataIn,

    output logic    [DATA_WIDTH - 1 : 0]        RXout,
    output logic                                RXdone,
    output logic                                parityError
);

    localparam indexBits = $clog2(PACKET_SIZE);

    logic   [indexBits - 1 : 0]     index;
    logic   [CLK_BITS - 1 : 0]      clkCount;

    logic                           regInMeta;
    logic                           regIn;
    logic                           parity;

    logic    [DATA_WIDTH - 1 : 0]    dataOut;
    logic                           dataDone;

    // Remove Problems due to Metastability
    always_ff @(posedge clk) begin
        regInMeta <= dataIn;
        regIn <= regInMeta;
    end



    typedef enum logic [1:0] {
        IDLE,
        START,
        RECEIVE,
        DONE
    } 
    state_t;

    state_t state;

    always_ff @(posedge clk) begin
        if (rst) begin
            dataOut <= 0;
            state <= IDLE;
            index <= 1'b0;
            clkCount <= 0;
            dataDone <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    clkCount <= 0;
                    index <= 0;
                    dataOut <= 0;
                    dataDone <= 0;

                    if (regIn == 1'b0) begin    // Start Condition
                        state <= START;
                    end
                    else begin
                        state <= IDLE;
                    end
                end

                START: begin
                    if (clkCount == ((clk_per_bit - 1) >> 1)) begin
                        clkCount <= 0;
                        state <= RECEIVE;
                    end
                    else begin
                        clkCount <= clkCount + 1;
                        state <= START;
                    end

                end

                RECEIVE: begin

                    if (clkCount < clk_per_bit - 1) begin
                        clkCount <= clkCount + 1;
                        state <= RECEIVE;
                    end

                    else begin
                        clkCount <= 0;
                        if (index < DATA_WIDTH) begin
                            dataOut[index] <= regIn;
                            index <= index + 1;
                            state <= RECEIVE;
                        end
                        else if (index == DATA_WIDTH && PARITY_BITS > 0) begin
                            parity <= regIn;
                            state <= DONE;
                        end
                        else begin
                            state <= DONE;
                        end
                    end
                end

                DONE: begin
                    if (clkCount < clk_per_bit - 1) begin
                        clkCount <= clkCount + 1;
                        state <= DONE;
                    end
                    else begin
                        clkCount <= 0;
                        state <= IDLE;
                        dataDone <= 1'b1;
                        index <= 0;
                        RXout <= dataOut;
                    end
                end

                default: begin
                    state <= IDLE;
                end
                
            endcase
        end 
    end

    always_comb begin
        RXdone = dataDone;
        if (PARITY_BITS > 0) begin
            parityError = (^RXout) ^ parity;
        end
        else begin
            parityError = 0;
        end
    end
endmodule

module UART_TX #(
    parameter CLK_BITS = 8,         // bits for adjustable BAUD rate, min BAUD = F_CLK / (2^CLK_BITS)
    parameter DATA_WIDTH = 8,
    parameter STOP_BITS = 1,        // either 1 or 2 stop bits
    parameter PARITY_BITS = 1,      // can be set to 0
    parameter PACKET_SIZE = DATA_WIDTH + STOP_BITS + PARITY_BITS + 1 
    // Total Packet Size = DATA_WIDTH + STOP_BITS + 1 Start Bit + 1 Parity Bit
) ( 
    input  logic                                clk,
    input  logic                                rst,

    input  logic      [CLK_BITS - 1 : 0]        clk_per_bit,
    input  logic      [DATA_WIDTH - 1 : 0]      dataIn,
    input  logic                                TXen,

    output logic                                TXout,
    output logic                                TXdone,
    output logic                                busy
);

    localparam indexBits = $clog2(PACKET_SIZE);

    logic   [PACKET_SIZE - 1 : 0]       packet;
    logic                               parityBit;
    logic   [indexBits - 1 : 0]         index;
    logic   [CLK_BITS - 1 : 0]          clkCount;

    typedef enum logic [1:0] {
        IDLE,
        TRANSMIT,
        DONE
    } 
    state_t;

    state_t state;

    always_comb begin
        parityBit = ^dataIn;    // 0 for even number of 1's, 1 for odd number of 1's
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            TXout       <= 1'b1;
            state       <= IDLE;
            busy        <= 1'b0;
            index       <= 1'b0;
            clkCount    <= 0;
            TXdone      <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    TXout       <= 1'b1;
                    index       <= 1'b0;
                    clkCount    <= 0;
                    TXdone      <= 0;

                    if (TXen) begin
                        if (PARITY_BITS > 0) begin
                            packet <= {{STOP_BITS{1'b1}}, parityBit, dataIn, 1'b0};
                        end 
                        else begin
                            packet <= {{STOP_BITS{1'b1}}, dataIn, 1'b0};
                        end
                        //                ^                         ^
                        //                |                         |
                        //              Stop                      Start
                        busy <= 1'b1;
                        state <= TRANSMIT;
                    end
                    else begin
                        state <= IDLE;
                    end
                end

                TRANSMIT: begin
                    TXout <= packet[index];

                    if (clkCount < clk_per_bit - 1) begin
                        clkCount <= clkCount + 1;
                        state <= TRANSMIT;
                    end

                    else begin
                        clkCount <= 0;
                        if (index == PACKET_SIZE - 1) begin
                            state <= DONE;
                        end
                        else begin
                            index <= index + 1;
                            state <= TRANSMIT;
                        end
                    end
                end

                DONE: begin
                    state       <= IDLE;
                    busy        <= 1'b0;
                    TXdone      <= 1'b1;
                    index       <= 1'b0;
                    clkCount    <= 0;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end 
    end
endmodule
