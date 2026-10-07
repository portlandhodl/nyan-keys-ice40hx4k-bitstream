/**
 * @auth: Reese Russell
 * @date: 10/10/23
 * @desc: keys -> debounce -> SPI frame push to the MCU
 *
 * Protocol (FPGA is the SPI master, MCU is an RX-only SPI slave):
 *  - Frame: [0xA5] [keys 7:0] [keys 15:8] ... [keys 63:56] [CRC-8], 80 bits
 *    back-to-back, mode 0, MSB first. Unused pad bits are 1 (released).
 *  - A frame is sent as soon as the debounced key state differs from the last
 *    frame sent, right after reset, every REFRESH_CYCLES, and again if the MCU
 *    does not ack within ACK_TIMEOUT_CYCLES.
 *  - After a frame the FPGA waits for a rising edge on spi_keys_ack. The edge
 *    is captured asynchronously, so pulses of any width are seen. Edges that
 *    arrive while a frame is being sent are ignored.
 *
 * Everything runs on the 78MHz PLL clock - a single clock domain.
 */

module spi_keys #(
    parameter NUM_KEYS           = 61,
    parameter DEBOUNCE_PRESCALE  = 8192,     // clocks per debounce tick (105us @ 78MHz)
    parameter DEBOUNCE_TICKS     = 127,      // ~13.3ms @ 78MHz
    parameter REFRESH_CYCLES     = 3900000,  // 50ms @ 78MHz
    parameter ACK_TIMEOUT_CYCLES = 78000,    // 1ms @ 78MHz, measured from frame start
    parameter HALF_BIT_CLKS      = 2         // SCLK = 78MHz / 4 = 19.5MHz
    ) (
    // Globals
    input  wire clk_g_i,
    input  wire rstn_g_i,
    input  wire spi_keys_ack,

    // SPI Interface - Global
    output wire spi_clk_g_o,
    output wire spi_mosi_g_o,

    // Key Interface - Global
    input wire [NUM_KEYS-1:0] keys_i_g
    );

    localparam DATA_BYTES = (NUM_KEYS + 7) / 8;
    localparam KEYS_PAD   = DATA_BYTES * 8;
    localparam CNT_MAX    = (REFRESH_CYCLES > ACK_TIMEOUT_CYCLES) ? REFRESH_CYCLES : ACK_TIMEOUT_CYCLES;
    localparam CNT_W      = $clog2(CNT_MAX + 1);

    localparam S_IDLE = 2'd0;
    localparam S_SEND = 2'd1;
    localparam S_WAIT = 2'd2;

    wire                clk;
    wire                pll_locked;
    wire [NUM_KEYS-1:0] keys;
    wire                tx_done;

    reg  [1:0]          state;
    reg  [NUM_KEYS-1:0] keys_sent;
    reg                 force_send;
    reg  [CNT_W-1:0]    since_start;

    /**
     * Clocking
     */
    `ifdef __ICARUS__
        // Simulation bypass PLL - Since no models are available
        assign clk        = clk_g_i;
        assign pll_locked = 1'b1;
    `else
        wire clk_pll;

        // Core clock generation - 12MHz * 52 / 8 = 78MHz
        SB_PLL40_CORE #(
            .FEEDBACK_PATH("SIMPLE"),
            .DIVR(4'b0000),       // DIVR =  0
            .DIVF(7'b0110011),    // DIVF = 51
            .DIVQ(3'b011),        // DIVQ =  3
            .FILTER_RANGE(3'b001) // FILTER_RANGE = 1
        ) g_pll (
            .LOCK(pll_locked),
            .RESETB(1'b1),
            .BYPASS(1'b0),
            .REFERENCECLK(clk_g_i),
            .PLLOUTGLOBAL(clk_pll)
        );

        // Buffer the output of the pll before use.
        SB_GB pll_fabric_buffer(
            .USER_SIGNAL_TO_GLOBAL_BUFFER(clk_pll),
            .GLOBAL_BUFFER_OUTPUT(clk)
        );
    `endif

    /**
     * Reset synchronizer - asserts asynchronously, releases synchronously,
     * and holds the core in reset until the PLL has locked.
     */
    reg [1:0] rst_sync;
    wire      rstn = rst_sync[1];

    always @(posedge clk or negedge rstn_g_i) begin
        if (!rstn_g_i)        rst_sync <= 2'b00;
        else if (!pll_locked) rst_sync <= 2'b00;
        else                  rst_sync <= {rst_sync[0], 1'b1};
    end

    /**
     * Keyboard keys interface - synchronizers + eager debounce
     */
    keys #(
        .keys          (NUM_KEYS),
        .PRESCALE      (DEBOUNCE_PRESCALE),
        .DEBOUNCE_TICKS(DEBOUNCE_TICKS)
    ) keys_interface (
        .clk_i   (clk),
        .rst_n_i (rstn),
        .keys_i  (keys_i_g),
        .keys_o  (keys)
    );

    /**
     * MCU ack capture
     *  - ack_seen is clocked by the ack pin itself so a pulse of any width
     *    sets it. It is held clear except while waiting for an ack, so an
     *    early or stuck-high ack can never acknowledge a frame it did not see.
     */
    reg       ack_clr;
    reg       ack_seen;
    reg [1:0] ack_sync;

    always @(posedge spi_keys_ack or posedge ack_clr) begin
        if (ack_clr) ack_seen <= 1'b0;
        else         ack_seen <= 1'b1;
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ack_clr  <= 1'b1;
            ack_sync <= 2'b00;
        end else begin
            ack_clr  <= (state != S_WAIT);
            ack_sync <= {ack_sync[0], ack_seen};
        end
    end

    /**
     * Frame scheduler
     *  - since_start counts clocks since the last frame started. It provides
     *    both the ack timeout (in S_WAIT) and the periodic refresh (in S_IDLE).
     *  - The 61-bit compare is registered to meet 78MHz (+1 clock). keys_changed
     *    is one clock stale after keys_sent updates, but that is always inside
     *    S_SEND, which lasts a whole frame.
     */
    reg  keys_changed;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) keys_changed <= 1'b0;
        else       keys_changed <= (keys != keys_sent);
    end

    wire want  = keys_changed || force_send || (since_start >= REFRESH_CYCLES);
    wire start = (state == S_IDLE) && want;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state       <= S_IDLE;
            keys_sent   <= {NUM_KEYS{1'b1}};
            force_send  <= 1'b1;            // report the initial state right away
            since_start <= {CNT_W{1'b0}};
        end else begin
            if (since_start != CNT_MAX)
                since_start <= since_start + 1'b1;

            case (state)
                S_IDLE: begin
                    if (want) begin
                        keys_sent   <= keys;
                        force_send  <= 1'b0;
                        since_start <= {CNT_W{1'b0}};
                        state       <= S_SEND;
                    end
                end
                S_SEND: begin
                    if (tx_done)
                        state <= S_WAIT;
                end
                S_WAIT: begin
                    if (ack_sync[1]) begin
                        state <= S_IDLE;
                    end else if (since_start >= ACK_TIMEOUT_CYCLES) begin
                        force_send <= 1'b1;     // no ack - send the frame again
                        state      <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

    /**
     * SPI frame transmitter - unused pad bits are sent as 1 (released)
     */
    // One extra pad bit avoids a zero-width replication when NUM_KEYS is a
    // multiple of 8; it is truncated off the top by the assignment.
    wire [KEYS_PAD-1:0] tx_data = {{(KEYS_PAD - NUM_KEYS + 1){1'b1}}, keys};

    spi_frame_tx #(
        .DATA_BYTES   (DATA_BYTES),
        .HALF_BIT_CLKS(HALF_BIT_CLKS)
    ) frame_tx (
        .clk  (clk),
        .rst_n(rstn),
        .start(start),
        .data (tx_data),
        .busy (),
        .done (tx_done),
        .sclk (spi_clk_g_o),
        .mosi (spi_mosi_g_o)
    );

endmodule
