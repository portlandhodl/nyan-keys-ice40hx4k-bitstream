`timescale 1ns/1ps
`default_nettype none

/**
 * System test for spi_keys (keys -> debounce -> frame push -> MCU)
 *
 * MCU model (mirrors the STM32 firmware):
 *  - RX-only SPI slave, mode 0. After FRAME_BITS bits the DMA completes; the
 *    ISR re-arms (SPI reset - bits arriving meanwhile are lost), validates
 *    sync + CRC, and pulses ack (default 2ns, narrower than one FPGA clock)
 *    only for a good frame.
 *  - Stall watchdog: a partial frame with no SCLK activity for WD_CYCLES is
 *    discarded (SPI reset) so a lost SCLK edge can't misalign later frames.
 *  - Fault injection: dropped/extra SCLK edges, stuck-high ack, MCU not
 *    acking, MCU missing whole frames.
 *
 * Bus monitor (independent of the MCU model):
 *  - Counts every SCLK edge; frames are exactly FRAME_BITS with no gaps.
 *  - MOSI setup/hold around SCLK rising.
 *  - A new frame only starts after an ack edge, or after the ack timeout.
 *
 * Debounce, refresh and ack timeout are shortened via parameters. In
 * simulation the PLL is bypassed, so clk_g_i is the 78MHz core clock.
 */
module tb_spi_keys;
    localparam NUM_KEYS    = 61;
    localparam DATA_BYTES  = (NUM_KEYS + 7) / 8;
    localparam FRAME_BITS  = 8 + DATA_BYTES * 8 + 8;
    localparam PRESCALE    = 4;
    localparam TICKS       = 4;
    localparam REFRESH     = 6000;
    localparam ACK_TIMEOUT = 1500;
    localparam HALF_BIT    = 2;
    localparam DEBOUNCE    = PRESCALE * TICKS;
    localparam FRAME_CLKS  = FRAME_BITS * 2 * HALF_BIT;
    localparam WD_CYCLES   = 300;         // MCU stall watchdog
    localparam REARM_CLKS  = 30;          // MCU ISR entry + SPI reset/re-arm (~0.4us)
    localparam real HALF_CLK = 6.41;      // 78MHz

    reg                 clk  = 1'b0;
    reg                 rstn = 1'b0;
    reg                 ack  = 1'b0;
    reg  [NUM_KEYS-1:0] keys_in = {NUM_KEYS{1'b1}};
    wire                sclk;
    wire                mosi;

    integer errors = 0;
    integer cycle  = 0;

    spi_keys #(
        .NUM_KEYS          (NUM_KEYS),
        .DEBOUNCE_PRESCALE (PRESCALE),
        .DEBOUNCE_TICKS    (TICKS),
        .REFRESH_CYCLES    (REFRESH),
        .ACK_TIMEOUT_CYCLES(ACK_TIMEOUT),
        .HALF_BIT_CLKS     (HALF_BIT)
    ) dut (
        .clk_g_i     (clk),
        .rstn_g_i    (rstn),
        .spi_keys_ack(ack),
        .spi_clk_g_o (sclk),
        .spi_mosi_g_o(mosi),
        .keys_i_g    (keys_in)
    );

    always #(HALF_CLK) clk = ~clk;
    always @(posedge clk) cycle <= cycle + 1;

    function [7:0] crc8_byte(input [7:0] crc, input [7:0] b);
        integer k;
        begin
            crc8_byte = crc ^ b;
            for (k = 0; k < 8; k = k + 1)
                crc8_byte = crc8_byte[7] ? ((crc8_byte << 1) ^ 8'h07) : (crc8_byte << 1);
        end
    endfunction

    /**
     * Bus monitor
     */
    integer  bus_bits         = 0;
    integer  bus_frames       = 0;
    integer  bus_frame_start  = -1000000;
    integer  last_ack_cycle   = -1000000;
    integer  last_frame_end   = -1000000;
    realtime t_mosi           = 0;
    realtime t_sclk_rise      = -1000;

    always @(posedge ack) last_ack_cycle = cycle;

    always @(mosi) begin
        if (rstn && ($realtime - t_sclk_rise) < HALF_BIT * 2 * HALF_CLK - 0.01) begin
            $display("[%0d] ERROR MOSI hold violation", cycle);
            errors = errors + 1;
        end
        t_mosi = $realtime;
    end

    always @(posedge sclk) begin
        if (($realtime - t_mosi) < HALF_BIT * 2 * HALF_CLK - 0.01) begin
            $display("[%0d] ERROR MOSI setup violation", cycle);
            errors = errors + 1;
        end
        if (bus_bits % FRAME_BITS == 0) begin
            // A new frame may only start after an ack for the previous one,
            // or once the ack timeout has expired.
            if (bus_frames > 0 && last_ack_cycle < last_frame_end &&
                cycle - bus_frame_start < ACK_TIMEOUT) begin
                $display("[%0d] ERROR frame started %0d clocks after the last one with no ack",
                         cycle, cycle - bus_frame_start);
                errors = errors + 1;
            end
            bus_frame_start = cycle;
        end else if ($realtime - t_sclk_rise > 2 * HALF_BIT * 2 * HALF_CLK + 0.01) begin
            $display("[%0d] ERROR gap inside a frame", cycle);
            errors = errors + 1;
        end
        t_sclk_rise = $realtime;
        bus_bits    = bus_bits + 1;
        if (bus_bits % FRAME_BITS == 0) begin
            bus_frames     = bus_frames + 1;
            last_frame_end = cycle;
        end
    end

    /**
     * MCU model
     */
    reg [FRAME_BITS-1:0] mcu_sr;
    reg [NUM_KEYS-1:0]   mcu_keys;               // last validated key state
    integer              mcu_bits        = 0;
    integer              mcu_good        = 0;
    integer              mcu_bad         = 0;
    integer              mcu_wd_resets   = 0;
    integer              mcu_good_cycle  = 0;
    integer              mcu_last_edge   = 0;
    reg                  mcu_rearming    = 1'b0;
    reg                  mcu_enable      = 1'b1;  // ack good frames
    reg                  ack_stuck       = 1'b0;  // hold ack high
    integer              ack_delay       = 10;    // clocks from re-arm to ack
    real                 ack_width       = 2.0;   // ns
    integer              drop_edges      = 0;     // ignore the next N SCLK edges
    integer              extra_edges     = 0;     // count N phantom SCLK edges
    integer              miss_frames     = 0;     // drop the next N whole frames

    task mcu_frame_done;
        reg [FRAME_BITS-1:0] fr;
        reg [7:0]            crc;
        reg [8*DATA_BYTES-1:0] payload;
        integer b;
        begin
            fr           = mcu_sr;
            mcu_bits     = 0;
            mcu_rearming = 1'b1;
            repeat (REARM_CLKS) @(posedge clk);   // ISR latency + SPI reset/re-arm
            mcu_rearming = 1'b0;
            crc = 8'h00;
            for (b = 0; b < DATA_BYTES; b = b + 1) begin
                payload[8*b +: 8] = fr[FRAME_BITS-9-8*b -: 8];
                crc = crc8_byte(crc, payload[8*b +: 8]);
            end
            if (fr[FRAME_BITS-1 -: 8] !== 8'hA5 || fr[7:0] !== crc) begin
                mcu_bad = mcu_bad + 1;
            end else begin
                if (payload[8*DATA_BYTES-1:NUM_KEYS] !== {(8*DATA_BYTES-NUM_KEYS){1'b1}}) begin
                    $display("[%0d] ERROR pad bits not 1: %b", cycle, payload[8*DATA_BYTES-1:NUM_KEYS]);
                    errors = errors + 1;
                end
                mcu_keys       = payload[NUM_KEYS-1:0];
                mcu_good       = mcu_good + 1;
                mcu_good_cycle = cycle;
                // The SPI is re-armed already, so keep receiving while the
                // ack pulse goes out
                if (mcu_enable && !ack_stuck) fork
                    begin
                        repeat (ack_delay) @(posedge clk);
                        ack = 1'b1;
                        #(ack_width);
                        ack = ack_stuck;
                    end
                join_none
            end
        end
    endtask

    always @(posedge sclk) begin
        if (drop_edges > 0) begin
            drop_edges = drop_edges - 1;
        end else if (!mcu_rearming) begin
            if (mcu_bits == 0 && miss_frames > 0) begin
                // MCU is not listening for this whole frame
                miss_frames  = miss_frames - 1;
                mcu_rearming = 1'b1;
                fork
                    begin
                        repeat (FRAME_CLKS - 2) @(posedge clk);
                        mcu_rearming = 1'b0;
                    end
                join_none
            end else begin
                mcu_sr        = {mcu_sr[FRAME_BITS-2:0], mosi};
                mcu_bits      = mcu_bits + 1;
                mcu_last_edge = cycle;
                if (extra_edges > 0) begin
                    extra_edges = extra_edges - 1;
                    mcu_sr      = {mcu_sr[FRAME_BITS-2:0], mosi};
                    mcu_bits    = mcu_bits + 1;
                end
                if (mcu_bits >= FRAME_BITS)
                    mcu_frame_done;
            end
        end
    end

    // Stall watchdog
    always @(posedge clk) begin
        if (mcu_bits != 0 && !mcu_rearming && cycle - mcu_last_edge > WD_CYCLES) begin
            mcu_bits      = 0;
            mcu_wd_resets = mcu_wd_resets + 1;
        end
    end

    always @(ack_stuck) ack = ack_stuck;

    /**
     * Helpers
     */
    task tick(input integer n);
        repeat (n) @(posedge clk);
    endtask

    // Wait until the MCU holds a validated state equal to keys_in
    task expect_state(input integer timeout, input [8*48-1:0] what);
        integer t;
        begin
            t = 0;
            while (mcu_keys !== keys_in && t < timeout) begin
                @(posedge clk);
                t = t + 1;
            end
            if (mcu_keys !== keys_in) begin
                $display("[%0d] ERROR %0s: MCU has %h expected %h", cycle, what, mcu_keys, keys_in);
                errors = errors + 1;
            end
        end
    endtask

    // Wait for n more good frames at the MCU
    task wait_good(input integer n, input integer timeout, input [8*48-1:0] what);
        integer start, t;
        begin
            start = mcu_good;
            t     = 0;
            while (mcu_good < start + n && t < timeout) begin
                @(posedge clk);
                t = t + 1;
            end
            if (mcu_good < start + n) begin
                $display("[%0d] ERROR timeout waiting for frame: %0s", cycle, what);
                errors = errors + 1;
            end
        end
    endtask

    // Let everything go idle (no frame in flight, nothing pending)
    task settle;
        begin
            tick(2 * DEBOUNCE + 2 * (FRAME_CLKS + REARM_CLKS + ack_delay + 10));
        end
    endtask

    localparam SETTLE_MAX = 2 * DEBOUNCE + 3 * (FRAME_CLKS + REARM_CLKS + 30);

    integer i, t0, gap, lat, f0, b0, max_lat;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_spi_keys.vcd");
            $dumpvars(0, tb_spi_keys);
        end

        fork
            begin
                #(HALF_CLK * 2 * 3000000);
                $display("FAIL tb_spi_keys: global timeout");
                $finish;
            end
        join_none

        // ---------------------------------------------------------------
        $display("[test] key held through reset is reported right after reset");
        keys_in[7] = 1'b0;
        tick(20);
        @(negedge clk) rstn = 1'b1;
        t0 = cycle;
        wait_good(1, FRAME_CLKS + 100, "initial frame");
        $display("        reset -> first frame %0d clocks", mcu_good_cycle - t0);
        expect_state(1, "initial state");

        // ---------------------------------------------------------------
        $display("[test] press latency on an idle bus");
        settle;
        @(negedge clk) keys_in[7] = 1'b1;
        settle;
        max_lat = 0;
        for (i = 0; i < 8; i = i + 1) begin
            @(negedge clk) keys_in[3 + i * 7] = ~keys_in[3 + i * 7];
            t0 = cycle;
            wait_good(1, FRAME_CLKS + 100, "press");
            lat = mcu_good_cycle - t0;
            if (lat > max_lat) max_lat = lat;
            settle;
        end
        expect_state(1, "press");
        $display("        key edge -> MCU has validated frame: %0d clocks (%0.2f us @ 78MHz)",
                 max_lat, max_lat / 78.0);
        $display("        of which FPGA (key edge -> last SCLK edge): %0d clocks (%0.2f us)",
                 max_lat - REARM_CLKS, (max_lat - REARM_CLKS) / 78.0);
        // 2 sync + 1 debounce + 1 schedule + frame + MCU re-arm/validate
        if (max_lat > FRAME_CLKS + REARM_CLKS + 6) begin
            $display("ERROR press latency %0d > %0d", max_lat, FRAME_CLKS + REARM_CLKS + 6);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------
        $display("[test] periodic refresh while idle");
        wait_good(1, REFRESH + FRAME_CLKS + 200, "refresh 1");
        f0 = mcu_good_cycle;
        wait_good(1, REFRESH + FRAME_CLKS + 200, "refresh 2");
        gap = mcu_good_cycle - f0;
        $display("        refresh interval %0d clocks", gap);
        if (gap < REFRESH || gap > REFRESH + 10) begin
            $display("ERROR refresh interval %0d", gap);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------
        $display("[test] contact bounce settles to the final state");
        for (i = 0; i < 60; i = i + 1) begin
            @(negedge clk) keys_in[17] = ~keys_in[17];
            keys_in[42] = $random;
        end
        keys_in[17] = 1'b0;
        keys_in[42] = 1'b0;
        expect_state(SETTLE_MAX, "bounce");

        // ---------------------------------------------------------------
        $display("[test] change while a frame is in flight is sent right after the ack");
        for (i = 0; i < 20; i = i + 1) begin
            settle;
            @(negedge clk) keys_in[20] = ~keys_in[20];
            wait (mcu_bits == 10 + 3 * i);
            @(negedge clk) keys_in[21 + i] = ~keys_in[21 + i];
            t0 = cycle;
            expect_state(2 * FRAME_CLKS + REARM_CLKS + ack_delay + DEBOUNCE + 50, "mid-frame change");
        end

        // ---------------------------------------------------------------
        $display("[test] wide ack pulse");
        ack_width = 1000.0;
        @(negedge clk) keys_in[0] = ~keys_in[0];
        expect_state(SETTLE_MAX, "wide ack");
        settle;
        ack_width = 2.0;

        // ---------------------------------------------------------------
        $display("[test] MCU busy (no ack) -> FPGA retries at the ack timeout");
        settle;
        mcu_enable = 1'b0;
        f0 = bus_frames;
        @(negedge clk) keys_in[1] = ~keys_in[1];
        tick(4 * ACK_TIMEOUT + 100);
        $display("        %0d frames in 4 timeouts while unacked", bus_frames - f0);
        if (bus_frames - f0 < 4 || bus_frames - f0 > 5) begin
            $display("ERROR expected 4-5 retries, saw %0d", bus_frames - f0);
            errors = errors + 1;
        end
        @(negedge clk) keys_in[2] = ~keys_in[2];
        mcu_enable = 1'b1;
        expect_state(ACK_TIMEOUT + SETTLE_MAX, "after busy MCU");

        // ---------------------------------------------------------------
        $display("[test] stuck-high ack never acknowledges a frame");
        settle;
        ack_stuck = 1'b1;
        f0 = bus_frames;
        @(negedge clk) keys_in[4] = ~keys_in[4];
        tick(3 * ACK_TIMEOUT + 100);
        if (bus_frames - f0 < 3) begin
            $display("ERROR stuck ack was taken as an ack (%0d frames)", bus_frames - f0);
            errors = errors + 1;
        end
        ack_stuck = 1'b0;
        @(negedge clk) keys_in[5] = ~keys_in[5];
        expect_state(ACK_TIMEOUT + SETTLE_MAX, "after stuck ack");

        // ---------------------------------------------------------------
        $display("[test] MCU misses a whole frame -> retried");
        settle;
        miss_frames = 1;
        @(negedge clk) keys_in[6] = ~keys_in[6];
        t0 = cycle;
        expect_state(ACK_TIMEOUT + SETTLE_MAX, "missed frame");
        $display("        recovered in %0d clocks", cycle - t0);

        // ---------------------------------------------------------------
        $display("[test] lost SCLK edge (bit slip) -> watchdog resync + retry");
        for (i = 0; i < 10; i = i + 1) begin
            settle;
            b0 = mcu_bad;
            drop_edges = 1 + (i % 3);
            @(negedge clk) keys_in[30 + i] = ~keys_in[30 + i];
            t0 = cycle;
            expect_state(2 * ACK_TIMEOUT + SETTLE_MAX, "lost edge");
            if (cycle - t0 < ACK_TIMEOUT - FRAME_CLKS) begin
                $display("ERROR lost-edge frame was accepted (recovered in %0d)", cycle - t0);
                errors = errors + 1;
            end
        end
        $display("        watchdog resets so far: %0d", mcu_wd_resets);

        // ---------------------------------------------------------------
        $display("[test] extra SCLK edge (glitch) -> CRC reject + retry");
        for (i = 0; i < 10; i = i + 1) begin
            settle;
            b0 = mcu_bad;
            extra_edges = 1 + (i % 3);
            @(negedge clk) keys_in[45 + i] = ~keys_in[45 + i];
            expect_state(3 * ACK_TIMEOUT + SETTLE_MAX, "extra edge");
            if (mcu_bad == b0) begin
                $display("ERROR corrupted frame was not rejected");
                errors = errors + 1;
            end
        end

        // ---------------------------------------------------------------
        $display("[test] random key patterns");
        for (i = 0; i < 40; i = i + 1) begin
            @(negedge clk);
            keys_in = {$random, $random};
            expect_state(SETTLE_MAX, "random pattern");
        end

        // ---------------------------------------------------------------
        $display("[test] random typing with random faults");
        for (i = 0; i < 300; i = i + 1) begin
            @(negedge clk);
            keys_in[$unsigned($random) % NUM_KEYS] = $random;
            case ($unsigned($random) % 16)
                0: drop_edges  = 1;
                1: extra_edges = 1;
                2: miss_frames = 1;
                default: ;
            endcase
            tick($unsigned($random) % (FRAME_CLKS * 2));
        end
        drop_edges  = 0;
        extra_edges = 0;
        miss_frames = 0;
        expect_state(4 * ACK_TIMEOUT + SETTLE_MAX, "random typing + faults");

        $display("        bus frames %0d, MCU good %0d, rejected %0d, watchdog resets %0d",
                 bus_frames, mcu_good, mcu_bad, mcu_wd_resets);
        if (errors == 0) $display("PASS tb_spi_keys");
        else             $display("FAIL tb_spi_keys: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
