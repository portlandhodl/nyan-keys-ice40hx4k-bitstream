`timescale 1ns/1ps
`default_nettype none

/**
 * Unit test for the SPI frame transmitter (rtl/spi_frame_tx.v)
 *
 *  - Reference CRC-8/SMBUS is checked against the standard check value
 *    (crc8("123456789") == 0xF4) so the FPGA and firmware agree on the spec.
 *  - Random payloads are sent and the received bit stream must be exactly
 *    [SYNC][data bytes][CRC] with no gaps and exactly FRAME_BITS clocks.
 *  - SCLK period, MOSI setup/hold and idle levels are checked.
 *  - Runs for both HALF_BIT_CLKS = 2 (19.5MHz) and 1 (39MHz).
 */
module tb_spi_frame_tx;
    localparam DATA_BYTES = 8;
    localparam FRAME_BITS = 8 + DATA_BYTES * 8 + 8;
    localparam real HALF_CLK = 6.41;    // 78MHz

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #(HALF_CLK) clk = ~clk;

    integer errors = 0;

    function [7:0] crc8_byte(input [7:0] crc, input [7:0] b);
        integer k;
        begin
            crc8_byte = crc ^ b;
            for (k = 0; k < 8; k = k + 1)
                crc8_byte = crc8_byte[7] ? ((crc8_byte << 1) ^ 8'h07) : (crc8_byte << 1);
        end
    endfunction

    // Two DUTs, one per SCLK rate, driven by the same stimulus
    reg                     start = 1'b0;
    reg  [DATA_BYTES*8-1:0] data;
    wire [1:0]              busy, done, sclk, mosi;

    spi_frame_tx #(.DATA_BYTES(DATA_BYTES), .HALF_BIT_CLKS(2)) dut2 (
        .clk(clk), .rst_n(rst_n), .start(start), .data(data),
        .busy(busy[0]), .done(done[0]), .sclk(sclk[0]), .mosi(mosi[0]));

    spi_frame_tx #(.DATA_BYTES(DATA_BYTES), .HALF_BIT_CLKS(1)) dut1 (
        .clk(clk), .rst_n(rst_n), .start(start), .data(data),
        .busy(busy[1]), .done(done[1]), .sclk(sclk[1]), .mosi(mosi[1]));

    // Per-DUT receiver + timing monitor
    reg  [FRAME_BITS-1:0] rx       [0:1];
    integer               nbits    [0:1];
    realtime              t_mosi   [0:1];
    realtime              t_rise   [0:1];
    realtime              t_rise_p [0:1];

    initial begin
        nbits[0] = 0; nbits[1] = 0;
        t_mosi[0] = 0; t_mosi[1] = 0;
        t_rise[0] = -1000; t_rise[1] = -1000;
        t_rise_p[0] = -1000; t_rise_p[1] = -1000;
    end

    `define RX_MONITOR(N, HALF_BIT) \
        always @(mosi[N]) begin \
            if (rst_n && busy[N] && ($realtime - t_rise[N]) < (HALF_BIT * 2 * HALF_CLK - 0.01)) begin \
                $display("ERROR dut%0d MOSI hold violation", HALF_BIT); errors = errors + 1; \
            end \
            t_mosi[N] = $realtime; \
        end \
        always @(posedge sclk[N]) begin \
            if (($realtime - t_mosi[N]) < (HALF_BIT * 2 * HALF_CLK - 0.01)) begin \
                $display("ERROR dut%0d MOSI setup violation", HALF_BIT); errors = errors + 1; \
            end \
            if (nbits[N] > 0 && (($realtime - t_rise[N]) - HALF_BIT * 4 * HALF_CLK > 0.01 || \
                                 ($realtime - t_rise[N]) - HALF_BIT * 4 * HALF_CLK < -0.01)) begin \
                $display("ERROR dut%0d SCLK period %0.2f (gap in frame?)", HALF_BIT, $realtime - t_rise[N]); \
                errors = errors + 1; \
            end \
            t_rise[N] = $realtime; \
            rx[N]     = {rx[N][FRAME_BITS-2:0], mosi[N]}; \
            nbits[N]  = nbits[N] + 1; \
        end

    `RX_MONITOR(0, 2)
    `RX_MONITOR(1, 1)

    task check_frame(input integer n, input integer half_bit);
        reg [7:0] crc;
        reg [FRAME_BITS-1:0] exp;
        integer b;
        begin
            crc = 8'h00;
            exp[FRAME_BITS-1 -: 8] = 8'hA5;
            for (b = 0; b < DATA_BYTES; b = b + 1) begin
                exp[FRAME_BITS-9-8*b -: 8] = data[8*b +: 8];
                crc = crc8_byte(crc, data[8*b +: 8]);
            end
            exp[7:0] = crc;
            if (nbits[n] != FRAME_BITS) begin
                $display("ERROR dut%0d sent %0d SCLK pulses, expected %0d", half_bit, nbits[n], FRAME_BITS);
                errors = errors + 1;
            end
            if (rx[n] !== exp) begin
                $display("ERROR dut%0d frame\n  got %h\n  exp %h", half_bit, rx[n], exp);
                errors = errors + 1;
            end
            if (sclk[n] !== 1'b0) begin
                $display("ERROR dut%0d SCLK not idle low after frame", half_bit);
                errors = errors + 1;
            end
            nbits[n] = 0;
        end
    endtask

    // +dump writes every frame (wire order, hex) for the firmware cross-check
    integer dump_fd = 0;

    integer f, c, tdone;
    reg [7:0] check;
    reg [8*9-1:0] str;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_spi_frame_tx.vcd");
            $dumpvars(0, tb_spi_frame_tx);
        end

        if ($test$plusargs("dump"))
            dump_fd = $fopen("frames.hex", "w");

        // Reference CRC must match the published CRC-8/SMBUS check value
        str   = "123456789";
        check = 8'h00;
        for (c = 8; c >= 0; c = c - 1)
            check = crc8_byte(check, str[8*c +: 8]);
        if (check !== 8'hF4) begin
            $display("ERROR reference crc8(\"123456789\") = %h, expected f4", check);
            errors = errors + 1;
        end

        data = 0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        for (f = 0; f < 200; f = f + 1) begin
            case (f)
                0:       data = {DATA_BYTES*8{1'b0}};
                1:       data = {DATA_BYTES*8{1'b1}};
                2:       data = 64'h0123456789abcdef;
                default: data = {$random, $random};
            endcase
            @(negedge clk) start = 1'b1;
            @(negedge clk) start = 1'b0;
            // the slow DUT finishes last
            tdone = 0;
            while (!(busy == 2'b00) && tdone < 1000) begin
                @(posedge clk);
                tdone = tdone + 1;
            end
            #1;
            if (dump_fd != 0)
                $fdisplay(dump_fd, "%h", rx[0]);
            check_frame(0, 2);
            check_frame(1, 1);
            repeat ($unsigned($random) % 4) @(posedge clk);
        end

        // start is ignored while busy - only one frame may come out
        data = 64'hfeedfacecafebeef;
        @(negedge clk) start = 1'b1;
        repeat (20) @(negedge clk);
        start = 1'b0;
        while (busy != 2'b00) @(posedge clk);
        #1;
        check_frame(0, 2);
        check_frame(1, 1);

        if (dump_fd != 0)
            $fclose(dump_fd);
        if (errors == 0) $display("PASS tb_spi_frame_tx");
        else             $display("FAIL tb_spi_frame_tx: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
