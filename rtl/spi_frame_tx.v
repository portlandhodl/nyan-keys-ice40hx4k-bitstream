/**
 * Nyan Keys frame transmitter - SPI master, mode 0 (CPOL=0, CPHA=0)
 *
 * Shifts one frame out back-to-back with no gaps between bytes:
 *
 *   [SYNC] [data byte 0] ... [data byte DATA_BYTES-1] [CRC-8]
 *
 *  - Every byte is sent MSB first. Data byte i is data[8*i +: 8].
 *  - CRC-8 is polynomial 0x07, init 0x00, no reflection, no final xor
 *    (CRC-8/SMBUS), calculated over the data bytes only. It is computed
 *    serially as the bits go out, so it adds no latency.
 *  - SCLK = clk / (2 * HALF_BIT_CLKS). MOSI changes together with the SCLK
 *    falling edge, giving HALF_BIT_CLKS clocks of setup and hold around
 *    every rising edge.
 *  - start is accepted when busy is low; data is captured on that clock.
 */
module spi_frame_tx #(
    parameter       DATA_BYTES    = 8,
    parameter       HALF_BIT_CLKS = 2,
    parameter [7:0] SYNC          = 8'hA5
    ) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,
    input  wire [DATA_BYTES*8-1:0] data,
    output reg                     busy,
    output reg                     done,
    output reg                     sclk,
    output reg                     mosi
    );

    localparam BODY_BITS  = 8 + DATA_BYTES * 8;     // sync + data
    localparam FRAME_BITS = BODY_BITS + 8;          // + crc
    localparam IDX_W      = $clog2(FRAME_BITS);
    localparam PH_W       = (HALF_BIT_CLKS > 1) ? $clog2(2 * HALF_BIT_CLKS) : 1;

    // Frame body in wire order - sync first, then data bytes 0..N-1
    reg [BODY_BITS-1:0] body;
    integer i;
    always @(*) begin
        body[BODY_BITS-1 -: 8] = SYNC;
        for (i = 0; i < DATA_BYTES; i = i + 1)
            body[BODY_BITS-9-8*i -: 8] = data[8*i +: 8];
    end

    reg [BODY_BITS-1:0] sr;       // sr[MSB] is the bit currently on MOSI
    reg [7:0]           crc;      // running crc, then the crc shift register
    reg [IDX_W-1:0]     bit_idx;  // index of the bit currently on MOSI
    reg [PH_W-1:0]      ph;

    wire       in_data  = (bit_idx >= 8) && (bit_idx < BODY_BITS);
    wire [7:0] crc_step = {crc[6:0], 1'b0} ^ ((crc[7] ^ mosi) ? 8'h07 : 8'h00);
    wire [7:0] crc_nxt  = in_data ? crc_step : crc;
    wire [IDX_W-1:0] nxt_idx = bit_idx + 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy    <= 1'b0;
            done    <= 1'b0;
            sclk    <= 1'b0;
            mosi    <= 1'b0;
            sr      <= {BODY_BITS{1'b0}};
            crc     <= 8'h00;
            bit_idx <= {IDX_W{1'b0}};
            ph      <= {PH_W{1'b0}};
        end else begin
            done <= 1'b0;

            if (!busy) begin
                if (start) begin
                    busy    <= 1'b1;
                    sr      <= body;
                    mosi    <= body[BODY_BITS-1];
                    crc     <= 8'h00;
                    bit_idx <= {IDX_W{1'b0}};
                    ph      <= {PH_W{1'b0}};
                end
            end else if (ph == 2 * HALF_BIT_CLKS - 1) begin
                // End of a bit: SCLK falls and the next bit goes out
                sclk <= 1'b0;
                ph   <= {PH_W{1'b0}};
                if (bit_idx == FRAME_BITS - 1) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    mosi <= 1'b0;
                end else begin
                    bit_idx <= nxt_idx;
                    if (nxt_idx < BODY_BITS) begin
                        sr   <= sr << 1;
                        mosi <= sr[BODY_BITS-2];
                        crc  <= crc_nxt;
                    end else if (nxt_idx == BODY_BITS) begin
                        mosi <= crc_nxt[7];
                        crc  <= crc_nxt << 1;
                    end else begin
                        mosi <= crc[7];
                        crc  <= crc << 1;
                    end
                end
            end else begin
                if (ph == HALF_BIT_CLKS - 1)
                    sclk <= 1'b1;
                ph <= ph + 1'b1;
            end
        end
    end

endmodule
