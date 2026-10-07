/**
 * Eager per-key debouncer.
 *
 *  - Inputs are asynchronous switch pins, so each one is passed through a
 *    2-FF synchronizer before it is used.
 *  - A single shared prescaler produces a tick every PRESCALE clocks. Each key
 *    only needs a small counter of ticks instead of a full-width cycle
 *    counter, which saves ~1000 LCs on a 61 key board.
 *  - A state change is accepted instantly once the key's lockout has expired,
 *    after which further changes are ignored for DEBOUNCE_TICKS ticks.
 *
 *  Lockout period = (DEBOUNCE_TICKS-1 .. DEBOUNCE_TICKS) * PRESCALE / f(clk_i)
 *  Default: 127 * 8192 / 78MHz ~= 13.3ms
 */
module keys #(
    parameter keys           = 61,
    parameter PRESCALE       = 8192,
    parameter DEBOUNCE_TICKS = 127
    ) (
    input                  clk_i,
    input                  rst_n_i,
    input  wire [keys-1:0] keys_i,
    output reg  [keys-1:0] keys_o
    );

    localparam PRE_W = (PRESCALE > 1) ? $clog2(PRESCALE) : 1;
    localparam CNT_W = $clog2(DEBOUNCE_TICKS + 1);

    // Input synchronizer - switch pins are asynchronous to clk_i
    reg [keys-1:0] keys_meta;
    reg [keys-1:0] keys_sync;

    always @(posedge clk_i) begin
        keys_meta <= keys_i;
        keys_sync <= keys_meta;
    end

    // Shared debounce time base
    reg [PRE_W-1:0] prescaler;
    wire            tick = (prescaler == PRESCALE - 1);

    always @(posedge clk_i) begin
        if (rst_n_i == 1'b0 || tick)
            prescaler <= {PRE_W{1'b0}};
        else
            prescaler <= prescaler + 1'b1;
    end

    reg [CNT_W-1:0] counter [keys-1:0];

    integer key;

    // Debouncing Logic
    always @(posedge clk_i) begin
        for (key = 0; key < keys; key = key + 1) begin
            if (rst_n_i == 1'b0) begin
                counter[key] <= {CNT_W{1'b0}};
                keys_o[key]  <= keys_sync[key];
            end else begin
                if (counter[key] == DEBOUNCE_TICKS) begin
                    if (keys_o[key] != keys_sync[key]) begin
                        counter[key] <= {CNT_W{1'b0}};
                        keys_o[key]  <= keys_sync[key];
                    end
                end else if (tick) begin
                    counter[key] <= counter[key] + 1'b1;
                end
            end
        end
    end

endmodule
