`timescale 1ns/1ps
`default_nettype none

/**
 * Unit test for the eager debouncer (rtl/keys.v)
 *
 * Invariants checked on every clock for every key:
 *  1. keys_o only ever takes the value of the synchronized input.
 *  2. After keys_o changes it is locked for at least (TICKS-1)*PRESCALE clocks.
 *  3. Liveness - keys_o never disagrees with a settled input for longer than
 *     the lockout window plus the synchronizer latency.
 * Directed checks:
 *  - Eager response: an unlocked key reaches keys_o in exactly 3 clocks.
 *  - Bounce rejection: chatter inside the lockout window is ignored.
 */
module tb_keys;
    localparam KEYS     = 8;
    localparam PRESCALE = 4;
    localparam TICKS    = 6;
    localparam LOCK_MIN = (TICKS - 1) * PRESCALE;
    localparam LOCK_MAX = TICKS * PRESCALE + 3;

    reg             clk = 1'b0;
    reg             rst_n = 1'b0;
    reg  [KEYS-1:0] keys_i = {KEYS{1'b1}};
    wire [KEYS-1:0] keys_o;

    integer errors = 0;
    integer cycle  = 0;

    keys #(.keys(KEYS), .PRESCALE(PRESCALE), .DEBOUNCE_TICKS(TICKS)) dut (
        .clk_i  (clk),
        .rst_n_i(rst_n),
        .keys_i (keys_i),
        .keys_o (keys_o)
    );

    always #5 clk = ~clk;

    // Input delayed to line up with keys_o as seen by this monitor
    // (2 synchronizer stages + the keys_o register)
    reg [KEYS-1:0] in_d1, in_d2, in_d3;
    reg [KEYS-1:0] keys_o_prev;
    integer        last_change  [0:KEYS-1];
    integer        disagree_for [0:KEYS-1];
    integer        k;

    initial for (k = 0; k < KEYS; k = k + 1) begin
        last_change[k]  = -1000000;
        disagree_for[k] = 0;
    end

    always @(posedge clk) begin
        cycle       <= cycle + 1;
        in_d1       <= keys_i;
        in_d2       <= in_d1;
        in_d3       <= in_d2;
        keys_o_prev <= keys_o;
        if (rst_n) begin
            for (k = 0; k < KEYS; k = k + 1) begin
                if (keys_o[k] !== keys_o_prev[k]) begin
                    if (keys_o[k] !== in_d3[k]) begin
                        $display("[%0d] ERROR key %0d changed to %b but synchronized input was %b",
                                 cycle, k, keys_o[k], in_d3[k]);
                        errors = errors + 1;
                    end
                    if (cycle - last_change[k] < LOCK_MIN) begin
                        $display("[%0d] ERROR key %0d changed again after %0d clocks (min %0d)",
                                 cycle, k, cycle - last_change[k], LOCK_MIN);
                        errors = errors + 1;
                    end
                    last_change[k] = cycle;
                end
                if (keys_o[k] !== in_d3[k])
                    disagree_for[k] = disagree_for[k] + 1;
                else
                    disagree_for[k] = 0;
                if (disagree_for[k] > LOCK_MAX && in_d2[k] === in_d3[k]) begin
                    $display("[%0d] ERROR key %0d stuck at %b for %0d clocks",
                             cycle, k, keys_o[k], disagree_for[k]);
                    errors = errors + 1;
                    disagree_for[k] = 0;
                end
            end
        end
    end

    task tick(input integer n);
        repeat (n) begin
            @(posedge clk);
            #1;
        end
    endtask

    integer i, t0;

    initial begin
        $dumpfile("tb_keys.vcd");
        $dumpvars(0, tb_keys);

        // Reset - output must load the current (synchronized) input
        keys_i = 8'b1010_0101;
        tick(10);
        @(negedge clk) rst_n = 1'b1;
        tick(1);
        if (keys_o !== 8'b1010_0101) begin
            $display("ERROR reset value %b", keys_o);
            errors = errors + 1;
        end

        // Changes during the post-reset lockout must be deferred
        @(negedge clk) keys_i[0] = 1'b0;
        tick(3);
        if (keys_o[0] !== 1'b1) begin
            $display("ERROR key 0 changed during post-reset lockout");
            errors = errors + 1;
        end
        tick(LOCK_MAX);
        if (keys_o[0] !== 1'b0) begin
            $display("ERROR key 0 not updated after lockout");
            errors = errors + 1;
        end

        // Eager response: an idle (unlocked) key reaches the output in 3 clocks
        tick(LOCK_MAX);
        @(negedge clk) keys_i[2] = 1'b0;
        t0 = cycle;
        while (keys_o[2] !== 1'b0 && cycle - t0 < 20) tick(1);
        if (cycle - t0 != 3) begin
            $display("ERROR eager latency %0d clocks, expected 3", cycle - t0);
            errors = errors + 1;
        end

        // Bounce rejection: chatter right after the press is ignored
        for (i = 0; i < LOCK_MIN - 4; i = i + 1) begin
            @(negedge clk) keys_i[2] = ~keys_i[2];
            if (keys_o[2] !== 1'b0) begin
                $display("ERROR bounce leaked through on key 2");
                errors = errors + 1;
            end
        end
        @(negedge clk) keys_i[2] = 1'b0;
        tick(LOCK_MAX);

        // Random chatter on all keys - invariants are checked by the monitor
        for (i = 0; i < 20000; i = i + 1) begin
            @(negedge clk);
            if (($random & 7) == 0)
                keys_i = keys_i ^ (1 << ($unsigned($random) % KEYS));
        end
        tick(LOCK_MAX * 2);
        if (keys_o !== keys_i) begin
            $display("ERROR final state %b != input %b", keys_o, keys_i);
            errors = errors + 1;
        end

        if (errors == 0) $display("PASS tb_keys");
        else             $display("FAIL tb_keys: %0d error(s)", errors);
        $finish;
    end

endmodule

`default_nettype wire
