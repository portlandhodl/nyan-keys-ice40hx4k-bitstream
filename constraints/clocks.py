# nextpnr pre-pack timing constraints.
# The 12MHz input constraint is propagated through SB_PLL40_CORE, so the
# 78MHz core clock (clk_g_int) is constrained automatically.
ctx.addClock("clk_g_i", 12)
