# Nyan Keys - An FPGA Keyboard Bitstream

A Lattice ICE40HX4K parallel keys interface for mechanical keyboards with integrated debouncing cores per key. This IP 
essentially performs the following functions 
 1. Consumes the state of every keys in parallel once per clock cycle. (12MHz)
 2. Keeps track of the key state changes and debouncing [eagerly](https://github.com/qmk/qmk_firmware/blob/master/docs/feature_debounce_type.md).
 3. Sends over state changes to the SPI slave as they occour.
 4. Sends over a state ever 50 milliseconds regardless.

## FPGA IP

FPGA IP is designed to be build using [Yosys](https://github.com/YosysHQ/yosys) and [Nextpnr](https://github.com/YosysHQ/nextpnr). This parallel keys interface has been validated on the Lattice ICE40HX4K 144TQFP FPGAs.
The IP is designed and seperated into 3 main parts.

 - Key switch cores (keys.v)
 - SPI frame transmitter, master MODE 0 CPOL=0, CPHA=0 (spi_frame_tx.v)
 - Frame scheduler, ack handshake and clocking (spi_keys.v)

### Key Switching Core
Each key (switch) in the nyan keys physical design will generate one key switch 'core'. The key switch core is the programmable logic that is used to read the keys state and perform debouncing as well as produce an output state that is passed to a SPI slave device. This IP acts as the master.

All mehcnial switches have some form of bounce to them. The term bounce is used to refer to the total time a singal takes
to settle. _As a real world example when you press a Cherry MX Blue/Green switch it could take up to 5ms before the singal has
stopped bouncing between high and low._ Removing debouncing logic on a keyboard would have the effect of the user seeing
double key presses.

In it's simplest form this design outputs a one hot vector that represents the states of each key as 0 or 1 (_high_ or _low_) and send that over a SPI interface as a master when either of the following conditions occours.

 - The array of keys changes state and that keys debounce timer is not active 
 - 50ms has passed since the last broadcast of a keys state.
 
outputs of the Lattice Ice40hx series ICs.

 - __Key state__ - Represents the physical key. [Pressed/Released]
 - __Logic Level__ - The logic level of net to the key itself.
 - __Direction State__ - Internal debounced logic level of the one hot key state register.

| Key State | Logic Level | Direction State |
| --------- | ----------- | --------------- |
| Depressed | Low         | Low             |
| Released  | High        | High            |


The actual mechanism for debouncing is incredibly simple. It involves an up counter that locks out the state change of a key until it has reached a threshold value. Compared to the original Nyan Keys FPGA design, which had an up and down counter, using an up-only counter allows for the design to have a smaller footprint. It also enables instant response to switch state changes as long as the debounce period has elapsed.

Every key input first passes through a 2-FF synchronizer. A single shared prescaler generates a debounce tick every `DEBOUNCE_PRESCALE` (8192) clocks, and each key has a small 7-bit counter of ticks. After reset the counters start at 0, so key changes are locked out for one debounce period. Once a key's counter reaches `DEBOUNCE_TICKS` (127) its lockout has expired and the next change is applied immediately. The counter then resets to 0, and further changes (bounce) are ignored until it reaches `DEBOUNCE_TICKS` again. With the defaults that is ~13.3ms at 78MHz. The whole design runs on the single 78MHz PLL clock.

The bit vector size depends on the number of keys on a keyboard. A 61-key, 60% board needs 61 bits, but SPI transfers whole bytes, so 8 bytes (64 bits) of key state are sent per frame.

### SPI Protocol

The FPGA pushes frames to the STM32 ([nyan-keys-stm32-firmware](https://github.com/portlandhodl/nyan-keys-stm32-firmware) `fpga-push` branch), which runs SPI2 as an RX-only slave.

| Byte | Content |
| ---- | ------- |
| 0    | Sync `0xA5` |
| 1..8 | Key state, byte 1 = keys[7:0]. 1 = released, 0 = pressed. Unused pad bits are 1 |
| 9    | CRC-8/SMBUS (poly 0x07, init 0x00) over bytes 1..8 |

 - SPI mode 0 (CPOL=0, CPHA=0), MSB first, 80 bits back-to-back with no gaps. SCLK = 78MHz / 4 = 19.5MHz (`HALF_BIT_CLKS`).
 - A frame is sent as soon as the debounced state differs from the last frame sent, immediately after reset, every 50ms, and again if no ack arrives within 1ms.
 - After each good frame the MCU pulses `spi_keys_ack`. The FPGA captures the rising edge asynchronously, so any pulse width works. Edges during a frame, or a line stuck high, never count as an ack.
 - There is no chip select, so alignment is kept by the MCU instead. It resets SPI2 before arming each frame, rejects bad sync/CRC frames without acking (so the FPGA resends), and discards a partial frame that stalls for 50µs (a lost SCLK edge).
 - Latency: key edge to the last SCLK edge of the frame is 322 clocks (4.13µs). That is 2 synchronizer, 1 debounce and 2 scheduling clocks plus the 320 clock frame.

| Pin | Signal | MCU |
| --- | ------ | --- |
| 28  | `rstn_g_i` (in) | PC0 `keys_fpga_resetn`, held low until the MCU is ready to receive |
| 29  | `spi_mosi_g_o`  | PC1 SPI2_MOSI |
| 31  | `spi_keys_ack` (in) | PC2 `keys_ack` |
| 32  | `spi_clk_g_o`   | PB10 SPI2_SCK |

## Building and Testing

Requires [Yosys](https://github.com/YosysHQ/yosys), [nextpnr-ice40](https://github.com/YosysHQ/nextpnr), [IceStorm](https://github.com/YosysHQ/icestorm) and [Icarus Verilog](https://github.com/steveicarus/iverilog) (12+).

```
make test             # run all self-checking testbenches
make sim-tb_spi_keys  # run one testbench and dump build/tb_spi_keys.vcd
make                  # synthesize, place & route -> build/spi_keys.bin
make prog             # program with iceprog
```

| Testbench               | Covers |
| ----------------------- | ------ |
| `sim/tb_keys.v`         | Debouncer: reset load, post-reset lockout, 3 clock eager latency, bounce rejection, random chatter with continuous lockout/liveness checks |
| `sim/tb_spi_frame_tx.v` | Frame transmitter: exact bit stream (sync, data, CRC checked against the CRC-8/SMBUS check value), no gaps, SCLK period, MOSI setup/hold, at 19.5MHz and 39MHz |
| `sim/tb_spi_keys.v`     | Full design with an MCU model that mirrors the firmware: initial frame after reset, press latency, refresh, bounce, changes mid-frame, 2ns and 1µs ack pulses, MCU busy, stuck-high ack, missed frames, lost and extra SCLK edges, random typing with random faults |

`make crosscheck FW=../nyan-keys-stm32-firmware` runs frames captured from the transmitter testbench through the firmware's own `NyanKeysFrameValid()`. It checks that every FPGA frame validates and that every single-bit corruption is rejected.

Debounce and refresh periods are parameters, so the testbenches shorten them to keep simulations fast. In simulation the PLL is bypassed (`__ICARUS__`).

`constraints/clocks.py` constrains the 12MHz input. nextpnr carries the constraint through the PLL, so the 78MHz core clock is timing checked. Without it nextpnr only checks against 12MHz.

### Future

One of the major features that could be implemented at a later time would be the use of in memory per switch debounce counter thresholds.
This means that instead of having a global state of all switches are debounced in at count value 8'bxxxxxxxx. The user could tune each switch
to the lowest possible latency before bouncing occours. This would work extremely well for designs that have multiple switch types.
