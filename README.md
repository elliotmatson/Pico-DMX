# Pico-DMX

![example branch parameter](https://github.com/jostlowe/Pico-DMX/actions/workflows/Arduino-lint.yml/badge.svg?branch=arduino-port)

A library for outputting and inputting the DMX512-A lighting control protocol from a RaspberryPi Pico

## About

The RaspberryPi Pico provides an exciting new addition to the market on low-cost easy-to-use microcontroller board. The Pico has plenty of features making it particularly interesting for lighting control hackers:

* Lighting control software can potentially become quite large. A large chunk of flash is required to run networked protocols such as Art-Net and sACN (ANSI e1.31). The Pico has a whopping 2MB of flash at disposal, which leaves plenty of room for your networked DMX software.
* Storing DMX universes requires a lot of RAM (>512B for each full DMX frame). With other microcontrollers, RAM is a scarce resource and manipulating DMX universes can quickly consume the entire RAM of the microcontroller. The Pico has a solid 264kB of RAM, leaving plenty of room for dmx frame manipulation.
* Libraries for sending DMX on other microcontrollers tend to rely on either bit-banging a GPIO or a hack on a hardware UART to allow the UART to assert a sufficiently long BREAK and MAB at the beginning of each transfer. Bit-banging consumes a processor core for the duration of a transfer and is extremely sensitive to interrupts. Using a hardware UART is less computationally intensive, but most low-cost microcontrollers only have 1 or 2 hardware UARTs, limiting the number of parallel universes one can transmit or receive. The Programmable IO (PIO) modules of the Pico gives the possibility to create what is _basically_ hardware support for custom IO protocols (such as DMX). The Pico features 2 PIO modules with 4 State Machines each, making it possible to input or output up to 8 universes _in parallel_. Thats more universes than you can shake a proverbial stick at.
* As if the PIO wasn't impressive enough, the Pico has a powerful 10 channel DMA controller. In conjunction with the PIO, the DMA makes it possible to input or output entire DMX universes with only a handful of instructions.
* The Pico has a rather odd combination of 2 ARM Cortex M0+ cores running at speeds up to 133MHz. The dual core architecture could provide a huge benefit in processing DMX universes as computationally demanding data processing can be offloaded on the second core.

This library implements a DMX input and DMX output for the RPi Pico, leveraging its powerful DMA and PIO features. 


## Installation
The Pico-DMX library should be available in both the Arduino libraries and the PlatformIO libraries. Pico-DMX can also be used in `pico-sdk`-based projects.

The Pico-DMX library can also be manually added to any project in the same way as any other Arduino library. See the [Arduino guide](https://www.arduino.cc/en/guide/libraries) for more information. 

## Usage

The library is based around the `DmxOutput` and `DmxInput` classes, representing a single DMX output or input. 

### Outputting DMX
The `DmxOutput` class is simply instantiated, and requires no further arguments.

```C++
   DmxOutput myDmxOutput;
```

After instantiation, the DMX output must be assigned to a pin. The `.begin(uint pin)` method initializes the DMX output and binds it to a pin. To start a DMX output on GPIO1 on the Pico, call

```C++
   myDmxOutput.begin(1);
```

The library defaults to using `pio0` as the underlying PIO instance. If you want to use `pio1` as the underlying PIO, add the PIO you want to use as an argument

```C++
   myDmxOutput.begin(1, pio1);
```

Even though the DMX output is initialized and ready to party, the output is still idle, waiting for a universe to transmit. A universe is simply an array of `uint8_t`, with a maximum length of 513. The zero'th value in the array is the packet start code and should be set to 0 for ordinary DMX data packets. Let's make a universe with 3 channels (plus the start code), and set channel 1 to full.

```C++
   uint universe_length;
   uint8_t universe[universe_length + 1]; 
   universe[1] = 255;
```

To send the universe, call the `.write(...)` method to send the universe _wooshing_ down your DMX line. 

```C++
   myDmxOutput.write(universe, universe_length + 1);
```

The `.write(...)` method is not blocking, and executes immediately. To check the status of your DMX transmission, you can call `.busy()`to check if the DMX output is done transmitting your universe.

```C++
   while(myDmxOutput.busy()) {
        // Patiently wait, or do other computing stuff
   }
```

See the [examples](examples/) for complete examples on how to use the DMX output

### Inputting DMX
The library also enables DMX inputs through the `DmxInput` class. The DMX input can either read an entire universe or just a couple specified channels. Let's say the Pico controls a simple RGB LED, and we want to read the first three channels on the DMX universe to control our RGB LED. First, instantiate your DMX input, specifying what pin you want to use (GPIO 0 in our case), what channel you want to read from (channel 1), and how many channels you want to read (3 channels in total)

```C++
   DmxInput myDmxInput;
   uint dmx_pin = 0;
   uint start_channel = 1;
   uint num_channels = 3;
   myDmxInput.begin(dmx_pin, start_channels, num_channels);
```

The DMX Input is now ready to receive your DMX data. Before we start receiving DMX data, we want to create a buffer where we can keep our received DMX channels:

```C++
   uint8_t buffer[num_channels]; 
```

Use the `.read(...)` method to read the 3 channels for our RGB fixture into our buffer.

```C++
   myDmxInput.read(buffer);
```

The `.read(...)` method blocks until it receives a valid DMX packet. Much like the `DmxOutput`, the zero'th channel in the DMX packet is the start code. You should always check that the start code is the one defined for DMX (zero) to ensure you have a valid DMX packet, unless you are messing around with other protocols such as RDM, in which case check it is their valid start codes.

As an alternative to the blocking `.read(...)` method, you can also start asynchronous buffer updates via the `.read_async(...)` method. This way, the buffer is automatically updated when DMX data comes in.
Optionally, you can also pass a pointer to a callback-function that will be called everytime a new DMX frame has been received, processed and has been written to the buffer. This callback-function will be called with one parameter which is the instance that has received the new data. This way, you can use one callback-function to react on data from multiple universes. See this example below:

```C++
   void __isr dmxDataRecevied(DmxInput* instance) {
     // A DMX frame has been received :-)
     // Toggle some LED, depending on which pin the data arrived
     uint ledPin = instance->pin() + 8;
     digitalWrite(ledPin, !digitalRead(ledPin));
   }

   myDmxInput.read_async(buffer, dmxDataRecevied);
```

### A note on DMX interfaces sending "partial universes" (= fewer channels)
There are multiple universes that can be configured to send less than 512 channels per frame. Some interfaces do this automatically without an option to configure this feature.

The reason why this is done is that if not all channels of a universe are in use, one can send a "shorter" frame but send this frame more often per second (= increase the refresh rate). The specification of DMX512 allows this.

The problem arises if `start_channel + num_channels` is larger than the number of channels sent by the interface since the code of DmxInput waits for this specific amount of channels until the callback is being triggered. So if the amount of channels arriving at the input, the callback will be triggered at a later point in time, not at the end of a DMX frame.

## Voltage Transceivers
The Pico itself cannot be directly hooked up to your DMX line, as DMX operates on RS485 logic levels, 
which do not match the voltage levels of the GPIO pins on the Pico. 

Fortunately TLL to RS485 transceivers are easily available. Simple transceiver modules can be bought through online retailers for only a couple of dollars. These tend to use the MAX485 series of voltage level transceivers, which work great for most purposes. If you're planning to implement DMX on an industrial level, your device should have some kind of EMC protection. Many RS485 transceivers are available that have galvanic isolation between the TLL side and the RS485 side. These should be the preferred option.

## Modifying .pio files

Unfortunately, the mbed-based Arduino core has no native support for compiling .pio files yet.
However, we can compile them manually using `pioasm`. The result is a header file that can be included into the arduino sources.

Manual compilation requires cloning the pico sdk, compiling the `pioasm` tool, and running it like so:
`pioasm src/DmxInput.pio src/DmxInput.pio.h`


## Many outputs from one state machine: `DmxStreamOutput`

`DmxStreamOutput` drives up to 32 pins from a single PIO state machine and DMA channel. Each 32-bit word is one 4 µs DMX bit-time across every pin. You build frames with `DmxFrameEncoder`, which lets every line have its own data, channel count, break and MAB, and optional transceiver DE pins in the same word.

```cpp
#include <DmxFrameEncoder.h>
#include <DmxStreamOutput.h>

DmxStreamOutput out;
uint8_t universes[12][513];  // [0] = start code
dmxstream::Lane lanes[12];
uint32_t frame[dmxstream::maxFrameWords()];

void setup() {
    out.begin(/*pinBase=*/1, /*pinCount=*/12, /*idleMask=*/0xFFF);
    for (int i = 0; i < 12; i++) {
        lanes[i].enabled = true;
        lanes[i].dataBit = i;
        lanes[i].slots = universes[i];
        lanes[i].slotCount = 513;
    }
}

void loop() {
    if (!out.busy()) {
        size_t n = dmxstream::encodeFrame(frame, sizeof frame / 4, lanes, 12);
        out.send(frame, n);  // double-buffer for back-to-back frames
    }
}
```

- `busy()` stays true until the last bit-time has left the pins, so a new frame never cuts off the previous one. `DmxOutput::busy()` returns as soon as the DMA finishes.
- The state machine is never restarted between frames, because break and MAB are part of the data.
- DE: an enabled lane holds DE high for the whole frame; `releaseDe` drops it right after the lane's packet (RDM requests). A disabled lane drops DE, unless `holdDe` is set: then its driver stays on with the line at mark, e.g. for an output sitting a frame out or a pin lent to a pixel stream. The last word of a frame stays on the pins until the next one.
- With no lane enabled, `encodeFrame` writes idle words: one, or `minWords` when given.
- The encoder is plain C++. Its host test runs with:

```bash
c++ -std=c++17 -I src test/host/test_frame_encoder.cpp src/DmxFrameEncoder.cpp -o /tmp/t && /tmp/t
```

- On a 150 MHz RP2350, encoding 12 full universes takes about 2.8 ms per frame (22.8 ms frame time).
- Requires pico-sdk 2.x (RP2040 or RP2350, including GPIOs above 31 on the RP2350B).

## Receiving any packet length: `DmxReceiver`

`DmxReceiver` runs one DMX input on one PIO state machine. DMA writes into an endless ring buffer, so no interrupt is needed. `poll()` feeds the received words to a `DmxFrameParser`, which splits them into packets at each break. Packets of any length (1–513 slots) work. Upstream `DmxInput` waits for a fixed byte count, so shorter packets never complete and later ones land misaligned.

- **Breaks:** the PIO program reports a break by how long the line stays low, not by the byte value. A receiver that joins mid-packet and is out of sync still sees the next break.
- **Bad packets:** packets with framing errors are counted and dropped.
- **Shared pins:** with `claimPin = false`, the receiver can listen to a pin that something else drives, which is useful for loopback tests.
- **Host test:**

```bash
c++ -std=c++17 -I src test/host/test_frame_parser.cpp src/DmxFrameParser.cpp src/DmxFrameEncoder.cpp -o /tmp/p && /tmp/p
```
