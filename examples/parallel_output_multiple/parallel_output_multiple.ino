
/*
 * Copyright (c) 2021 Jostein Løwer
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Description:
 * Outputs 8 DMX universes simultaneously from a single PIO state machine
 * using GPIO pins 0-7.
 *
 * Each universe shares the same channel data in this example, but in practice
 * each universe[] array can contain independent channel values.
 *
 * Pin mapping:
 *   GPIO 0 -> universe 0
 *   GPIO 1 -> universe 1
 *   ...
 *   GPIO 7 -> universe 7
 *
 * Wiring: connect each pin through an RS-485 transceiver to a separate DMX line.
 *
 * NOTE: The DmxOutput{2,4,8,16,32}.pio.h headers must be generated from the
 * .pio source files in extras/ before compiling:
 *   pioasm extras/DmxOutput8.pio src/DmxOutput8.pio.h  (and similarly for 2/4/16/32)
 */

#include <Arduino.h>
#include <DmxOutputMultiple.h>

// Use 8 simultaneous DMX universes.
// Supported values: 2, 4, 8, 16, 32
DmxOutputMultiple<8> dmxOutput;

#define NUM_UNIVERSES 8
#define UNIVERSE_LENGTH 512

// One byte array per universe (start code + channel data)
uint8_t universeData[NUM_UNIVERSES][UNIVERSE_LENGTH + 1];

// Pointer array required by DmxOutputMultiple::write()
uint8_t *universes[NUM_UNIVERSES];

void setup()
{
    // Build the pointer array
    for (int u = 0; u < NUM_UNIVERSES; u++)
    {
        universes[u] = universeData[u];

        // Start code = 0x00 (standard DMX)
        universeData[u][0] = 0x00;
    }

    // Set channel 1 of universe 0 to full, all others to 0
    universeData[0][1] = 255;

    // Start the 8 DMX outputs on GPIO pins 0-7 (all on pio0)
    DmxOutputMultiple<8>::return_code result = dmxOutput.begin(0, pio0);
    if (result != DmxOutputMultiple<8>::SUCCESS)
    {
        // Initialisation failed — halt and blink the LED
        pinMode(LED_BUILTIN, OUTPUT);
        while (true)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }
}

void loop()
{
    // Send all 8 universes in parallel
    dmxOutput.write(universes, UNIVERSE_LENGTH + 1);

    // Wait for the frame to finish transmitting before writing the next one
    while (dmxOutput.busy())
    {
        // Optionally do other work here
    }

    // Small delay between frames (optional; DMX allows back-to-back frames)
    delay(1);
}
