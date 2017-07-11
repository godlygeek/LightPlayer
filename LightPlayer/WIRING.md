## High Level

| Quaternary Lights Shift Register | Arduino Uno w/ SeedStudio SD Shield | Adafruit Simple RF M4 Receiver | APA106 Strip | DC 12v to DC 5v converter |
| -------------------------------- | ----------------------------------- | ------------------------------ | ------------ | ------------------------- |
|                                  | A0                                  | D0                             |              |                           |
|                                  | A1                                  | D1                             |              |                           |
|                                  | A2                                  | D2                             |              |                           |
|                                  | A3                                  | D3                             |              |                           |
|                                  | D6                                  |                                | DIN          |                           |
| SRCLK (Clock)                    | D8                                  |                                |              |                           |
| RCLK (Latch)                     | D9                                  |                                |              |                           |
| SER (Serial)                     | D10                                 |                                |              |                           |
| OE (OutputEnable)                | GND                                 | GND                            | GND          | -                         |
| VCC                              | 5V                                  | +5V                            | VDD          | +                         |

## APA106 Strip

| 1st APA106 | 2nd APA106      | N-1th APA106 | Nth APA106  |
| ---------- | --------------- | ------------ | ----------- |
| VDD [1]    | VDD [1]         | VDD [1]      | VDD [1]     |
| GND [1]    | GND [1]         | GND [1]      | GND [1]     |
| DIN        |                 |              |             |
| DOUT       | DIN             |              |             |
|            | DOUT            | DIN          |             |
|            |                 | DOUT         | DIN         |
|            |                 |              | DOUT        |

[1] Each APA106 is wired in parallel with a 0.1uF decoupling capacitor between VDD and GND.
