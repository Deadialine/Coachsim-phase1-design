# Channel map and physical qualification plan

The simulation configuration is frozen as `coachsim-sim-v2.0`. The physical study configuration remains provisional until one-channel hardware proof and a four-channel qualification run succeed.

| Logical channel | Rate | Unit | Physical candidate | Qualification required |
| --- | --- | --- | --- | --- |
| emg_ch1 | 2000 Hz | raw ADC count | MyoWare RAW, volar landmark position | Confirm module output, ADC range/reference, offset and gain |
| emg_ch2 | 2000 Hz | raw ADC count | MyoWare RAW, next quarter of ring toward radial side | Confirm actual module count and placement |
| emg_ch3 | 2000 Hz | raw ADC count | MyoWare RAW, opposite ring position | Confirm actual module count and placement |
| emg_ch4 | 2000 Hz | raw ADC count | MyoWare RAW, final quarter position | Confirm actual module count and placement |
| ax,ay,az | 100 Hz | g | MPU-6050 at 0x69 | Check ranges, scale and rigid dorsal hand orientation |
| gx,gy,gz | 100 Hz | degrees/s | Same MPU-6050 | Check ranges, bias and axis mapping |
| t0_unix | session start | Unix seconds | DS1307 at 0x68 | Read valid time and oscillator status; document UTC setting |

The plan establishes ownership of a MyoWare sensor, not possession of four modules. ADC model, exact ESP32 variant/pins and analogue range have not been supplied. Do not present a guessed pinout as a build-ready electrical design. Select an ADC capable of the measured required aggregate rate and document channel skew. The simulation's 12-bit 0–4095 range is a software assumption, not a verified ADC specification.

The existing Arduino record uses Mega D20/D21 and effective pull-ups to 5 V. That wiring cannot simply be copied to ESP32 GPIO. Resolve voltage compatibility and level translation using the actual ESP32 and breakout specifications, then measure idle bus voltages. Keep MPU AD0 HIGH for address 0x69; DS1307 remains at 0x68. Archive a wiring diagram and independent measurement before participant connection.

Use battery-isolated participant-connected sensors and follow [MyoWare setup guidance](https://learn.sparkfun.com/tutorials/getting-started-with-the-myoware-20-muscle-sensor-ecosystem/all). Verify whether USB, oscilloscope or charger connections defeat isolation. A simulated run cannot certify electrical protection.

Physical freeze record must include board and ADC IDs, channel count, sensor serials, firmware commit, gain/reference/range, measured rate and skew, anti-alias test, wiring/power arrangement, placement diagram, date and responsible operator. Store deviations instead of silently reducing channels to rescue a run.
