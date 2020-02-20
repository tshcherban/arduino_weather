nRF24 packet format (in progress)

11 bit -  temperature (-40..85, 1250 of 2048 steps; resolution 0.1 degree; 0_-40.0, 400_0, 1250_+85.0)
            possible to reduce by 1 bit lowering upper range to +62.4 (or lower to -30 and upper to +72.4)
            or by 2 bits for range 0..+51.2

7  bit -  humidity (0..100, 100 of 127 steps; 0_0%, 100_100%, 127_invalid value or disabled)

17 bit -  pressure (300hPa-1100hPa, 80k of 131072 steps; resolution 1Pa; 30000_300hpa, 110000_1100hPa)
            possible to reduce by 1 bit reducing lower range, 300 hPa is too low for human either

12 bit -  batt voltage (1000-5096mV, 4096 of 4096 steps; resolution - 1mV; 0_1000mV, 4096_5096mV; )
            possible to reduce by 1 bit if rise lower limit to 1800 mV (most of sensors and arduino itself not running at this voltage either) and reducing upper range to 3848 mV, which is more than enough for two AA/AAA or coin cell
