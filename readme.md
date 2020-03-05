nRF24 packet format (in progress)

payload 47 bit, currently more because of data types overhead and packet counter added

temperature
11 bit [-40..+85], 1251 of 2048 steps; resolution 0.1 degree; 0_-40.0, 400_0, 1250_+85
possible to reduce by 1 bit [-40..+62.3] or [-30..+72.3]
or by 2 bits [0..+51.1]

humidity
7  bit [0..100] %, 100 of 127 steps; resolution 1%, 0_0%, 100_100%, 127_invalid value or disabled

pressure
17 bit [300..1100] hPa, 80k of 131072 steps; resolution 1Pa; 30000_300hpa, 110000_1100hPa
possible to reduce by 1 bit [44465..110000], (300 hPa is too low for human either, 44465 Pa is ~6430m)

batt voltage
12 bit [1000..5095] mV, 4096 of 4096 steps; resolution - 1mV; 0_1000mV, 4095_5095mV
possible to reduce by 1 bit [1800..3847] mV (most of sensors and arduino itself not running below 1800 mV either, 3848 mV - is more than enough for two AA/AAA or coin cell)