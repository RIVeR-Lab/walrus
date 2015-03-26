int16[10] temp_sense        # in tenths of a deg C
uint8 water_sense           # bit0 = Water Sensor 1, bit1 = Water Sensor 2 .... bit5 = Water Sensor 6
uint16[2] tension           # raw ADC output (0-4095)
uint32 pressure             # in Pa
int16 board_temp            # in hudredths of a deg C
uint16 humidity             # in hudredths of a %
uint8 batt_present          # bit0 = Battery 1 Present, bit1 = Battery 2 Present, bit2 = Battery 3 Present, bit3 = Battery 4 Present
int16[4] ucell_current      # in mA   
int16[4] ucell_voltage      # in mV
int16[4] ucell_charge       # in %
int16[4] ucell_temp         # in hundredths of a deg C
int16[4] ucell_avgcurr      # in mA
int16[4] lcell_current      # in mA
int16[4] lcell_voltage      # in mV
int16[4] lcell_charge       # in %
int16[4] lcell_temp         # in hundredths of a deg C
int16[4] lcell_avgcurr      # in mA
