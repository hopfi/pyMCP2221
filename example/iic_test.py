import usb.core
import usb.util
import mcp2221
import time
import logger
import ctypes


dig_T1 = 28181
dig_T2 = 27029
dig_T3 = 50
def BME280_compensate_T(adc_T):
    # Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
    # t_fine carries fine temperature as global value 
    # Calculates T in counts? To get real Temperature value multiply result by resolution of 0.01degC
    var1 = ((((adc_T>>3) - (dig_T1<<1))) * (dig_T2)) >> 11
    var2 = (((((adc_T>>4) - (dig_T1)) * ((adc_T>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
    t_fine = var1 + var2
    T  = (t_fine * 5 + 128) >> 8
    return T, t_fine


dig_P1=36835
dig_P2=-10440
dig_P3=3024
dig_P4=8789
dig_P5=-128
dig_P6=-7
dig_P7=12300
dig_P8=-12000
dig_P9=5000
def BME280_compensate_P(adc_P, t_fine):
    # Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits). 
    # Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    var1 = (t_fine) - 128000
    var2 = var1 * var1 * dig_P6
    var2 = var2 + ((var1*dig_P5)<<17)
    var2 = var2 + ((dig_P4)<<35)
    var1 = ((var1 * var1 * dig_P3)>>8) + ((var1 * dig_P2)<<12)
    var1 = ((((1)<<47)+var1))*(dig_P1)>>33
    if (var1 == 0):
        return 0; # avoid exception caused by division by zero
    p = 1048576 - adc_P
    p = (((p<<31)-var2)*3125)/var1
    var1 = (dig_P9 * (int(p)>>13) * (int(p)>>13)) >> 25
    #p1 = ctypes.c_uint.from_buffer(ctypes.c_float(p)).value
    #var1 = (dig_P9 * (p1>>13) * (p1>>13)) >> 25
    var2 = int(dig_P8 * p) >> 19
    #p2 = ctypes.c_uint.from_buffer(ctypes.c_float(dig_P8 * p)).value
    #var2 = p2 >> 19
    p = (int(p + var1 + var2) >> 8) + ((dig_P7)<<4)
    #p3 = ctypes.c_uint.from_buffer(ctypes.c_float(p + var1 + var2)).value
    #p = (p3 >> 8) + ((dig_P7)<<4)
    return p


dig_H1=75
dig_H2=341
dig_H3=0
dig_H4=371
dig_H5=50
dig_H6=30
def bme280_compensate_H(adc_H, t_fine):
    # Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits). 
    # Output value of “47445” represents 47445/1024 = 46.333 %RH 
    v_x1_u32r = t_fine - 76800
    v_x1_u32r = (((((adc_H << 14) - (dig_H4 << 20) - (dig_H5 * v_x1_u32r)) + 16384) >> 15) * (((((((v_x1_u32r * dig_H6) >> 10) * (((v_x1_u32r * dig_H3) >> 11) + 32768)) >> 10) + 2097152) * dig_H2 + 8192) >> 14))
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (dig_H1)) >> 4))
    # v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r)
    if v_x1_u32r < 0:
        v_x1_u32r = 0
    else:
        v_x1_u32r = v_x1_u32r

    #v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r)
    if v_x1_u32r > 419430400:
        v_x1_u32r = 419430400
    else:
        v_x1_u32r = v_x1_u32r

    return (v_x1_u32r>>12)


wl = logger.wave_logger()

gpio0_off = {"value": 0, "dir": "OUT", "mode": "GPIO"}

gpio_config0 = [[True, 0, True, "OUT"], \
        [True, 0, True, "IN"], \
        [False, 1, False, "OUT"], \
        [False, 0, False, "IN"]]
gpio_config1 = [[True, 1, True, "OUT"], \
        [True, 0, True, "IN"], \
        [False, 1, False, "OUT"], \
        [False, 0, False, "IN"]]

afboard = mcp2221.MCP2221(0x04D8, 0x00DD)
afboard.connect_usb()
afboard.config_usb([0,0])
afboard.set_gpio_sram(gpio0_off)
#afboard.set_GPIOs(gpio_config1)

i2c_write = [   0x90, 0x00, 0x0A, 0x60,
                0xAB, 0xCD, 0xEF, 0x23,
                0xDD, 0xEE, 0xAA, 0xDD,
                0xFF, 0xFF, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00 ]

i2c_cancel = [  0x10, 0x00, 0x10, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00 ]

# print(afboard.dev.write(0x03, i2c_write, 1000))
# print(afboard.dev.read(0x83, 64, 1000))
# print(afboard.dev.write(0x03, i2c_cancel, 1000))
# print(afboard.dev.read(0x83, 64, 1000))
# print(afboard.dev.write(0x03, i2c_cancel, 1000))
# print(afboard.dev.read(0x83, 64, 1000))

addr = 0x76 #0x77 #0x76
id_data = [0xD0]
weathData_regs = [0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE]

data = [0xF4]

print(afboard.get_status())
afboard.set_parameter(i2c_set_speed=True, i2c_speed=40)
#afboard.i2c_cancel_transaction()
# print(afboard.i2c_write(addr, id_data, len(id_data), mcp2221.MCP2221.C_I2C_MODE_NORMAL))
# print(afboard.i2c_read(addr, 1, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
# print(afboard.i2c_read_data())
# print(afboard.i2c_read(addr, 1, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
# print(afboard.i2c_read_data())

## Read trimming parameters
# dig_T1, dig_T2, dig_T3
# Result is: 15, 6E, 95, 69, 32, 00 
# unsigned dig_T1=0x6E15
# signed dig_T2=0x6995
# signed dig_T3=0032
data = [0x88]
print(afboard.i2c_write(addr, data, len(data), mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read(addr, 6, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read_data())

# dig_P1 - dig_P9
# Result is: E3, 8F, 38, D7, D0, 0B, 55, 22, 80, FF, F9, FF, 0C, 30, 20, D1, 88, 13
# unsigend dig_P1=x8FE3
# sigend dig_P2=xD738
# sigend dig_P3=x0BD0
# sigend dig_P4=x2255
# sigend dig_P5=xFF80
# sigend dig_P6=xFFF9
# sigend dig_P7=x300C
# sigend dig_P8=xD120
# sigend dig_P9=x1388
data = [0x8E]
print(afboard.i2c_write(addr, data, len(data), mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read(addr, 18, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read_data())

# dig_H1 - dig_H6
# Result is: 4B
# unsigned dig_H1 = x4B
data = [0xA1]
print(afboard.i2c_write(addr, data, len(data), mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read(addr, 1, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read_data())
# Result is: 55, 01, 00, 17(170), 23(3)(2), 03, 1E
# signed dig_H2=x0155
# unsigned dig_H3=x00
# signed dig_H4=x173
# signed dig_H5=x32
# signed dig_H6=x1E
data = [0xE1]
print(afboard.i2c_write(addr, data, len(data), mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read(addr, 7, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
print(afboard.i2c_read_data())

data = [0xD0]
for i in range(1,140,1):
    data.append(i)

data1 = data[0:60]
data2 = data[61:120]
data3 = data[121:]

while True:
    print(afboard.i2c_write(addr, data1, len(data), mcp2221.MCP2221.C_I2C_MODE_REPSTART))
    print(afboard.i2c_write(addr, data2, len(data), mcp2221.MCP2221.C_I2C_MODE_REPSTART))
    print(afboard.i2c_write(addr, data3, len(data), mcp2221.MCP2221.C_I2C_MODE_NORMAL))
    
    #print(afboard.i2c_read(addr, 18, mcp2221.MCP2221.C_I2C_MODE_NORMAL))
    #print(afboard.i2c_read_data())
    #time.sleep(0.1)