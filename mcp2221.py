import usb.core
import usb.util
import serial

class MCP2221:

    C_MESSAGE_LEN = 64
    C_TIMEOUT = 1000

    C_CC_STATUS = 0x10
    C_CC_SET_PARAM = 0x10
    C_CC_RD_FLASH = 0xB0
    C_CC_WR_FLASH = 0xB1
    C_CC_FLASH_PW = 0xB2
    C_CC_I2C_WRITE = 0x90
    C_CC_I2C_WRITE_REPSTART = 0x92
    C_CC_I2C_WRITE_NOSTOP = 0x94
    C_CC_I2C_READ = 0x91
    C_CC_I2C_READ_REPSTART = 0x93
    C_CC_I2C_READ_DATA = 0x40
    C_CC_SET_SRAM = 0x60
    C_CC_GET_SRAM = 0x61
    C_CC_SET_GPIO = 0x50
    C_CC_GET_GPIO  = 0x51
    C_CC_RESET = 0x70
    C_FLASH_SUBCODE = { "chip_settings": 0x00, \
                        "gp_settings": 0x01, \
                        "wr_usb_man_descr": 0x02, \
                        "wr_usb_prod_descr": 0x03, \
                        "wr_usb_serial_nr": 0x04 }

    C_READSIZE, C_READUNITL = "readsize", "readuntil"
    C_ADC_ALL, C_ADC_1, C_ADC_2, C_ADC_3 = "ADC_ALL", "ADC_1", "ADC_2", "ADC_3"
    C_GPIO_IN, C_GPIO_OUT = "IN", "OUT"
    C_GPIO_SSPND        = "SSPND"
    C_GPIO_LED_URX      = "LED_URx"
    C_GPIO_GPIO         = "GPIO"
    C_GPIO_INTRPT_DET   = "INTRPT_DET"
    C_GPIO_LED_UTX      = "LED_UTx"
    C_GPIO_ADC          = "ADC"
    C_GPIO_CLK_OUT      = "CLK_OUT"
    C_GPIO_DAC          = "DAC"
    C_GPIO_LED_I2C      = "LED_I2C"

    C_CLK_DC0           = "50"
    C_CLK_DC25          = "25"
    C_CLK_DC50          = "50"
    C_CLK_DC75          = "75"

    C_VREF_VDD          = "VDD"
    C_VREF_VRM          = "VRM"
    C_VRM_1024          = "1024"
    C_VRM_2048          = "2048"
    C_VRM_4096          = "4096"
    C_VRM_OFF           = "OFF"

    C_I2C_MODE_NORMAL   = "NORMAL"
    C_I2C_MODE_REPSTART = "REPSTART"
    C_I2C_MODE_NOSTOP   = "NOSTOP"



    def __init__(self, vendorID, productID):
        self.vendorID = vendorID
        self.productID = productID
        self.serDevice = serial.Serial()

    def config_uart(self, port, baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, 
                                stopbits=serial.STOPBITS_ONE, rd_timeout=None, wr_timeout=None, termination="\n"):
        self.serDevice.port = port
        self.serDevice.baudrate = baudrate
        self.serDevice.bytesize = bytesize
        self.serDevice.parity = parity
        self.serDevice.stopbits = stopbits
        self.serDevice.timeout = rd_timeout
        self.serDevice.write_timeout = wr_timeout
        self.uartTermination = termination
        return 0
    
    def connect_uart(self):
        self.serDevice.open()
        assert self.serDevice.is_open == True
        return 0

    def disconnect_uart(self):
        self.serDevice.close()
        assert self.serDevice.is_open == False
        return 0

    def uart_read(self, size=None, mode=None):
        if mode == None:
            mode = MCP2221.READSIZE

        if size == None:
            size = 1

        if mode == MCP2221.READSIZE:
            ret = self.serDevice.read(size)
        elif mode == MCP2221.READUNITL:
            ret = self.serDevice.read_until(self.uartTermination, size)
        else:
            raise ValueError("Value did not match the available choices!")
        
        return ret

    def uart_write(self, data):
        assert self.serDevice.write(data) == len(data)
        return 0

    def set_ID(self, vendorID, productID):
        self.vendorID = vendorID
        self.productID  = productID

    def get_ID(self):
        return [self.vendorID, self.productID]

    def connect_usb(self):
        self.dev = usb.core.find(idVendor=self.vendorID, idProduct=self.productID)
        if self.dev is None:
            raise ValueError('Device not found')
        
        return self.dev

    def config_usb(self, config):
        # set the active configuration. With no arguments, the first
        # configuration will be the active one
        self.dev.set_configuration()

        # get an endpoint instance
        self.cfg = self.dev.get_active_configuration()
        self.intf = self.cfg[(config[0],config[1])]
        self.ep_read = self.intf[0]
        self.ep_write = self.intf[1]
        assert self.ep_read is not None
        assert self.ep_write is not None

        # claim the device
        usb.util.claim_interface(self.dev, self.intf)

        return 0

    def write_data(self, msg, timeout=1000):
        assert self.dev.write(self.ep_write, msg, timeout) == len(msg)
        return 0

    def read_data(self, timeout=1000):
        ret = self.dev.read(self.ep_read, MCP2221.C_MESSAGE_LEN, timeout)
        return ret

    def send_data(self, msg, timeout=1000):
        self.write_data(msg, timeout)
        ret = self.read_data(timeout)
        return ret

    def get_status(self):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_STATUS
        ret = self.send_data(msg, timeout=1000)
        return ret

    def set_parameter(self, i2c_cancel=False, i2c_set_speed=False, i2c_speed=0):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_SET_PARAM
        if i2c_cancel == True:
            msg[2] = 0x10

        if i2c_set_speed == True:
            msg[3] = 0x20
            msg[4] = i2c_speed #Calculated by i2c_freq = hex(int(12e6 / i2c_speed) - 2)

        ret = self.send_data(msg, timeout=1000)

        return ret

    def i2c_cancel_transaction(self):
        ret = self.set_parameter(i2c_cancel=True)
        return ret[2]

    def i2c_set_speed(self, i2c_speed):
        ret = self.set_parameter(i2c_set_speed=True, i2c_speed=i2c_speed)
        return [ret[3], ret[4]]

    def i2c_get_info(self):
        ret = self.set_parameter()
        i2c_length =  (int(ret[10]) << 8) + int(ret[9])
        i2c_num_of_bytes = (int(ret[12]) << 7) + int(ret[11])
        i2c_buffer = ret[13]
        i2c_divider = ret[14]
        i2c_timeout = ret[15]
        i2c_address = (int(ret[17]) << 8) + int(ret[16])
        ret = [i2c_length, i2c_num_of_bytes, i2c_buffer, i2c_divider, i2c_timeout, i2c_address, ret[22], ret[23], ret[24], ret[25]]

    def get_revision(self):
        ret = self.get_status()
        return [ret[46], ret[47], ret[48], ret[49]]

    def get_adc_value(self, adc_id):
        #0:adc1-3, 1:adc1, 2:adc2, 3:adc3
        ret = self.get_status()
        adc_value = []

        if adc_id == MCP2221.ADC_ALL:
            adc_value.append(int(ret[51]) << 8) + int(ret[50])
            adc_value.append(int(ret[53]) << 8) + int(ret[52])
            adc_value.append(int(ret[55]) << 8) + int(ret[54])
        elif adc_id == MCP2221.ADC_1:
            adc_value = (int(ret[51]) << 8) + int(ret[50])
        elif adc_id == MCP2221.ADC_2:
            adc_value = (int(ret[53]) << 8) + int(ret[52])
        elif adc_id == MCP2221.ADC_3:
            adc_value = (int(ret[55]) << 8) + int(ret[54])
        else:
            raise ValueError("Value did not match the available choices!")

        return adc_value
    
    def adc_cnt2int(cnt, vref):
        return (vref / (2**10 - 1)) * cnt
    
    def get_flash(self, subcode):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_RD_FLASH
        msg[1] = MCP2221.C_FLASH_SUBCODE[subcode]
        ret = self.send_data(msg, timeout=1000)
        
        return ret

    def get_flash_all(self):
        for code in MCP2221.C_FLASH_SUBCODE:
            self.flash[code] = self.get_flash(self, code)

        return 0
    
    def set_flash(self, subcode, data):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_WR_FLASH
        msg[1] = hex(subcode)
        msg.extend(data)
        ret = self.send_data(msg, timeout=1000)

        return ret[1]

    def set_chip_settings(self, general=None, clk_out_div=None, dac_settings=None, adc_intrpt_settings=None, pid_vid_settings=None, usb_pwr_settings=None, flash_pw=None ):
        content = self.get_flash(MCP2221.C_FLASH_SUBCODE["chip_settings"])[4:13].copy()
        if general != None:
            content[0] = general
        if clk_out_div != None:
            content[1] = clk_out_div
        if dac_settings != None:
            content[3] = dac_settings
        if adc_intrpt_settings != None:
            content[3] = adc_intrpt_settings
        if pid_vid_settings != None:
            content[4] = (pid_vid_settings[0] & 0x0F)
            content[5] = (pid_vid_settings[0] >> 8)
            content[6] = (pid_vid_settings[1] & 0x0F)
            content[7] = (pid_vid_settings[1] >> 8)
        if usb_pwr_settings != None:
            content[8] = usb_pwr_settings[0]
            content[9] = usb_pwr_settings[1]
        if flash_pw != None:
            content[10] = flash_pw[0]
            content[11] = flash_pw[1]
            content[12] = flash_pw[2]
            content[13] = flash_pw[3]
            content[14] = flash_pw[4]
            content[15] = flash_pw[5]
            content[16] = flash_pw[6]
            content[17] = flash_pw[7]

        ret = self.set_flash(MCP2221.C_FLASH_SUBCODE["chip_settings"], content)
        return ret
    
    def get_gpio_config(self, id, gpio):
            if gpio["value"] == 0:
                value = 0b0
            elif gpio["value"] == 1:
                value = 0b1
            else:
                raise ValueError("Value did not match the available choices!")

            if gpio["dir"] == MCP2221.C_GPIO_OUT:
                direction = 0b0
            elif gpio["dir"] == MCP2221.C_GPIO_IN:
                direction = 0b1
            else:
                raise ValueError("Value did not match the available choices!")

            if id == 0:
                if gpio["mode"] == MCP2221.C_GPIO_SSPND:
                    mode = 0b010
                elif gpio["mode"] == MCP2221.C_GPIO_LED_URX:
                    mode = 0b001
                elif gpio["mode"] == MCP2221.C_GPIO_GPIO:
                    mode = 0b000
                else:
                    raise ValueError("Value did not match the available choices!")
            elif id == 1:
                if gpio["mode"] == MCP2221.C_GPIO_INTRPT_DET:
                    mode = 0b100
                elif gpio["mode"] == MCP2221.C_GPIO_LED_UTX:
                    mode = 0b011
                elif gpio["mode"] == MCP2221.C_GPIO_ADC:
                    mode = 0b010
                elif gpio["mode"] == MCP2221.C_GPIO_CLK_OUT:
                    mode = 0b001
                elif gpio["mode"] == MCP2221.C_GPIO_GPIO:
                    mode = 0b000
                else:
                    raise ValueError("Value did not match the available choices!")
            elif id == 2:
                if gpio["mode"] == MCP2221.C_GPIO_DAC:
                    mode = 0b011
                elif gpio["mode"] == MCP2221.C_GPIO_ADC:
                    mode = 0b010
                elif gpio["mode"] == MCP2221.C_GPIO_CLK_OUT:
                    mode = 0b001
                elif gpio["mode"] == MCP2221.C_GPIO_GPIO:
                    mode = 0b000
                else:
                    raise ValueError("Value did not match the available choices!")
            elif id ==3:
                if gpio["mode"] == MCP2221.C_GPIO_DAC:
                    mode = 0b011
                elif gpio["mode"] == MCP2221.C_GPIO_ADC:
                    mode = 0b010
                elif gpio["mode"] == MCP2221.C_GPIO_LED_I2C:
                    mode = 0b001
                elif gpio["mode"] == MCP2221.C_GPIO_GPIO:
                    mode = 0b000
                else:
                    raise ValueError("Value did not match the available choices!")
            else:
                raise ValueError("Value did not match the available choices!")


            config = (value << 4) + (direction << 3) + mode
            return config

    def set_flash_gpio(self, gpio0=None, gpio1=None, gpio2=None, gpio3=None):
        content = self.get_flash(MCP2221.C_FLASH_SUBCODE["chip_settings"])[4:13].copy()
        if gpio0 != None:
            content[0] = self.get_gpio_config(0, gpio0)
        if gpio1 != None:
            content[1] = self.get_gpio_config(1,gpio1)
        if gpio2 != None:
            content[2] = self.get_gpio_config(2, gpio2)
        if gpio3 != None:
            content[3] = self.get_gpio_config(3, gpio3)

        ret = self.set_flash(MCP2221.C_FLASH_SUBCODE["gpio_settings"], content)
        return ret
    
    def set_flash_usb_manu(self, manufacturer_desc):
        byte_string = manufacturer_desc.encode("utf-16")
        length = len(byte_string)
        content = [0x00] * MCP2221.C_MESSAGE_LEN
        content[0] = length + 2
        content[1] = 0x03
        for i in range(0,length):
            content[i+2] = byte_string[i]

        ret = self.set_flash(MCP2221.C_FLASH_SUBCODE["wr_usb_man_descr"], content)
        return ret

    def set_flash_usb_prod(self, product_desc):
        byte_string = product_desc.encode("utf-16")
        length = len(byte_string)
        content = [0x00] * MCP2221.C_MESSAGE_LEN
        content[0] = length + 2
        content[1] = 0x03
        for i in range(0,length):
            content[i+2] = byte_string[i]

        ret = self.set_flash(MCP2221.C_FLASH_SUBCODE["wr_usb_prod_descr"], content)
        return ret

    def set_flas_usb_sn(self, serial_number):
        byte_string = serial_number.encode("utf-16")
        length = len(byte_string)
        content = [0x00] * MCP2221.C_MESSAGE_LEN
        content[0] = length + 2
        content[1] = 0x03
        for i in range(0,length):
            content[i+2] = byte_string[i]

        ret = self.set_flash(MCP2221.C_FLASH_SUBCODE["wr_usb_serial_nr"], content)
        return ret

    def send_flash_pw(self, pw):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_FLASH_PW
        for i in range(0,len(pw)):
            msg[i+2] = pw[i]

        ret = self.send_data(msg, timeout=1000)

        return ret[1]

    def i2c_write(self, addr, data, length, mode=None):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN

        if mode == None:
            mode = MCP2221.C_I2C_MODE_NORMAL

        payload_length = len(data)

        if mode == MCP2221.C_I2C_MODE_NORMAL:
            msg[0] = MCP2221.C_CC_I2C_WRITE
            #assert length == payload_length
        elif mode == MCP2221.C_I2C_MODE_REPSTART:
            msg[0] = MCP2221.C_CC_I2C_WRITE_REPSTART
        elif mode == MCP2221.C_I2C_MODE_NOSTOP:
            msg[0] = MCP2221.C_CC_I2C_WRITE_NOSTOP
        else:
            raise ValueError("Value did not match the available choices!")

        addr = (addr << 1)

        msg[1] = (length & 0xFF)
        msg[2] = ((length & 0xFF00) >> 8)
        msg[3] = addr
        msg[4:4+payload_length] = data

        ret = self.send_data(msg, timeout=1000)

        return ret
    
    def i2c_read(self, addr, length, mode):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN

        if mode == MCP2221.C_I2C_MODE_NORMAL:
            msg[0] = MCP2221.C_CC_I2C_READ
        elif mode == MCP2221.C_I2C_MODE_REPSTART:
            msg[0] = MCP2221.C_CC_I2C_READ_REPSTART
        else:
            raise ValueError("Value did not match the available choices!")

        read_length = length
        addr = (addr << 1) + 0x01

        msg[1] = (read_length & 0xFF)
        msg[2] = ((read_length & 0xFF00) >> 8)
        msg[3] = addr

        ret = self.send_data(msg, timeout=1000)

        return ret

    def i2c_read_data(self):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_I2C_READ_DATA

        ret = self.send_data(msg, timeout=1000)

        return ret

    def get_GPIOs(self):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_GET_GPIO

        ret = self.send_data(msg, timeout=1000)

        return ret
    
    def set_GPIOs(self, gpio_config):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_SET_GPIO

        baseByte = 2
        for gpio in gpio_config:
            if gpio[0] == True:
                msg[baseByte + 0] = 0x01
            elif gpio[0] == False:
                msg[baseByte + 0] = 0x00
            else:
                raise ValueError("Value did not match the available choices!")

            if gpio[1] == 1:
                msg[baseByte + 1] = 0x01
            elif gpio[1] == 0:
                msg[baseByte + 1] = 0x00
            else:
                raise ValueError("Value did not match the available choices!")

            if gpio[2] == True:
                msg[baseByte + 2] = 0x01
            elif gpio[2] == False:
                msg[baseByte + 2] = 0x00
            else:
                raise ValueError("Value did not match the available choices!")

            if gpio[3] == "IN":
                msg[baseByte + 3] = 0x01
            elif gpio[3] == "OUT":
                msg[baseByte + 3] = 0x00
            else:
                raise ValueError("Value did not match the available choices!")

            baseByte = baseByte + 4

        ret = self.send_data(msg, timeout=1000)

        return ret

    def set_SRAM(self, clk_out_div=0, dac_vref=0, dac_value=0, adc_vref=0, intrp_det=0, set_gpio=0, gpio_config=[0,0,0,0]):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_SET_SRAM
        msg[2] = clk_out_div
        msg[3] = dac_vref
        msg[4] = dac_value
        msg[5] = adc_vref
        msg[6] = intrp_det
        msg[7] = set_gpio
        msg[8] = gpio_config[0]
        msg[9] = gpio_config[1]
        msg[10] = gpio_config[2]
        msg[11] = gpio_config[3]

        ret = self.send_data(msg, timeout=2000)

        return ret[1]

    def set_clk_out_div(self, dc=None, div=2):
        load_en = 0x01
        if dc==None:
            dc = MCP2221.C_CLK_DC50

        if dc == MCP2221.C_CLK_DC0:
            dc_value = 0b00
        elif dc == MCP2221.C_CLK_DC25:
            dc_value = 0b01
        elif dc == MCP2221.C_CLK_DC50:
            dc_value = 0b10
        elif dc == MCP2221.C_CLK_DC75:
            dc_value = 0b11
        else:
            raise ValueError("Value did not match the available choices!")
        
        clk_out_div_cfg = (load_en << 7) + (dc_value << 4) + div
        ret = self.set_SRAM(clk_out_div=clk_out_div_cfg)
        return ret

    def set_dac_vref(self, dac_vrm, dac_vref_sel):
        load_en = 0x01

        if dac_vrm == MCP2221.C_VRM_4096:
            vrm = 0x11
        elif dac_vrm == MCP2221.C_VRM_2048:
            vrm = 0x10
        elif dac_vrm == MCP2221.C_VRM_1024:
            vrm = 0x01
        elif dac_vrm == MCP2221.C_VRM_OFF:
            vrm = 0x00
        else:
            raise ValueError("Value did not match the available choices!")

        if dac_vref_sel == MCP2221.C_VREF_VRM:
            vref_sel = 0x01
        elif dac_vref_sel == MCP2221.C_VREF_VDD:
            vref_sel = 0x00
        else:
            raise ValueError("Value did not match the available choices!")

        dac_vref_cfg = (load_en << 7) + (vrm << 1) + vref_sel
        ret = self.set_SRAM(dac_vref=dac_vref_cfg)
        return ret

    def set_dac_value(self, dac_out_value):
        load_en = 0x01
        
        if dac_out_value > 31:
            raise ValueError("Value is too high!")
        elif dac_out_value < 0:
            raise ValueError("Value cannot be lower than 0!")

        dac_value_cfg = (load_en << 7) + dac_out_value
        ret = self.set_SRAM(dac_value=dac_value_cfg)
        return ret

    def set_adc_vref(self, adc_vrm, adc_vref_sel):
        load_en = 0x01

        if adc_vrm == MCP2221.C_VRM_4096:
            vrm = 0x11
        elif adc_vrm == MCP2221.C_VRM_2048:
            vrm = 0x10
        elif adc_vrm == MCP2221.C_VRM_1024:
            vrm = 0x01
        elif adc_vrm == MCP2221.C_VRM_OFF:
            vrm = 0x00
        else:
            raise ValueError("Value did not match the available choices!")

        if adc_vref_sel == MCP2221.C_VREF_VRM:
            vref_sel = 0x01
        elif adc_vref_sel == MCP2221.C_VREF_VDD:
            vref_sel = 0x00
        else:
            raise ValueError("Value did not match the available choices!")

        adc_vref_cfg = (load_en << 7) + (vrm << 1) + vref_sel
        ret = self.set_SRAM(adc_vref=adc_vref_cfg)
        return ret

    def set_pos_edge(self, pos_edge_trig):
        load_en = 0x01
        pos_edge_en = 0x01

        if pos_edge_trig == True:
            pos_edge_value = 0x1
        elif pos_edge_trig == False:
            pos_edge_value = 0x0
        else:
            raise ValueError("Value did not match the available choices!")

        pos_edge_cfg = (load_en << 7) + (pos_edge_en << 4) + (pos_edge_value << 3)
        ret = self.set_SRAM(intrp_det=pos_edge_cfg)
        return ret
    
    def set_neg_edge(self, neg_edge_trig):
        load_en = 0x01
        neg_edge_en = 0x01

        if neg_edge_trig == True:
            neg_edge_value = 0x1
        elif neg_edge_trig == False:
            neg_edge_value = 0x0
        else:
            raise ValueError("Value did not match the available choices!")

        neg_edge_cfg = (load_en << 7) + (neg_edge_en << 2) + (neg_edge_value << 1)
        ret = self.set_SRAM(intrp_det=neg_edge_cfg)
        return ret

    def clear_intrpt_flag(self):
        load_en = 0x01
        clear_flag = (load_en << 7) + 0x01
        ret = self.set_SRAM(intrp_det=clear_flag)
        return ret

    def set_gpio_sram(self, gpio0=None, gpio1=None, gpio2=None, gpio3=None):
        load_en = 0x80
        sram = self.get_SRAM()
        gpio_cfg = sram[22:26]
        if gpio0 != None:
            gpio_cfg[0] = self.get_gpio_config(0, gpio0)
        if gpio1 != None:
            gpio_cfg[1] = self.get_gpio_config(1, gpio1)
        if gpio2 != None:
            gpio_cfg[2] = self.get_gpio_config(2, gpio2)
        if gpio3 != None:
            gpio_cfg[3] = self.get_gpio_config(3, gpio3)

        ret = self.set_SRAM(set_gpio=load_en, gpio_config=gpio_cfg)
        return ret

    def get_SRAM(self):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_GET_SRAM
        self.sram = self.send_data(msg, timeout=1000)
        return self.sram
    
    def reset_chip(self):
        msg = [0x00] * MCP2221.C_MESSAGE_LEN
        msg[0] = MCP2221.C_CC_RESET
        msg[1] = 0xab
        msg[2] = 0xcd
        msg[3] = 0xef

        self.write_data(msg, self.C_TIMEOUT)
        return 0
