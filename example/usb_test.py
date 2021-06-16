# import time
# import board
# import digitalio
# 
# led = digitalio.DigitalInOut(board.G0)
# led.direction = digitalio.Direction.OUTPUT
#      
# while True:
#     led.value = True
#     time.sleep(0.5)
#     led.value = False
#     time.sleep(0.5)

import usb.core
import usb.util
import sys
import time


def hid_set_report(dev, report):
    """ Implements HID SetReport via USB control transfer """
    dev.ctrl_transfer(
        0x41,  # REQUEST_TYPE_CLASS | RECIPIENT_INTERFACE | ENDPOINT_OUT
        9,     # SET_REPORT
        0x200, # "Vendor" Descriptor Type + 0 Descriptor Index
        0,     # USB interface № 0
        report # the HID payload as a byte array -- e.g. from struct.pack()
    )

def hid_get_report(dev):
    """ Implements HID GetReport via USB control transfer """
    return dev.ctrl_transfer(
        0xA1,  # REQUEST_TYPE_CLASS | RECIPIENT_INTERFACE | ENDPOINT_IN
        1,     # GET_REPORT
        0x200, # "Vendor" Descriptor Type + 0 Descriptor Index
        0,     # USB interface № 0
        64     # max reply size
    )


# find our device
#dev = usb.core.find(idVendor=0x046D, idProduct=0xC62B)
dev = usb.core.find(idVendor=0x04d8, idProduct=0x00dd)
dev.reset()
time.sleep(0.5)
dev = usb.core.find(idVendor=0x04D8, idProduct=0x00DD)


# was it found?
if dev is None:
    raise ValueError('Device not found')


for cfg in dev:
    sys.stdout.write(str(cfg.bConfigurationValue) + '\n')
    for intf in cfg:
        sys.stdout.write('\t' + \
                         str(intf.bInterfaceNumber) + \
                         ',' + \
                         str(intf.bAlternateSetting) + \
                         '\n')
        for ep in intf:
            sys.stdout.write('\t\t' + \
                             str(ep.bEndpointAddress) + \
                             str(ep) + \
                             '\n')


dev.reset()
if dev.is_kernel_driver_active(0) == True:
    dev.detach_kernel_driver(0)

# set the active configuration. With no arguments, the first
# configuration will be the active one
#dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(1,0)]
#ep = intf[0]

ep = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT)

#assert ep is not None
# claim the device
# usb.util.claim_interface(dev, intf)



msg = [ 0x51, 0x00, 0x01, 0x00,
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
        0x00, 0x00, 0x00, 0x51 ]

msg2 = b'\x51\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

resp = dev.ctrl_transfer(0x21, 0x9, 0x040, 0x00, msg)
print(resp)

hid_set_report(dev, msg2)
ans = hid_get_report(dev)
print(ans)

# assert dev.write(0x02, msg2, 1000) == len(msg)
# time.sleep(1)
# ret = dev.read(0x82, 50, 1000)
# print(ret)

usb.util.dispose_resources(dev)