import struct
import time

import hid

# HID device vendor and product IDs
VENDOR_ID = 0x3318
PRODUCT_ID = 0x0424

# HID report lengths
HID_REPORT_LEN = 64
HID_OUTPUT_REPORT_LEN = 3

# HID report indexes for roll, pitch1, and pitch2
HID_REPORT_INDEX_ROLL = 0
HID_REPORT_INDEX_PITCH1 = 2
HID_REPORT_INDEX_PITCH2 = 4

# HID output report index for LED control
HID_OUTPUT_REPORT_INDEX_LED = 0

# HID output report values for turning LEDs on and off
HID_OUTPUT_REPORT_LED_OFF = 0
HID_OUTPUT_REPORT_LED_ON = 1

# Initialize the HID device
try:
    device = hid.device()
    device.open(VENDOR_ID, PRODUCT_ID)
    print("HID device opened")
except Exception as ex:
    print("Error opening HID device:", ex)
    exit()

# Continuously read HID reports and display roll and pitch data
while True:
    try:
        # Read an HID report
        data = device.read(HID_REPORT_LEN, timeout_ms=100)

        # Parse the roll and pitch data from the HID report
        roll, pitch1, pitch2 = struct.unpack("<hhh", data[:6])

        # Convert the roll and pitch data from integer to float
        roll = roll / 100.0
        pitch1 = pitch1 / 100.0
        pitch2 = pitch2 / 100.0

        # Display the roll and pitch data
        print("Roll: {:.2f} Pitch1: {:.2f} Pitch2: {:.2f}".format(roll, pitch1, pitch2))

        # Toggle the LED on the HID device
        output_report = [HID_OUTPUT_REPORT_LED_OFF]
        output_report[HID_OUTPUT_REPORT_INDEX_LED] = HID_OUTPUT_REPORT_LED_ON
        device.send_feature_report(output_report[:HID_OUTPUT_REPORT_LEN])

    except KeyboardInterrupt:
        print("Keyboard interrupt, exiting...")
        break

    except Exception as ex:
        print("Error reading HID report:", ex)
        time.sleep(1)

# Close the HID device
device.close()
print("HID device closed")
