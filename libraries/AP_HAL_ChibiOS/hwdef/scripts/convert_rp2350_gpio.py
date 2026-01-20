#!/usr/bin/env python3
"""
Convert RP2350 GPIO functions from table format to dictionary format.
Functions are mapped to their column positions: F0=0, F1=1, F2=2, F3=3, F4=4, F5=5, F6=6, F7=7, F8=8, F9=9, F10=10, F11=11
"""

# SIO, PIO0, PIO1 and PIO2 can connect to all GPIO pins and are controlled by software.
# GPIO table data - cleaned format with 'x' for unused functions
# Each line: GPIO F0 F1 F2 F3 F4 F5 F6 F7 F8 F9 F10 F11
gpio_data = """
0 x     SPI0_RX   UART0_TX  I2C0_SDA PWM0_A  SIO PIO0 PIO1 PIO2 QMI_CS1n      USB_OVCUR_DET  x
1 x     SPI0_CSn  UART0_RX  I2C0_SCL PWM0_B  SIO PIO0 PIO1 PIO2 TRACECLK      USB_VBUS_DET   x
2 x     SPI0_SCK  UART0_CTS I2C1_SDA PWM1_A  SIO PIO0 PIO1 PIO2 TRACEDATA0    USB_VBUS_EN    UART0_TX
3 x     SPI0_TX   UART0_RTS I2C1_SCL PWM1_B  SIO PIO0 PIO1 PIO2 TRACEDATA1    USB_OVCUR_DET  UART0_RX
4 x     SPI0_RX   UART1_TX  I2C0_SDA PWM2_A  SIO PIO0 PIO1 PIO2 TRACEDATA2    USB_VBUS_DET   x
5 x     SPI0_CSn  UART1_RX  I2C0_SCL PWM2_B  SIO PIO0 PIO1 PIO2 TRACEDATA3    USB_VBUS_EN    x
6 x     SPI0_SCK  UART1_CTS I2C1_SDA PWM3_A  SIO PIO0 PIO1 PIO2  x            USB_OVCUR_DET  UART1_TX
7 x     SPI0_TX   UART1_RTS I2C1_SCL PWM3_B  SIO PIO0 PIO1 PIO2  x            USB_VBUS_DET   UART1_RX
8 x     SPI1_RX   UART1_TX  I2C0_SDA PWM4_A  SIO PIO0 PIO1 PIO2 QMI_CS1n      USB_VBUS_EN    x
9 x     SPI1_CSn  UART1_RX  I2C0_SCL PWM4_B  SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET  x
10 x    SPI1_SCK  UART1_CTS I2C1_SDA PWM5_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET   UART1_TX
11 x    SPI1_TX   UART1_RTS I2C1_SCL PWM5_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN    UART1_RX
12 HSTX SPI1_RX   UART0_TX  I2C0_SDA PWM6_A  SIO PIO0 PIO1 PIO2 CLOCK_GPIN0   USB_OVCUR_DET  x
13 HSTX SPI1_CSn  UART0_RX  I2C0_SCL PWM6_B  SIO PIO0 PIO1 PIO2 CLOCK_GPOUT0  USB_VBUS_DET   x
14 HSTX SPI1_SCK  UART0_CTS I2C1_SDA PWM7_A  SIO PIO0 PIO1 PIO2 CLOCK_GPIN1   USB_VBUS_EN    UART0_TX
15 HSTX SPI1_TX   UART0_RTS I2C1_SCL PWM7_B  SIO PIO0 PIO1 PIO2 CLOCK_GPOUT1  USB_OVCUR_DET  UART0_RX
16 HSTX SPI0_RX   UART0_TX  I2C0_SDA PWM0_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET   x
17 HSTX SPI0_CSn  UART0_RX  I2C0_SCL PWM0_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN    x
18 HSTX SPI0_SCK  UART0_CTS I2C1_SDA PWM1_A  SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET  UART0_TX
19 HSTX SPI0_TX   UART0_RTS I2C1_SCL PWM1_B  SIO PIO0 PIO1 PIO2 QMI_CS1n      USB_VBUS_DET   UART0_RX
20 x    SPI0_RX   UART1_TX  I2C0_SDA PWM2_A  SIO PIO0 PIO1 PIO2 CLOCK_GPIN0   USB_VBUS_EN    x
21 x    SPI0_CSn  UART1_RX  I2C0_SCL PWM2_B  SIO PIO0 PIO1 PIO2 CLOCK_GPOUT0  USB_OVCUR_DET  x
22 x    SPI0_SCK  UART1_CTS I2C1_SDA PWM3_A  SIO PIO0 PIO1 PIO2 CLOCK_GPIN1   USB_VBUS_DET   UART1_TX
23 x    SPI0_TX   UART1_RTS I2C1_SCL PWM3_B  SIO PIO0 PIO1 PIO2 CLOCK_GPOUT1  USB_VBUS_EN    UART1_RX
24 x    SPI1_RX   UART1_TX  I2C0_SDA PWM4_A  SIO PIO0 PIO1 PIO2 CLOCK_GPOUT2  USB_OVCUR_DET  x
25 x    SPI1_CSn  UART1_RX  I2C0_SCL PWM4_B  SIO PIO0 PIO1 PIO2 CLOCK_GPOUT3  USB_VBUS_DET   x
26 x    SPI1_SCK  UART1_CTS I2C1_SDA PWM5_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN    UART1_TX
27 x    SPI1_TX   UART1_RTS I2C1_SCL PWM5_B  SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET  UART1_RX
28 x    SPI1_RX   UART0_TX  I2C0_SDA PWM6_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET
29 x    SPI1_CSn  UART0_RX  I2C0_SCL PWM6_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN
30 x    SPI1_SCK  UART0_CTS I2C1_SDA PWM7_A  SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET  UART0_TX
31 x    SPI1_TX   UART0_RTS I2C1_SCL PWM7_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET   UART0_RX
32 x    SPI0_RX   UART0_TX  I2C0_SDA PWM8_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN
33 x    SPI0_CSn  UART0_RX  I2C0_SCL PWM8_B  SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET
34 x    SPI0_SCK  UART0_CTS I2C1_SDA PWM9_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET   UART0_TX
35 x    SPI0_TX   UART0_RTS I2C1_SCL PWM9_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN    UART0_RX
36 x    SPI0_RX   UART1_TX  I2C0_SDA PWM10_A SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET
37 x    SPI0_CSn  UART1_RX  I2C0_SCL PWM10_B SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET
38 x    SPI0_SCK  UART1_CTS I2C1_SDA PWM11_A SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN    UART1_TX
39 x    SPI0_TX   UART1_RTS I2C1_SCL PWM11_B SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET  UART1_RX
40 x    SPI1_RX   UART1_TX  I2C0_SDA PWM8_A  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET
41 x    SPI1_CSn  UART1_RX  I2C0_SCL PWM8_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN
42 x    SPI1_SCK  UART1_CTS I2C1_SDA PWM9_A  SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET  UART1_TX
43 x    SPI1_TX   UART1_RTS I2C1_SCL PWM9_B  SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET   UART1_RX
44 x    SPI1_RX   UART0_TX  I2C0_SDA PWM10_A SIO PIO0 PIO1 PIO2 x             USB_VBUS_EN
45 x    SPI1_CSn  UART0_RX  I2C0_SCL PWM10_B SIO PIO0 PIO1 PIO2 x             USB_OVCUR_DET
46 x    SPI1_SCK  UART0_CTS I2C1_SDA PWM11_A SIO PIO0 PIO1 PIO2 x             USB_VBUS_DET   UART0_TX
47 x    SPI1_TX   UART0_RTS I2C1_SCL PWM11_B SIO PIO0 PIO1 PIO2 QMI_CS1n      USB_VBUS_EN    UART0_RX
"""

def parse_gpio_line(line):
    """
    Parse a GPIO line by splitting on whitespace and filtering out 'x' values.
    Each line: GPIO F0 F1 F2 F3 F4 F5 F6 F7 F8 F9 F10 F11
    """
    parts = line.strip().split()
    if not parts:
        return []
    
    gpio_num = parts[0]
    functions = parts[1:]  # F0 through F11 (up to 12 functions)
    
    result = []
    for func_num, func_name in enumerate(functions):
        # Skip 'x' placeholders
        if func_name.strip() != 'x':
            pin_func = f"P{gpio_num}:{func_name}"
            result.append((pin_func, func_num))
    
    return result

# Process all lines
output_lines = []
for line in gpio_data.strip().split('\n'):
    if line.strip():
        entries = parse_gpio_line(line)
        for pin_func, func_num in entries:
            # Format with tabs for alignment like the example
            output_lines.append(f'"{pin_func}"\t:\t{func_num},')

# Print output
print('\n'.join(sorted(output_lines)))

# Also save to file
with open('/home/buzz2/ardupilot/libraries/AP_HAL_ChibiOS/hwdef/scripts/rp2350_gpio_output.txt', 'w') as f:
    f.write('\n'.join(sorted(output_lines)))
    f.write('\n')

print(f"\n\nOutput saved to rp2350_gpio_output.txt")
print(f"Total entries: {len(output_lines)}")
