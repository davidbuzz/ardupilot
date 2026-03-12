# sideproject - this python should inject a ctrl-enter keypress as if the user just typed it, every 10 seconds.

# pip3 install pyautogui --break-system-packages

# python -m pip install keyboard --break-system-packages
# sudo python -m pip install keyboard --break-system-packages

# import pyautogui
# import time
# while True:
#     print ("Pressing ctrl-enter...")
#     pyautogui.hotkey('ctrl', 'enter')
#     time.sleep(30)

# the above doesnt inject it into other applications, and i want it to.
import time
import keyboard
while True:
    print ("Pressing ctrl-enter...")
    keyboard.press_and_release('ctrl+enter')
    time.sleep(15)

# needs run with sudo


