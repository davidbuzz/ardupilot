# click the regular mounse button ever5 5 
# # seconds and beep a sound, on linux.

import os
import time
import pyautogui


while True:
    print("Clicking the mouse...")
    pyautogui.click()
    #cmd = 'beep -f 1000 -l 500'  # Beep at 1000 Hz for 500 ms
    #os.system(cmd)
    time.sleep(5)  # Wait for 5 seconds before the next click
