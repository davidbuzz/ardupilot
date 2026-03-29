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
secs_counter=0
while True:
    print ("Pressing ctrl-enter...")
    keyboard.press_and_release('ctrl+enter')
    time.sleep(5)
    secs_counter += 5
    if secs_counter >= 3600:
        print ("typing 'please continue analysis' and then enter...")
        #keyboard.write('go-ahead to finish PIOUART RX/TX end-to-end on hardware')
        keyboard.write('continue analysis')
        keyboard.press_and_release('enter')
        secs_counter = 0

# needs run with sudo


