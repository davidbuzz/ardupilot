# sideproject - this python should inject a 1 press, and then a backspace keypress as if the user just typed it, every 5 seconds.

# pip3 install pyautogui --break-system-packages

# python -m pip install keyboard --break-system-packages
# sudo python -m pip install keyboard --break-system-packages

# this is useful to auto-ok claude code vscode plugin.

# import pyautogui
# import time
# while True:
#     print ("Pressing ctrl-enter...")
#     pyautogui.hotkey('ctrl', 'enter')
#     time.sleep(30)

# also, once-per-hour, type 'continue' and press enter after.

# the above doesnt inject it into other applications, and i want it to.
import time
import keyboard
secs_counter = 0
while True:
    print ("Pressing '1' and then a backspace...")
    keyboard.press_and_release('1')
    time.sleep(0.1)
    keyboard.press_and_release('backspace')
    keyboard.press_and_release('backspace')
    time.sleep(5)
    secs_counter += 5
    if secs_counter >= 3600:
        print ("Pressing 'continue' and then enter...")
        keyboard.write('continue')
        keyboard.press_and_release('enter')
        secs_counter = 0


# needs run with sudo

