# sideproject - this python should inject a ctrl-enter keypress as if the user just typed it, every 10 seconds.

# sudo pip install pynput --break-system-packages

import time
from pynput.keyboard import Key, Controller

keyboard = Controller()
secs_counter = 0

while True:
    print("Pressing ctrl+enter...")
    with keyboard.pressed(Key.ctrl):
        keyboard.press(Key.enter)
        keyboard.release(Key.enter)
    time.sleep(5)
    secs_counter += 5

# needs run with sudo


