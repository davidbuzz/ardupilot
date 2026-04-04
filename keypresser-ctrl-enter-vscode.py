# sideproject - this python should inject a ctrl-enter keypress as if the user just typed it, every 10 seconds.

# sudo apt install xdotool

import subprocess
import time

window_name = "Visual Studio Code" # paste the wndiw name substring .
window_id = 0

while True:
    print("Pressing ctrl+enter...")

    # find the right window
    result = subprocess.run(
        ['xdotool', 'search', '--name', window_name],
        capture_output=True, text=True
    )
    window_id = result.stdout.strip().split('\n')[0]
    print(f"VS Code window ID: {window_id}")
    # Send ctrl+enter to the window
    subprocess.run(['xdotool', 'key', '--window', window_id, 'ctrl+Return'])

    time.sleep(5)


#run with sudo?