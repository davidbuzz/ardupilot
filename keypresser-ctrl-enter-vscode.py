# sideproject - this python should inject a ctrl-enter keypress as if the user just typed it, every 10 seconds.

# sudo apt install xdotool

import subprocess
import time

window_name = "Visual Studio Code" # paste the wndiw name substring .
window_id = 0

seconds_conter = 0

while True:
    print("Nudging VSCODE...")

    seconds_conter += 5
    print(f"Seconds since start: {seconds_conter}")

    # find the right window
    result = subprocess.run(
        ['xdotool', 'search', '--name', window_name],
        
        capture_output=True, text=True
    )
    window_id = result.stdout.strip().split('\n')[0]

    # find its window geometry
    result = subprocess.run(
        ['xdotool', 'getwindowgeometry', '--shell', window_id],
        capture_output=True, text=True
    )
    #print(result.stdout)
    # WINDOW=41943044
    # X=0
    # Y=0
    # WIDTH=2560
    # HEIGHT=1440
    # SCREEN=0
    # parse result.stdout to get width and height
    lines = result.stdout.strip().split('\n')
    width = 0
    height = 0
    for line in lines:
        if line.startswith("WIDTH="):
            width = int(line.split('=')[1])
        elif line.startswith("HEIGHT="):
            height = int(line.split('=')[1])
    #print(f"Window width: {width}, height: {height}")

    #If you use --window, coordinates are relative to that window's top-left corner.

    # move mous to an off-set from the bottom-right abd click there.
    offset_x = 300
    offset_y = 150

    click_spot_x = width - offset_x
    click_spot_y = height - offset_y
    #print(f"Clicking at: {click_spot_x}, {click_spot_y}")

    cmd = f'xdotool mousemove --window {window_id} {click_spot_x} {click_spot_y} click 1'
    #print(f"Running command: {cmd}")
    subprocess.run(['bash', '-c', cmd])

    # time for the click to take effect
    time.sleep(1)

    #print(f"VS Code window ID: {window_id}")
    # Send ctrl+enter to the window
    subprocess.run(['xdotool', 'key', '--window', window_id, 'ctrl+Return'])

        # every so often.. say once per minute...
    if seconds_conter % 60 == 0:
        print("Resetting counter...")
        seconds_conter = 0
        #type DEL key.
        subprocess.run(['xdotool', 'key', '--window', window_id, 'Delete'])
        # wait 1/2 sec.
        time.sleep(0.5)
        # type esc key.
        subprocess.run(['xdotool', 'key', '--window', window_id, 'Escape'])
        # wait 1/2 sec.
        time.sleep(0.5)



    time.sleep(5)




#run with sudo?
