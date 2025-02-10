#!/usr/bin/python3
# File name   : setup.py for PIPPY
# Date        : 2020/11/24

import os

def replace_num(file, initial, new_num):
    """Replace a specific line in a file with a new line."""
    newline = ""
    with open(file, "r") as f:
        for line in f.readlines():
            if line.startswith(initial):
                line = f"{new_num}\n"
            newline += line
    with open(file, "w") as f:
        f.write(newline)

def run_command(command, max_retries=3):
    """Run a shell command with retry mechanism."""
    for _ in range(max_retries):
        if os.system(command) == 0:
            return True
    return False

# Paths
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)

# Update system and install packages
commands = [
    "sudo apt update",
    "sudo apt -y dist-upgrade",
    "sudo apt clean",
    "sudo pip3 install --break-system-packages -U pip",
    "sudo apt-get install -y python-dev python3-pip libfreetype6-dev libjpeg-dev build-essential",
    "sudo -H pip3 install --break-system-packages --upgrade luma.oled",
    "sudo apt-get install -y i2c-tools",
    "sudo apt-get install -y python3-smbus",
    "sudo pip3 install --break-system-packages icm20948",
    "sudo pip3 install --break-system-packages flask",
    "sudo pip3 install --break-system-packages flask_cors",
    "sudo pip3 install --break-system-packages websockets",
]

for cmd in commands:
    run_command(cmd)

# Modify /boot/config.txt
config_modifications = [
    ('/boot/config.txt', '#dtparam=i2c_arm=on', 'dtparam=i2c_arm=on'),
    ('/boot/config.txt', '[all]', '[all]\ngpu_mem=128'),
    ('/boot/config.txt', 'camera_auto_detect=1', '#camera_auto_detect=1\nstart_x=1'),
    ('/boot/config.txt', 'camera_auto_detect=1', '#camera_auto_detect=1')
]

for file, initial, new_num in config_modifications:
    try:
        replace_num(file, initial, new_num)
    except Exception as e:
        print(f"Error modifying {file}: {e}")

# Install specific packages
package_commands = [
    "sudo pip3 install --break-system-packages opencv-contrib-python==4.5.3.56",
    "sudo pip3 uninstall --break-system-packages -y numpy",
    "sudo pip3 install --break-system-packages numpy==1.21",
    "sudo apt-get -y install libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev",
    "sudo pip3 install --break-system-packages imutils zmq pybase64 psutil",
    "sudo apt-get install -y util-linux procps hostapd iproute2 iw haveged dnsmasq",
    "sudo pip3 install --break-system-packages pi-ina219"
]

for cmd in package_commands:
    run_command(cmd)

# Clone and install create_ap
if run_command(f"cd {thisPath} && cd .. && sudo git clone https://github.com/oblique/create_ap"):
    try:
        os.system(f"cd {thisPath} && cd .. && cd create_ap && sudo make install")
    except Exception as e:
        print(f"Error installing create_ap: {e}")

# Update rc.local
# replace_num('/etc/rc.local', 'exit 0', f'cd {thisPath} && sudo python3 webServer.py &\nexit 0')

print('Completed!')

# Reboot system
# os.system("sudo reboot")
