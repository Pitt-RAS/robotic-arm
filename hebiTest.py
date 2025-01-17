#!/usr/bin/env python3

import hebi
from math import pi, sin
from time import sleep, time

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate
sleep(2.0)

family_name = "fourGrowers"
module_name = "Wrist" #Elbow, Shoulder, Base

group = lookup.get_group_from_names([family_name], [module_name])

if group is None:
    print('Group not found: Did you forget to set the module family and name above?')
    exit(1)

group_command = hebi.GroupCommand(group.size)
group_feedback = hebi.GroupFeedback(group.size)

# Start logging in the background
group.start_log('logs', mkdirs=True)

freq_hz = 0.5                 # [Hz]
freq = freq_hz * 2.0 * pi  # [rad / sec]
amp = pi * 0.25           # [rad] (45 degrees)

duration = 8              # [sec]
start = time()
t = time() - start

while t < duration:
    # Even though we don't use the feedback, getting feedback conveniently
    # limits the loop rate to the feedback frequency
    group.get_next_feedback(reuse_fbk=group_feedback)
    t = time() - start

    group_command.position = amp * sin(freq * t)
    group.send_command(group_command)
"""group.command_lifetime=1000.0
group_command.position = 0
group.send_command(group_command)
group_command.position = 3.14
group.send_command(group_command)


for i in range(0, 100):
    print(i/10)
    group_command.position = i/10
    group.send_command(group_command)
    sleep(0.3)"""


# Stop logging. `log_file` contains the contents of the file
log_file = group.stop_log()

if log_file is not None:
    hebi.util.plot_logs(log_file, 'position')
