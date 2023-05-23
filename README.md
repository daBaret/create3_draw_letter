# Create3 Draw Letter
This package containts a simple state machine and ables to command the robot to drive and draw a letter.

## To launch
Run:
```bash
ros2 launch create3_state_machine create3_state_machine.launch.py
```

In another terminal call the service to start the mission:
```bash
ros2 service call /input_word create3_state_machine_msgs/srv/String word:\ \'N\'\
```
The available letters are **SEVN**.