JOYをロボットのコマンドに変換するノード

5_msgs

標準のメッセージファイルは次のようになっています。パッケージを参照してください。

```
# button states (0: released, 1: pressed)
bool square
bool cross
bool circle
bool triangle
bool l1
bool r1
bool l2
bool r2
bool command_l
bool command_r
bool joy_l
bool joy_r
bool home
bool capture

# joystick axes (-1.0 to 1.0)
float32 left_joystick_x
float32 left_joystick_y
float32 right_joystick_x
float32 right_joystick_y

# trigger values (0.0 to 1.0)
float32 left_trigger
float32 right_trigger

float32 cross_up
float32 cross_left

# deadzone for left and right joysticks
float32 left_joystick_deadzone
float32 right_joystick_deadzone
```

## publisher



