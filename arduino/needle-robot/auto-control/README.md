# Auto Control for Needle Robot

This project contains the Arduino code for controlling the needle-steering
robot using the automated software stack.  This code allows for various
commands to control the two motors and it sends back responses and sensor
readings back to the host, both through the serial port connection.

## Questions

### What are the effects of baud rate?

A higher baud rate will cause the Serial port buffer to fill up quicker (11ms
for 115,200 baud and 133ms for 9,600 baud).  So if you do a lot of work or take
too long parsing the data from the serial port, then it may fill up and start
ejecting data quicker than you can get to it.

Taken from [here](https://forum.arduino.cc/t/what-are-the-effects-of-baud-rate/4871)

The serial receive buffer can only hold 64 bytes.  The number of bytes
currently taken up in the buffer is obtained by `Serial.available()`.

### Does the baud rate effect speed of serial port function calls?

Both the reading and writing have buffers.  The read buffer is 64 bytes large.
Reading a character from the read buffer is very fast and is independent from
the baud rate.  You know there is something in the read buffer using
`Serial.available()`.  A higher baud rate will have two effects on reading:

1. characters come in faster, and can therefore be parsed more quickly
2. if characters come in faster than they are read, the buffer can overflow and
   portions of the message through the serial port can be lost.  If this is
   happening, you may be able to reduce the sizes or frequency of messages
   being sent, reduce the baud rate, or increase the buffer size for the serial
   ports.

The write buffer is 63 bytes large.  Writing a character is very fast and is
independent from the baud rate **only if** you do no exceed the write buffer
size.  You can know how much space is in the write buffer with
`Serial.availableForWrite()`.  A higher baud rate will increase the speed with
which the write buffer will be written to the serial port.  So if you are
writing often or larger quantities than 63 bytes, then the baud rate will cause
blocking on write operations.


### Can we increase the serial buffer size?

It does not look like an easy thing to do.  The file
`/usr/share/arduino/hardware/archlinux-arduino/avr/cores/arduino/HardwareSerial.h`
has two macro-defined variables called `SERIAL_TX_BUFFER_SIZE` and
`SERIAL_RX_BUFFER_SIZE` which are the serial buffer sizes for sending and
receiving respectively (I think).  Some places online have suggested to
manually edit the values in this file, which is part of the Arduino IDE
installation (I would not recommend this approach).

Others have suggested that the compilation be given new values on the
command-line to override the ones specified in `HardwareSerial.h`.  This would
certainly work, but I don't see a way to specify the compiler flags on the
command-line through the Arduino IDE.

There are ways to specify command-line compiler flags using `platform.txt` for
the specific platform you are targetting.  This also requires you to modify
system-installed files.  You can place a `platform.local.txt` that overrides
values of the `platform.txt` file, but again in a system-installation location.
You can copy the hardware specification into a directory in your home and make
changes there.  Like these other suggested solutions, this will be applied to
all sketches for that particular hardware architecture, which may or may not be
what you want (it's certainly not what I want).

Otherwise, there is a project to compile and push arduino sketches from a
Makefile (the package is called `arduino-mk`).  Or you can have the IDE give
verbose output on the exact commands that are executed, then you can copy,
paste into the terminal, and then modify the command before executing.


### What is the most efficient way to read from the serial port in Arduino code?

One character at a time only when `Serial.available()` returns a number bigger
than zero.  In such a case, you are reading from a ring buffer.


## Commands

Each command or message in both directions begins with the `<` character and
ends with the `>` character.  All characters before the `<` beginning character
are discarded.  Within the beginning and end, the pieces are separated by
forward slashes.

There are two modes of communication, binary and text.  First, we will describe
the text commands.

Note: all number values are 32-bit integer values.  They are at a small enough
resolution that integer values are sufficient.

For text commands, the maximum message size to the arduino is 35 bytes
(currently).  The maximum message size from the arduino to the host is 75 bytes
(currently) for a single message.

For binary commands, the maximum message size to the arduino is 12 bytes
(currently).  The maximum message size from the arduino to the host is 24 bytes
(currently).


### Supported text commands

- `<help>`
  - send the possible commands with their descriptions over the serial port.
    Each one is printed individually as
    `<help-command/{command}/{description}>`

- `<settings>`
  - this will cause the Arduino code to send many messages back, one for each
    setting within the Arduino code.  These come back as
    `<setting/{name}/{value}>`, e.g.,
    `<setting/force sensor calibration factor/418110>`

- `<state>`
  - print the state of the robot.  It will report on the absolute linear and
    rotary positions relative to the zero position in micrometers (for linear
    position) and millidegrees (for rotation), which is the position it was at
    when the arduino started up, and on the force sensor reading in micro
    Newtons.  It will result in the
    `<current-state/
      {linear-abs-position}/
      {rotary-abs-rotation}/
      {linear-velocity}/
      {rotary-velocity}/
      {force-sensor-reading}
      >` reply (without the newlines)
  - Example: `<state>` results in `<current-state/13210/754500/-1000/18000/53400>`
    which means 13.210 mm linearly forward from the home position, 754.5
    degrees clockwise on the rotatary motor from the home position, 1 mm per
    second backwards velocity on the linear actuator, 18 degrees per second
    clockwise on the rotary motor, and 53.4 milli Newtons, which equates to
    about 5.45 grams of mass with the force of gravity.

- `<send-binary/[on|off]>`
  - Set the binary send type on or off.  Off means to send as plain text.
    This is only applicable to the message types that are implemented in both
    text and binary format.
  - Example: `<send-binary/on>` tells the arduino to send messages in binary
    format instead of the default text format.

- `<stream-force/[on|off]>`
  - Tells the arduino to turn on or off the streaming of the force sensor
    readings.  Turning it on means that every time a new force sensor reading
    is available, it will be sent from the arduino as
    `<force/{force-sensor-reading}>` messages.
  - Example: `<stream-force/on>` will turn on streaming of force sensor
    readings to the host from the Arduino as `<force/{force-sensor-reading}>`
    messages.

- `<stream-state-on/{interval}>`
  - Tells the arduino to stream `<current-state/...>` messages at a particular
    `{interval}` in microseconds.
  - Example: `<stream-state-on/1000000>` will cause the arduino to send the
    `<current-state/...>` message once every second.

- `<stream-state-off>`
  - turn off streaming of state from arduino


### Supported text messages from the arduino

- `<help-command/{command}/{description}>`
  - Describes an available text command.
  - Many of these are given in response to the `<help>` command

- `<setting/{name}/{value}>`
  - Gives the name for a setting and its value.
  - Many of these are given in response to the `<settings>` command

- `<current-state/{linear-abs-position}/{rotary-abs-position}/{linear-velocity}/{rotary-velocity}/{force-sensor-reading}>`
  - Gives the current state
  - In response to `<state>` command
  - Can be output at regular intervals as well

- `<force/{force-sensor-reading}>`
  - Gives just the force sensor reading
  - Can be output at regular intervals


### Text commands yet to be supported

TODO: tare the force sensor
TODO: change the force sensor calibration factor?
TODO: get raw force sensor value?
TODO: set maximum motor angular acceleration

- `<linear-abs/{position}/{speed}>`
  - rotate the linear motor such that it moves the linear platform to the
    `{position}` micrometer absolute location (relative to the point it's at
    when the arduino starts up).  Move at the specified `{speed}` micrometers
    per second.
  - Example: `<linear-abs/0/750>` will move to the zero position (i.e., where
    it was when the arduino started up), and will move there at a speed of 750
    micrometers per second.

- `<linear-rel/{position}/{speed}>`
  - rotate the linear motor such that it moves the linear platform to the
    `{position}` micrometer location forward from the current position.  Move
    at the specified `{speed}` micrometers per second.  Give a negative number
    for `{position}` to move backwards.
  - Example: `<linear-rel/-500/750>` will move backwards by 0.5 mm, and will
    move there at a maximum speed of 0.75 mm per second.

- `<linear-velocity/{velocity}>`
  - Move at the given velocity until given a new linear command, meaning does
    not stop at a particular location, just moves at a constant speed.  It is
    velocity instead of speed because you can give a negative value to make it
    move backwards.  It is in units of micrometers per second.
  - Example: `<linear-velocity/-2150>` will move backwards at 2.15 mm/s until a
    new command is given.

- `<rotary-abs/{position}/{speed}>`
  - rotate the rotary motor to the given `{position}` in milli-degrees.  This
    value can be larger than 360,000 which will mean past one full rotation.
    From the home position, you can command five full rotations clockwise by
    passing the value 1,800,000 for the position.  The `{speed}` is given as
    millidegrees per second.
  - Example: `<rotary-abs/1800000/36000>` will rotate clockwise for five full
    rotations from the home position at a speed of 0.1 revolutions per second
    (or 36 degrees per second).

- `<rotary-rel/{position}/{speed}>`
  - Rotate the rotary motor `{position}` millidegrees clockwise from where it
    currently is (give a negative number to go counter-clockwise) at the given
    `{speed}` in millidegrees per second (so 360000 is 1 Hz, which at this
    resolution with 32-bits, you can specify up to 5.9 kHz which is much faster
    than this little motor can achieve).

- `<rotary-velocity/{velocity}>`
  - rotate the rotary motor at the given `{velocity}` given in milli-degrees
    per second.  A positive value will move clockwise and a negative value will
    move counter-clockwise.
  - Example: `<rotary-velocity/-180000>` will command it to move
    counter-clockwise at 0.5 revolutions per second (or -180 degrees per
    second).

### Text messages from the arduino yet to be supported

None for now.

### Binary format

Binary messages also begin with the `<` character and end with the `>`
character, for consistency.  After the `<` character, the `B` character
indicates a binary message followed by a single byte indicating the command
type.  So, for a binary message that has a 8 byte payload, and is command
number 97, it would be formatted as

`<BA{payload}>`

where `A` has the ACII value of 65, and `{payload}` would consist of 8 bytes.
In this example, the full message would be 8 + 4 = 12 bytes.  The message has a
three byte header, and a one byte tail along with whatever the size of the
payload may be.  The size of `{payload}` is known from the message type and
must exactly match the expected size.

The payload will frequently have values that are multiple bytes long (like a
32-bit signed integer).  This protocol expects these to be sent in the
Big-Endian format, e.g., the bits of the 32-bit integer should be in order from
most significant bit to least significant bit.

Messages from the host to the arduino have command codes from 65 to 90, which
map to the ASCII characters `A` through `Z` (uppercase).

Messages from the arduino have command codes from 97 to 122, which map to the
ASCII characters `a` through `z` (lowercase).

I kept the command codes within the alphabetic range for easy documentation and
partial readability of binary messages within a serial terminal.  In the
documentation below, I specify just the command code followed by the arguments
(with their respective sizes) and explanation.

### Supported binary commands

None yet

### Supported binary messages from the arduino

- `s`: current state, equal to `current-state` text command
  - payload of 20 bytes
  - `linear-abs-position` (32-bit signed integer): absolute linear position
    from home position in micrometers
  - `rotary-abs-position` (32-bit signed integer): absolute clockwise rotation
    from home position in milli-degrees
  - `linear-velocity` (32-bit signed integer): velocity of linear actuator in
    micrometers per second
  - `rotary-velocity` (32-bit signed integer): clockwise angular velocity in
    milli-degrees per second
  - `force-sensor-reading` (32-bit signed integer): last force sensor reading
    in micro-Newtons.

- `f`: force sensor reading, equal to `force` text command
  - payload of 4 bytes
  - `force-sensor-reading`


### Binary commands yet to be supported

- `A`: equal to `linear-abs` text command
  - payload of 8 bytes
  - `position` (32-bit signed integer): absolute linear position in micrometers
  - `speed` (32-bit unsigned integer): max speed in micrometers per second

- `B`: equal to `linear-rel` text command
  - payload of 8 bytes
  - `position` (32-bit signed integer): relative linear position in micrometers
  - `speed` (32-bit unsigned integer): max speed in micrometers per second

- `C`: equal to `linear-velocity` text command
  - payload of 4 bytes
  - `velocity` (32-bit signed integer): velocity in micrometers per second

- `D`: equal to `rotary-abs` text command
  - payload of 8 bytes
  - `position` (32-bit signed integer): absolute rotation position in milli-degrees
  - `speed` (32-bit unsigned integer): max speed in milli-degrees per second

- `E`: equal to `rotary-rel` text command
  - payload of 8 bytes
  - `position` (32-bit signed integer): relative rotation position in milli-degrees
  - `speed` (32-bit unsigned integer): max speed in milli-degrees per second

- `F`: equal to `rotary-velocity` text command
  - payload of 4 bytes
  - `velocity` (32-bit unsigned integer): clockwise velocity in milli-degrees per second


### Binary messages from the arduino yet to be supported

None for now.


## TODO Notes

TODO: figure out a way to specify more than one command to be queued at a time.
Do we want to have a new command supercede the current command?  Or to be put
into a queue?  Perhaps we could have them queue up by default, but then have a
command that clears the current queue and another command to query the current
queue.

TODO: do I want to do position control or velocity control?  Adam (I think) has
some good velocity control code I could probably borrow.

TODO: do I want the controller to live on the Arduino or on the host machine?
- If on the Arduino, it will increase amount of computation
- If on the host, it could be subject to lag from message sending and parsing
- How fast can it parse a full message with the entire path? (or some lookahead
  time of like one second's worth of states?)
- If on the host, would do like the Kuka, and have a frequency at which the
  messages are expected.  We could use the existing interface to implement
  this, where we do rotary-rel and linear-rel with velocity and enough
  positional difference to last a little longer than it takes for the next
  message to come in, which would supercede the previous one immediately.
  Again, this results in lag, which may be accounted for by the host
  controller.

TODO: emit the current state when we do a force sensor reading - i.e., emit
force (in milliNewtons?), rotation angle, linear displacement, and linear motor
rotation angle.

TODO: I think I want to do a PID position controller with velocity limits and
probably pretty conservative acceleration limits.  So I think the command to
send to the Arduino would be a target velocity for that time step, but the
controller, which lives on the host machine is performing position control
using the sensor readings from the arduino.

TODO: can I read from the force sensor more frequently than 11 Hz?  One website
seemed to think so.  That would be beneficial if I can read the force value
pretty quickly.
