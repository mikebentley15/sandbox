# Virtual Serial Ports Using Socat

https://jamesthom.as/2021/01/virtual-serial-ports-using-socat/
Jan 08, 2021
 
How can you test and debug programs using serial port communication when you
don’t have access to a physical serial port? Using the socat utility to create
virtual serial ports which pipe port traffic to custom executables using
stdin/stdout.

Here is the magic command you need:

```
socat -d -d -v pty,rawer,link=<PORT_NAME> EXEC:<COMMAND>,pty,rawer
```

It took me a while to find the correct combination of options to make this
work. Here’s what I’ve used...

The first pty options creates the virtual serial port. It turns out terminal
echo using `rawer`. The port is created using the file id given by the link
option. The `EXEC` option will run the executable (given by the `COMMAND`
value) and connect to the virtual serial port using stdin & stdout.

The `-d -d -v` options are used to print virtual port stream values to the
console as well as the target.

## Why would you want to do this?

Recently, I’ve been working on an embedded system which used a physical serial
port for communication. Application commands and device responses were
exchanged using the device’s serial port. As part of the development, there was
a series of system tests which ran from the development machine, sending
example commands to the device and then checking the responses against expected
values.

As part of the testing process, I wanted to run the system tests in the CI/CD
pipeline. This came with the challenge that the test environment would not have
the device connected to the serial port. Thinking about how to handle this, I
eventually settled on a neat solution using the socat utility.

Using the tool, I could create a virtual serial port in the test environment.
This virtual serial port was configured to pipe all data transferred to an
executable (using stdin and stdout). The embedded code was compiled into an
executable for the test platform - using a custom runner script and a mock
serial API which read from and wrote to stdin and stdout. The systems test just
needed to use the virtual port name to run in the test environment without
modification.

Being able to create virtual serial ports (which pipe data to stdin/stdout or
custom executables) is a really useful feature for testing and debugging
programs which use serial ports for communication!

# My Own Experience

It is nice to be able to "fake" a true serial port channel.  This can be done
as described above.  You can alternatively create two fake serial ports that
are connected to each other and communicate on one of them manually.

```
socat -d -d -v \
  pty,rawer,link=serial-connection-a \
  pty,rawer,link=serial-connection-b
```

Your program connects to one of them (e.g., `./serial-connection-a`) while
you manually connect to the other.  There are many ways to connect to it, but
perhaps the easiest is with the `screen` command


## Manual Serial Communication


### screen

```
screen ./serial-connection-b <BAUD_RATE>
```

where `<BAUD_RATE>` is the used baud rate for the serial communication, such as
9600.  Anything you type (or paste) into that screen session will be echoed
directly through the serial connection, and vice versa.  To kill the screen
session, press `Ctrl-A` and then `:kill`.  You will need to recreate the socat
virtual connection again if you want to use this virtual serial port.


### Arduino IDE Serial Monitor

You can use the serial monitor for real serial ports.  It doesn't easily see
the ones created by `socat`.  Perhaps there is something that can be done to
change that.


### tinyserial

This package comes with an app called `com`.  Usage is simple:

```
com <FILE> <BAUD>
```

where `<FILE>` is the serial port device, like `/dev/ttyACM0` or the link
created from `socat`, and `<BAUD>` is the baud rate.  Press `Ctrl-A` to exit
and `Ctrl-X` to list the status.


### picocom

A stripped-down version of `minicom`.  It's easy to use

```
picocom -b <BAUD> <FILE>
```

For example

```
picocom -b 9600 ~/serial-connection-a
```

You can get help with `Ctrl-A` then `Ctrl-H` to show you commands like changing
the baud rate, sending messages using hex values for characters, showing port
settings, etc.  This seems to be quite featured for doing simple manual
communication on a serial port connection, even a faked one through `socat`.


### minicom

Looks like there's a lot of setup here and ability to configure and use modems
to setup serial port communications.  Too many options that are now irrelevant.


### Conclusion

Of all of these tools, I think `picocom` is the easiest to use for the features
I care about.  However, `screen` is the easiest to remember since I use it for
multiplexing terminal emulation (i.e., tabs within a terminal).
