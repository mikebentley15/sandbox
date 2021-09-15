The USB serial port is taken up by Micropython and its REPL interactive shell.  However, I can run code through the REPL that takes control of the input/output stream until it returns.

Some information is found here:

https://forum.micropython.org/viewtopic.php?t=7325

It appears from the `test_polling.py` example, that the stdin buffer only
returns as available when a newline is given.
