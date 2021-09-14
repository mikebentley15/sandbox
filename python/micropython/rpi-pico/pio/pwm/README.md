## Goal

We want to control a stepper motor at 16th microsteps where 200 full steps is a
full rotation.  This means a full rotation will be 3,200 microsteps.  Using a
PIO state machine, I hope to specify a value to the TX queue of that state
machine to control the speed of that motor.  The direction pin can be
controlled by the software, that's fine (that's just a pin that gets set to
high or low to indicate forward or backward, and shouldn't need to be triggered
often).

The signal is being sent to an A4988 stepper motor controller chip.  These
chips essentially will move one step for each pulse it receives.  Therefore, to
have one revolution per second, we would simply send 3,200 steps (where both
the high and low values are set for at least one microsecond) equally spaced
apart.

The maximum speed I will require is probably 3 revolutions per second, if that.
That amounts to 9,600 steps per second, which is easily doable with a 125 MHz
clock.  The trouble is that I want to control many different step sizes below
that.  The control given is likely to be an amount to sleep between pulses.  I
think the slowest speed I want to support would be one step per second, so
controlling anywhere between 9,600 steps to one step per second using a single
32-bit value sent to the PIO state machine.

For 10,000 steps per second (rounded up from 9,600 steps per second), we would
have a time distance between steps of 0.0001 seconds, or 100 microseconds.  For
1 step per second, we would have a time distance between steps of 1 second, or
1,000,000 microseconds.  Perhaps if the passed number could mean the number of
microseconds to wait longer than the 100 base microseconds.

If we have the state machine running at 1,000,000 Hz, then each clock cycle is
one microsecond.  The base implementation (for an initialized value of zero)
should then take 200 cycles to complete.  If `y` represents the value passed in
(for microseconds beyond the 100 microsecond base timing), then the number of
cycles should be equal to `100 + y`.  This can easily be done with a 32-bit
integer specifying the number of microseconds (in fact, we only need 20 bits to
express that number).  With such an implementation, the number of steps per
second becomes

|  y  | steps per sec | delta |
|-----|---------------|-------|
|  0  | 10,000        |   -   |
|  1  |  9,901        |  99   |
|  2  |  9,804        |  97   |
|  3  |  9,709        |  95   |
|  4  |  9,615        |  94   |
|  5  |  9,524        |  91   |
|  6  |  9,434        |  90   |
|  7  |  9,346        |  88   |
|  8  |  9,259        |  87   |
|  9  |  9,174        |  85   |
| 10  |  9,091        |  83   |
| 11  |  9,009        |  82   |
| 12  |  8,929        |  80   |
| 13  |  8,850        |  79   |
| 14  |  8,772        |  78   |
| 15  |  8,696        |  76   |
| 16  |  8,621        |  75   |
| 17  |  8,547        |  74   |
| 18  |  8,475        |  72   |
| 19  |  8,403        |  72   |
| 20  |  8,333        |  70   |
| 21  |  8,264        |  69   |
| 22  |  8,197        |  67   |
| 23  |  8,130        |  67   |
| 24  |  8,065        |  65   |
| 25  |  8,000        |  65   |
| 26  |  7,937        |  63   |
| 27  |  7,874        |  63   |
| 28  |  7,812        |  62   |
| 29  |  7,752        |  60   |
| 30  |  7,692        |  60   |
| 31  |  7,634        |  58   |
| 32  |  7,576        |  58   |
| 33  |  7,519        |  57   |
| 34  |  7,463        |  56   |
| 35  |  7,407        |  56   |
| 36  |  7,353        |  54   |
| 37  |  7,299        |  54   |
| 38  |  7,246        |  53   |
| 39  |  7,194        |  52   |
| 40  |  7,143        |  51   |
| 41  |  7,092        |  51   |
| 42  |  7,042        |  50   |
| 43  |  6,993        |  49   |
| 44  |  6,944        |  49   |
| 45  |  6,897        |  47   |
| 46  |  6,849        |  48   |
| 47  |  6,803        |  46   |
| 48  |  6,757        |  46   |
| 49  |  6,711        |  46   |
| 50  |  6,667        |  44   |
| ... |    ...        | ...   |
| 100 |  5,000        | ...   |
| ... |    ...        |  ...  |
| 999,900 |  1      |  ...  |


If instead we were to go at 2 MHz speed and have `200 + y` cycles, it would then become

|  y  | steps per sec | delta |
|-----|---------------|-------|
|  0  |   10,000      |   50  |
|  1  |    9,950      |   50  |
|  2  |    9,901      |   49  |
|  3  |    9,852      |   49  |
|  4  |    9,804      |   48  |
|  5  |    9,756      |   48  |
|  6  |    9,709      |   47  |
|  7  |    9,662      |   47  |
|  8  |    9,615      |   47  |
|  9  |    9,569      |   46  |
| 10  |    9,524      |   45  |
| 11  |    9,479      |   45  |
| 12  |    9,434      |   45  |
| 13  |    9,390      |   44  |
| 14  |    9,346      |   44  |
| 15  |    9,302      |   44  |
| 16  |    9,259      |   43  |
| 17  |    9,217      |   42  |
| 18  |    9,174      |   43  |
| 19  |    9,132      |   42  |
| 20  |    9,091      |   41  |
| 21  |    9,050      |   41  |
| 22  |    9,009      |   41  |
| 23  |    8,969      |   40  |
| 24  |    8,929      |   40  |
| 25  |    8,889      |   40  |
| 26  |    8,850      |   39  |
| 27  |    8,811      |   39  |
| 28  |    8,772      |   39  |
| 29  |    8,734      |   38  |
| 30  |    8,696      |   38  |
| 31  |    8,658      |   38  |
| 32  |    8,621      |   37  |
| 33  |    8,584      |   37  |
| 34  |    8,547      |   37  |
| 35  |    8,511      |   36  |
| 36  |    8,475      |   36  |
| 37  |    8,439      |   36  |
| 38  |    8,403      |   36  |
| 39  |    8,368      |   35  |
| 40  |    8,333      |   35  |
| 41  |    8,299      |   34  |
| 42  |    8,264      |   35  |
| 43  |    8,230      |   34  |
| 44  |    8,197      |   33  |
| 45  |    8,163      |   34  |
| 46  |    8,130      |   33  |
| 47  |    8,097      |   33  |
| 48  |    8,065      |   32  |
| 49  |    8,032      |   33  |
| 50  |    8,000      |   32  |
| ... |    ...        |  ...  |
| 100 |    6,667      |  ...  |
| ... |    ...        |  ...  |
| 1,999,800 |  1      |  ...  |

However, going at the full speed of 125 MHz, we have `12,500 + y` cycles per
pulse (if that's even possible, but I think it may be).  That would make a 

|  y  | steps per sec | delta |
|-----|---------------|-------|
|   0   |   10000  |  1  |
|   1   |   9999  |  1  |
|   2   |   9998  |  1  |
|   3   |   9998  |  0  |
|   4   |   9997  |  1  |
|   5   |   9996  |  1  |
|   6   |   9995  |  1  |
|   7   |   9994  |  1  |
|   8   |   9994  |  0  |
|   9   |   9993  |  1  |
|   10   |   9992  |  1  |
|   11   |   9991  |  1  |
|   12   |   9990  |  1  |
|   13   |   9990  |  0  |
|   14   |   9989  |  1  |
|   15   |   9988  |  1  |
|   16   |   9987  |  1  |
|   17   |   9986  |  1  |
|   18   |   9986  |  0  |
|   19   |   9985  |  1  |
|   20   |   9984  |  1  |
|   21   |   9983  |  1  |
|   22   |   9982  |  1  |
|   23   |   9982  |  0  |
|   24   |   9981  |  1  |
|   25   |   9980  |  1  |
|   26   |   9979  |  1  |
|   27   |   9978  |  1  |
|   28   |   9978  |  0  |
|   29   |   9977  |  1  |
|   30   |   9976  |  1  |
|   31   |   9975  |  1  |
|   32   |   9974  |  1  |
|   33   |   9974  |  0  |
|   34   |   9973  |  1  |
|   35   |   9972  |  1  |
|   36   |   9971  |  1  |
|   37   |   9970  |  1  |
|   38   |   9970  |  0  |
|   39   |   9969  |  1  |
|   40   |   9968  |  1  |
|   41   |   9967  |  1  |
|   42   |   9967  |  0  |
|   43   |   9966  |  1  |
|   44   |   9965  |  1  |
|   45   |   9964  |  1  |
|   46   |   9963  |  1  |
|   47   |   9963  |  0  |
|   48   |   9962  |  1  |
|   49   |   9961  |  1  |
|   50   |   9960  |  1  |
| ... |     ...       |  ...  |
| 100 |    9,921      |  ...  |
| ... |    ...        |  ...  |
| 124,987,500 |  1      |  ...  |

If this is achievable, then I would like to make it happen because it would be
amazing resolution.  The limiting factors are the limited instruction set
(e.g., no arithmatic or any way to set a value larger than 31), the limited
registers (just x and y, unless you also use osr and isr), and the limited
space of instructions (32 instructions total shared between all state machines
on a single PIO - of which we have two).


## PIO Emulation Scripts

`*.mon` files are to be run by the PIO Emulator found here:

https://github.com/soundpaint/rp2040pio

This is to be cloned, then compiled with `make`.  Then you can run these files
by starting the server in one shell

```
java -jar jar/rp2040pio_server.jar
```

and then launching the `*.mon` file using the monitor (suppose we are using
`blink_fastest.mon` here in the `rp2040/`, which is the current directory).

```
java -jar jar/rp2040pio_monitor.jar --file=blink_fastest.mon
```

This will execute the commands one at a time from that file.  It's a good idea to also launch some of the observers, each on in their own separate shell

```
java -jar jar/rp2040pio_fifoobserver.jar
```

```
java -jar jar/rp2040pio_codeobserver.jar
```

```
java -jar jar/rp2040pio_diagram.jar
```

```
java -jar jar/rp2040pio_gpioobserver.jar
```
