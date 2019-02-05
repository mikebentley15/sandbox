The file lbr4_trajopt.py is the one to run to generate an optimized trajectory
and command it in V-REP. It offers a command line option interface. To see
all the options, in a terminal type

    python lbr4_trajopt.py -h

If you wanted to run problem 1.1 with default falues, you would type

    python lbr4_trajopt.py -p 1.1

OR you can also do the long form of

    python lbr4_trajopt.py --problem 1.1

If you want to run problem 1.2 with a gamma value of 0.1:

    python lbr4_trajopt.py -p 1.2 -g 0.1

Long form:

    python lbr4_trajopt.py --problem 1.2 --gamma 0.1

If you want to run problem 2 with a gamma value of 0.1 and beta value of 5:

    python lbr4_trajopt.py -p 2 -g 0.1 -b 5

Long form:

    python lbr4_trajopt.py --problem 2 --gamma 0.1 --beta 5