I had a code base I wanted to run with Clang.  I tried compiling with clang,
only to have the compiler crash on code the GCC compiled just fine.  After I
found the culprit line, I realized my mistake, but it is still a bug in Clang.

I have two versions of the reduced bug.  The first was created using CReduce.
The second was created manually after identifying the one line of code that I
could comment out to cause it to stop crashing the compiler.

- minimal-creduce.cpp: generated from CReduce automatically from my source file
- minimal-manual-1.cpp: generated manually trying to recreate the conditions
  that cause the crash, first version
- minimal-manual-2.cpp: second version of the manually generated example.

This crashes on Ubuntu 18.04 using Clang++ 9 and Clang++ 10.

```
$ clang++-9 --version
clang version 9.0.0-2~ubuntu18.04.2 (tags/RELEASE_900/final)
Target: x86_64-pc-linux-gnu
Thread model: posix
InstalledDir: /usr/bin
```

```
$ clang++-10 --version
clang version 10.0.0-4ubuntu1~18.04.2 
Target: x86_64-pc-linux-gnu
Thread model: posix
InstalledDir: /usr/bin
```

The problem can be reproduced simply with

```
clang++-9 minimal-creduce.cpp
```

with any combination of clang 9 and 10 with any of the three files provided.
