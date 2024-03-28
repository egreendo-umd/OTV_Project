# OTV_Project

Create and develop on individual modules of the OTV, input and output is essential. 

Main.ino is intended to be what the OTV actually runs, using the various individual programs as "libraries".

Main: 

Propulsion:

Navigation:
    - Currently messed up, base working code is still there but I have a lot to clean up.

Overhead Vision System:

Payload: 

CI/CD:
    - Using pre-made formats specific to Arduino
    - https://github.com/arduino/compile-sketches#readme 
    
In case someone isn't familiar with the architecture format:

Dividing by topic
In general, divide your code along natural divisions in the program. Each file should contain a group of functions that do related things, e.g. manipulate the same data structure, are associated with the same class or closely-related group of classes, etc. Imagine another program that might use some of these functions: include in one file all the functions that this other program would require. Or try to write an English description of what's in each file: good divisions can usually be described succinctly.

Examples:

the set of functions that handle access to your database
the set of functions that handle access a device (a serial port, the graphics screen)
functions that do file or graphical input/output
the implementation of an abstract data type, e.g. a linked list or a binary search tree
a group of functions to do related numerical computations, e.g. a matrix manipulation package or a set of statistics functions
A large program might be divided into several such files, plus a main program file. The main file contains the function main, as well as miscellaneous functions that don't seem to fit anywhere else and/or seem specialized to this particular program.

There is no one "right" way to divide up a large program. Some programmers divide their programs into many, very tiny files. Some use a smaller number of larger files. The best choice depends not only on individual taste but on what sort of task the program is doing.

- Harvey Mudd College