Iwaki Interaction Manager
=========================

https://github.com/maxipesfix/iwaki
 

## ABOUT

This is the distribution of the Iwaki interaction manager. It includes two relevant libraries:

<pre>
queueio.a
iwaki.a
</pre>

and a sample application:

<pre>
soundboard
</pre>

A manual can be found in the PROJECTDIR/doc directory.

## TO INSTALL AND RUN on Linux or Mac OS X

### PREREQUISITES on Linux or Mac OS X

* RE2 regular expression library: http://code.google.com/p/re2/ 
  * make testinstall throws a "symbols not found" error on OSX, but that's OK.
* Additionally, to run the Soundboard app, you will need to install 
  * ncurses 
  * on Linux, gstreamer (on ubuntu it is the package `gstreamer0.10-tools`).
  * on Max OS X, nothing (Soundboard uses preinstalled `afplay`, which doesn't play ogg audio)


### TO INSTALL on Linux or Mac OS X

Make sure that CMakeLists.txt in this directory includes the correct location
of your RE2 library and header files.

In the PROJECTDIR directory (the location of this README) issue the following:

<pre>
mkdir build
cd build
cmake ..
make
</pre>

The library binaries are installed into:
PROJECTDIR/build/queueio/ and 
PROJECTDIR/build/iwaki/ 

directories. The soundboard executable is installed into:
PROJECTDIR/build/soundboard/bin/ .

Issuing 

<pre>
make install 
</pre>

moves header files and static libraries to /usr/local/include/ and usr/local/lib/ 

You are done!

### TO RUN On Linux or Mac OS X

To run soundboard app from PROJECTDIR/build directory, type:

<pre>
./soundboard/bin/soundboard -t 0.1 -d DEBUG4 -l log1 -p ../soundboard/scripts -i initialize_im.georgi.xml -s PROJECTDIR/soundboard/sounds -x
</pre>

where PROJECTDIR is the absolute path to the root of the iwaki installation tree (and the location of this file).

For more details on soundboard app, refer to README inside iwaki/soundboard/ directory.


## TO INSTALL AND RUN on Windows

Current Windows port is built using MinGW framework which does not support RE2 regular expression library. 
Other features appear to work well.

### PREREQUISITES on Windows

* Cmake (tested with cmake-gui)
* MinGW (tested with a 32-bit version)
* ncurses (download and build for MinGW32)
* Git Bash is helpful

### TO INSTALL on Windows

* Run cmake-gui with generators set to "MinGW Makefiles". 
* Using MinGW Shell or Git Bash terminal, inside your build directory (usually, iwaki/build), issue 

<pre>
mingw32-make
</pre>


### TO RUN on Windows

To run soundboard app from iwaki/build directory, use either Windows command line prompt (not Git Bash or MinGW Shell, 
as they don't work well with ncurses), or from a Windows explorer, and execute iwaki/soundboard.bat script. 
You should hear a heartbeat sound. 

For more details on soundboard app, refer to README inside iwaki/soundboard/ directory.

## LICENSE



###Iwaki Interaction Manager.

Copyright (C) 2012-2013 Maxim Makatchev, Reid Simmons, Carnegie Mellon University.


###Soundboard: play sounds in response to keyboard according to Iwaki scripts.

Copyright (C) 2012-2013 Maxim Makatchev.


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.