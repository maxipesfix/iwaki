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



## PREREQUISITES


* RE2 regular expression library: http://code.google.com/p/re2/
* Additionally, to run the Soundboard app, you will need to install 
  * ncurses 
  * gstreamer (ubuntu package gstreamer0.10-tools)


## TO INSTALL on Linux


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
sudo make install 
</pre>

moves header files and static libraries to /usr/local/include/ and usr/local/lib/ 

You are done!

To run soundboard app from PROJECTDIR/build directory, type:

<pre>
./soundboard/bin/soundboard -t 0.1 -d DEBUG4 -l log1 -p ../soundboard/scripts -i initialize_im.georgi.xml -s PROJECTDIR/soundboard/sounds -x
</pre>

where PROJECTDIR is the absolute path to the root of the iwaki installation tree (and the location of this file).

For more details on soundboard app, refer to README inside iwaki/soundboard/ directory.



## LICENSE



Iwaki Interaction Manager
Copyright (C) 2012-2013 Maxim Makatchev, Reid Simmons, 
Carnegie Mellon University.

Soundboard: play sounds in response to keyboard according to Iwaki scripts
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