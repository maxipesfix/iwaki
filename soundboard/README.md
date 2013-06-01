Soundboard 
=====

Soundboard is an example app that uses Iwaki interaction manager library. 


## How to run?

To run Sounboard, type:

<pre>
soundboard 
-t < period of the main loop in seconds > 
-d < debug level > 
-l < location and prefix of the log file > 
-p < path of the top-level script directory containing init file > 
-i < name of the init file > 
-s < absolute path of the sound files > 
-x
</pre>

For example, from the h/iwaki/build directory on my computer I type:
<pre>
./soundboard/bin/soundboard -t 0.1 -d DEBUG4 -l log1 -p ~/h/iwaki/soundboard/scripts -i initialize_im.georgi.xml -s ~/h/iwaki/soundboard/sounds -x
</pre>

For a complete list of options run:
<pre>
soundboard -h
</pre>

## What can it do?

It contains recipes to demonstrate some capabilities of Iwaki interaction manager:

  - Timed actions: it will make a sound of a heartbeat at random intervals between 1 and 4 seconds.
  - Random actions: if you type "h" it will speak one of the two phrases, at random.
  - Conditionals/branching: if you time "how", it will say "Good, and you?" and then, depending on your answer "y" (yes, good) or "n" (not good) it will say different follow-ups.

## License

Copyright (C) 2012-2013 Maxim Makatchev. Released under GPLv3.