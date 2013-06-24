/*****************************************************************************
 * PROJECT: Soundboard
 *
 * FILE: soundboard.cc
 *
 * ABSTRACT: an example use of the Iwaki library. This is a simple application
 * that takes terminal input and plays sound files according to the recipes.
 *
 * SAMPLE COMMAND LINE: ./soundboard/bin/soundboard -t 0.1 -d DEBUG4 -l log1
 *                       -p ../soundboard/scripts
 *                       -i initialize_im.georgi.xml
 *                       -s ~/h/iwaki/soundboard/sounds -x
 *
 * Soundboard: play sounds in response to keyboard according to Iwaki scripts
 * Copyright (C) 2012-2013 Maxim Makatchev.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 ****************************************************************/

#include <iostream>
#include <sstream>
#include "soundboard.h"
#include "log.h"
#include <string>
#include <csignal>
#include <algorithm>
#include <ncurses.h> /* for getch() */

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
     

#include <math.h>


//using namespace std;
using std::string;
using std::iostream;


/**
 * GLOBALS
 * */

InteractionManager im;
TextUI textUI;
string sounds_path; /* path to the directory with sound files */

#define KB_ENTER int('\n')

/*
 * Process the output Action queue.
 * */

void dispatchActionsFromOutputQueue() {

    std::list<Action>::iterator action_it = im.output_queue.begin();

    FILE_LOG(logDEBUG4) << "Action queue has length: " <<
        im.output_queue.size();
    while( action_it != im.output_queue.end() ) {
            /* hndAction:
             * here we dispatch the action from the queue.
             * Generally, since an action can take time longer than allowed
             * for the tick of the main loop,
             * hndAction and hndActionCompletionStatus
             * should be done asynchronously from this process. */
        ActionStatus astat;
        hndAction(*action_it, astat);
            /* hndActionCompletionStatus:
             * here we pass the action completion status to the IM. We
             * do this synchroneously because in our example hndAction takes very
             * short time (it creates a new gstreamer process) and the ActionStatus
             * is returned right away, before the gstreamer finishes
             * playing the sound. Generally,
             * hndActionCompletionStatus should be called whenever an
             * asychnroneous ActionStatus signal arrives . */
        hndActionCompletionStatus(astat);
        
        action_it++;
        im.output_queue.pop_front();
    }
}

/*
 * handle action completion status
 * */
void hndActionCompletionStatus(ActionStatus &astat) {

    FILE_LOG(logDEBUG3) << "IM received action completion status: ";
    astat.print(logDEBUG3);

    im.processActionCompletionStatus(astat);
    im.ptree.print(logDEBUG3);	
}

/*
 * handle user input
 * */
void hndUserInput(string &utterance) {
    FILE_LOG(logINFO) << "INFO: received user utterance: " + utterance;
    textUI.push_msg("INFO: received user utterance: " + utterance);

    Atom new_atom;
    VarSlot v1, v2, v3, v4, v5, v6;

    v1.name = "type";
    v1.val = "im";
    v1.unique_mask = true;
	
    v2.name = "subtype";
    v2.val = "user";
    v2.unique_mask = true;
	
    v3.name = "uu_unhandled";
    v3.val = "true";

    v4.name = "uu_string";
    v4.val = utterance;

    v5.name = "uu_timestamp";
    v5.val = to_string((double)getSystemTimeMSec()/1000.0);

    v6.name = "id";
    v6.val = "1";


    new_atom.varslots.push_back(v1);
    new_atom.varslots.push_back(v2);
    new_atom.varslots.push_back(v3);
    new_atom.varslots.push_back(v4);
    new_atom.varslots.push_back(v5);
    new_atom.varslots.push_back(v6);
    
    im.input_queue.push_back(new_atom);
}

void updateKbBuffer(string &kb_buffer, int ch) {
    char buffer[8];
    sprintf (buffer, "%c", ch);
    if ((ch >= 32) && (ch <= 126)) {
            /* legal visible characters */
        kb_buffer += string(buffer);
    } else if (ch == KEY_BACKSPACE) {
            /* apparently this symbol is already defined */
        if (!kb_buffer.empty()) {
            kb_buffer = kb_buffer.substr(0, kb_buffer.size() - 1);
        }
    } else if (ch == KB_ENTER) {

            /* hndUserInput */
        hndUserInput(kb_buffer);
        kb_buffer = "";
    }
}




int main (int argc, char **argv)
{       /* parse argument line */
    opterr = 0;
    char* cvalue;
    string logfile_name;
    bool text_ui = false;
    string debug_level;
    unsigned int timer_period_microsec = 1000000;
    int c;
    UICommand ui_command;
    int ch;             /* char for runtime keyboard input */
    string kb_buffer;   /* the keyboard buffer master copy */

    while (1)
    {
        static struct option long_options[] =
            {
                    /* These options set a flag. */
                    //{"verbose", no_argument,       &verbose_flag, 1},
                    //{"brief",   no_argument,       &verbose_flag, 0},
                    /* These options don't set a flag.
                       We distinguish them by their indices. */
                {"help",    no_argument,       0, 'h'},
                {"text_ui",    no_argument,       0, 'x'},
                {"log", required_argument, 0, 'l'},
                {"debug",   required_argument, 0, 'd'},
                {"timer",   required_argument, 0, 't'},
                {"init",       required_argument, 0, 'i'},
                {"atoms",       required_argument, 0, 'a'},
                {"sounds",       required_argument, 0, 's'},
                {"path",       required_argument, 0, 'p'},
                {0, 0, 0, 0}
            };
            /* getopt_long stores the option index here. */
        int option_index = 0;
        
        c = getopt_long (argc, argv, "hxl:d:t:i:a:p:s:",
                         long_options, &option_index);
        
            /* Detect the end of the options. */
        if (c == -1)
            break;
     
        switch (c) {
             case 0:
               /* If this option set a flag, do nothing else now. */
               if (long_options[option_index].flag != 0)
                 break;
               printf ("option %s", long_options[option_index].name);
               if (optarg)
                 printf (" with arg %s", optarg);
               printf ("\n");
               break;

            case 'h':
                cout << "\
Usage: imcore [OPTION]... \n\
";
                cout << "Mandatory arguments to long options are mandatory for short options too.\n\
";
                cout << "\
  -d --debug DEBUG_LEVEL     DEBUG_LEVEL is one of the \n              \
                             following: ERROR, WARNING, INFO, \n       \
                             DEBUG, DEBUG1, DEBUG2, DEBUG3, DEBUG4\n\
";
                cout << "\
  -t --timer TIMER_PERIOD    if present, this option causes \n \
                             IM to run in timer mode, with every TIMER_PERIOD\n\
                             seconds IM state update is forced. If the option is\n\
                             not present IM runs in callback mode\n\
";
                cout << "\
  -l --log LOG_FILE          if present, this option causes \n        \
                             all the debug output be directed to the LOG_FILE\n\
";
                cout << "\
  -i --init INIT_FILE        name of the IM init file that overrides\n\
                             default initialize_im.xml\n\
";
                cout << "\
  -a --atoms DEFAULTS_FILE   name of the atom defaults file that overrides\n\
                             default default_atoms.xml\n\
";
                cout << "\
  -p --path PATH             path to recipes, actions and functions directories\n\
";
                cout << "\
  -s --sounds PATH           absolute path to sounds files \n\
";
                cout << "\
  -x --text_ui               turn on text ui. will supress normal debug and error \n\
                             terminal output\n\
";
                
                cout << "\
  -h --help                  This info." << endl;
                return -1;
            case 'x':
                text_ui = true;
                break;
            case 'd':
                cvalue = optarg;
                debug_level=string((const char*) cvalue);
                break;
            case 'l':
            {
                cvalue = optarg;
                logfile_name = (string)((const char*) cvalue);
                if (!setLogFile(logfile_name)) {
                    return -1;
                }
                break;
            }
            case 't':
            {
                cvalue = optarg;
                std::string  cvalue_str = string((const char*) cvalue);
                double timer_period_sec = string_to_double(cvalue_str);
                if ((timer_period_sec > 1.0)||(timer_period_sec <= 0)) {
                    cout << "Timer period should be positive and not more \
                    than 1.0 seconds." << endl;
                 return -1;
                }
                timer_period_microsec =
                    (unsigned int) floor(timer_period_sec*1000000);
                break;
            }
            case 'i':
            {
                cvalue = optarg;
                im.init_file_name = (string) ((const char*) cvalue);
                break;
            }
            case 'a':
            {
                cvalue = optarg;
                im.default_atoms_file_name = (string) ((const char*) cvalue);
                break;
            }
            case 'p':
            {
                cvalue = optarg;
                im.script_path = (string) ((const char*) cvalue);
                break;
            }
            case 's':
            {
                cvalue = optarg;
                sounds_path = (string) ((const char*) cvalue);
                break;
            }
            default:
                 cout <<
                     "Unknown option. Try 'soundboard --help' for more information.\n";
                 return -1;
        }
    }

            /* Print any remaining command line arguments (not options). */
    if (optind < argc)
    {
        printf ("non-option ARGV-elements: ");
        while (optind < argc)
            printf ("%s ", argv[optind++]);
        putchar ('\n');
    }

    FILELog::ReportingLevel() =
        FILELog::FromString(debug_level.empty() ? "DEBUG1" : debug_level);


    /* Verify that the command line args were passed */
    if (sounds_path.empty()) {
      cout << "ERROR: Don't know where sound files are." << endl; 
      cout << "       Use -s option to pass the absolute path to the sound file directory."  << endl;
      return -1;
    }
    
        /* set IM params */
    im.timer_period_microsec = timer_period_microsec;
    
    if (!im.initialize())
    {
        FILE_LOG(logERROR)  << "Initialization of IM failed.";
    } else {
        FILE_LOG(logINFO) << "Initialization of IM succeeded.";
    }
        //im.printRecipes();
        //im.printTriggerables();

        /* initialize text UI */
    if ( text_ui ) {
        textUI.init();
    }

    FILE_LOG(logINFO) << "Running timer with the period of " <<
        timer_period_microsec << " microseconds." << endl;
        /******************************
         *
         * Main loop
         * 
         ******************************/
    while (1) {
        double start_tick = getSystemTimeMSec()/1000.0;
        FILE_LOG(logDEBUG4) <<
            "############ Re-entering the main loop at: "
                            << setprecision(20) << start_tick ;
        
        im.doTick();
        
            /* Dispatch actions collected at the output queue */
        dispatchActionsFromOutputQueue();

        
            /* Process keyboard input */
        ch = getch();

        updateKbBuffer(kb_buffer, ch);
            /* if running with text_ui, do the terminal screen update */
        if (text_ui) {
                /* pass kb_buffer to textUI for printing */
            textUI.keyboardBuffer = kb_buffer;  
            ui_command =  textUI.update(im, ch);
            if ( ui_command == uiQuit) {
                textUI.close();
                FILE_LOG(logINFO) << "TextUI requested quit. Goodbye.";
                return -1;
            }
        }

            /*
             * Logging
             * */
        
            /* check if need to swap log file */
        FILE* pStream = Output2FILE::Stream();
            // fseek( pStream, 0, SEEK_END );
        int logfile_size = ftell( pStream );
        FILE_LOG(logDEBUG3) << "Logfile size: " << logfile_size;
        if (logfile_size > MAX_LOGFILE_SIZE ) {
                /* move on to a new logfile */
            if (!setLogFile(logfile_name)) {
                return -1;
            }
        }
        
        double end_tick = getSystemTimeMSec()/1000.0;
        
        FILE_LOG(logDEBUG4) <<
            "############ Done tick at: " << setprecision(20) <<
            end_tick ;
        
        FILE_LOG(logDEBUG4) <<
            "############ Cycle took: " << setprecision(20) <<
            (end_tick - start_tick) << " seconds.";
        
            /** print sybcycle timings **/
        if (im.timing_dump_counter == 0) {
            FILE_LOG(logINFO) << "Print subcycle timings...";
            for (std::map<string, std::list<double> >::const_iterator
                     map_it = im.subcycle_timings.begin();
                 map_it != im.subcycle_timings.end(); map_it++) {
                string timings_str;
                
                for (std::list<double>::const_iterator
                         list_it = map_it->second.begin();
                     list_it != map_it->second.end(); list_it++) {
                    timings_str = timings_str + to_string(*list_it) + ", ";
                }
                
                FILE_LOG(logINFO) << "Timing subcycle " << map_it->first
                                  << ": " << timings_str;
            }
            im.ptree.print(logINFO);
            im.getGlobalBindings()->print(logINFO);
            FILE_LOG(logINFO) << "Print subcycle timings...";   
        }
        
        if ((end_tick - start_tick) < (timer_period_microsec/1000000.0)) {
            FILE_LOG(logDEBUG4) <<
                "########### intentially wasting: " << setprecision(20) <<
                ((timer_period_microsec/1000000.0) - (end_tick - start_tick)) <<
                " seconds."; 
            usleep(timer_period_microsec - (end_tick - start_tick)*1000000.0);
        } else {
            FILE_LOG(logWARNING) <<
                "########### sampling period is short of: " << setprecision(20) <<
                ((end_tick - start_tick) - (timer_period_microsec/1000000.0)) <<
                " seconds.";
            im.ptree.print(logWARNING);
            im.getGlobalBindings()->print(logWARNING);
        }

            /*
             * End logging
             * */
    }
        /*********
         * End of main loop
         * *******/
    
}
