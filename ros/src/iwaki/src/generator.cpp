/*****************************************************************************
 * PROJECT: iwaki
 *
 * FILE: generator.cpp
 *
 * ABSTRACT: This is an example executor of Iwaki actions: a sound file player.
 * It relies on gstreamer library and executes a console command,
 * so, most probably only runs on Unix.
 *
 * Iwakishi: a ROS wrapper around Iwaki interaction manager library.
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
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <string>
#include <fstream>

#include "ros/ros.h"
#include "iwaki/ActionMsg.h"
#include "iwaki/ActionStatusMsg.h"

using namespace std;

using std::string;
using std::iostream;

/**
 * GLOBALS
 * */

string sounds_path;
ros::Publisher actionStatus_pub;

bool executeAction(const iwaki::ActionMsg &anActionMsg) {
    string full_filename;
    
        /* in the future generate may handle other actions, perhaps */
    if (anActionMsg.name == "generate_utterance") {
        string utterance_file;
        string utterance_token;
        string utterance_string;
        
            /* get the args*/
        for (std::vector<iwaki::ArgSlot>::const_iterator arg_it = anActionMsg.args.begin();
             arg_it != anActionMsg.args.end(); arg_it++) {
             cout  << "  arg name: " << arg_it->name << endl;
             cout  << "  arg value: " << arg_it->value << endl;
             cout  << "  arg type: " << arg_it->type << endl;
            if (arg_it->name == "utterance_file") {
                utterance_file = arg_it->value;
            } else if (arg_it->name == "utterance_token") {
                utterance_token = arg_it->value;
            } else if (arg_it->name == "utterance_string") {
                utterance_string = arg_it->value;
            }
        }

        if (utterance_file != "_NO_VALUE_") {
                /* action included utterance file, ignore the rest */


            full_filename = sounds_path + "/" + utterance_file;

            ifstream sound_file(full_filename.c_str());
            
            if ( !sound_file)
            {
                cout << "Can't find sound file:" << full_filename << endl;
                return false;
            }
                /* using & to run gstreamer as a new processes to allow for
                 * asychnronous execution. -q quiets it although this
                 * doesn't prevent it with messing text_ui without &. */
            string exec_str = "gst-launch-0.10 -q playbin uri=file://" +
                full_filename + " &";
            const char *exec_cstr = (const char*)exec_str.c_str();
            int res = system(exec_cstr);

           
                
            cout  << "System() returned value: " << res
                  << " when asked to execute command: " << exec_str << endl;
        }
    } else {
        cout << "Don't know what to do with the action named: " << anActionMsg.name
             << endl;
    }
    return true;
}


/*
 * actionCallback implements the callback on the subscribed topic,
 * the actions Iwakishi interaction manager.
 * */

void actionCallback(const iwaki::ActionMsg::ConstPtr& anActionMsg_p) {
    cout << "Generator received action: " << endl;
    cout << "       action name: " << anActionMsg_p->name << endl;
    cout << "         action id: " << anActionMsg_p->id << endl;
    cout << "      action actor: " << anActionMsg_p->actor << endl;

    if (anActionMsg_p->actor != "generator") {
        return;
    }


    if (!executeAction(*anActionMsg_p)) {
        cout << "There was an error executing action named: "
             << anActionMsg_p->name << ", action id: "
             << anActionMsg_p->id << endl;
    }
    
    iwaki::ActionStatusMsg aStatusMsg;
            /* send an action completed status when done */
    aStatusMsg.status = "completed";
    aStatusMsg.executor = "generator";
    aStatusMsg.action_id = anActionMsg_p->id;

    actionStatus_pub.publish(aStatusMsg);
    cout << "Generator sent status completed for action id: "
         << anActionMsg_p->id << endl;
    return;

}




int main (int argc, char **argv)
{

        /* before reading the app's arglist */
    ros::init(argc, argv, "generator");
    
    opterr = 0;
    char* cvalue;
    int c;

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
                {"sounds",   required_argument, 0, 's'},
                {0, 0, 0, 0}
            };
            /* getopt_long stores the option index here. */
        int option_index = 0;
        
        c = getopt_long (argc, argv, "hs:",
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
  -s --sounds SOUND_DIR          sound file directory \n\
";            
                cout << "\
  -h --help                  This info." << endl;
                return -1; 
            case 's':
            {
                cvalue = optarg;
                sounds_path = (string) ((const char*) cvalue);
                break;
            }   
            default:
                 cout << "Unknown option. Try 'generator --help' for more information.\n";
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

    if (sounds_path.empty()) {
        cout << "Missing path to sounds directory. Pass it with a command line option -s."
             << endl;
    }


        /* ROS stuff */
    ros::NodeHandle n;
    ros::Subscriber action_sub = n.subscribe("IwakiAction", 1000, actionCallback);
        /* publisher object is global so that callback can see it */
    actionStatus_pub = n.advertise<iwaki::ActionStatusMsg>("IwakiActionStatus", 1000);
        /******************************
         *
         * Main loop
         * 
         ******************************/
    ros::Rate loop_rate(10);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
