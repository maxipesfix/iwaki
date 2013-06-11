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
         << anActionMsg_p->id << cout;
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
        
        c = getopt_long (argc, argv, "hl:d:s:",
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
            }   
            default:
                 cout << "Unknown option. Try 'imcore --help' for more information.\n";
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
