#include <iostream>
#include "log.h"
#include "iwaki.h"
#include <string>


#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include <fstream>

using std::string;
using std::iostream;

extern string sounds_path;

bool executeAction(Action &an_action) {
    string full_filename;

        /* in the future generate may handle other actions, perhaps */
    if (an_action.name == "generate_utterance") {
        string utterance_file;
        string utterance_token;
        string utterance_string;
        
            /* get the args*/
        for (std::list<ArgSlot>::iterator arg_it = an_action.args.begin();
             arg_it != an_action.args.end(); arg_it++) {
            FILE_LOG(logDEBUG1)  << "  arg name: " << arg_it->name;
            FILE_LOG(logDEBUG1)  << "  arg value: " << arg_it->value;
            FILE_LOG(logDEBUG1)  << "  arg type: " << arg_it->type;
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

            full_filename = sounds_path + utterance_file;

            ifstream sound_file(full_filename.c_str());
            
            if ( !sound_file)
            {
                FILE_LOG(logERROR) << "Can't find sound file:" << full_filename;
                return false;
            }
                /* using & to run gstreamer as a new processes to allow for
                 * asychnronous execution. -q quiets it although this
                 * doesn't prevent it with messing text_ui without &. */
            string exec_str = "gst-launch-0.10 -q playbin uri=file://" +
                full_filename + " &";
            const char *exec_cstr = (const char*)exec_str.c_str();
            int res = system(exec_cstr);
                
            FILE_LOG(logINFO)  << "System() returned value: " << res
                                << " when asked to execute command: " <<
                exec_str;
        }
    }
    return true;
}


void hndAction(Action &an_action, ActionStatus &astat) {

    FILE_LOG(logDEBUG1) << "Generator received action:";
    FILE_LOG(logDEBUG1)  << "  action name: " << an_action.name;
    FILE_LOG(logDEBUG1)  << "  action id: " << an_action.id;
    FILE_LOG(logDEBUG1)  << "  action actor: " << an_action.actor;
        /* make sure the action is intended for us */
    if (an_action.actor != "generator") {
        return;
    }

    if (!executeAction(an_action)) {
         FILE_LOG(logERROR)  << "There was an error executing action named: "
                             << an_action.name << ", action id: "
                             << an_action.id;
    }
    
        /* get the args*/
    for (std::list<ArgSlot>::iterator arg_it = an_action.args.begin();
         arg_it != an_action.args.end(); arg_it++) {
        FILE_LOG(logDEBUG1)  << "  arg name: " << arg_it->name;
        FILE_LOG(logDEBUG1)  << "  arg value: " << arg_it->value;
        FILE_LOG(logDEBUG1)  << "  arg type: " << arg_it->type;
    }


    
        /* send an action completed status when done */
    astat.status = "completed";
    astat.executor = "generator";
    astat.action_id = an_action.id;

    FILE_LOG(logDEBUG1) << "Generator sent status completed for action id: "
                        << an_action.id;
    return;
}
