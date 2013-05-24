/*****************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * (c) Copyright 2009-2013 Maxim Makatchev, Reid Simmons,
 * Carnegie Mellon University. All rights reserved.
 *
 * FILE: outputs.cc
 *
 * ABSTRACT: outputs (action) methods
 *
 ****************************************************************/

#include <iostream>
#include <sstream>
#include "iwaki.h"
#include "log.h"
#include <string>
#include <csignal>

#include <sys/time.h>
#include <unistd.h>

#include <math.h>
#ifndef M_PI
#define M_PI 3.14159
#endif

//using namespace std;
using std::string;
using std::iostream;

/* the globals are defined in imcore.cc */
extern InteractionManager im;



/********************************
 * Task execution
 * ******************************/

bool InteractionManager::dispatchActionCommandAbort(int action_id, string &anActor) {

    FILE_LOG(logINFO) << "Attempting to abort action with id: " << action_id;

    if (this->actions.find(ABORT_ACTION_NAME) != this->actions.end()) {
            /* If action has been loaded from action scripts */
            /* create a new action object to send around to executors */
        Action an_action = this->actions[ABORT_ACTION_NAME];
        an_action.id = action_id;
        an_action.actor = anActor;                /* actor has to be the same as
                                                   * in the action being aborted */

            /* push action to the output queue */
        this->output_queue.push_back(an_action);
            /* publish the action on blackboard 
               BoostBBEntry<Action> bbAction(ACTION_DISPATCH_BB_VAR, Poll_Mode);
               bbAction = an_action;
            */
            /* TODO: copy action defaults to body element (like tomeout and if_timeout)
             * in case they are not set in body element */
    } else {
        FILE_LOG(logERROR) <<
            "Error in IM: could not find the following abort action in defaults file: " <<
            ABORT_ACTION_NAME;
    }

    
    return true;
}


/**
 * This is where the actions specified by the action body tag get 
 * dispatched.
 * recipe_name and active_element id are needed to access and update
 * random outcome history in derandomize method.
 * */

bool InteractionManager::dispatchAction(BodyElement &element1,
                                        Conjunction &bindings,
                                        int action_id, string recipe_name,
                                        int active_element) {
    FILE_LOG(logINFO) << "Attempting to dispatch action: " << element1.name;
    FILE_LOG(logDEBUG1) << "With its node bindings:";
    bindings.print(logDEBUG1);
    if (element1.actor == "user") {
        this->executeUserAction(element1, bindings, action_id);
            /* executing user action just puts on the list of pending actions and
             * always returns false initially. */
        return false;
    }
    else { /** actor is not a user (but some executor) */
        if (this->actions.find(element1.name) != this->actions.end()) {
            /* If action has been loaded from action scripts */
            /* create a new action object to send around to executors */
            Action an_action = this->actions[element1.name];
            an_action.id = action_id;

                                            /* pass body element attributes to action object.
                                             * Consider doing it in a separate function */
            if ((!element1.actor.empty()) && (element1.actor != "_NO_VALUE_")) {
                an_action.actor = element1.actor;
            } else if (an_action.actor.empty() || (an_action.actor == "_NO_VALUE_")) {
                FILE_LOG(logERROR) << "No actor specified in action: " << an_action.name
                                   << ", id: " << an_action.id << ", actor: " <<
                    an_action.actor;
            }

            if ((!element1.priority.empty()) && (element1.priority != "_NO_VALUE_")) {
                an_action.priority = element1.priority;
            } else {
                FILE_LOG(logDEBUG4) << "No priority specified in action: " << an_action.name
                                   << ", id: " << an_action.id << ", actor: " <<
                    an_action.actor << ", using default priority: " << an_action.priority;
            }
            
            FILE_LOG(logINFO) << "Created new action to dispatch. name: " <<
                an_action.name << ", id: " << an_action.id << ", actor: " <<
                an_action.actor << ", priority: " << an_action.priority;

            BodyElement derandElement = element1.derandomize(recipe_name,
                                                             (unsigned int) active_element,
                                                             this->history);

            for (unsigned int j=0;
                 j < this->history.recipeHistories[recipe_name].bodyElementHistories[active_element].outcome_age.size();
                 j++) {
                FILE_LOG(logDEBUG3) << "Returned outcome " << j << ", age: " <<
                    this->history.recipeHistories[recipe_name].bodyElementHistories[active_element].outcome_age[j];
            }
 
                /* bind the parameters of the action to their values.
                 * It's an IM method because actions are defined in queueio
                 * and do not know about conjunctions (bindings) */
                /* Also update he global bindings before binding action args
                 * so that the latter gets fresh globals */
            this->updateGlobalBindings();
            this->bindActionArgs(an_action, derandElement.args, bindings);
            an_action.return_args = element1.return_args;
                // this->bindActionArgs(an_action, element1.args, bindings);
            this->evalActionDatablocks(an_action);
                /* TOREMOVE: debugging output */
            an_action.print(logDEBUG2);
                /*push action onto the action queue */
            this->output_queue.push_back(an_action);
                /* publish the action on blackboard 
                   BoostBBEntry<Action> bbAction(ACTION_DISPATCH_BB_VAR, Poll_Mode);
                   bbAction = an_action;
                */
                /* TODO: copy action defaults to body element (like tomeout and if_timeout)
                 * in case they are not set in body element */
        } else {
            FILE_LOG(logERROR) <<
                "Error in IM: nothing known about how to dispatch action: " <<
                element1.name;
        }
    }
    return true;
}

/** executing user actions means putting it on the list of pending user actions 
 * so that the sem parser will match incoming actions against the list, 
 * will get the action_id and advance the relevant node just like
 * completed robot's actions do. */
	
bool InteractionManager::executeUserAction(BodyElement &element1,\
	 	Conjunction &bindings, int action_id) {
	
	FILE_LOG(logDEBUG1) << "Adding the user action on pending action list: " << element1.name;
	
	this->pending_user_actions.insert( make_pair(action_id, element1) );
	return true;
}
