/*****************************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * (c) Copyright 2009-2013 Maxim Makatchev, Reid Simmons,
 * Carnegie Mellon University. All rights reserved.
 *
 * FILE: soundboard.cc
 *
 * ABSTRACT: an example use of the library. This is a simple application
 * that takes terminal input and plays sound files according to the recipes.
 *
 ****************************************************************/

#ifndef SOUNDBOARD_H
#define SOUNDBOARD_H

#include "iwaki.h"
#include <iostream>
#include <deque>


/* if uncommented, the action is considered completed right
 * after it is dispatched. When using true executive modules,
 * comment this define out.
 * */
#define NO_EXEC

#define TICK_BB_VAR "tick"
#define TICK_FORMAT "int"

#define OUTPUT_IM_STRING "output_im_string"
#define OUTPUT_IM_FORMAT "{int, string}"
#define INVITE_IM_STRING "invite_im_string"
#define INVITE_IM_FORMAT "{int, string}"

#define ACTION_DISPATCH_BB_VAR "action_dispatch"
#define ACTION_STATUS_BB_VAR "action_status"

#define ACTION_TIMING_BB_VAR "action_timing"
#define ACTION_TIMING_RESPONSE_BB_VAR "action_timing_response"
#define ACTION_TIMING_RESPONSE_FORMAT "{int, float}"


#define BB_USER_RECORD "user_record"
#define BB_USER_RECORD_TYPE User
#define MAX_UI_MSGS 100
#define MAX_LOGFILE_SIZE 100000000
#define MAX_ELEMENT_AGE 1000
#define MAX_UI_TICK_ID 100000

#define ABORT_ACTION_NAME "abort_action"

enum UICommand {uiQuit, uiNone};
enum VerbosityLevel {Brief, Verbose, WithBodyElements};

/**
 * TextUI is the object that provides terminal-based user interface
 * */
class TextUI{
  public:
        // default constructor
    TextUI(): verbosity(Brief), msgPaneHeight(5), minTreeHeight(3), tick_id(0) { }
    
    UICommand update(InteractionManager &im);
    void init();
    void close();
    void printPlanTree(PlanTree &tr, const int &tree_height);
    void printHeader();
    void printMessages(const int &msg_height);
    void push_msg(const string &msg);
    void processBodyElementDescriptions();
    void setBodyElementDescriptions(Node &aNode);
    void updateBodyElementDescriptions(Node &aNode);
    void updateBodyElementDescription(Node &aNode, int element_id);
    string makeFutureActionDescription(BodyElement &element);
    string makePastActionDescription(BodyElement &element);
    string makeActionDescription(BodyElement &element, int active_element, int element_id);
    std::map<string, int> getAtomSubtypeCounts(Conjunction &gBindings);
    void printGlobals(Conjunction &gBindings);
  public:
    VerbosityLevel verbosity;
    int msgPaneHeight;
    int minTreeHeight;
    std::list< string > messages;
    int tick_id;
};


/* action executor's function: in this case, a sound generator */
bool executeAction(Action &an_action);
void hndAction(Action &an_action, ActionStatus &astat);

/* Action completion status handler */
void hndActionCompletionStatus(ActionStatus &astat);

#endif // GIM_H
