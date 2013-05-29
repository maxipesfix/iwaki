/*****************************************************************************
 * PROJECT: Soundboard
 *
 * FILE: soundboard.h
 *
 * ABSTRACT: an header file for the Soundboard example.
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
 * */

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

  public:
    UICommand update(InteractionManager &im, int ch);
    void init();
    void close();
    void printPlanTree(PlanTree &tr, const int &tree_height);
    void printHeader();
    void printMessages(const int &msg_height);
    void printKeyboardBuffer();
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
    string keyboardBuffer;
};


/* action executor's function: in this case, a sound generator */
bool executeAction(Action &an_action);
void hndAction(Action &an_action, ActionStatus &astat);

/* Action completion status handler */
void hndActionCompletionStatus(ActionStatus &astat);

#endif // GIM_H
