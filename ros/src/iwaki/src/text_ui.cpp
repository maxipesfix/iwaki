/*****************************************************************************
 * PROJECT: Iwakishi
 *
 * FILE: text_ui.cc
 *
 * ABSTRACT: an ncurses library based text interface with Iwaki interaction
 * manager.
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
*/


#include <iostream>
#include <sstream>
#include "iwakishi.h"
#include <algorithm>
#include <string>

#include <locale.h>
#include <ncursesw/ncurses.h>

#define KEY_SMALL_Q 113
#define KEY_SMALL_V 118
#define KEY_CONTROL_SMALL_Q 17
#define KEY_CONTROL_SMALL_V 22
#define MAX_NODE_WIDTH 20
#define ROOT_NODE_WIDTH 4


UICommand TextUI::update(InteractionManager &im, int ch) {
    UICommand ui_command = uiNone;
    int nrow, ncol, msg_height, tree_height, globals_height = 1, header_height = 2;
    int keyboard_buffer_height = 1;


        /*
         * decide the layout dimensions
         * */
    getmaxyx(stdscr, nrow, ncol);    /*  figure out screen geomery */

    if (nrow <= this->msgPaneHeight + header_height + this->minTreeHeight) {
        msg_height = 0;               /* no msg pane if screen is short */
    } else {
        msg_height = this->msgPaneHeight;
    }
    tree_height = nrow - header_height - msg_height - globals_height -
        keyboard_buffer_height;


        /*
         * do the printing
         */
    clear();

    move(0,0);
    this->printHeader();

    move(header_height, 0);
    this->printPlanTree(im.ptree, tree_height);

    move(header_height + tree_height, 0);
    this->printGlobals( *im.getGlobalBindings() );

    move(header_height + tree_height + globals_height, 0);
    this->printMessages(msg_height);

    move(header_height + tree_height + globals_height + msg_height, 0);
    this->printKeyboardBuffer();

        /*
         * read user input
         * (do it outside) of TextUI::update
         * 
         ch = getch();	*/	/* If raw() hadn't been called
                                 * we have to press enter before it
				 * gets to the program 		*/
    
    if ( ch == KEY_CONTROL_SMALL_Q ) {
        printw("Goodbye");
        ui_command = uiQuit;
    } else if ( ch == KEY_CONTROL_SMALL_V ) {  /* toggle verbose mode */
        move(1,0);
        if (this->verbosity == WithBodyElements) {
            printw("brief");
            this->verbosity = Brief;           
        } else if (this->verbosity == Brief) {
            printw("verbose");
            this->verbosity = Verbose;
        } else { /* second to last verbosity, Verbose */
            printw("with body elements");
            this->verbosity = WithBodyElements;
        }
    }

    
    refresh();			/* Print it on to the real screen */

        /* tick_id is used as a counter for dynamic features, like random choices */
    if (this->tick_id >= MAX_UI_TICK_ID) {
        this->tick_id = 0; 
    } else {
        this->tick_id++;
    }
    
    return ui_command;
}

void TextUI::init() {
    setlocale(LC_CTYPE,"");       /* for Unicode support */
    initscr();			/* Start curses mode */
    raw();			/* Line buffering disabled	*/
    keypad(stdscr, TRUE);	/* We get F1, F2 etc..		*/
    noecho();			/* Don't echo() while we do getch */
    timeout(0);
}


void TextUI::close() {
    endwin();			/* End curses mode */
}


void TextUI::printPlanTree(PlanTree &ptree, const int &tree_height) {
    int y = 0, x = 0, dy = 0, depth_old = 0, depth = 0, dx = 0;
    
    getyx(stdscr, y, x);
    
    if (this->verbosity == Brief) {                           /* brief view */
        tree<Node>::iterator sib2=ptree.tr.begin();
        tree<Node>::iterator end2=ptree.tr.end();
        while (( sib2!=end2 ) && (dy < tree_height)) {
            dx = ptree.tr.depth(sib2);
            move(y + dy, x + dx);
            printw( (*sib2).recipe_name.c_str() );
            ++sib2;
            ++dy;
        }
    } else if (this->verbosity == Verbose) {         /* verbose view
                                                     * includes one liner of number of currently active
                                                     * element, total number of elements
                                                     * */
        tree<Node>::iterator sib2=ptree.tr.begin();
        tree<Node>::iterator end2=ptree.tr.end();
        while (( sib2 != end2 ) && (dy < tree_height)) {
            dx = ptree.tr.depth(sib2);
            move(y + dy, x + dx);
            printw( (*sib2).recipe_name.c_str() );
                                                    /* print recipe status string
                                                     * */
            if (sib2->recipe_name != "ROOT") {      /* don't print details for ROOT */
                string recipe_status = " " + to_string(sib2->node_id) + " " +
                    to_string(sib2->active_element) + " " +
                    to_string(sib2->body.elements.size());
                printw(recipe_status.c_str());
            }
            ++sib2;
            ++dy;
        }
    } else {                                        /* verbosity == WithBodyElements */
        tree<Node>::iterator sib2_old;
        tree<Node>::iterator sib2=ptree.tr.begin();
        tree<Node>::iterator end2=ptree.tr.end();
        while (( sib2 != end2 ) && (dy < tree_height)) {
            depth_old = depth;
            depth = ptree.tr.depth(sib2);                /* assuming ROOT's depth is 0 */
            dx = ROOT_NODE_WIDTH + (depth-1) * MAX_NODE_WIDTH;
                                                         /* compute dx and dy */
            if (sib2->recipe_name == "ROOT") {
                dy = 0;
                dx = 0;
            } else if (depth > depth_old) {                     /* this node is the child of the
                                                                 * previous one, including first ROOT's
                                                                 * child */
                dy += 1 + ((sib2_old->recipe_name == "ROOT") ? 0 : sib2_old->active_element);
            } else {
                                                         /* the node is a new branch (must be ROOT's child,
                                                         * since there is only one child possible), so
                                                         * second or following ROOT's child */
                dy += 1 + sib2_old->body.elements.size();
                
            }
            
            move(y + dy, x + dx);
            
            if (sib2->recipe_name == "ROOT") {           /* don't print details for ROOT */ 
                string root_name = "ROOT";
                printw( root_name.c_str() );                                                  
            }  else  {                                  
                string recipe_status = "-" + sib2->recipe_name + ":" + to_string(sib2->node_id) + " " +
                    to_string(sib2->active_element) + " " + to_string(sib2->body.elements.size());
                printw(recipe_status.c_str());
                    /* update body element descriptions */
                if (sib2->bodyElementDescriptionsHaveBeenSet) {
                    this->updateBodyElementDescriptions(*sib2);
                } else {
                    this->setBodyElementDescriptions(*sib2);
                }
                    /* print body elements */
                for (int i = 0; (unsigned int)i < sib2->body.elements.size(); i++) {
                    string body_element_blurp = ((i == sib2->active_element) ? "*":" ") +
                        to_string(i) + "." + sib2->element_descriptions[i];
                    move(y + dy + i + 1, x + dx);
                    printw(body_element_blurp.c_str());
                }
            }
            sib2_old = sib2;
            ++sib2;
        } 
    }
}

void TextUI::printHeader() {
    string header = "[ctrl-v]erbose/brief [ctrl-q]uit [ctrl-h]elp";
    printw( header.c_str() );
}

void TextUI::printMessages(const int &msg_height) {
    int y, x, dy = msg_height - 1, dx = 0;
    getyx(stdscr, y, x);

    std::list<string>::iterator msg_it = this->messages.begin();
                                                        
    while  (( msg_it != this->messages.end() ) && (dy >= 0)) {  /* print from the bottom
                                                                 * of the pane */
        move(y + dy, x + dx);
        printw( msg_it->c_str() );            
        ++msg_it;
        --dy;
    }
}

void TextUI::printKeyboardBuffer() {
    printw( this->keyboardBuffer.c_str() );
}


void TextUI::push_msg(const string &msg) {
    if (this->messages.size() >= MAX_UI_MSGS) {
        this->messages.pop_back();
    }
    this->messages.push_front(NowTime() + " " + msg); /* by default print timestamp*/
}
string TextUI::makeActionDescription(BodyElement &element, int active_element, int element_id) {
    if (element_id <= active_element) {
        return makePastActionDescription(element);
    } else {
        return makeFutureActionDescription(element);
    }

}
/* generate a short description of an action body element */
string TextUI::makeFutureActionDescription(BodyElement &element) {
    string result;
    int rand_element;
    std::ostringstream output; 
    
    if (element.element_type != "action") {
        return "_ERROR_";
    }

    result = element.name + " ";

    if (element.random) {
        result += "rnd ";

        if (element.random_elements.size() == 0) {
            return "_ERROR_";
        }
            
        rand_element = (this->tick_id) %  element.random_elements.size();
        result = result + "p=";
        output << setprecision(2) << element.element_probs[rand_element];
        result += output.str();
        result += " ";
            /* the random element is element.random_elements[rand_element] */

        std::list<ArgSlot>::iterator arg_it = element.random_elements[rand_element].args.begin();
        while (arg_it != element.random_elements[rand_element].args.end()) {
            if (arg_it->name == "utterance_file") {
                result += abbreviateString(arg_it->value, 10, 15); 
            }
            arg_it++;
        }
    } else {
    
        std::list<ArgSlot>::iterator arg_it = element.args.begin();
        while (arg_it != element.args.end()) {
            if (arg_it->name == "utterance_file") {
                result += abbreviateString(arg_it->value, 10, 15); 
            }
            arg_it++;
        }
    }

    return result;
    
}

/* generate a short description of an action body element */
string TextUI::makePastActionDescription(BodyElement &element) {
    string result;
    int rand_element;
    std::ostringstream output; 
    
    if (element.element_type != "action") {
        return "_ERROR_";
    }

    result = element.name + " ";

    if (element.random) {
        result += "rnd ";

        if (element.random_elements.size() == 0) {
            return "_ERROR_";
        }
            
        rand_element = element.chosen_outcome;  /* the only difference between past and future versions*/
        result = result + "p=";
        output << setprecision(2) << element.element_probs[rand_element];
        result += output.str();
        result += " ";
            /* the random element is element.random_elements[rand_element] */

        std::list<ArgSlot>::iterator arg_it = element.random_elements[rand_element].args.begin();
        while (arg_it != element.random_elements[rand_element].args.end()) {
            if (arg_it->name == "utterance_file") {
                result += abbreviateString(arg_it->value, 10, 15); 
            }
            arg_it++;
        }
    } else {
    
        std::list<ArgSlot>::iterator arg_it = element.args.begin();
        while (arg_it != element.args.end()) {
            if (arg_it->name == "utterance_file") {
                result += abbreviateString(arg_it->value, 10, 15); 
            }
            arg_it++;
        }
    }

    return result;
    
}


void TextUI::setBodyElementDescriptions(Node &aNode) {
    
    aNode.element_descriptions.clear();
    for (int i = 0; (unsigned int)i < aNode.body.elements.size(); i++) {
        aNode.element_descriptions.push_back("");    /* first initialize the vector element*/
        this->updateBodyElementDescription(aNode, i);
    }
    aNode.bodyElementDescriptionsHaveBeenSet = true;
}

/*
 * This differs from setBodyElementDescriptions in that it only updates
 * descriptions of those elements that need to be, like random actions, for
 * example (and for now).
 */
void TextUI::updateBodyElementDescriptions(Node &aNode) {

    for (int i = max(0, aNode.active_element); (unsigned int)i < aNode.body.elements.size(); i++) {
        if (aNode.body.elements[i].random) {
            this->updateBodyElementDescription(aNode, i);
        }
    }
}


void TextUI::updateBodyElementDescription(Node &aNode, int element_id) {
                                                          /* does element_id make sense? */
    if ( ( (unsigned int)element_id >= aNode.body.elements.size() ) ||
         ( element_id < 0 ) ) {
        return;
    }

        /* do it */
    if (aNode.body.elements[element_id].element_type == "action") {
        
        aNode.element_descriptions[element_id] = "action " +
            makeActionDescription(aNode.body.elements[element_id], aNode.active_element, element_id);
    } else if (aNode.body.elements[element_id].element_type == "goal") {
        aNode.element_descriptions[element_id] = "goal";
            /* TODO: add summary of the goal formula */
    } else if (aNode.body.elements[element_id].element_type == "assignment") {
        aNode.element_descriptions[element_id] = "assignment";
            /* TODO add summary of the assignment formula */
    } else {                                                   /* unknown body element */
        aNode.element_descriptions[element_id] = "Unknown type: " +
            aNode.body.elements[element_id].element_type;
        return;
    }
}



/*
 * produces map of counts of atom subtypes on global bindings
 * */
std::map<string, int> TextUI::getAtomSubtypeCounts(Conjunction &gBindings) {

    std::map<string, int> counts_map;

    for (std::vector< Atom >::iterator atom_it=gBindings.atoms.begin();
         atom_it != gBindings.atoms.end(); atom_it++) {
        if (counts_map.count(atom_it->readSlotVal("subtype")) == 0) {
            counts_map[atom_it->readSlotVal("subtype")] = 1;
        } else {
            counts_map[atom_it->readSlotVal("subtype")] += 1;
        }
    }
    
    return counts_map;
    
}

/*
 * Prints counts of atom subtypes in global bindings
 * */
void TextUI::printGlobals(Conjunction &gBindings) {
    int nrow, ncol, y, x, dy = 0, dx = 0;

    getmaxyx(stdscr, nrow, ncol); 
    getyx(stdscr, y, x);

    std::map<string, int> atomSubtypeCounts = this->getAtomSubtypeCounts(gBindings);
    
    std::map<string, int>::iterator entry_it = atomSubtypeCounts.begin();
    while (( entry_it != atomSubtypeCounts.end() ) &&
           (dx <= ncol - (int)entry_it->first.size() - 3)) {  /* leave the space for
                                                          * subtype name and 1 digit count */
        move(y + dy, x + dx);
        string count_string = entry_it->first + ":" + to_string(entry_it->second) + " ";
        printw( count_string.c_str() );            
        ++entry_it;
        dx += count_string.size();
    }
}
