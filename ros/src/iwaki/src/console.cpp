/*****************************************************************************
 * PROJECT: iwaki
 *
 * FILE: console.cpp
 *
 * ABSTRACT: This is a simple app that reads user's terminal input and
 * sends it to Iwakishi ROS wrapper around Iwaki interaction manager.
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
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <getopt.h>
#include <ncursesw/ncurses.h> /* for getch() */

#include "ros/ros.h"
#include "iwaki/AtomMsg.h"

//using namespace std;
using std::string;
using std::iostream;


/**
 * GLOBALS
 * */
#define KB_ENTER int('\n')
#define KEY_CONTROL_SMALL_C 3

void publishBufferAtom(string &kb_buffer,  ros::Publisher &atom_pub) {
        /* first, create atom_msg */
    iwaki::AtomMsg newAtomMsg;

    iwaki::VarSlot v1, v2, v3, v4, v5;

    v1.name = "type";
    v1.val = "im";
    v1.unique_mask = true;

    v2.name = "subtype";
    v2.val = "user";
    v2.unique_mask = true;
	
    v3.name = "uu_unhandled";
    v3.val = "true";

    v4.name = "uu_string";
    v4.val = kb_buffer;

    v5.name = "id";
    v5.val = "1";
    
    newAtomMsg.varslots.push_back(v1);
    newAtomMsg.varslots.push_back(v2);
    newAtomMsg.varslots.push_back(v3);
    newAtomMsg.varslots.push_back(v4);
    newAtomMsg.varslots.push_back(v5);

    atom_pub.publish(newAtomMsg);
}


void renderKbLines(std::list<string> &kb_lines) {
    int nrow, ncol, dy;
    clear();
    getmaxyx(stdscr, nrow, ncol);
    dy = nrow - 1;

    move(0,0);
    
    std::list<string>::iterator msg_it = kb_lines.begin();
    while  (( msg_it != kb_lines.end() ) && (dy >= 0)) {  /* print from the bottom
                                                                 * of the pane */
        move(0 + dy, 0);
        printw( msg_it->c_str() );            
        ++msg_it;
        --dy;
    }
    refresh();
}

void updateKbBuffer(string &kb_buffer, int ch, ros::Publisher &atom_pub) {
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
    } else if (ch == KEY_CONTROL_SMALL_C) {
        endwin(); /* end curses mode */
        ros::shutdown();
    } else if (ch == KB_ENTER) {
        publishBufferAtom(kb_buffer, atom_pub);
        kb_buffer = "";
    }
}

void updateKbLines(std::list<string> &kb_lines, int &ch, ros::Publisher &atom_pub) {
    char buffer[8];
    string kb_buffer = *kb_lines.begin();
    
    sprintf (buffer, "%c", ch);
    if ((ch >= 32) && (ch <= 126)) {
            /* legal visible characters */
        kb_buffer += string(buffer);
            /* update buffer */
        kb_lines.pop_front();
        kb_lines.push_front(kb_buffer);
    } else if (ch == KEY_BACKSPACE) {
            /* apparently this symbol is already defined */
        if (!kb_buffer.empty()) {
            kb_buffer = kb_buffer.substr(0, kb_buffer.size() - 1);
                /* update buffer */
            kb_lines.pop_front();
            kb_lines.push_front(kb_buffer);
        }
    } else if (ch == KEY_CONTROL_SMALL_C) {
        endwin(); /* end curses mode */
        ros::shutdown();
    } else if (ch == KB_ENTER) {
        publishBufferAtom(kb_buffer, atom_pub);
        kb_buffer = "";
        kb_lines.push_front(kb_buffer);
    }

}

/**
 * Main
 */
int main(int argc, char **argv) {
    int ch;
    std::list<string> kb_lines;

        /* initalize kb_lines */
    kb_lines.push_back("");
    
        /* before reading the app's arglist */
     ros::init(argc, argv, "console");

        /* init text UI*/
    setlocale(LC_CTYPE,"");       /* for Unicode support */
    initscr();			/* Start curses mode */
    raw();			/* Line buffering disabled	*/
    keypad(stdscr, TRUE);	/* We get F1, F2 etc..		*/
    noecho();			/* Don't echo() while we do getch */
    timeout(0);
    
    

      /* ROS stuff */
    ros::NodeHandle n;
    ros::Publisher atom_pub = n.advertise<iwaki::AtomMsg>("IwakiInputAtoms", 1000);
        /******************************
         *
         * Main loop
         * 
         ******************************/
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ch = getch();
        updateKbLines(kb_lines, ch, atom_pub);
        renderKbLines(kb_lines);
        loop_rate.sleep();
    }
}
