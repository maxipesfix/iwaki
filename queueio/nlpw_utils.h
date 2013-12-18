/****************************************************************
 * FILE: utils.h
 *
 * ABSTRACT: A collection of useful utilities.
 *
 * Copyright (C) 2009-2013 Maxim Makatchev
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
 ****************************************************************/


#ifndef NLPW_UTILS_H
#define NLPW_UTILS_H

#ifndef __MINGW32__
#define USE_RE2
#endif

#include <algorithm>
#include <string>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <vector>
#include "log.h"

#ifdef USE_RE2
#include <re2/re2.h>
#include <re2/filtered_re2.h>
#endif

using namespace std;

#ifdef USE_RE2
using namespace re2;
#endif

struct RemoveWhitespace
{
  bool operator()(char c)
  {
    return (c =='\r' || c =='\t' || c == ' ' || c == '\n');
  }
};


/* convert stuff to string */
template <class T>
inline std::string to_string (const T& t)
{
    std::stringstream ss;
    ss << setprecision(20) << t;
    return ss.str();
}

inline std::string abbreviateString ( const std::string &inputString,
                                      int prefix_size, int suffix_size)
{
    if (inputString.size() <= (unsigned int)(prefix_size + suffix_size) ) {
        return inputString;
    }

    return inputString.substr(0, prefix_size) + ".." +
        inputString.substr(inputString.size() - suffix_size, suffix_size);
}


/* convert string to a number */
inline int string_to_number ( const std::string &Text )
//Text not by const reference so that the function can be used with a
{                               //character array as argument
    std::stringstream ss(Text);
    int result;
    return ss >> result ? result : 0;
}

inline double string_to_double ( const std::string &Text )
//Text not by const reference so that the function can be used with a
{                               //character array as argument
    std::stringstream ss(Text);
    double result;
    return ss >> result ? result : 0;
}

inline void toLowerCase(std::string &str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
}

inline std::string toLowerCaseReturns(std::string &str)
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

inline std::vector<std::string> &split(const std::string &s, char delim,
                                       std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}



inline std::list<std::string> splitIntoListRemoveWhitespace(const std::string &s, char delim) {
    std::list<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        item.erase( std::remove_if(
                        item.begin(), item.end(),
                        RemoveWhitespace()),
                    item.end() );
        if (!item.empty()) {
            elems.push_back(item); /* push only non-empty strings */
        }
    }
    return elems;
}



/* remove weekdate which is the first th space-separated chunk
 * in Www Mmm dd yyyy time*/
inline string getTimeStamp(string desired_delim) {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    string full_string = string(asctime(now));

        /* remove trailing newline character returned by asctime */
    full_string = full_string.substr(0, full_string.size() - 1);
    
    std::vector<std::string> chunks;
    char delim=' ';
    split(full_string, delim, chunks);
    if (chunks.size() < 5) {
        FILE_LOG(logERROR)  << "Malformed output of C time function: " << full_string;
        return "_NO_VALUE_";
    }
    return chunks[4] + desired_delim + chunks[1] + desired_delim + chunks[2] +
        desired_delim  + chunks[3];

}


/* remove time which is the 4th space-separated chunk
 * in Www Mmm dd time yyyy*/
inline string removeTimeFromFullDateString(string full_string) {
    std::vector<std::string> chunks;
    char delim=' ';
    split(full_string, delim, chunks);
    if (chunks.size() < 5) {
        FILE_LOG(logERROR)  << "Malformed output of C time function: " << full_string;
        return "_NO_VALUE_";
    }
    return chunks[0] + " " + chunks[1] + " " + chunks[2] + " " + chunks[4];

}



/*  Returns the today's date as a string of the following form:
 *  Www Mmm dd yyyy
 *  */

inline string getCurrentNormalizedDate() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    string norm_date = string(asctime(now));

    return removeTimeFromFullDateString(norm_date);
}


/*  Returns the a date offset days in the future (or past) as a string
 *  of the following form:
 *  Www Mmm dd yyyy
 *  */

inline string getFuturePastNormalizedDate(int offset) {
    time_t t = time(0);   // get time now
    t += (time_t) 24*3600*offset;
    struct tm * tm_struct = localtime( & t );
    string norm_date = string(asctime(tm_struct));
   
    return removeTimeFromFullDateString(norm_date);
}

inline int getCurrentSecond() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_sec;
}


inline int getCurrentMinute() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_min;
}


inline int getCurrentHour() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_hour;
}


inline int getCurrentYear() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_year + 1900;
}

inline int getCurrentMonth() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_mon + 1; /* adjust 0-11 to 1-12 */
}

inline int getCurrentMonthdayInt() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_mday;
}

inline int getCurrentWeekdayInt() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_wday;
}

inline std::string convertWeekdayIntToString(int weekday_int) {
    switch (weekday_int) {
        case 0:
            return "sunday";
        case 1:
            return "monday";
        case 2:
            return "tuesday";
        case 3:
            return "wednesday";
        case 4:
            return "thursday";
        case 5:
            return "friday";
        case 6:
            return "saturday";
        default:
            return "_NO_VALUE_";
    }
}

inline int convertWeekdayStringToInt(string weekday_str) {
    if (weekday_str == "sunday") {
        return 0;
    } else if (weekday_str == "monday") {
        return 1;
    } else if (weekday_str == "tuesday") {
        return 2;
    } else if (weekday_str == "wednesday") {
        return 3;
    } else if (weekday_str == "thursday") {
        return 4;
    } else if (weekday_str == "friday") {
        return 5;
    } else if (weekday_str == "saturday") {
        return 6;
    } else {
        return -1;
    }
}

inline bool isWeekday(string temp_ref) {
    if ((temp_ref == "sunday")||(temp_ref == "monday")
        ||(temp_ref == "tuesday")||(temp_ref == "wednesday")
        ||(temp_ref == "thursday")||(temp_ref == "friday")
        ||(temp_ref == "saturday")) {
        return true;
    } else {
        return false;
    }
}

inline string getNearestFutureWeekdayNormalizedDate(string weekday) {
    int weekday_int = convertWeekdayStringToInt(weekday);
    
    if (weekday_int < 0) {
        FILE_LOG(logERROR)  << "Weekday not recognized for conversion to int: "
                            << weekday;
        return "_NO_VALUE_";
    }
    int cur_weekday_int = getCurrentWeekdayInt();
    if (cur_weekday_int <= weekday_int) {
            /* nearest future weekday is this week */
        return getFuturePastNormalizedDate( weekday_int - cur_weekday_int );
    } else {
        return getFuturePastNormalizedDate( weekday_int - cur_weekday_int + 7 );
    }    
}

/* Grounded expression is the one that doesn't have unescaped special symbols for
 * $-vars and @-functions */

inline bool isGroundedExpression(string expr) {
    return ((expr.find("$")==string::npos) && (expr.find("@")==string::npos));
}

inline string toString(list<string> listStr) {
    string res;
    for (list<string>::iterator listStr_it = listStr.begin();
         listStr_it != listStr.end(); listStr_it++) {
        string delim = (++listStr_it == listStr.end())?"":",";
        listStr_it--;
        res += *listStr_it + delim;
    }
    return res;
}

#endif // NLPW_UTILS_H
       
