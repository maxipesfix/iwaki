/*****************************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * FILE: exparser.cc
 *
 * ABSTRACT: expression parser
 *
 * Iwaki Interaction Manager
 * Copyright (C) 2009-2013 Maxim Makatchev, Reid Simmons,
 * Carnegie Mellon University.
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

#include <iostream>
#include <sstream>
#include <map>
#include <stack>
#include <queue>
#include <deque>
#include "iwaki.h"
#include "exparser.h"
#include "log.h"
#include <string>
//#include <Python.h>
#include <csignal>

#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159
#endif

using namespace std;
//using std::string;
//using std::iostream;

/* globals */

std::map<std::string, Operator> operators;
std::map<std::string, Function> functions;
std::map<std::string, Operator> string_operators;
std::map<std::string, Function> string_functions;

void initExpressionParsing() {

        /* all operators should be single character and are not allowed in function names */
    Operator an_op1("#", 10, ASSOC_RIGHT, true); /* unary minus */
    Operator an_op2("^", 9, ASSOC_RIGHT, false);
    Operator an_op3("*", 8, ASSOC_LEFT, false);
    Operator an_op4("/", 8, ASSOC_LEFT, false);
    Operator an_op5("%", 8, ASSOC_LEFT, false);
    Operator an_op6("+", 5, ASSOC_LEFT, false);
    Operator an_op7("-", 5, ASSOC_LEFT, false);
    
    operators[an_op1.op] = an_op1;
    operators[an_op2.op] = an_op2;
    operators[an_op3.op] = an_op3;
    operators[an_op4.op] = an_op4;
    operators[an_op5.op] = an_op5;
    operators[an_op6.op] = an_op6;
    operators[an_op7.op] = an_op7;

    Operator string_op1("+", 10, ASSOC_LEFT, false); /* default concatenation */
    string_operators[string_op1.op] = string_op1;
    
    Function fun1("rand", 2);
    Function fun2("randi", 2);
    functions[fun1.fun] = fun1;
    functions[fun2.fun] = fun2;
    
    Function sfun1("@PartialMatch", 2);
    Function sfun2("@FullMatch", 2);
    string_functions[sfun1.fun] = sfun1;
    string_functions[sfun2.fun] = sfun2;
}

bool ExpressionParser::convertIntoStandardUnits2(string &num_str, string &unit_orig) {
    if (!unit_orig.empty()) {
            /* if units are empty string then just return was received */
            /* find the unit record */
        list<UnitRecord>::iterator unit_it = this->measures.begin();
        while (unit_it != this->measures.end()) {
            if (unit_it->unit == unit_orig) {
                    /* unit record found, do conversion */
                double value = string_to_double(num_str);
                value = value * unit_it->factor;
                num_str = to_string(value);
                unit_orig = unit_it->standard_unit;
                return true;
            }
            unit_it++;
        }
            /* unit record not found */
        FILE_LOG(logERROR) << "Unknown unit: " << unit_orig << ".";
        return false;
    }
    return true;
}


/* return the variable name tocken starting from the particular index of the string
 * Also return the end infex of the token (one past token).
 * */
string ExpressionParser::getToken(string anExp, size_t prefix_ind, size_t &end_ind) {
    end_ind=anExp.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890_",
                                    (prefix_ind + 1));
    FILE_LOG(logDEBUG4) << "Got token: " <<
        anExp.substr(prefix_ind + 1, end_ind - prefix_ind - 1) << 
        " for expression: " << anExp;
    FILE_LOG(logDEBUG4) << "prefix_ind: " << prefix_ind << " end_ind:" << end_ind; 
    return anExp.substr(prefix_ind + 1, end_ind - prefix_ind - 1);
}



/* dereference the varname using bindings and im node vars and return the actual value */
string ExpressionParser::getValue(string varname) {
        /* check if the varname comes from local node bindings */
    FILE_LOG(logDEBUG4) << "Cheking node bindings for varname: " << varname;
    this->bindings.print(logDEBUG4);
    string value = this->bindings.readAtomVarVal(varname);
    FILE_LOG(logDEBUG4) << "Got value: " << value << " for varname: " << varname;
    if (value == "_NOT_FOUND_") {
        FILE_LOG(logERROR) << "\
varname: " << varname << " has not been found in local bindings. \n\
$ has scope only within a given recipe bindings.";
    }
    return value;
}



/* evaluate in-string expression in terms of node bindings, im root bindings and im
 * global vars.
 * Returns the evaluated expression and units strings
 * */
string ExpressionParser::eval(string aType, string anExp, string &unit) {

  if ((aType=="string")||(aType=="direct")){
      
          //return this->evalStringExpression(anExp);
        string result;
        if (!this->evalStringExpressionShuntingYard(anExp, unit, result)) {
            result = anExp; /* return original expression if any failure */
            FILE_LOG(logERROR)
                << "String evaluation error. Returning original input expression: "
                << result;
        }
        FILE_LOG(logDEBUG4) << "Returning evaluated string expression: " <<
            result;
        return result;
        
    }
    else if (aType=="number") {
        string result;
        this->evalNumberExpressionShuntingYard(anExp, unit, result);
        FILE_LOG(logDEBUG4) << "Returning evaluated number expression: " <<
            result;
        return result;
    }
    else {
        FILE_LOG(logERROR) << "unknown type: " << aType << " of the expression: "
                           << anExp;
        return "_NO_VALUE_";
    }
}

string ExpressionParser::evalStringExpression(string anExp) {
    /* the only thing we currently do with strings is substitution */   

    return this->substitute(anExp);
        
}

/**
 * do simple variable substitutions
 */
string ExpressionParser::substitute(string anExp) {
    /* the only thing we currently do with strings is substitution */  

    size_t end_ind = 0;
    size_t prefix_ind = anExp.find("$"); /* variable prefix is $ */
        
    if ( prefix_ind != string::npos) {
            /* get the variable name and dereference it */
        string varname = this->getToken(anExp, prefix_ind, end_ind);
        string value = this->getValue(varname);

        if (value == "_NOT_FOUND_") {
            FILE_LOG(logERROR) << "unknown varname: " << varname <<
                " in the expression: " << anExp;
                /* the whole expression becomes _NO_VALUE_*/
            return "_NO_VALUE_";
        } else if (value == "_NO_VALUE_") {
            FILE_LOG(logERROR) << "uninitialized variable: " << varname <<
                " in the expression: " << anExp;
                /* the whole expression becomes _NO_VALUE_*/
            return "_NO_VALUE_";
        }

        
        if (end_ind == string::npos) {
                /* the token ends the string */
            return anExp.substr(0, prefix_ind) + value;
        } else {
                /* there is some chunk of the string to process after the token
                 * recurse */
            return anExp.substr(0, prefix_ind) + value +
                this->substitute(anExp.substr(end_ind));
        }
    } else {
            /* a value string */
        return anExp;
    }
        
}

/*
 * New parsing algorithm 
 * */

int op_preced(const string op)
{
    if (operators.count(op) > 0) {
        return operators[op].prec;
    }
    return 0;
}

int string_op_preced(const string op)
{
    if (string_operators.count(op) > 0) {
        return string_operators[op].prec;
    }
    return 0;
}



bool op_left_assoc(const string op)
{
    if (operators.count(op) > 0) {
        return (operators[op].assoc == ASSOC_LEFT);
    }
    return false;
}

bool string_op_left_assoc(const string op)
{
    if (string_operators.count(op) > 0) {
        return (string_operators[op].assoc == ASSOC_LEFT);
    }
    return false;
}



unsigned int op_arg_count(const string op)
{
    if (operators.count(op) > 0) {
        return operators[op].unary ? 1 : 2;
    } else if (functions.count(op) > 0) {
        return functions[op].arg_count;
    } else {
        return 0;
    }
}

unsigned int string_op_arg_count(const string op)
{
    if (string_operators.count(op) > 0) {
        return string_operators[op].unary ? 1 : 2;
    } else if (string_functions.count(op) > 0) {
        return string_functions[op].arg_count;
    } else {
        return 0;
    }
}



bool ExpressionParser::is_operator(const string op) {
    return (operators.count(op) > 0);
}

bool ExpressionParser::is_string_operator(const string op) {
    return (string_operators.count(op) > 0);
}


bool ExpressionParser::is_function(const string op) {
    return (functions.count(op) > 0);
}

bool ExpressionParser::is_string_function(const string op) {
    return (string_functions.count(op) > 0);
}

/* dereference the varname using bindings and im node vars and return the actual value */
bool ExpressionParser::is_variable(const string varname) {
        /* check if the varname comes from local node bindings */
    FILE_LOG(logDEBUG4) << "Checking node bindings for varname: " << varname;
    this->bindings.print(logDEBUG4);
    string value = this->bindings.readAtomVarVal(varname);
    FILE_LOG(logDEBUG4) << "Got value: " << value << " for varname: " << varname;
    if (value == "_NOT_FOUND_") {
        FILE_LOG(logERROR) << "varname: " << varname
                           << " has not been found in local bindings.";
        return false;
    }
    return true;
}


#define is_ident(c)     ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z'))

/*
 * reads next operator, function, number, of paren/comma or returns false if cannot
 * parse such a term
 * */
bool ExpressionParser::readTerm(const string &input, unsigned int &strpos,
                                string &term, TermType &term_type) {

    bool dot_happened = false;
    term = "";
    string unit = "";
    term_type = UNKNOWN;

        /* skip leading white space */
    while ((strpos < input.length()) && ( input.substr(strpos, 1) == " ")) {
        strpos++;
    }
    
    while ( strpos < input.length() ) {

        string new_char = input.substr(strpos, 1);

        FILE_LOG(logDEBUG4) << "Processing char: " << new_char;
        FILE_LOG(logDEBUG4) << "term_type so far: " << term_type;
        if (term.empty() && (term_type == UNKNOWN)) {
            if (new_char == "(") {
                term = new_char;
                term_type = LEFT_PAREN;
                strpos++;
                return true;        
            } else if (new_char == ")") {
                term = new_char;
                term_type = RIGHT_PAREN;
                strpos++;
                return true; 
            } else if (new_char == ",") {
                term = new_char;
                term_type = COMMA;
                strpos++;
                return true;
            } else if (new_char.find_first_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_")
                       != string::npos) {
                term = new_char;
                term_type = FUNCTION;
                strpos++;
            } else if (new_char.find_first_of("$") != string::npos) {
                    /* skip the leading $ */
                term_type = VARIABLE;
                strpos++;
            } else if (new_char.find_first_of("0123456789.") != string::npos) {
                term = new_char;
                term_type = NUMBER;
                strpos++;

                if (new_char ==".") {
                    dot_happened = true;
                }
            } else if (is_operator(new_char)) {
                term = new_char;
                term_type = OPERATOR;
                strpos++;
                return true;
            } else {
                FILE_LOG(logERROR) << "Invalid character as a beginning of a term: " << input;
                return false;
            }
        } else {
                /* term is not empty, then term_type is probably determined already */
            if (term_type == FUNCTION) {
                if (new_char.find_first_of("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_")
                    != string::npos) {
                    term += new_char;
                    strpos++;
                } else {
                        /* finished reading function term, verify it exists */
                    return is_function(term);
                }
            } else if (term_type == VARIABLE) {
                if (new_char.find_first_of("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_")
                    != string::npos) {
                    term += new_char;
                    strpos++;
                } else {
                        /* finished reading variable term, verify it exists */
                    return is_variable(term);
                }
            } else if (term_type == NUMBER) {
                    /* allow of units like 5s */
                if (new_char==".") {
                    if (dot_happened) { /* second dot in a number */
                        FILE_LOG(logERROR) << "Second dot in number expression: " << input;
                        return false;
                    } else {
                        dot_happened = true;
                        term +=new_char;
                        strpos++;
                    }
                } else if (new_char.find_first_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ") != string::npos) {
                    unit += new_char;
                    strpos++;
                } else if (new_char.find_first_of("0123456789") != string::npos) {
                    if (unit.empty()) {
                        term += new_char;
                        strpos++;
                    } else {
                        FILE_LOG(logERROR) << "Wrong unit syntax in number expression: "
                                           << input;
                        return false;
                    }
                } else {
                        /* done scanning a number */
                        /* convert it to standard units */
                    return this->convertIntoStandardUnits2(term, unit);
                }
            }
        }      
    }

        /* got here because reached the end of the input string */
    if (term_type == OPERATOR) {
        return is_operator(term);
    } else if (term_type == FUNCTION) {
        return is_function(term);
    } else if (term.empty()) {
            /* strpos was out of bounds to start with */
        return false;
    } else {
        return true;
    }
}


/*
 * Shunting yard algorithm to convert to postfix
 * */


bool ExpressionParser::shuntingYard(const string &input, deque<pair<string, TermType> > &output_q) {
    unsigned int strpos = 0;
    string term;
    TermType last_term_type = UNKNOWN; /* need to detect unary minus */
    TermType term_type;
    stack<pair<string, TermType> > op_stack;
    
    FILE_LOG(logDEBUG4) << "Converting into reverse polish, input: " << input;
    
    while ( readTerm(input, strpos, term, term_type) ) {
        FILE_LOG(logDEBUG4) << "Read term: " << term << ", of type: " << term_type;
       
        if ( (term_type == NUMBER) || (term_type == VARIABLE) ) {
            output_q.push_back( make_pair(term, term_type) );
        } else if (term_type == FUNCTION) {
            op_stack.push ( make_pair(term, term_type) );
        } else if (term_type == COMMA) {
            bool paren_encountered = false;
            while (!op_stack.empty()) {
                if (op_stack.top().second == LEFT_PAREN) {
                    paren_encountered = true;
                    break;
                } else {
                    output_q.push_back(op_stack.top());
                    op_stack.pop();
                }
            }
            if (!paren_encountered) {
                FILE_LOG(logERROR) << "Separator or parentheses mismatched" << input;
                return false;
            }
            
        } else if (term_type == OPERATOR) {

            if ((term == "-") &&
                ((last_term_type == LEFT_PAREN)||(last_term_type == OPERATOR)||
                 (last_term_type == UNKNOWN))) {
                    /* this is an unary - */
                term = "#";
            }
            
            while (!op_stack.empty()) {
                if ((op_stack.top().second == OPERATOR) &&
                    ((op_left_assoc(term) && (op_preced(term) <= op_preced(op_stack.top().first)  ))
                     || ( op_preced(term) < op_preced(op_stack.top().first) ) ) ) {
                    output_q.push_back(op_stack.top());
                    op_stack.pop();  
                } else {
                    break;
                }
            }
            op_stack.push( make_pair(term, term_type) );
        } else if (term_type == LEFT_PAREN) {
            op_stack.push( make_pair(term, term_type) );
        } else if (term_type == RIGHT_PAREN) {
            bool paren_encountered = false;
            while (!op_stack.empty()) {
                if (op_stack.top().second == LEFT_PAREN) {
                    paren_encountered = true;
                    break;
                } else {
                    output_q.push_back(op_stack.top());
                    op_stack.pop();
                }      
            }
            if (!paren_encountered) {
                FILE_LOG(logERROR) << "Separator or parentheses mismatched: " << input;
                return false;
            }
                /* pop the left parenthesis from the stack, but not onto the output queue. */
            op_stack.pop();
                /* If the token at the top of the stack is a function token,
                 * pop it onto the output queue. */
            if ((!op_stack.empty()) && (op_stack.top().second == FUNCTION) ) {
                output_q.push_back(op_stack.top());
                op_stack.pop();
            }
        } else {
            FILE_LOG(logERROR) << "Unknown token: " << term << ", in input: " << input;
            return false;
        }
            /* this is used to detect the unary minus */
        last_term_type = term_type;
    }

        /* When there are no more tokens to read:
         * While there are still operator tokens in the stack */
    while (!op_stack.empty()) {
        if ((op_stack.top().second == LEFT_PAREN) || (op_stack.top().second == RIGHT_PAREN)) {
            FILE_LOG(logERROR) << "Parentheses mismatched in input: " << input;
            return false; 
        }
        output_q.push_back(op_stack.top());
        op_stack.pop();
    }
    
    return true;
}


bool ExpressionParser::evalNumberExpressionShuntingYard(string anExp, string &unit, string &res) {
    deque<pair<string, TermType> > output_q;
    deque<pair<string, TermType> > arg_dq;

    int step = 0;
    
    this->shuntingYard(anExp, output_q);

        /* debug output */
    FILE_LOG(logDEBUG4) << "Reverse polish queue:";
    for (deque<pair<string, TermType> >::iterator oq_it = output_q.begin();
         oq_it != output_q.end();
         oq_it++){
        FILE_LOG(logDEBUG4) << "output_q val: " << oq_it->first << ", term type: "
                            << oq_it->second;
    }

    while (!output_q.empty()) {
        if ((output_q.front().second == NUMBER) || (output_q.front().second == VARIABLE)) {
            arg_dq.push_back(output_q.front());
         
        } else if ((output_q.front().second == OPERATOR) || (output_q.front().second == FUNCTION)) {
            unsigned int nargs = op_arg_count(output_q.front().first);
            FILE_LOG(logDEBUG4) << "Shunting step: " << step << " = ";
            ++step;
            if (arg_dq.size() < nargs) {
                FILE_LOG(logERROR) << "Not enough args for operator/function: " <<
                    output_q.front().first;
            }
            FILE_LOG(logDEBUG4) << "Shunting op: " << output_q.front().first;
            for (int i = nargs ; i > 0; i--) {
                FILE_LOG(logDEBUG4) << "Shunting arg: " << (arg_dq.end()-i)->first
                                    << ", term type: " << (arg_dq.end()-i)->second;
            }
            this->evalFunctionOrOperator(output_q.front(),
                                         arg_dq,
                                         res);
            arg_dq.erase(arg_dq.end() - nargs, arg_dq.end());
            arg_dq.push_back( make_pair(res, NUMBER) );
        }
        output_q.pop_front();
    }

        /* If there is only one value in the stack, that value is the result of the calculation. */
    if (arg_dq.size()==1) {
        res = (arg_dq.begin()->second == VARIABLE) ?
            this->getValue(arg_dq.begin()->first) : arg_dq.begin()->first;
        return true;
    }
    FILE_LOG(logERROR) << "Expression has too many values: " << anExp;
    return false;

}

bool ExpressionParser::evalFunctionOrOperator( pair<string, TermType> &op_type_pair, 
                             deque<pair<string, TermType> > &arg_dq,
                             string &res) {
    if (op_type_pair.second == OPERATOR) {
        if (op_type_pair.first == "+") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = to_string(string_to_double(arg1) + string_to_double(arg2));
        } else if (op_type_pair.first == "-") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = to_string(string_to_double(arg1) - string_to_double(arg2));
        }  else if (op_type_pair.first == "*") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = to_string(string_to_double(arg1) * string_to_double(arg2));
        } else if (op_type_pair.first == "/") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = to_string(string_to_double(arg1) / string_to_double(arg2));
        } else if (op_type_pair.first == "#") {
            string arg1 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = to_string(-string_to_double(arg1));
        } else if (op_type_pair.first == "^") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = to_string(pow(string_to_double(arg1), string_to_double(arg2)));
        } else {
            FILE_LOG(logERROR) << "Unimplemented operator: " <<
                op_type_pair.first;
            return false;
        }
    } else {
        if (op_type_pair.first == "randi") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            double rseed = ((double)rand()/(double)RAND_MAX);
            rseed = string_to_double(arg1) +
                rseed * (string_to_double(arg2)-string_to_double(arg1));
            res = to_string(floor(rseed));
        } else if (op_type_pair.first == "rand") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            double rseed = ((double)rand()/(double)RAND_MAX);
            rseed = string_to_double(arg1) +
                rseed * (string_to_double(arg2)-string_to_double(arg1));
            res = to_string(rseed);
        } else {
            FILE_LOG(logERROR) << "Unimplemented function: " <<
                op_type_pair.first;
            return false;
        }
    }
    return true;
}

/*
 * reads next operator, function, stringterm, of paren/comma or returns false if cannot
 * parse such a term
 * */
bool ExpressionParser::readStringTerm(const string &input, unsigned int &strpos,
                                string &term, TermType &term_type) {

    bool escaped = false;
    term = "";
    term_type = UNKNOWN;
    
    while ( strpos < input.length() ) {

        string new_char = input.substr(strpos, 1);

        FILE_LOG(logDEBUG4) << "Processing char: " << new_char;
        FILE_LOG(logDEBUG4) << "term_type so far: " << term_type;
        if (term.empty() && (term_type == UNKNOWN) && (!escaped)) {
                    /* meaning nothing has been parsed yet */
            if (new_char == "\\") {
                    /* skip leading \ TOFIX HOW TO END THE STRING TERM
                     * AND WHAT TO DO WITH EMPTY STRING CONCATENATION OPS */
                term_type = UNKNOWN;               /* already UNKNOWN, but again for clarity */
                strpos++;
                escaped = true;
            } else if (new_char.find_first_of("@")          /* string function prefix */
                       != string::npos) {
                    /* include the leading @ */
                term = new_char;
                term_type = FUNCTION;
                strpos++;
            } else if (new_char.find_first_of("$") != string::npos) {
                    /* skip the leading $ */
                term_type = VARIABLE;
                strpos++;
            } else if (is_string_operator(new_char)) {
                term = new_char;
                term_type = OPERATOR;
                strpos++;
                return true;
            } else {                   /* all other chars can be a start of a string term */
                term = new_char;
                term_type = STRINGTERM;
                strpos++;
            }
        } else {
                /* term has started being parsed */
            if (term_type == FUNCTION) {
                if (new_char.find_first_of("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_")
                    != string::npos) {
                    term += new_char;
                    strpos++;
                } else {
                                       /* finished reading function term, verify it exists */
                    return is_string_function(term);
                }
            } else if (term_type == VARIABLE) {
                if (new_char.find_first_of("0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_")
                    != string::npos) {
                    term += new_char;
                    strpos++;
                } else {
                                       /* finished reading variable term, verify it exists */
                    return is_variable(term);
                }
            } else if (term_type == STRINGTERM) {
                                         /* escaped sequence, care only if it's backslash */
                if ((!escaped) && (new_char=="\\")) {
                    escaped = true;
                                         /* skip leading backslash */
                    strpos++;                  
                } else if ((!escaped) && (new_char.find_first_of("$@") != string::npos)) {
                        /* end of string term*/
                    return true;
                } else if (escaped && (new_char.find_first_of("(),") != string::npos)) {
                        /* end of string term*/
                    strpos--;                     /* recapture leading backslash to ensure
                                                   * new term will start with an escape */
                    return true;
                } else {
                        /* keep on scanning the string term */
                    term += new_char;
                    strpos++;
                    escaped = false;
                }
            } else if (term_type == UNKNOWN) {
                    /* unknown term after first character has been parsed,
                     * means it started with a backslash */
                if (escaped && new_char=="(") {
                    term = new_char;
                    term_type = LEFT_PAREN;
                    strpos++;
                    return true;  
                } else if (escaped && new_char==")") {
                    term = new_char;
                    term_type = RIGHT_PAREN;
                    strpos++;
                    return true;  
                }  else if (escaped && new_char==",") {
                    term = new_char;
                    term_type = COMMA;
                    strpos++;
                    return true;  
                } else {
                        /* if starting escape is not paren or comma, then it started a string term */
                    escaped = false;                     /* reset escape flag */
                    term = new_char;
                    term_type = STRINGTERM;
                    strpos++;
                }
            } else {
                FILE_LOG(logERROR) << "Unhandled term type : " << term_type;
            }
        }      
    }

        /* got here because reached the end of the input string */
    if (term_type == OPERATOR) {
        return is_operator(term);
    } else if (term_type == FUNCTION) {
        return is_function(term);
    } else if (term.empty()) {
            /* strpos was out of bounds to start with */
        return false;
    } else {
        return true;
    }
}



/*
 * Shunting yard algorithm to convert to postfix.
 * Concatenation is an implicit operator (+)
 * Special treatment of $, @, (, ), comma.
 * No special treatment of double and single quotes.
 * */

bool ExpressionParser::shuntingYardString(const string &input, deque<pair<string, TermType> > &output_q) {
    unsigned int strpos = 0;
    string term;
    string fake_term;                  /* need for implicit + operator */
    TermType last_term_type = UNKNOWN; /* need to detect unary minus */
    TermType term_type;
    stack<pair<string, TermType> > op_stack;
    
    FILE_LOG(logDEBUG4) << "Converting into reverse polish, input: " << input;
    
    while ( readStringTerm(input, strpos, term, term_type) ) {
        FILE_LOG(logDEBUG4) << "Read term: " << term << ", of type: " << term_type;
       
        if ( (term_type == STRINGTERM) || (term_type == VARIABLE) ) {
            
                /* see if we should insert implicit + operator here */
            if ((last_term_type == STRINGTERM) || (last_term_type == VARIABLE)
                || (last_term_type == RIGHT_PAREN)) {

                    /* pretend that we had + operator term */
                fake_term = "+";
                while (!op_stack.empty()) {
                    if ((op_stack.top().second == OPERATOR) &&
                        ((string_op_left_assoc(fake_term) &&
                          (string_op_preced(fake_term) <= string_op_preced(op_stack.top().first)))
                         || ( string_op_preced(fake_term) < string_op_preced(op_stack.top().first)))) {
                        output_q.push_back(op_stack.top());
                        op_stack.pop();  
                    } else {
                        break;
                    }
                }
                op_stack.push( make_pair(fake_term, OPERATOR) );
                    /* stop pretending */
            } 
            
            output_q.push_back( make_pair(term, term_type) );
        } else if (term_type == FUNCTION) {
                /* see if we should insert implicit + operator here */
            if ((last_term_type == STRINGTERM) || (last_term_type == VARIABLE)
                || (last_term_type == RIGHT_PAREN)) {

                    /* pretend that we had + operator term */
                fake_term = "+";
                while (!op_stack.empty()) {
                    if ((op_stack.top().second == OPERATOR) &&
                        ((string_op_left_assoc(fake_term) &&
                          (string_op_preced(fake_term) <= string_op_preced(op_stack.top().first)))
                         || ( string_op_preced(fake_term) < string_op_preced(op_stack.top().first)))) {
                        output_q.push_back(op_stack.top());
                        op_stack.pop();  
                    } else {
                        break;
                    }
                }
                op_stack.push( make_pair(fake_term, OPERATOR) );
                    /* stop pretending */
            }
            
            op_stack.push ( make_pair(term, term_type) );
        } else if (term_type == COMMA) {
            bool paren_encountered = false;
            while (!op_stack.empty()) {
                if (op_stack.top().second == LEFT_PAREN) {
                    paren_encountered = true;
                    break;
                } else {
                    output_q.push_back(op_stack.top());
                    op_stack.pop();
                }
            }
            if (!paren_encountered) {
                FILE_LOG(logERROR) << "Separator or parentheses mismatched: " << input;
                return false;
            }
            
        } else if (term_type == OPERATOR) {
            
            while (!op_stack.empty()) {
                if ((op_stack.top().second == OPERATOR) &&
                    ((string_op_left_assoc(term) &&
                      (string_op_preced(term) <= string_op_preced(op_stack.top().first)))
                     || ( string_op_preced(term) < string_op_preced(op_stack.top().first)))) {
                    output_q.push_back(op_stack.top());
                    op_stack.pop();  
                } else {
                    break;
                }
            }
            op_stack.push( make_pair(term, term_type) );
        } else if (term_type == LEFT_PAREN) {
            op_stack.push( make_pair(term, term_type) );
        } else if (term_type == RIGHT_PAREN) {
            bool paren_encountered = false;
            while (!op_stack.empty()) {
                if (op_stack.top().second == LEFT_PAREN) {
                    paren_encountered = true;
                    break;
                } else {
                    output_q.push_back(op_stack.top());
                    op_stack.pop();
                }      
            }
            if (!paren_encountered) {
                FILE_LOG(logERROR) << "Separator or parentheses mismatched" << input;
                return false;
            }
                /* pop the left parenthesis from the stack, but not onto the output queue. */
            op_stack.pop();
                /* If the token at the top of the stack is a function token,
                 * pop it onto the output queue. */
            if ((!op_stack.empty()) && (op_stack.top().second == FUNCTION) ) {
                output_q.push_back(op_stack.top());
                op_stack.pop();
            }
        } else {
            FILE_LOG(logERROR) << "Unknown token: " << term << ", in input: " << input;
            return false;
        }
            /* this is used to detect the unary minus */
        last_term_type = term_type;
    }

        /* When there are no more tokens to read:
         * While there are still operator tokens in the stack */
    while (!op_stack.empty()) {
        if ((op_stack.top().second == LEFT_PAREN) || (op_stack.top().second == RIGHT_PAREN)) {
            FILE_LOG(logERROR) << "Parentheses mismatched in input: " << input;
            return false; 
        }
        output_q.push_back(op_stack.top());
        op_stack.pop();
    }
    
    return true;
}



bool ExpressionParser::evalStringExpressionShuntingYard(string anExp, string &unit, string &res) {
    deque<pair<string, TermType> > output_q;
    deque<pair<string, TermType> > arg_dq;

    int step = 0;
    
    if (!this->shuntingYardString(anExp, output_q)) {
        return false;
    }

        /* debug output */
    FILE_LOG(logDEBUG4) << "Reverse polish queue:";
    for (deque<pair<string, TermType> >::iterator oq_it = output_q.begin();
         oq_it != output_q.end();
         oq_it++){
        FILE_LOG(logDEBUG4) << "output_q val: " << oq_it->first << ", term type: "
                            << oq_it->second;
    }

    while (!output_q.empty()) {
        if ((output_q.front().second == STRINGTERM) || (output_q.front().second == VARIABLE)) {
            arg_dq.push_back(output_q.front());
         
        } else if ((output_q.front().second == OPERATOR) || (output_q.front().second == FUNCTION)) {
            unsigned int nargs = string_op_arg_count(output_q.front().first);
            FILE_LOG(logDEBUG4) << "Shunting step: " << step;
            ++step;
            if (arg_dq.size() < nargs) {
                FILE_LOG(logERROR) << "Not enough args for operator/function: " <<
                    output_q.front().first;
            }
            FILE_LOG(logDEBUG4) << "Shunting op: " << output_q.front().first;
            for (int i = nargs ; i > 0; i--) {
                FILE_LOG(logDEBUG4) << "Shunting arg: " << (arg_dq.end()-i)->first
                                    << ", term type: " << (arg_dq.end()-i)->second;
            }
            if (!this->evalStringFunctionOrOperator(output_q.front(), arg_dq, res)) {
                return false;
            }
            FILE_LOG(logDEBUG4) << "Result of the shunting step evaluation: " << res;
            arg_dq.erase(arg_dq.end() - nargs, arg_dq.end());
            arg_dq.push_back( make_pair(res, STRINGTERM) );
        }
        output_q.pop_front();
    }

        /* If there is only one value in the stack, that value is the result of the calculation. */
    if (arg_dq.size()==1) {
        res = (arg_dq.begin()->second == VARIABLE) ?
            this->getValue(arg_dq.begin()->first) : arg_dq.begin()->first;
        return true;
    }
    FILE_LOG(logERROR) << "Expression has too many values: " << anExp;
    return false;
}



bool ExpressionParser::evalStringFunctionOrOperator( pair<string, TermType> &op_type_pair, 
                             deque<pair<string, TermType> > &arg_dq,
                             string &res) {
    if (op_type_pair.second == OPERATOR) {
        if (op_type_pair.first == "+") {               /* concatenation */
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
            res = arg1 + arg2;
        } else {
            FILE_LOG(logERROR) << "Unimplemented string operator: " <<
                op_type_pair.first;
            return false;
        }
    } else {
        if (op_type_pair.first == "@PartialMatch") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
#ifdef USE_RE2 
                        /* TODO: make sure arg2 has exactly one sub-pattern () */
            RE2::PartialMatch(arg1, arg2, &res);
            FILE_LOG(logDEBUG4) << "Evaluated PartialMatch with arg1: " << arg1 <<
                ", arg2: " << arg2 << ", result: " << res ;
#else
            FILE_LOG(logERROR) << 
              "This binary of Iwaki has been compiled without regular expression support:";
            FILE_LOG(logERROR) << "@PartialMatch function is not available.";
            FILE_LOG(logERROR) << "arg1: " << arg1 << ", arg2: " << arg2;
            return false;
#endif
        } else if (op_type_pair.first == "@FullMatch") {
            string arg1 = ((arg_dq.end()-2)->second == VARIABLE) ?
                getValue((arg_dq.end()-2)->first) :  (arg_dq.end()-2)->first;
            string arg2 = ((arg_dq.end()-1)->second == VARIABLE) ?
                getValue((arg_dq.end()-1)->first) :  (arg_dq.end()-1)->first;
#ifdef USE_RE2
                        /* TODO: make sure arg2 has exactly one sub-pattern () */
            RE2::FullMatch(arg1, arg2, &res);
            FILE_LOG(logDEBUG4) << "Evaluated FullMatch with arg1: " << arg1 <<
                ", arg2: " << arg2 << ", result: " << res ;
#else
            FILE_LOG(logERROR) << 
              "This binary of Iwaki has been compiled without regular expression support:";
            FILE_LOG(logERROR) << "@FullMatch function is not available.";
            FILE_LOG(logERROR) << "arg1: " << arg1 << ", arg2: " << arg2;
            return false;
#endif
        } else {
            FILE_LOG(logERROR) << "Unimplemented string function: " <<
                op_type_pair.first;
            return false;
        } 
    }
    return true;
}
