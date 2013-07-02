/*****************************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * FILE: formula.cc
 *
 * ABSTRACT: formula methods
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

//using namespace std;
using std::string;
using std::iostream;


/********************************************/
/* VarSlot methods */
/********************************************/
bool VarSlot::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  //cout << "Processing slot." << endl;
  
  /* All varslots should have a name */
  if (!pElem->Attribute("name")) {
    FILE_LOG(logERROR) << "Error: slot has no name attribute.";
    return false;
  } 
  else {
    this->name = pElem->Attribute("name");
  }
  /* relation */
  if (pElem->Attribute("rel")) {
    this->relation = pElem->Attribute("rel");
  }
  /* value */
  if (pElem->Attribute("val")) {
    this->val = pElem->Attribute("val");
  }
  /* binding */
  if (pElem->Attribute("var")) {
    this->var = pElem->Attribute("var");
  }

  if (pElem->Attribute("type")) {
    this->type = pElem->Attribute("type");
  }
#ifdef USE_RE2
                             /* if relations involves RE matching, precompile regular
                              * expression, if there are no $-vars inside */
  if (isREFunction(this->relation) && (this->relation.find("$")==string::npos) &&
      (this->relation.find("@")==string::npos)) {
          /* want to evaluate since there may be backslash escape chars. Do it always instead of checking
           * for backslash since this is done once during loading */
      Conjunction empty_bindings;
      ExpressionParser exparser(empty_bindings);
      string unit="_NO_VALUE_";
      string val1 = exparser.eval("string", this->val, unit);
      
      this->re_p = new RE2(val1, RE2::Quiet);
      if (!this->re_p->ok()) {
          FILE_LOG(logERROR) << "Error parsing RE: " << this->val << ", which was evaluated into: "
                             << val1;
          FILE_LOG(logERROR) << this->re_p->error(); 
      }
          //this->re_p = &re;
  }
#endif
  // cout << "Slot's name:"<< this->name << ", rel:"<< this->relation
  //   << ", val:"<< this->val << ", var:" << this->var <<endl;
  return loadOkay;
}

void VarSlot::print() {
    char buffer[80];
    sprintf(buffer, "          %-15.15s %-15.15s %-15.15s %-10.10s %-10.10s", this->name.c_str(), this->val.c_str(), this->var.c_str(), this->type.c_str(), this->relation.c_str());
    FILE_LOG(logDEBUG) << (string)buffer;
}

void VarSlot::print(TLogLevel log_level) {
    char buffer[80];
    sprintf(buffer, "          %-15.15s %-15.15s %-15.15s %-10.10s %-10.10s", this->name.c_str(), this->val.c_str(), this->var.c_str(), this->type.c_str(), this->relation.c_str());
    FILE_LOG(log_level) << (string)buffer;
}

void VarSlot::printWithLabels() {
    FILE_LOG(logDEBUG) << "            VarSlot " <<     \
        "name: " << this->name << ", " <<               \
        "relation: " << this->relation << ", " <<       \
        "val: " << this->val << ", " <<                 \
        "var: " << this->var << "," <<                  \
        "type: " << this->type;
}


#ifdef USE_RE2
/*
 * val2 here is the global slot value, this->val is the value in the condition
 * */
bool VarSlot::evalRERelation(string &val2, string &val1, string &relation, string &type,
                           Conjunction &new_bindings) {
    FILE_LOG(logDEBUG4) << "Evaluating RE string relation: " <<
        val2 << " " << relation << " " << this->val << " of type: " << type;

    if (relation=="PartialMatch") {
        if (this->re_p == NULL) {              /* there is no precompiled pattern */
            return RE2::PartialMatch(val2, val1);
        } else {                               /* there is a precompiled pattern */
            return RE2::PartialMatch(val2, *(this->re_p));
        }
    } else if (relation=="FullMatch") {
        if (this->re_p == NULL) {              /* there is no precompiled pattern */
            return RE2::FullMatch(val2, val1);
        } else {                               /* there is a precompiled pattern */
            return RE2::FullMatch(val2, *(this->re_p));
        }
    } else {
        FILE_LOG(logERROR) << "Unknown RE relation between strings.";
    }
    
    return false;
}
#endif

/*
 * val2 here is the global slot value, this->val is the value in the condition
 * */
bool VarSlot::evalStringRelation(string &val2, string &relation, string &type,
                           Conjunction &new_bindings, HowComplete howcomplete) {
    FILE_LOG(logDEBUG4) << "Evaluating string relation " << howcomplete <<
        " relation: " << val2 << " " << relation <<
        " " << this->val << " of type: " << type;

    string val1 = this->val;
    size_t end_ind = 0;
    size_t prefix_ind = val1.find("$"); /* variable prefix is $ */
    size_t fun_prefix_ind =
        val1.find_first_of("@"); /* string function prefix is @ */
        /* Any functions? This characters indicate presence of functions in
         * a string expression */
        /* evaluate expression in this-val (val1) if necessary */
    
    if ((prefix_ind != string::npos) || (fun_prefix_ind != string::npos)
        || ((val1.find("\\") != string::npos) 
#ifdef USE_RE2
        && (this->re_p == NULL)
#endif
        )) {
            /* could do everything as while loop, but we want to quickly
             * eliminate the case when there are no $, @ or backslash in the expression
             * and not to define exparser for that case for maximum speed, since
             * that is a common case */
        ExpressionParser exparser(new_bindings);
        string varname, value;
            /* first see if all $-vars are defined in bindings. If not,
             * return true if howcomplete == Partial, error if == Complete.
             * We want this to be done here, because exparses does not provide
             * this kind of diagnostics to the external caller. */
        while (prefix_ind != string::npos) {
            varname = exparser.getToken(val1, prefix_ind, end_ind);
            value = exparser.getValue(varname);
            if ((value == "_NOT_FOUND_")||(end_ind == string::npos)) {
                break;
            }
            val1 = val1.substr(end_ind);
            prefix_ind = val1.find("$");
        }

        if (value ==  "_NOT_FOUND_") {
            if (howcomplete == Partial) {
                return true;
            } else {
                    /* Complete matching, not found $-var, through an error */
                FILE_LOG(logERROR) << "Complete evaluation impossible due to unknown varname: "
                                   << varname <<
                    " in the expression: " << this->val;
                return false;
            }
        }
        
            /* OK, here all the $-vars can be resolved via new_bindings, do just this */

        string unit="_NO_VALUE_";
        FILE_LOG(logDEBUG4) << "Evaluating slot val: " << this->val;
        val1 = exparser.eval(type, this->val, unit);
        
    } else {       
        val1 = this->val;
    }

        /* for strings we have equality and substr for now */
    if ((relation=="=") || (relation=="equal")) {
        return (val1==val2);
    } else if ((relation=="!=") || (relation=="not equal"))  {
        return (val1!=val2);
    } else if (relation=="substr") {
        return (val2.find(val1)!=string::npos);
    } else if (isREFunction(relation)) {
#ifdef USE_RE2
        return this->evalRERelation(val2, val1, relation, type, new_bindings);
#else 
        FILE_LOG(logERROR) << "This binary of Iwaki has been built without regular expression support.";
        FILE_LOG(logERROR) << "relation: " << relation << ", val1: " << val1 << ", val2:" << val2;
        return false;
#endif
    } else {
        FILE_LOG(logERROR) << "Unknown relation between strings.";
    }
    
    return false;
}


/*
 * val2 here is the global slot value, this->val is the value in the condition
 * */
bool VarSlot::evalNumberRelation(string &val2, string &relation, string &type,
                           Conjunction &new_bindings, HowComplete howcomplete) {
    FILE_LOG(logDEBUG4) << "Evaluating number relation " << howcomplete <<
        " relation: " << val2 << " " << relation <<
        " " << this->val << " of type: " << type;

        /* quick _NO_VALUE_ check */
    if (this->val == "_NO_VALUE_") {
        if ((relation=="=") || (relation=="equal") ||
            (relation=="<=") || (relation=="leq") ||
             (relation==">=") || (relation=="geq")) {
            if (val2=="_NO_VALUE_") {return true;} else {return false;}
        } else if ((relation=="!=") || (relation=="not equal")) {
            if (val2=="_NO_VALUE_") {return false;} else {return true;}
        } else {
            return false;
        }
    }
    
    string val1 = this->val;
    size_t end_ind = 0;
    size_t prefix_ind = val1.find("$"); /* variable prefix is $ */
    size_t fun_prefix_ind =
        val1.find_first_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ+-*/^");
        /* Any functions? This characters indicate presence of functions in
         * a numeric expression */
        /* evaluate expression in this-val (val1) if necessary */
    
    if ((prefix_ind != string::npos) || (fun_prefix_ind != string::npos)) {
            /* could do everything as while loop, but we want to quickly
             * eliminate the case when there are no $ in the expression and
             * not to define exparser for that case for maximum speed, since
             * that is a common case */
        ExpressionParser exparser(new_bindings);
        string varname, value;
            /* first see if all $-vars are defined in bindings. If not,
             * return true if howcomplete == Partial, error if == Complete.
             * We want this to be done here, because exparses does not provide
             * this kind of diagnostics to the external caller. */
        while (prefix_ind != string::npos) {
            varname = exparser.getToken(val1, prefix_ind, end_ind);
            value = exparser.getValue(varname);
            if ((value == "_NOT_FOUND_")||(end_ind == string::npos)) {
                break;
            }
            val1 = val1.substr(end_ind);
            prefix_ind = val1.find("$");
        }

        if (value ==  "_NOT_FOUND_") {
            if (howcomplete == Partial) {
                return true;
            } else {
                    /* Complete matching, not found $-var, through an error */
                FILE_LOG(logERROR) << "Complete evaluation impossible due to unknown varname: "
                                   << varname <<
                    " in the expression: " << this->val;
                return false;
            }
        }
        
            /* OK, here all the $-vars can be resolved via new_bindings, do just this */

        string unit="_NO_VALUE_";
        FILE_LOG(logDEBUG4) << "Evaluating slot val: " << this->val;
        val1 = exparser.eval(type, this->val, unit);
        
    } else {       
        val1 = this->val;
    }


        /* do the number comparison */
    if ((relation=="=") || (relation=="equal")) {
        return (string_to_double(val1)==string_to_double(val2));
    } else if ((relation=="!=") || (relation=="not equal")) {
        return (string_to_double(val1)!=string_to_double(val2));
    } else if ((relation=="<") || (relation=="less")) {
        return (string_to_double(val2)<string_to_double(val1));
    } else if ((relation==">") || (relation=="greater")) {
        return (string_to_double(val2)>string_to_double(val1));
    } else if ((relation=="<=") || (relation=="leq")) {
        return (string_to_double(val2)<=string_to_double(val1));
    } else if ((relation==">=") || (relation=="geq")) {
        return (string_to_double(val2)>=string_to_double(val1));
    } else {
        FILE_LOG(logERROR) << "Unknown relation between numbers.";
    }
    
    return false;
}

/*
 * val2 here is the global slot value, this->val is the value in the condition
 * */
bool VarSlot::evalRelation(string &val2, string &relation, string &type,
                           Conjunction &new_bindings, HowComplete howcomplete) {
    FILE_LOG(logDEBUG4) << "Evaluating " << howcomplete <<
        " relation: " << val2 << " " << relation <<
        " " << this->val << " of type: " << type;

    if (type=="string")  {
        return this->evalStringRelation(val2, relation, type, new_bindings, howcomplete);
    } else if (type=="number") {
        return this->evalNumberRelation(val2, relation, type, new_bindings, howcomplete);
    } else {
        FILE_LOG(logERROR) << "Unknown type of argslot.";
    }
    return false;
}

bool VarSlot::unify(VarSlot &varslot2, Conjunction &new_bindings, HowComplete howcomplete) {
	/** subsumption rules */
	
    if (this->name != varslot2.name) {
        return false;
    } else if (this->relation == "bind") {
            /* this is a binding without a condition predicate does not affect unification */
        return true;
    } else if (this->evalRelation(varslot2.val,
                                  this->relation, varslot2.type, new_bindings, howcomplete)) {
            /* used type from varslot2 (globals) to allow type in precond to be optional */
            /* predicate succeeded*/
        return true;
    }
    else {
        return false;
    }
    
}


/********************************************/
/* Atom methods */
/********************************************/
bool Atom::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  
  //cout << "Processing atom." << endl;
  
  if (pElem->Attribute("quantifier")) {
    this->quantifier = pElem->Attribute("quantifier");
  }

  if (pElem->Attribute("delete")) {
      if ((string)pElem->Attribute("delete")=="now") {
          this->toBeDeleted = Now;
      } else  if ((string)pElem->Attribute("delete")=="when_not_used") {
          this->toBeDeleted = WhenNotUsed;
      }
  }
  
  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();
  
  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement())
  {
    string level1tag = pLevel1Node->Value();
    // cout << "In atom, level 1 tag: " << level1tag << endl;
    if (level1tag=="slot") {
      VarSlot varslot;
      if (!varslot.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Could not load atom's slot";
        return false;
      } else { 
        this->varslots.push_back(varslot); 
      }
    } else 
    {
      FILE_LOG(logERROR) << "Error: In atom, unknown level 1 tag: " << level1tag;
      return false;
    }
  }
  return loadOkay;
}

void Atom::print() {
    FILE_LOG(logDEBUG) << "       * atom with quantifier:" << this->quantifier;
    FILE_LOG(logDEBUG) << "       * atom with toBeDeleted flag:" <<
        this->toBeDeleted2String();
    string node_ids_string;
    for (std::list<int>::iterator node_id_it = this->node_ids.begin();
         node_id_it != this->node_ids.end(); node_id_it++) {
        node_ids_string += to_string(*node_id_it);
        node_ids_string += " ";
        
    }
    FILE_LOG(logDEBUG) << "       * atom with node_ids: " << node_ids_string;
    list<VarSlot>::iterator a_varslot = varslots.begin();
    while (a_varslot!=varslots.end()) {
        a_varslot->print();
        a_varslot++;
    }
}

void Atom::print(TLogLevel log_level) {
    FILE_LOG(log_level) << "       * atom with quantifier:" << this->quantifier;
    FILE_LOG(log_level) << "       * atom with toBeDeleted flag:" <<
        this->toBeDeleted2String();
    string node_ids_string;
    for (std::list<int>::iterator node_id_it = this->node_ids.begin();
         node_id_it != this->node_ids.end(); node_id_it++) {
        node_ids_string += to_string(*node_id_it);
        node_ids_string += " ";
    }
    FILE_LOG(log_level) << "       * atom with node_ids: " << node_ids_string;
    list<VarSlot>::iterator a_varslot = varslots.begin();
    while (a_varslot!=varslots.end()) {
        a_varslot->print(log_level);
        a_varslot++;
    }
}

void Atom::printWithLabels() {
	FILE_LOG(logDEBUG) << "        atom";
	FILE_LOG(logDEBUG) << "          quantifier:" << this->quantifier;
        FILE_LOG(logDEBUG) << "       * atom with toBeDeleted flag:" <<
            this->toBeDeleted2String();
        string node_ids_string;
        for (std::list<int>::iterator node_id_it = this->node_ids.begin();
             node_id_it != this->node_ids.end(); node_id_it++) {
            node_ids_string += to_string(*node_id_it);
            node_ids_string += " ";
        }
        FILE_LOG(logDEBUG) << "       * atom with node_ids: " << node_ids_string;
	list<VarSlot>::iterator a_varslot = varslots.begin();
        FILE_LOG(logDEBUG) << "          varslots";
        while (a_varslot!=varslots.end()) {
            a_varslot->printWithLabels();
            a_varslot++;
  	}
}

string Atom::toBeDeleted2String() {
    if (this->toBeDeleted == NotYet) {
        return "NotYet";
    } else if (this->toBeDeleted == WhenNotUsed)  {
        return "WhenNotUsed";
    }  else if (this->toBeDeleted == Now)  {
        return "Now";
    } else {
        return "_NO_VALUE_";
    }
}

/* new atom unification uses complete conjunction bindings to allow for
 * cross-referencing $-vars to other atoms within the conjunction.
 * if howcomplete == "partial" unification will not fail if $-vars are unresolved */
bool Atom::unify(Atom &atom2, Conjunction &new_bindings, HowComplete howcomplete) {
    bool res = true;
    FILE_LOG(logDEBUG4) << "------Trying to unify atoms with complete flag == " << howcomplete;
    this->print(logDEBUG4);
    FILE_LOG(logDEBUG4) << "------and: ";
    atom2.print(logDEBUG4);
    FILE_LOG(logDEBUG4) << "------end of atoms we are trying to unify. ";

        /* if atom2 has toBeDeleted anything but NotYet, fail to unify.
         * atom2 is normally from the global atoms */
    if (atom2.toBeDeleted != NotYet) {
        FILE_LOG(logDEBUG4) << "Failed to unify since atom2 toBeDeleted="
                            << atom2.toBeDeleted;
        return false;
    }
      
    for(list<VarSlot>::iterator a_varslot1 = this->varslots.begin();
        a_varslot1 != this->varslots.end(); a_varslot1++) {
        bool varslot_matched = false;
        VarSlot slot_binding;
        for(list<VarSlot>::iterator a_varslot2 = atom2.varslots.begin();\
            a_varslot2 != atom2.varslots.end(); a_varslot2++) {
            if (a_varslot1->unify((*a_varslot2), new_bindings, howcomplete)) {
                varslot_matched = true;
                break;
            }
        }
        if (varslot_matched) {
                /* do nothing, successful match of varslot */	
        } 
        else {        
            FILE_LOG(logDEBUG4) << "Varslot that didn't find a match:";
            a_varslot1->print(logDEBUG4);
            res=false;
            break;
        }
    }
    return res;	
}


bool Atom::hasVar(string var1) {
    for(list<VarSlot>::iterator varslot1=this->varslots.begin();        \
        varslot1!=this->varslots.end(); varslot1++) {
        if (varslot1->var==var1) {
            return true;
        }
    }
    return false;
}


/** 
 * Return the pointer to a slot by name
 * 
 * */

VarSlot* Atom::getSlotByName(string slot_name) {
    for(list<VarSlot>::iterator varslot1=this->varslots.begin();        \
        varslot1!=this->varslots.end(); varslot1++) {
        if (varslot1->name==slot_name) {
            return &(*varslot1);
        }
    }
        /* return NULL pointer if varslot by the name not found */  
    return 0;
} 


/** 
 * Return the pointer to a slot by var
 * 
 * */

VarSlot* Atom::getSlotByVar(string slot_var) {
    for(list<VarSlot>::iterator varslot1=this->varslots.begin();        \
        varslot1!=this->varslots.end(); varslot1++) {
        if (varslot1->var==slot_var) {
            return &(*varslot1);
        }
    }
        /* return NULL pointer if varslot by the name not found */  
    return 0;
} 



/** 
 * Return the value of a slot given var
 * 
 * */

string Atom::readSlotVal(string slot_name) {
	for(list<VarSlot>::iterator varslot1=this->varslots.begin(); \
		varslot1!=this->varslots.end(); varslot1++) {
		if (varslot1->name==slot_name) {
			return varslot1->val;
			}
		}
	return "_NOT_FOUND_";
} 

/** 
 * Return the var of a slot given the name
 * 
 * */

string Atom::readSlotVar(string slot_name) {
	for(list<VarSlot>::iterator varslot1=this->varslots.begin(); \
		varslot1!=this->varslots.end(); varslot1++) {
		if (varslot1->name==slot_name) {
			return varslot1->var;
			}
		}
	return "_NOT_FOUND_";
} 

/**
 * Return the value of a slot given the var name
 * */
string Atom::readVarVal(string var_name) {
	for(list<VarSlot>::iterator varslot1=this->varslots.begin(); \
		varslot1!=this->varslots.end(); varslot1++) {
            if (varslot1->var==var_name) {
                return varslot1->val;
            }
        }
	return "_NOT_FOUND_";
} 

/*
 * same as updateAtom() but does not add new slots found in atom2
 * */
void Atom::updateAtomValsOnly(Atom &atom2) {
	for(list<VarSlot>::iterator a_varslot2 = atom2.varslots.begin();\
		a_varslot2 != atom2.varslots.end(); a_varslot2++) {
		//bool found_slot = false;
		for(list<VarSlot>::iterator a_varslot1 = this->varslots.begin();\
		a_varslot1 != this->varslots.end(); a_varslot1++) {
			if (a_varslot1->name == a_varslot2->name) {
				a_varslot1->val = a_varslot2->val;
                                    /* do not allow rewriting types */
                                    //a_varslot1->type = a_varslot2->type;
                                    //	found_slot = true;
				break;
				}
			}

                    /* do not add new slots */
                    //if (!found_slot) {
                    // /** slot name does not exist yet in this atom, add it */
                    //this->varslots.push_back((*a_varslot2));	
                    //		}
        }
}


void Atom::updateAtom(Atom &atom2) {
	for(list<VarSlot>::iterator a_varslot2 = atom2.varslots.begin();\
		a_varslot2 != atom2.varslots.end(); a_varslot2++) {
		bool found_slot = false;
		for(list<VarSlot>::iterator a_varslot1 = this->varslots.begin();\
		a_varslot1 != this->varslots.end(); a_varslot1++) {
			if (a_varslot1->name == a_varslot2->name) {
				a_varslot1->val = a_varslot2->val;
                                    /* do not allow rewriting types */
                                    //a_varslot1->type = a_varslot2->type;
				found_slot = true;
				break;
				}
			}
		if (!found_slot) {
			/** slot name does not exist yet in this atom, add it */
			this->varslots.push_back((*a_varslot2));	
			}
		}
}

void Atom::setSlotVal(string slot_name, string slot_val) {
	bool found_slot = false;
	for(list<VarSlot>::iterator a_varslot1 = this->varslots.begin();\
		a_varslot1 != this->varslots.end(); a_varslot1++) {
		if (a_varslot1->name == slot_name) {
			a_varslot1->val = slot_val;
			found_slot = true;
			break;
			}
		}
	if (!found_slot) {
		/** slot name does not exist yet in this atom, add it */
		VarSlot new_varslot;
		new_varslot.name = slot_name;
		new_varslot.val = slot_val;
		this->varslots.push_back(new_varslot);	
		}
}

void Atom::setSlotUniqueMask(string slot_name, bool mask_val) {
	bool found_slot = false;
	for(list<VarSlot>::iterator a_varslot1 = this->varslots.begin();\
		a_varslot1 != this->varslots.end(); a_varslot1++) {
		if (a_varslot1->name == slot_name) {
			a_varslot1->unique_mask = mask_val;
			found_slot = true;
			break;
			}
		}
	if (!found_slot) {
		/** slot name does not exist yet in this atom, add it */
		VarSlot new_varslot;
		new_varslot.name = slot_name;
		new_varslot.unique_mask = mask_val;
		this->varslots.push_back(new_varslot);	
		}
}


bool Atom::setSlotValByVar(string slot_var, string slot_val) {
    for(list<VarSlot>::iterator a_varslot1 = this->varslots.begin();
        a_varslot1 != this->varslots.end(); a_varslot1++) {
        if (a_varslot1->var == slot_var) {
            a_varslot1->val = slot_val;
            return true;
        }
    }
    return false;
}

/* evaluate expressions inside value of a varslot */
void Atom::evalSlotVals(Conjunction &bindings) {
    for(list<VarSlot>::iterator a_varslot = this->varslots.begin();    \
        a_varslot != this->varslots.end(); a_varslot++) {
        ExpressionParser exparser(bindings);
        string unit="_NO_VALUE_";
        FILE_LOG(logDEBUG4) << "Evaluating slot val: " << a_varslot->val;
        a_varslot->val = exparser.eval(a_varslot->type, a_varslot->val, unit);
            /* returned unit is ignored here, should be standard unit */
    }
}

/* bind atom's $-bindings. gAtom is a reference to a corresponding atom in
 * the root bindings (globals). */

void Atom::bindBindings(Atom &gAtom, Atom &atom_bindings) {
    VarSlot* gSlot_p;
    
        /* iterate through
         * all the slots of the given atom and fill the vals of pure bindings */
    for(list<VarSlot>::iterator a_varslot = this->varslots.begin();     \
        a_varslot != this->varslots.end(); a_varslot++) {
            /* if slot is a binding, namely var is non-empty and is set.
             * Note that val as we allow defining a binding with a condition predicate
             * in a precondition (i.e. val and var can be non-empty and set)
             * IMPORTANT: semantics of val is different for bindings as opposed to
             * conditions. Namely, in a binding, val stores the value of the binding.
             * in a condition, val stores the value of the predicate */
        if ((a_varslot->var!="_NO_VALUE_") && (!a_varslot->var.empty())) {
            FILE_LOG(logDEBUG4) << "Trying to bind slot var: " << a_varslot->var << \
                "for slot name: " << a_varslot->name;
                /* get the value from the globals atom */
            VarSlot new_varslot;
            gSlot_p =  gAtom.getSlotByName(a_varslot->name);

            if (!gSlot_p) {
                FILE_LOG(logERROR) << "Cannot find slot named: " << a_varslot->name << \
                    " in global atom with 'this' = " << (gAtom.getSlotByName("this"))->val;
            } else {
                new_varslot.val =  gSlot_p->val;
                new_varslot.var = a_varslot->var;
                /* inherit type from global atom, to make type spec optional in precond */
                new_varslot.type = gSlot_p->type; 
                new_varslot.name = a_varslot->name;
                atom_bindings.varslots.push_back(new_varslot);
            }
        }
    }
        /* if 'this' slot was not copied as a $-binding, add one. As an exception, it will be in
         * the bindings without var value. */
    if (!atom_bindings.getSlotByName("this")) {
        VarSlot new_varslot;
        gSlot_p = gAtom.getSlotByName("this");
        if (!gSlot_p) {
            FILE_LOG(logERROR) << "Cannot find 'this' slot " << \
                "in global atom with 'subtype' = " << (gAtom.getSlotByName("subtype"))->val;
        }
        new_varslot.name = gSlot_p->name; /* "this" */
        new_varslot.val = gSlot_p->val;
        atom_bindings.varslots.push_back(new_varslot);
    }
}


bool Atom::checkSyntax() {
  if (this->readSlotVal("type").empty() || this->readSlotVal("subtype").empty() ||
      (this->readSlotVal("type")=="_NO_VALUE_") ||
      (this->readSlotVal("type")=="_NOT_FOUND_") ||
      (this->readSlotVal("subtype")=="_NO_VALUE_") ||
      (this->readSlotVal("subtype")=="_NOT_FOUND_")) {
    return false;
  } else {
    return true;
  }
}

/********************************************/
/* Conjunction methods */
/********************************************/

bool Conjunction::load(TiXmlElement* pElem) {
    bool loadOkay = true;
    
    TiXmlHandle hRoot=TiXmlHandle(pElem);
    TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();
    
    for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement()) {
        string level1tag = pLevel1Node->Value();
            // cout << "In formula, level 1 tag: " << level1tag << endl;
        if (level1tag=="atom") {
            Atom atom;
            if (!atom.load(pLevel1Node)){
                FILE_LOG(logERROR) << "Could not load atom";
                return false;
            } else { 
                this->atoms.push_back(atom);
            }
        } else
        {
            FILE_LOG(logERROR) << "Error: In conjunction, unknown level 1 tag: " << level1tag;
            return false;
        }
    }
    return loadOkay;
}



void Conjunction::print() {
    FILE_LOG(logDEBUG) << "      Conjunction of length:" << this->atoms.size();
    std::vector<Atom>::iterator an_atom = this->atoms.begin();
    while (an_atom != this->atoms.end()) {
  	an_atom->print();
  	an_atom++;
    }
}

void Conjunction::print(TLogLevel log_level) {
    FILE_LOG(log_level) << "      Conjunction of length:" << this->atoms.size();
    std::vector<Atom>::iterator an_atom = this->atoms.begin();
    while (an_atom != this->atoms.end()) {
  	an_atom->print(log_level);
  	an_atom++;
    }
}


/** 
 *  Returns the iterator to the atom matching three varslots (normally will
 * be used to match type, subtype and id). If no match found returns the 
 * iterator off the list.
 * */

vector<Atom>::iterator Conjunction::findAtom(string slotname1, string slotval1, \
                                           string slotname2, string slotval2,  
                                           string slotname3, string slotval3) {
    vector<Atom>::iterator atom1=this->atoms.begin();
    for(; atom1!=this->atoms.end(); atom1++) {
        if ((atom1->readSlotVal(slotname1)==slotval1)&&(atom1->readSlotVal(slotname2)==slotval2)&&(atom1->readSlotVal(slotname3)==slotval3)) {
            break;
        }
    }
    return atom1;
}

/** 
 *  Returns the iterator to the atom matching two varslots (normally will
 * be used to match type and subtype). If no match found returns the 
 * iterator off the list.
 * */

vector<Atom>::iterator Conjunction::findAtom(string slotname1, string slotval1, \
                                           string slotname2, string slotval2) {
    vector<Atom>::iterator atom1=this->atoms.begin();
    for(; atom1!=this->atoms.end(); atom1++) {
        if ((atom1->readSlotVal(slotname1)==slotval1)&&(atom1->readSlotVal(slotname2)==slotval2)) {
            break;
        }
    }
    return atom1;

}

/** 
 *  Returns the iterator to the atom matching two varslots (normally will
 * be used to match 'this' slot). If no match found returns the 
 * iterator off the list.
 * */

vector<Atom>::iterator Conjunction::findAtom(string slotname1, string slotval1) {
    vector<Atom>::iterator atom1=this->atoms.begin();
    for(; atom1!=this->atoms.end(); atom1++) {
        if ((atom1->readSlotVal(slotname1)==slotval1)) {
            break;
        }
    }
    return atom1;
}


/** 
 *  Returns pointer to the first atom in conjunctions that contains the given
 *  var slot. If none found, returns NULL pointer.
 * */

Atom* Conjunction::findAtomByVar(string var1) {
    Atom* atom_p = 0;
    for(vector<Atom>::iterator atom_it=this->atoms.begin();
        atom_it!=this->atoms.end(); atom_it++) {
        if (atom_it->hasVar(var1)) {
            return &(*atom_it);
        }
    }
    return atom_p;
}



/** 
 *  Returns the value corresponding to var_name from the FIRST atom it finds with
 *  the slot with this var_name in the given conjunction.
 *  Usage: This is the main way to retrieve a value of a binding,
 *  since a binding is a conjunction. Users must make sure their var_names are
 *  unique per conjunction to avoid an unexpected result.
 * */

string Conjunction::readAtomVarVal(string var_name) {
    for(vector<Atom>::iterator atom1=this->atoms.begin(); \
        atom1!=this->atoms.end(); atom1++) {
        string val = atom1->readVarVal(var_name);
        if (val != "_NOT_FOUND_") {
            return val;
        }
    }
    return "_NOT_FOUND_";
}



/**
 * update atoms inside the conjuction according to
 * gBindings and a value of 'this' slot. 
 * */

void Conjunction::updateBinding(Conjunction &gBindings) {
    string this_val1;
    
    for(std::vector< Atom >::iterator atom1_it=this->atoms.begin();             
        atom1_it != this->atoms.end(); atom1_it++) {
        this_val1 = atom1_it->readSlotVal("this");
            /* if atom is defunct, skip it */
        if (this_val1 == "_DEFUNCT_") {
            continue;
        }
        vector<Atom>::iterator atom2_it =               \
            gBindings.findAtom("this", this_val1);
        if (atom2_it == gBindings.atoms.end()) {
                /** no matching atom in gBindings, mark atom as removed
                 * by setting this val to _NO_VALUE_. */
            FILE_LOG(logDEBUG4)  << "No matching atom not found in the gBindings, \n\
invalidating local bindings for atom with 'this' = " << this_val1;
            atom1_it->setSlotVal("this", "_DEFUNCT_");
        } else {
                /* atom2 found */
            atom1_it->updateAtomValsOnly(*atom2_it);
                /* update toBeDeleted flag */
            atom1_it->toBeDeleted = atom2_it->toBeDeleted;            
        }
    }
}



/**
 * update atoms inside the conjuction according to
 * gBindings and a value of 'this' slot. Also, update global atoms
 * node_ids -- sorted list of nodes the atom is involved in.
 * 
 * */

void Conjunction::updateBinding(Conjunction &gBindings, const int node_id) {
    string this_val1;
    
    for(std::vector< Atom >::iterator atom1_it=this->atoms.begin();             
        atom1_it != this->atoms.end(); atom1_it++) {
        this_val1 = atom1_it->readSlotVal("this");
            /* if atom is defunct, skip it */
        if (this_val1 == "_DEFUNCT_") {
            continue;
        }
        vector<Atom>::iterator atom2_it =               \
            gBindings.findAtom("this", this_val1);
        if (atom2_it == gBindings.atoms.end()) {
                /** no matching atom in gBindings, mark atom as removed
                 * by setting this val to _NO_VALUE_. */
            FILE_LOG(logDEBUG4)  << "No matching atom not found in the gBindings, \n\
invalidating local bindings for atom with 'this' = " << this_val1;
            atom1_it->setSlotVal("this", "_DEFUNCT_");
        } else {
                /* atom2 found */
            atom1_it->updateAtomValsOnly(*atom2_it);
                /* update toBeDeleted flag */
            atom1_it->toBeDeleted = atom2_it->toBeDeleted;
            FILE_LOG(logDEBUG4)  << "About to insert in node_ids an id: " << node_id;
            FILE_LOG(logDEBUG4)  << "...for atom with this: " << this_val1;
            insertIntListSortedUnique(atom2_it->node_ids, node_id);
        }
    }
}




/**
 * bind $-bindings of atoms inside the conjuction according to
 * gBindings and a Matching.  Store
 * the bindings in new_bindings
 * */

void Conjunction::bindBindings(Conjunction &gBindings, Conjunction &new_bindings,
                               Matching &aMatching) {
    
    for(std::vector< Match >::iterator aMatch_it=aMatching.mapping.begin();             
        aMatch_it != aMatching.mapping.end(); aMatch_it++) {
        if (aMatch_it->match_type == Locked) {
                /* do the binding between locked atoms only */
            Atom atom_bindings;
            this->atoms[aMatch_it->id1].bindBindings(gBindings.atoms[aMatch_it->id2],
                                                     atom_bindings);
            new_bindings.atoms.push_back(atom_bindings);
        }
    }
}

/**
 * Unify conjunction without binding (of a whilecondition, for example) and
 * the conjunction root bindings, for example.
 *
 * how_complete argument is used to specify whether the full expression evaluation
 * will be done, such as $-variable expansion.
 * Usally during run-time how_complete=Complete, but during the preprocessing
 * step how_complete=Partial as the $-vars are not yet instantiated.
 **/
bool Conjunction::unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings,
                                      std::vector< Match > &mapping,
                                      HowComplete howcomplete) {

    
        /* prepare to generate matchings: place permanent locks
         * on matches for atoms that are already locked. Actually, all atoms
         * should be locked here, since this is used for whilecondition */

        /* quick check: an empty conjunction unifies with anything */
    if (this->atoms.size() == 0) {
        return true;
    }
    
    Matching aMatching(this->atoms.size(), con2.atoms.size());
    if (!aMatching.init(*this, con2)) {
            /* If init matching failed due to a missing global binding atom, for example */
         FILE_LOG(logDEBUG3) << "Initializing matching failed.";
        return false;
    }

    FILE_LOG(logDEBUG4) << "Initialized matching: ";
    aMatching.print(logDEBUG4);

        /* none of the atoms are Locked in case of goal/assignpost matching
         * but just in case we use this for something else, try partial
         * match of locked atoms first */

        /* passes 2 and 3: eval expressions and unify */
    if (!this->unifyLockedAtoms(con2, new_bindings, aMatching, Partial)) {
            /* locked atoms were not unifiable. */
            /* move onto the next conjunction in the formula */
        return false;
    }

    while (aMatching.next()) {
            /* generate next aMatching. These ones should have all
             * atoms in con1 locked */
        FILE_LOG(logDEBUG4) << "Generated next matching: ";
        aMatching.print(logDEBUG4);

            /* pass 1: extract bindings: no need here */
        
            /* passes 2 and 3: eval expressions and unify */
        if (this->unifyLockedAtoms(con2, new_bindings, aMatching, howcomplete)) {
                /* all atoms in con1 got unified). */
            mapping = aMatching.mapping;
            return true;
        }
            /* clear bindings of non-unified match: no need */
    }

        /* went through all matchings, none unified */
    return false;

    
}


/**
 * Unify conjunction without binding (of a whilecondition, for example) and the conjunction 
 * root bindings, for example
 **/
bool Conjunction::unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings) {

    
        /* prepare to generate matchings: place permanent locks
         * on matches for atoms that are already locked. Actually, all atoms
         * should be locked here, since this is used for whilecondition */

        /* quick check: an empty conjunction unifies with anything */
    if (this->atoms.size() == 0) {
        return true;
    }
    
    Matching aMatching(this->atoms.size(), con2.atoms.size());
    if (!aMatching.init(*this, con2)) {
            /* If init matching failed due to a missing global binding atom, for example */
         FILE_LOG(logDEBUG3) << "Initializing matching failed.";
        return false;
    }

    FILE_LOG(logDEBUG4) << "Initialized matching: ";
    aMatching.print(logDEBUG4);

        /* all atoms must be Locked, we require this of node.whilecondition */

        /* passes 2 and 3: eval expressions and unify */
    return this->unifyLockedAtoms(con2, new_bindings, aMatching, Complete);
}



/**
 * Unify conjunction (of a precondition, for example) and the conjunction 
 * root bindings, for example
 **/
bool Conjunction::unify(Conjunction &con2, Conjunction &new_bindings) {
           
        /* create locked matching and locked bindings
         * and check if it's unifiable so far.
         * keep locked bindings around anyways -- saves time
         * when doing bindPureBinding*/
    
        /* prepare to generate matchings: place permanent locks
         * on matches for atoms that are already locked */

        /* quick check: an empty conjunction unifies with anything */
    if (this->atoms.size() == 0) {
        return true;
    }
    
    Matching aMatching(this->atoms.size(), con2.atoms.size());
    if (!aMatching.init(*this, con2)) {
            /* If init matching failed due to a missing global binding atom, for example */
        FILE_LOG(logDEBUG3) << "Initializing matching failed.";
        return false;
    }

    FILE_LOG(logDEBUG4) << "Initialized matching: ";
    aMatching.print(logDEBUG4);
        
        /* do an advanced check if atoms locked so far in con1
         * fail to unify. Use initMatching to find locks */

        /* pass 1: extract bindings */
    this->bindBindings(con2, new_bindings, aMatching);
    FILE_LOG(logDEBUG4) << "Bound the following bindings:";
    new_bindings.print(logDEBUG4);
    FILE_LOG(logDEBUG4) << "End of bounded bindings.";
    
        /* passes 2 and 3: eval expressions and unify */
    if (!this->unifyLockedAtoms(con2, new_bindings, aMatching, Partial)) {
            /* locked atoms were not unifiable. */
            /* move onto the next conjunction in the formula */
        return false;
    }

        /* clear possible new bindings from partial matching of init matching */
    new_bindings.atoms.clear();
    
    while (aMatching.next()) {
            /* generate next aMatching. These ones should have all
             * atoms in con1 locked */
        FILE_LOG(logDEBUG4) << "Generated next matching: ";
        aMatching.print(logDEBUG4);

            /* pass 1: extract bindings */
        this->bindBindings(con2, new_bindings, aMatching);
        FILE_LOG(logDEBUG4) << "Bound the following bindings:";
        new_bindings.print(logDEBUG4);
        FILE_LOG(logDEBUG4) << "End of bounded bindings.";
        
            /* passes 2 and 3: eval expressions and unify */
        if (this->unifyLockedAtoms(con2, new_bindings, aMatching, Complete)) {
                /* all atoms in con1 got unified). */
            return true;
        }
            /* clear bindings of non-unified match */
        new_bindings.atoms.clear();
    }

        /* went through all matchings, none unified */
    return false;
}




/* tries to unify atoms that are locked according to the matching.
 * if howcomplete = "partial" do not complain about missing references to $-vars,
 * unresolved vars will not affect unification.
 * if howcomplete = "full" all vars must be resolved. */
bool Conjunction::unifyLockedAtoms(Conjunction &con2, Conjunction &new_bindings,
                                   Matching &aMatching, HowComplete howcomplete) {


        /* there will be a few passes:
         * pass 1: binds $-vars to their values from globals (root bindings)
         *    done beforehand outside of this function
         * pass 2: evaluates vals (val may contain $-vars)
         * pass 3: actually do the unification */
        /* passes 2 & 3 are together for efficiency, since we may abort unification
         * earlier upon a failure within any varslot */
    
    for(std::vector< Match >::iterator aMatch_it=aMatching.mapping.begin();             
        aMatch_it != aMatching.mapping.end(); aMatch_it++) {
            /* do the unification between locked atoms only */
        if (( aMatch_it->match_type == Locked )&&
            ( !this->atoms[aMatch_it->id1].unify(
                  con2.atoms[aMatch_it->id2], new_bindings, howcomplete))) {
            return false;
        }
    }
       
    return true;
}


/********************************************/
/* Formula methods */
/********************************************/
bool Formula::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  
  // cout << "Processing formula in tag: " << pElem->Value() << endl;
  
  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();

  /* in case there are atoms without conjunction tag wrapped around them
   * assign them all to one conjunction -- this is for the case of a single
   * conjunction formula, when conjunction tag is optional */
  Conjunction conj;
  
  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement())
  {
    string level1tag = pLevel1Node->Value();
    // cout << "In formula, level 1 tag: " << level1tag << endl;
    if (level1tag=="conjunction") {
            /* Conjuction loading */
        Conjunction aConj;
        if (!aConj.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Could not load conjunction.";
        return false;
      } else { 
        this->disjuncts.push_back(aConj);
      }
    } else if (level1tag=="atom") {
      Atom atom;
      if (!atom.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Could not load atom.";
        return false;
      } else { 
        conj.atoms.push_back(atom);
      }
    } else
    {
      FILE_LOG(logERROR) << "Error: In formula, unknown level 1 tag: " << level1tag;
      return false;
    }
  }

  if (!conj.atoms.empty()) {
      this->disjuncts.push_back(conj);
  }
  return loadOkay;
}

void Formula::print() {
    vector<Conjunction>::iterator a_disjunct = disjuncts.begin();
    FILE_LOG(logDEBUG) << "    Formula";
    while (a_disjunct!=disjuncts.end()) {
  	a_disjunct->print();
  	a_disjunct++;
    }
}

void Formula::print(TLogLevel log_level) {
    vector<Conjunction>::iterator a_disjunct = disjuncts.begin();
    FILE_LOG(log_level) << "    Formula";
    while (a_disjunct!=disjuncts.end()) {
  	a_disjunct->print(log_level);
  	a_disjunct++;
    }
}

/* returns pointer to the first atom that contains the var slot*/
Atom* Formula::findAtomByVar(string var1) {

    Atom* atom_p = 0;
    vector<Conjunction>::iterator con1_it = this->disjuncts.begin();
    
    while (con1_it != this->disjuncts.end()) {
        atom_p =  con1_it->findAtomByVar(var1);
        if (atom_p) {
            return atom_p;
        }
  	con1_it++;
    }
    return atom_p;
}

bool Formula::bindTypeandSubtype(Formula precondition, string recipe_name) {
    for(vector<Conjunction>::iterator
            con1_it=this->disjuncts.begin();
        con1_it != this->disjuncts.end(); con1_it++) {
        
        for(vector<Atom>::iterator atom1_it=con1_it->atoms.begin();    
            atom1_it!=con1_it->atoms.end(); atom1_it++) {                    
            string var1 = atom1_it->readSlotVar("this");
            if (var1 == "_NOT_FOUND_") {
                FILE_LOG(logERROR) << "No 'this' slot in a goal or assignpost for node: "
                                   << recipe_name;
                FILE_LOG(logERROR) << "Offending atom:";
                atom1_it->print(logERROR);
                return false;
                    } else {
                    /* find type/subtype of the atom with 'this' var1 in
                             * preconditions*/
                Atom* precAtom_p = precondition.findAtomByVar(var1);
                    /* does var exist in the preconditions? */
                if (!precAtom_p) {
                    FILE_LOG(logERROR) << "No var: " << var1 <<
                        " found in preconditions of recipe: " <<
                        recipe_name;
                    return false;
                }
                    /* atom found OK, read its type and subtype */
                string type = precAtom_p->readSlotVal("type");
                string subtype = precAtom_p->readSlotVal("subtype");
                if ((type == "_NOT_FOUND_") || (type == "_NO_VALUE_")) {
                    FILE_LOG(logERROR) <<
                                "No type for atom with var: "
                                       << var1 <<
                        " found in preconditions of recipe: " <<
                        recipe_name;
                        } else if ((subtype == "_NOT_FOUND_") ||
                                   (subtype == "_NO_VALUE_")) {
                    FILE_LOG(logERROR) <<
                        "No subtype for atom with var: "
                                       << var1 <<
                        " found in preconditions of recipe: " <<
                        recipe_name;
                    return false;
                } else {
                        /* valid type and subtype found */
                    atom1_it->setSlotVal("type", type);
                    atom1_it->setSlotVal("subtype", subtype);
                }
            }
        }
    }
    return true;
}


/* bind missing "this" slot vals based on local bindings */
void Formula::bindThis(Conjunction &lBindings, string &recipe_name) {
    for(vector<Conjunction>::iterator con1_it=this->disjuncts.begin();       \
        con1_it != this->disjuncts.end(); con1_it++) {
		
            /** iterate through atoms1 */
        for(vector<Atom>::iterator atom1_it=con1_it->atoms.begin();     \
            atom1_it!=con1_it->atoms.end(); atom1_it++) {
                /* bind "this" slots according to node's bindings */
            string var1 = atom1_it->readSlotVar("this");
            if (var1 == "_NOT_FOUND_") {
                FILE_LOG(logERROR) << "No 'this' slot in atom for node: "
                                   << recipe_name;
                atom1_it->print(logERROR);
            } else if (var1 == "_NO_VALUE_") {
                FILE_LOG(logERROR) <<
                    "No var argument given for 'this' slot in atom for node: "
                                   << recipe_name;
                atom1_it->print(logERROR);
            } else {
                string val1 =  lBindings.readAtomVarVal(var1);
                if ((val1 == "_NOT_FOUND_") || (val1 == "_NO_VALUE_")) {
                    FILE_LOG(logERROR) <<
                        "No binding found in node bindings for var: "
                                       << var1 << " in node: " << recipe_name;
                } else {
                    atom1_it->setSlotVal("this", val1);
                }
            }
        }
    }
}





/**
 * Unify formula (of a goal) and the conjunction of assignpost.
 * howcomplete is Partial for preprocessing stem ($-vars will not be resolved,
 * matching succeeds by default when $-var is present)
 **/
bool Formula::unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings,
                                  std::vector< Match > &mapping, int &matched_con_id,
                                  HowComplete howcomplete) {
    bool unified = true; /* empty formula unifies with everything */
    matched_con_id = -1; /* empty formula's matched_con_id = -1 */
	
	/** iterate through conjunctions of the formula 
	 * and see if any of them unify with the con2. For
	 * now greedy match, return as soon as conjunct is unifable.
	 * The unification procedure is as follows:
	 * - make a candidate match between atoms of two conjunctions
	 * - do the pass to bind all pure bindings (working with copies)
	 * - do the pass to evaluate all val expressions, passing the 
	 * pure bindings from previous step for reference.
	 * - do the atom content unification.
	 *
	 * This procedure allows for cross-referencing to $-variables
	 * that are defined within the same conjunction.
	 * */
    FILE_LOG(logDEBUG4) << "Unifying without binding a formula that contains " <<
        this->disjuncts.size() << " conjunctions."; 	
    for(vector<Conjunction>::iterator con1=this->disjuncts.begin();
        con1 != this->disjuncts.end(); con1++) {
        matched_con_id++;

        if (con1->unifyWithoutBinding(con2, new_bindings, mapping, howcomplete)) { 
            unified = true;
            break;
        } else {
            unified = false;
        }
    }
    
    return unified;
}



/**
 * Unify formula (of a whileconditions) and the conjunction of old bindings
 * (of a parent node, or global root bindings, for example)
 **/
bool Formula::unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings) {
    bool unified = true; /** empty formula unifies with everything */
	
	/** iterate through conjunctions of the formula 
	 * and see if any of them unify with the con2. For
	 * now greedy match, return as soon as conjunct is unifable.
	 * The unification procedure is as follows:
	 * - make a candidate match between atoms of two conjunctions
	 * - do the pass to bind all pure bindings (working with copies)
	 * - do the pass to evaluate all val expressions, passing the 
	 * pure bindings from previous step for reference.
	 * - do the atom content unification.
	 *
	 * This procedure allows for cross-referencing to $-variables
	 * that are defined within the same conjunction.
	 * */
    FILE_LOG(logDEBUG4) << "Unifying without binding a formula that contains " << this->disjuncts.size() \
                        << " conjunctions."; 	
    for(vector<Conjunction>::iterator con1=this->disjuncts.begin();       \
        con1 != this->disjuncts.end(); con1++) {

        if (con1->unifyWithoutBinding(con2, new_bindings)) {
            unified = true;
            break;
        } else {
            unified = false;
        }
    }
    
    return unified;
}

/**
 * Unify formula (of a precond) and the conjunction of old bindings
 * (of a parent node, or global root bindings, for example)
 **/
bool Formula::unify(Conjunction &con2, Conjunction &new_bindings) {
    bool unified = true; /** empty formula unifies with everything */
	
	/** iterate through conjunctions of the formula 
	 * and see if any of them unify with the con2. For
	 * now greedy match, return as soon as conjunct is unifable.
	 * The unification procedure is as follows:
	 * - make a candidate match between atoms of two conjunctions
	 * - do the pass to bind all pure bindings (working with copies)
	 * - do the pass to evaluate all val expressions, passing the 
	 * pure bindings from previous step for reference.
	 * - do the atom content unification.
	 *
	 * This procedure allows for cross-referencing to $-variables
	 * that are defined within the same conjunction.
	 * */
    FILE_LOG(logDEBUG4) << "Unifying formula that contains " << this->disjuncts.size() \
                        << " conjunctions."; 	
    for(vector<Conjunction>::iterator con1=this->disjuncts.begin();       \
        con1 != this->disjuncts.end(); con1++) {

        if (con1->unify(con2, new_bindings)) {
            unified = true;
            break;
        } else {
            unified = false;
        }
    }
    
    return unified;
}

/* TODO: see if this is needed. Now evaluation of slot vals should use new_bindings were
 * the bindings to $-vars are stored.*/
void Formula::evalSlotVals() {

	/* iterate through conjunctions of the formula */
    FILE_LOG(logDEBUG4) << "Evaluating formula slot vals...";
    for(vector<Conjunction>::iterator con1=this->disjuncts.begin();       \
        con1 != this->disjuncts.end(); con1++) {
		
            /** iterate through atoms1 */
        for(vector<Atom>::iterator atom1=con1->atoms.begin();     \
            atom1!=con1->atoms.end(); atom1++) {
                //Conjunction bindings; /* just a placeholder */
                /* EXPERIMENTAL: pass the conjunction itself as local bindings for
                 * dereferencing $vars. TODO maybe change to proper list of bindings
                 * when formula allows multiple disjuncts. Otherwise the scope of
                 * dereferencing is the current conjunction only */
            atom1->evalSlotVals(*con1);
        }
    }
}


/************************************
 * Match methods
 * **********************************/


void Match::print() {
    FILE_LOG(logDEBUG2) << this->id1 << "  " << this->id2 << "  " << \
        (this->match_type==Unlocked?"Unlocked":"Locked");
}

void Match::print(TLogLevel log_level) {
    FILE_LOG(log_level) << this->id1 << "  " << this->id2 << "  " << \
        (this->match_type==Unlocked?"Unlocked":"Locked");
}

/************************************
 * Matching methods
 * **********************************/

/*
 * next permutation of a maching without groups (i.e. group matching). Just do
 * subset of proper size and permute it.
 * Using Algorithm T from Knuth fasc3a (generating combinations).
 * t is this->ids1.size()
 * n is this->ids2.size()
 * indexing of c_{t-1}...c_0 instead of c_{t}...c_1 like in Knuth.
 * */
bool Matching::nextPerm() {
    
    /* see if need to do subset first */
    if (this->combine_id == -1) {
        this->subset_ids.clear();
            /* if t==n subset is the whole set */
        if (this->ids1.size()==this->ids2.size()) {
            for (std::vector<Atom>::size_type jj = 1; jj <= this->ids1.size(); jj++) {
                this->subset_ids.push_back(jj - 1);
            }
        } else {
                /* set up combinations (subset). Subset_ids is all c_j in Knuth */
            for (std::vector<Atom>::size_type jj = 1; jj <= this->ids1.size(); jj++) {
                this->subset_ids.push_back(jj - 1);
            }
            this->subset_ids.push_back(this->ids2.size());
            this->subset_ids.push_back(0);
            this->j = this->ids1.size();
        }
    } else if (this->permute_id == this->total_permutes - 1) {
            /* if permutation counter maxed, and combine_id counter is less than
             * total_combines, then need a new subset */
        if (this->combine_id == this->total_combines - 1) {
            FILE_LOG(logERROR) << "\
Error: Requested nextPerm() beyond the number of total_combines: Shouldn't be getting here.";
            return false;
        } else {
                /* make a new combination (subset) */
            if (this->j > 0) {
                    /* T2 after visit */
                this->x = this->j;
                    /* T6 */
                this->subset_ids[ this->j - 1 ] = this->x;
                this->j--;
            } else {
                    /* T3 */
                if ( this->subset_ids[0] + 1 < this->subset_ids[1]) {
                    this->subset_ids[0]++;
                        /* return to T2 (visit and the rest) */
                } else {
                    this->j = 2;
                    
                        /* T4 */
                    this->subset_ids[ this->j - 2 ] = this->j - 2;
                    this->x = this->subset_ids[ this->j - 1] + 1;
                    while ( this->x == this->subset_ids[ this->j ] ) {
                        this->j++;
                        this->subset_ids[ this->j - 2 ] = this->j - 2;
                        this->x = this->subset_ids[ this->j - 1] + 1;
                    }
                        /* T5 */
                    if ( this->j > this->ids1.size()) {
                        FILE_LOG(logERROR) << "\
Error: Requested nextPerm() beyond the number of total tuples: Shouldn't be getting here.";
                        return false;
                    }
                        /* T6 */
                    this->subset_ids[ this->j - 1 ] = this->x;
                    this->j--;
                }
            }
        }
    }

    this->combine_id++; /* works for initial value -1 too */

        
        /*
         * visiting the combination (subset).
         * Algorithm L from Knuth 7.2.1.2: Generating all permutations.
         * permute_ids[1..ids1.size()] stand for a_1, a_n and refer to positions 1..ids1.size().
         * Minus 1 that and we get the references to
         * subset_ids[0..ids1.size()-1]. Hence dereferencing should be done from combine_ids
         * to subset_ids to ids2.
         * this->i stands for j in Knuth.
         * make a new permutation, increment permute_id
         *
         * */
    std::vector<Atom>::size_type i=0;
    std::vector<Atom>::size_type l=0;
    std::vector<Atom>::size_type k=0;
    std::vector<Atom>::size_type temp;
    if ((this->permute_id == -1) || (this->permute_id == this->total_permutes - 1)) {
            /* initialize/reset permute_ids*/
        this->permute_ids.clear();
        this->permute_ids.push_back(0); /* initialize a_0 to an impossibly small number. */
            /* initialize a_1 to a_n */
        for (std::vector<Atom>::size_type jj = 1; jj <= this->ids1.size(); jj++) {
            this->permute_ids.push_back(jj);
        }
        this->permute_id = 0;
            /* done with permuting for this case */
    } else {
            /* this is not the 0th permute */
            /* L2 */
        i = this->ids1.size() - 1;
        while (( this->permute_ids[i] >= this->permute_ids[i + 1]   )&&( i != 0 )) {
            i--;
        }
        FILE_LOG(logDEBUG2) << "L2 step. i = " << i;
        if (i != 0) {
                /* if i == 0 then done. Otherwise L3 */
            l = this->ids1.size();
            while ( this->permute_ids[i] >= this->permute_ids[l] ) {
                l--;
            }
                //FILE_LOG(logDEBUG2) << "L2 step. l = " << l;
            temp = this->permute_ids[i];
            this->permute_ids[i] = this->permute_ids[l];
            this->permute_ids[l] = temp;
            
                //FILE_LOG(logDEBUG2) << "L2 step. permute_ids[i] = " << permute_ids[i];
                //FILE_LOG(logDEBUG2) << "L2 step. permute_ids[l] = " << permute_ids[l];
                /* L4 */
            k = i + 1;
            l = this->ids1.size();
            while (k < l) {
                temp = this->permute_ids[k];
                this->permute_ids[k] = this->permute_ids[l];
                this->permute_ids[l] = temp;
                k++;
                l--;
            }
        } 
        this->permute_id++;
    }
 
        /* visit the eventual permutation: reconstruct the mapping within the group */
    this->mapping.clear();
    for (std::vector<Atom>::size_type  m = 0; m < this->ids1.size(); m++) {
        Match aMatch;
        aMatch.id1 = this->ids1[ m ];
            /* permute_ids indeces are 1...n, subset_ids are 0...n-1 */
        aMatch.id2 = this->ids2[ this->subset_ids[ this->permute_ids[m+1] - 1 ] ];
        aMatch.match_type = Locked;
        this->mapping.push_back(aMatch);
    }
    return true;
}


/*
 * generate the next matching. if all matchings have been generated return false.
 * Algorithm:
 * - form the map of matchings grouped based on type+subtype (supplied by init).
 *    - that includes vectors of indexes of root bindings that match type+subtype
 *      in ids2 of respective grops.
 * - for each group, do a matching, in turn. if a matching cannot be done
 *   at any point, return false.
 * - reassemble groups matchings into a single mapping.
 *
 * In this method: generate all tuples of group matchings.
 * */
bool Matching::next() {
        /* if this->combine_id == -1 then do the initialization first */
        /* this is the base matching */
    if (this->combine_id == -1) {
        for( std::map< string, Matching >::iterator group_it = this->groups.begin();
         group_it != this->groups.end(); group_it++) {
            /* group permute ids are updated in nextPerm */
                //group_it->second.combine_id = 0; 
            group_it->second.nextPerm();
        }
    } else {
        std::map< string, Matching >::iterator group_it = this->groups.begin();
        while ((group_it != this->groups.end()) &&
               (group_it->second.combine_id == group_it->second.total_combines - 1)) {
                    /* register full, reset within-group counters */
            group_it->second.combine_id = -1;
            group_it->second.permute_id = -1;
            group_it->second.nextPerm();
            group_it++;
        }

        if  (group_it != this->groups.end()) {
                //group_it->second.combine_id++;
            group_it->second.nextPerm();
        } else {
                /* went through all tuples */
            return false;
        }
    }

    this->combine_id++;

        /*
         * reassemble the group combinations in this->mapping
         * */
    for( std::map< string, Matching >::iterator group_it = this->groups.begin();
         group_it != this->groups.end(); group_it++) {

        for(std::vector< Match >::iterator aMatch_it=group_it->second.mapping.begin();
            aMatch_it != group_it->second.mapping.end(); aMatch_it++) {
            if (this->mapping[aMatch_it->id1].id1 != aMatch_it->id1) {
                FILE_LOG(logERROR) << "\
Error: Invalid mapping of base-level matching: all atoms in con1 should be\n \
       in the same order as they are in con1 and in this->ids1.";
            }
            this->mapping[aMatch_it->id1].id2 = aMatch_it->id2;
            this->mapping[aMatch_it->id1].match_type = Locked;
        }

    }
       
    return true;
}


/* lock matches (i.e. set lock flag) for atoms that share 'this' slot.
 * In effect, makes an appropriate seedMatching */
bool Matching::init(Conjunction &con1, Conjunction &con2) {
        /*
         * initialize the type+subtype groups, do the locking
         * based on inevitability (1 candidate for 1 mapping)
         * and 'this'.
         */
        /*
         * lock bmapping based on 'this'
         */
    std::vector<Atom>::size_type i=0;

    for(vector<Atom>::iterator atom1=con1.atoms.begin();  \
        atom1!=con1.atoms.end(); atom1++) {
        Match aMatch;
        aMatch.id1 = i;
        string this_val1 = atom1->readSlotVal("this");
        
        if (this_val1 == "_NOT_FOUND_") {
            FILE_LOG(logERROR) << "\
Error: 'this' slot not found in the following atom: ";
            atom1->print(logERROR);
                /* allow smooth operation if preconditions atom doesn't
                 * have 'this' slot */
            aMatch.match_type = Unlocked;
        } else if ((this_val1 == "") || (this_val1 == "_NO_VALUE_")) {
                /* unlocked atom */
            aMatch.match_type = Unlocked;
        } else {
        
                /* 'this' val is meaningful, find matching atom in con2*/
            std::vector<Atom>::size_type j=0;
            for(vector<Atom>::iterator atom2=con2.atoms.begin();  \
                atom2!=con2.atoms.end(); atom2++) {
                if (atom2->readSlotVal("this") == this_val1) {
                        /* found matching atom in con2*/
                    break;
                }
                j++;
            }
            if (j >= con2.atoms.size()) {
                    /* matching atom not found in con2 */
                FILE_LOG(logDEBUG4) << "\
Not found atom in con2 that matches 'this' slot with value: " << this_val1;
                return false;
            } else {
                aMatch.match_type = Locked;
                aMatch.id2 = j;
            }
        }

            /* got aMatch object, insert into mapping */
        this->mapping.push_back(aMatch);
        i++;
    }

    FILE_LOG(logDEBUG4) << "\
Created mapping of size: " << this->mapping.size();
    this->print(logDEBUG4);

    
        /* make record of an unlocked atom in group matchings */
    for (std::vector< Match >::size_type i=0; i!=this->mapping.size(); i++) {
        if (this->mapping[i].match_type == Unlocked) {
                /* get group id (atom signature) */
            string type1 = con1.atoms[this->mapping[i].id1].readSlotVal("type");
            string subtype1 = con1.atoms[this->mapping[i].id1].readSlotVal("subtype");
            string group_id = type1 + subtype1;
            
            if (type1.empty() || type1=="_NOT_FOUND_") {
                FILE_LOG(logERROR) << "\
Error: not found type in the following atom:";
                con1.atoms[this->mapping[i].id1].print(logERROR);
                return false;
            }

            if (subtype1.empty() || subtype1=="_NOT_FOUND_") {
                FILE_LOG(logERROR) << "\
Error: not found subtype in the following atom:";
                con1.atoms[this->mapping[i].id1].print(logERROR);
                return false;
            }

            
            if (!this->addAtomToGroup(i, group_id, con2)) {
                return false;
            }
        }
    }

        /* update total_combines in the base level Matching. Optional, just for
         * debugging reference */
    this->total_combines = 1;
    for( std::map< string, Matching >::iterator group_it = this->groups.begin();
         group_it != this->groups.end(); group_it++) {
        this->total_combines *= group_it->second.total_combines;
    }
    return true;
}

/*
 * Adds atom1 to a grouped matching based on type+subtype. Creates a group if none
 * exist for given type+subtype.
 * Returns false if a group has more ids1 than ids2
 * */
bool Matching::addAtomToGroup(std::vector<Atom>::size_type id1, string &group_id, Conjunction &con2) {
    if ( this->groups.count(group_id)==1 ) {
        
            /* group exists, add atom1 id to the indexes to match */
        this->groups[group_id].ids1.push_back(id1);

        if (this->groups[group_id].ids1.size() >
            this->groups[group_id].ids2.size()) {
                /* more atoms to match than candidates available, cannot unify */
            FILE_LOG(logDEBUG4) << "\
Failed to unify because ids1.size() > ids2.size() in group " << group_id << \
                ", namely: " << groups[group_id].ids1.size() <<      \
                " > " << this->groups[group_id].ids2.size();
            return false;
        }
        
            /* (n choose s) * s! = n*...*(n-s+1) */
        this->groups[group_id].total_combines *=
            (this->groups[group_id].ids2.size() - this->groups[group_id].ids1.size() + 1);
        this->groups[group_id].total_permutes *= this->groups[group_id].ids1.size(); /* (s! permutes) */
        
    } else {
            /* group does not exist, create one*/
        Matching &new_group = this->groups[group_id];
        new_group.ids1.push_back(id1);

            /* create a list of ids of potentially matching atoms from con2 */
        std::vector<Atom>::size_type id2 = 0;
        for(vector<Atom>::iterator atom2=con2.atoms.begin();    \
                atom2!=con2.atoms.end(); atom2++) {
            FILE_LOG(logDEBUG4) << "\
Got con2 atom signature " << this->getAtomSignature(*atom2) << ".";
            FILE_LOG(logDEBUG4) << "\
Comparing with new group id: " << group_id << "."; 
            if (this->getAtomSignature(*atom2)==group_id) {
                FILE_LOG(logDEBUG4) << "\
Comparison Succeeded."; 
                     /* check that id2 has not been locked already */
                bool locked = false;
                for (std::vector< Match >::size_type i=0; i!=this->mapping.size(); i++) {
                    if ((this->mapping[i].id2 == id2) &&
                        (this->mapping[i].match_type == Locked)) {
                        locked = true;
                        break;
                    }
                }
                    /* if id2 atom already had been locked the skip to next atom */
                if (!locked) {
                    FILE_LOG(logDEBUG4) << "\
Pushing id2: " << id2 << " into the group with id: " << group_id;
                    new_group.ids2.push_back(id2);
                }
            }
            id2++;
        }
        if (this->groups[group_id].ids2.size() == 0) {
                /* equiv to ids2.size < ids1.size at this point since ids1.size == 1 */
            FILE_LOG(logDEBUG4) << "\
Failed to unify as ids2.size()=0 for atom signature: " << group_id << "."; 
                /* no candidates available, cannot unify */
            return false;
        } 
        /* (n choose s) * s! = n when s=1 */        
        new_group.total_combines = new_group.ids2.size();
        new_group.total_permutes = 1;                              /* s! = 1 for s=1 */ 
    }
    return true;
}

string Matching::getAtomSignature(Atom &atom) {
    string type = atom.readSlotVal("type");
    string subtype = atom.readSlotVal("subtype");

    if (type.empty() || type=="_NOT_FOUND_") {
        FILE_LOG(logERROR) << "\
Error: not found type in the following atom:";
        atom.print(logERROR);
    }

    if (subtype.empty() || subtype=="_NOT_FOUND_") {
        FILE_LOG(logERROR) << "\
Error: not found subtype in the following atom:";
        atom.print(logERROR);
    }

    return type + subtype;
}

void Matching::print() {
    FILE_LOG(logDEBUG2) << "Mapping with sizes: " << this->ids1.size() << " and " << \
        this->ids2.size();
    FILE_LOG(logDEBUG2) << "Between ids1:";
    for(std::vector< std::vector<Atom>::size_type  >::iterator          \
            id_it=this->ids1.begin();                                   \
        id_it != this->ids1.end(); id_it++) {
        FILE_LOG(logDEBUG4) << *id_it;
    }
    FILE_LOG(logDEBUG4) << "...and ids2:";
    for(std::vector< std::vector<Atom>::size_type  >::iterator          \
            id_it=this->ids2.begin();                                   \
        id_it != this->ids2.end(); id_it++) {
        FILE_LOG(logDEBUG4) << *id_it;
    }
    FILE_LOG(logDEBUG4) << "combine_id: " << this->combine_id << " of total_combines: " << \
         this->total_combines;
    FILE_LOG(logDEBUG4) << "permute_id: " << this->permute_id << " of total_permutes: " << \
         this->total_permutes;   
    for(std::vector< Match >::iterator                           \
            aMatch_it=this->mapping.begin();                     \
        aMatch_it != this->mapping.end(); aMatch_it++) {
        aMatch_it->print(logDEBUG4);
    }
    FILE_LOG(logDEBUG2) << "subset_ids:";
    for(std::vector< std::vector<Atom>::size_type  >::iterator                           \
            subset_it=this->subset_ids.begin();                         \
        subset_it != this->subset_ids.end(); subset_it++) {
        FILE_LOG(logDEBUG4) << *subset_it;
    }
    FILE_LOG(logDEBUG2) << "permute_ids:";
    for(std::vector< std::vector<Atom>::size_type   >::iterator                                  \
            permute_it=this->permute_ids.begin();                         \
        permute_it != this->permute_ids.end(); permute_it++) {
        FILE_LOG(logDEBUG4) << *permute_it;
    }
    FILE_LOG(logDEBUG4) << "... and a total of " << this->groups.size() << \
        " group matchings.";
    for(std::map< string, Matching >::iterator                       \
            group_it=this->groups.begin();                           \
        group_it != this->groups.end(); group_it++) {
        FILE_LOG(logDEBUG2) << "    Group id: " << group_it->first;
        group_it->second.print();
    }
}


void Matching::print(TLogLevel log_level) {
    FILE_LOG(log_level) << "Mapping with sizes: " << this->ids1.size() << " and " << \
        this->ids2.size();
    FILE_LOG(log_level) << "Between ids1:";
    for(std::vector< std::vector<Atom>::size_type  >::iterator          \
            id_it=this->ids1.begin();                                   \
        id_it != this->ids1.end(); id_it++) {
        FILE_LOG(log_level) << *id_it;
    }
    FILE_LOG(log_level) << "...and ids2:";
    for(std::vector< std::vector<Atom>::size_type  >::iterator          \
            id_it=this->ids2.begin();                                   \
        id_it != this->ids2.end(); id_it++) {
        FILE_LOG(log_level) << *id_it;
    }
    FILE_LOG(log_level) << "combine_id: " << this->combine_id << " of total_combines: " << \
         this->total_combines;
    FILE_LOG(log_level) << "permute_id: " << this->permute_id << " of total_permutes: " << \
         this->total_permutes;   
    for(std::vector< Match >::iterator                           \
            aMatch_it=this->mapping.begin();                     \
        aMatch_it != this->mapping.end(); aMatch_it++) {
        aMatch_it->print(log_level);
    }
    FILE_LOG(log_level) << "subset_ids:";
    for(std::vector< std::vector<Atom>::size_type  >::iterator                           \
            subset_it=this->subset_ids.begin();                         \
        subset_it != this->subset_ids.end(); subset_it++) {
        FILE_LOG(log_level) << *subset_it;
    }
    FILE_LOG(log_level) << "permute_ids:";
    for(std::vector< std::vector<Atom>::size_type   >::iterator                                  \
            permute_it=this->permute_ids.begin();                         \
        permute_it != this->permute_ids.end(); permute_it++) {
        FILE_LOG(log_level) << *permute_it;
    }
    FILE_LOG(log_level) << "... and a total of " << this->groups.size() << \
        " group matchings.";
    for(std::map< string, Matching >::iterator                       \
            group_it=this->groups.begin();                           \
        group_it != this->groups.end(); group_it++) {
        FILE_LOG(log_level) << "    Group id: " << group_it->first;
        group_it->second.print(log_level);
    }
}
