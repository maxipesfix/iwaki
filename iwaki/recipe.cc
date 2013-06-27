/*****************************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * FILE: recipe.cc
 *
 * ABSTRACT: recipe parsing methods
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
#include <csignal>

#include <sys/time.h>
#include <unistd.h>

#include <math.h>

//using namespace std;
using std::string;
using std::iostream;

/********************************************/
/* BodyElement methods */
/********************************************/

/*
 * Return true if the body element is timed out.
 * */
bool BodyElement::isTimedOut() {
    if (this->timeout < 0) {             /* make negative timeout to never expire */
        return false;
    } else {
        return ( (getSystemTimeMSec()/1000.0)
                 >= (this->dispatch_time + this->timeout) ) ;
    }
}

/* mass assign args to all the random_elements of the element
 * */
void BodyElement::assignArgsToRandElements(Args &args) {
    for(vector<BodyElement>::iterator randElement_it = this->random_elements.begin();
        randElement_it < this->random_elements.end(); randElement_it++) {
        randElement_it->args = args;
    } 
}


/* mass assign return_args to all the random_elements of the element
 * */
void BodyElement::assignReturnArgsToRandElements(Args &return_args) {
    for(vector<BodyElement>::iterator randElement_it = this->random_elements.begin();
        randElement_it < this->random_elements.end(); randElement_it++) {
        randElement_it->return_args = return_args;
    } 
}


/* verify and assign default probabilities
 * */
bool BodyElement::verifyProbabilities() {
    int count_defaults = 0;
    double sum_probs = 0;
                                                  /* count default probs and sum of
                                                   * non-default ones */
    for(unsigned int i = 0;  i < this->element_probs.size(); i++) {
        if (this->element_probs[i] < 0) {
            count_defaults++;                     /* to be defaulted*/
        } else {
            sum_probs += this->element_probs[i];
        }
    }

    if (sum_probs > 1.0) {
        FILE_LOG(logERROR) << "Probability sum is greater than 1\n" <<
            "in body element action: " << this->name;
        return false;
    }

    if (count_defaults > 0) {
        double default_prob = (1.0 - sum_probs)/count_defaults;
        for(unsigned int i = 0;  i < this->element_probs.size(); i++) {
            if (this->element_probs[i] < 0) {
                this->element_probs[i] = default_prob;
            }
        }
    }
    return true;
}

/* derandomize body element. History of random outcomes is recorded
 * in recipes themselves, not in the nodes, since nodes get destroyed.
 * */
BodyElement BodyElement::derandomize(string recipe_name, unsigned int active_element,
                                     History &history) {

    unsigned int i = 0;
    double p=0;                                             /* cumulative probability */
    double normalizer = 0;                                  /* sum of probs of old enough */
                                                            /* outcomes */

    if (history.recipeHistories.count(recipe_name) == 0) {
        FILE_LOG(logERROR) <<
            "Recipe name not found in recipeHistories when derandomizing.";
        return *this;
    }
        
    if (active_element >= history.recipeHistories[recipe_name].bodyElementHistories.size()) {
        FILE_LOG(logERROR) <<
            "Active element number exceeds the size of bodyElementHistories.";
        return *this;
    }
    
    BodyElementHistory &elementHistory =
        history.recipeHistories[recipe_name].bodyElementHistories[active_element];
    
    if (!this->random) {
        return *this;
    }

    if (this->element_probs.size() == 0) {
        FILE_LOG(logERROR) << "Empty element_probs for a random element.";
        return *this;
    }

    if (elementHistory.outcome_age.size() != this->element_probs.size()) {
        FILE_LOG(logERROR) << "outcome_age and element_probs have different sizes.";
        return *this;
    }

    
    p = this->element_probs[0];
                                     
    double rseed = ((double)rand()/(double)RAND_MAX);
    FILE_LOG(logDEBUG3) << "rseed: " << rseed;

        /* compute the sum of probabilities of outcomes that are old enough */
    for (unsigned int j=0; j < elementHistory.outcome_age.size(); j++) {
        FILE_LOG(logDEBUG3) << "Age of outcome " << j << " is " <<
            elementHistory.outcome_age[j];
        if (elementHistory.outcome_age[j] > this->unique_within) {
            normalizer += this->element_probs[j];
        }
    }

    FILE_LOG(logDEBUG3) << "Normalizer: " << normalizer;
    
    if (normalizer == 0) {
        FILE_LOG(logERROR) << "All outcomes are too young, the unique_within is likely 0\n"
                           << "should be positive integer";
    }

        /* skip to the first old outcome */
    while ((i < elementHistory.outcome_age.size() - 1) &&
           (elementHistory.outcome_age[i] <= this->unique_within)) {
        i++;
    }
    FILE_LOG(logDEBUG3) << "First old outcome is " << i << ", age: " <<
        elementHistory.outcome_age[i];
        /* find an old outcome that contains rseed when scaled with the normalizer */
    while ((i < this->element_probs.size() - 1) && ((p/normalizer) < rseed)) {
        i++;
        p += this->element_probs[i];
        
            /* skip to the next old outcome
             * elementHistory.outcome_age.size() is better be equal to
             * this->element_probs.size()
             * */
        while ((i < elementHistory.outcome_age.size() - 1) &&
               (elementHistory.outcome_age[i] <= this->unique_within)) {
            i++;
        }
        FILE_LOG(logDEBUG3) << "Next old outcome is " << i << ", age: " <<
        elementHistory.outcome_age[i];
    }


    
        /*
          while ((i < this->element_probs.size() - 1)&&(p<rseed)) {
          i++;
          p += this->element_probs[i];
          }
        */
    
        /* i is the chosen outcome */
    FILE_LOG(logDEBUG3) << "Chosen outcome: " << i;
    
        /* update outcome age */
    elementHistory.outcome_age[i] = 0;
        /* age all outcomes */
    for (unsigned int j=0; j < elementHistory.outcome_age.size(); j++) {
        if (elementHistory.outcome_age[j] < MAX_ELEMENT_AGE) {
            elementHistory.outcome_age[j]++;               /* cap max age to prevent */
        }                                                  /* overflow */
        FILE_LOG(logDEBUG3) << "Updated outcome " << j << ", age: " <<
        elementHistory.outcome_age[j];
    }
    
        /* update the outcome history */
    elementHistory.outcome_history.push_front(i);
    if (elementHistory.outcome_history.size() > this->element_probs.size()) {
        elementHistory.outcome_history.pop_back();
    }

    this->chosen_outcome = i;
    
    return this->random_elements[i];
}

bool BodyElement::parseActionLevel2_preprocessRandom(TiXmlElement* pElem)
{
    TiXmlHandle hRoot=TiXmlHandle(pElem);
    TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();

                                                              /* do first pass to see
                                                               * if there is random tag
                                                               * anywhere in the element
                                                               * */
    this->random = false;
    
    for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement()) {
        string level1tag = pLevel1Node->Value();
        FILE_LOG(logDEBUG2) << "Level 2 tag: " << level1tag << endl;
    
        if (level1tag=="roboml:random") {
                                         /* ok there is a random tag, create
                                          * a vector of body element versions
                                          * */
                                         /* set random flag for the BodyElement */
            this->random = true;

                /* this snippet stores the TinyXML object in BodyElement */
            if (!this->pXmlElement) { 
                TiXmlNode* pElemCopy = pElem->Clone(); 
                this->pXmlElement = pElemCopy->ToElement(); 
                    /* debug output */ 
                    //this->printXML(this->pXmlElement); 
            } 

            
            if (pLevel1Node->Attribute("unique_within")) {
                this->unique_within =
                    string_to_number(pLevel1Node->Attribute("unique_within"));
                FILE_LOG(logDEBUG3) <<                                  \
                    "Parsed attribute 'unique_within'=" << this->unique_within;
            } else {
                FILE_LOG(logDEBUG3) <<                                  \
                    "Attribute 'unique_within' missing from random_outcome tag " << \
                    "in body element action: " << this->name;
                FILE_LOG(logDEBUG3) << "... using default unique_within = 0";
                this->unique_within = 0;
                
            }
            
            
            TiXmlHandle hRoot2=TiXmlHandle(pLevel1Node);
            TiXmlElement* pLevel3Node=hRoot2.FirstChildElement().Element();

            
            double p = 0;                                  /* outcome probability */
                                                           /* create a rand_element for
                                                            * every random outcome and
                                                            * parse the content of the
                                                            * random outcome */
            for(; pLevel3Node; pLevel3Node=pLevel3Node->NextSiblingElement()) {
                string level3tag = pLevel3Node->Value();
                FILE_LOG(logDEBUG1) << "Level 3 random tag: " << level3tag;
                if (level3tag=="roboml:random_outcome") {

                    BodyElement rand_element;
                                                        /* get the probability */
                    if (pLevel3Node->Attribute("p")) {                   
                        p = string_to_double(pLevel3Node->Attribute("p"));         
                        FILE_LOG(logDEBUG3) << "Parsed p: " << p;
                    } else {
                        FILE_LOG(logDEBUG3) <<                           \
                            "Attribute 'p' missing from random_outcome tag " << \
                            "in body element action: " << this->name;
                        FILE_LOG(logDEBUG3) <<                          \
                            "Will attempt to assign a default probability value.";
                        p = -1.0;
                    }
                    
                    rand_element.parseActionLevel2_preprocessRandom(pLevel3Node);
                    this->random_elements.push_back(rand_element);
                    this->element_probs.push_back(p);
                } else {
                    FILE_LOG(logERROR) << \
                        "BodyElement parsing error: Unknown level 3 tag \n" << \
                        "(should be random_outcome): " << level3tag <<  \
                        " in body element action: " << this->name;
                    return false;
                }
            }
            
        }
    }

                                                           /* second pass, parse the
                                                            * non-random content, assigning
                                                            * it to this element and all
                                                            * the rand_elements if they
                                                            * were created in the first
                                                            * pass*/

    pLevel1Node = hRoot.FirstChildElement().Element();
        
    for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement())
    {
        string level1tag = pLevel1Node->Value();
        FILE_LOG(logDEBUG2) << "Level 2 tag: " << level1tag << endl;
            
        if (level1tag=="roboml:args") {
                                                            /* make sure the argument
                                                             * list is empty since
                                                             * loading just pushes new
                                                             * args on the list */
            this->args.clear(); 
            if (!this->args.load(pLevel1Node)){
                FILE_LOG(logERROR) << "Could not load args from action: "
                                   << this->name;
                return false;
            } else {
                this->args.print(logDEBUG2);
                this->assignArgsToRandElements(this->args);      /* automatically works even
                                                                  * if no random elements
                                                                  * */
            }
        } else if (level1tag=="roboml:return_args") {
                                                                 /* make sure the argument
                                                                  * list is empty since
                                                                  * loading just pushes
                                                                  * new args on the list */
            this->return_args.clear(); 
            if (!this->return_args.load(pLevel1Node)){
                FILE_LOG(logERROR) << "Could not load return_args from action: "
                                   << this->name;
                return false;
            } else {
                this->return_args.print(logDEBUG2);
                this->assignReturnArgsToRandElements(this->return_args);
                                                                 /* automatically works even
                                                                  * if no random elements
                                                                  * */
            }
        } else if (level1tag=="roboml:random") {
                                                                 /* don't do anything, since
                                                                  * this has been taken care
                                                                  * of in the first pass */
        } else {
            FILE_LOG(logERROR) << "Unknown level 1 tag: " << level1tag << \
                " in body element action: " << this->name;
        }
    }

                                            /* finally, if this has random_elements,
                                             * verify probabilities and assign default
                                             * values if necessary */

    
    
    
    return true;
}


bool BodyElement::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  string level1tag = pElem->Value();
  // cout << "Processing body element." << endl;
  
  if (level1tag=="action") {
    this->element_type = "action";
    /* name */
    if (pElem->Attribute("name")) {
        this->name = pElem->Attribute("name");
    } 
    else { 
        cout << "Error: action has no name." << endl;
        return false;
    }
        /* value */
    if (pElem->Attribute("actor")) {
        this->actor = pElem->Attribute("actor");
    }

    if (pElem->Attribute("timeout")) {
        this->timeout = string_to_double(pElem->Attribute("timeout"));
    }
    
    if (pElem->Attribute("if_completed")) {
        this->if_completed = pElem->Attribute("if_completed");
    }
    if (pElem->Attribute("if_aborted")) {
        this->if_aborted = pElem->Attribute("if_aborted");
    }
    if (pElem->Attribute("if_failed")) {
        this->if_failed = pElem->Attribute("if_failed");
    }
    if (pElem->Attribute("if_timeout")) {
        this->if_timeout = pElem->Attribute("if_timeout");
    }
    if (pElem->Attribute("if_node_purged")) {
        this->if_node_purged = pElem->Attribute("if_node_purged");
    }

    if (pElem->Attribute("priority")) {
        this->priority = pElem->Attribute("priority");
    }
    
        /* binding */
    if (pElem->Attribute("action_space")) {
        this->action_space = pElem->Attribute("action_space");
    }
    
        /* parse the arguments if present */
    this->parseActionLevel2_preprocessRandom(pElem);
    
   // cout << "Action name:"<< this->name << ", actor:" << this->actor
   //     << ", action_space:"<< this->action_space << endl;
  } 
  else if (level1tag=="goal") {
    this->element_type = "goal";
    /* name (optional for goals) */
    if (pElem->Attribute("name")) {
      this->name = pElem->Attribute("name");
    } 
    /* goal initator */
    if (pElem->Attribute("initiator")) {
      this->initiator = pElem->Attribute("initiator");
    } 
    else {
      FILE_LOG(logERROR) << "Error: goal has no initiator.";
      return false;
    }
    /* recipe_name */
    if (pElem->Attribute("recipe_name")) {
      this->recipe_name = pElem->Attribute("recipe_name");
    }
    if (!this->formula.load(pElem)){
      FILE_LOG(logERROR) << "Error: could not load the formula in goal: " << this->name;
      return false;
    }
    //cout << "Goal name:"<< this->name << ", initiator:" << this->initiator
    //    << ", recipe_name:"<< this->recipe_name << endl;
  }
  else if (level1tag=="assignment") {
    this->element_type = "assignment";
    /* name (optional for assignments) */
    if (pElem->Attribute("name")) {
      this->name = pElem->Attribute("name");
    }
    if (!this->formula.load(pElem)){
      FILE_LOG(logERROR) << "Error: could not load the formula in assignment: " << this->name;
      return false;
    }
    //cout << "Assignment name:"<< this->name << endl;
  } 
  else {
    FILE_LOG(logERROR) << "Error: in body, unknown tag: " << level1tag;
  }
  return loadOkay;
}

void BodyElement::print() {
	FILE_LOG(logDEBUG1) << "  BodyElement " << "name: " << this->name << ", " << \
    "element_type: " << this->element_type << ", " <<                   \
    "recipe_name: " << this->recipe_name << ", " <<                     \
    "actor: " << this->actor << ", " <<                                 \
    "action_space: " << this->action_space << ", " <<                   \
    "initiator: " << this->initiator;
	this->formula.print();
}


/********************************************/
/* Body methods */
/********************************************/
bool Body::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  
  //cout << "Processing body" << endl;
  
  if (pElem->Attribute("order")) {
    this->order = pElem->Attribute("order");
  }

  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();
  
  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement())
  {
    string level1tag = pLevel1Node->Value();
    //cout << "In body, level 1 tag: " << level1tag << endl;
    if (level1tag=="action") {
      BodyElement action;
      if (!action.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Could not load action.";
        return false;
      } else { 
        this->elements.push_back(action);
      }
    } else if (level1tag=="goal") {
      BodyElement goal;
      if (!goal.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Could not load goal.";
        return false;
      } else { 
        this->elements.push_back(goal);
      }
    } else if (level1tag=="assignment") {
      BodyElement assignment;
      if (!assignment.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Could not load assignment.";
        return false;
      } else { 
        this->elements.push_back(assignment);
      }
    }
    else 
    {
      FILE_LOG(logERROR) << "Error: In atom, unknown level 1 tag: " << level1tag;
      return false;
    }
  }
  
  
  return loadOkay;
}

void Body::print() { 
	vector<BodyElement>::iterator an_element = elements.begin();
  	// go through body elements and see if the time is up
  FILE_LOG(logDEBUG1) << "* Body";
  while (an_element!=elements.end()) {
  	an_element->print();
  	an_element++;
  	}
}


/********************************************/
/* Recipe methods */
/********************************************/
bool Recipe::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  if (!pElem->Attribute("name")) {
    FILE_LOG(logERROR)  << "Error: Name missing in recipe."; 
    return false;
  } 
  this->name = pElem->Attribute("name");
  
  FILE_LOG(logINFO)  << "Loading recipe: \'" << this->name << "\'";
  
  if (!pElem->Attribute("priority")) {
      this->priority = 0;
  } else {
      this->priority = string_to_number(pElem->Attribute("priority"));
  }
  
  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();

  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement())
  {
    string level1tag = pLevel1Node->Value();
    //cout << "Level 1 tag: " << level1tag << endl;
    if (level1tag=="precondition") {
      if (!this->precondition.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Error: could not load precondition from recipe: " << this->name;
        return false;
      }
    } else if (level1tag=="whilecondition"){
        if (!this->whilecondition.load(pLevel1Node)){
            FILE_LOG(logERROR) << "Error: could not load whilecondition from recipe: " << this->name;
            return false;
        } else {
                /* set a recipe-level variable what to do if whilecondition fails */
            if (pLevel1Node->Attribute("if_failed")) {
                this->ifWhileconditionFailed = pLevel1Node->Attribute("if_failed");
            }
        }
    } else if (level1tag=="assignwhile"){
      if (!this->assignwhile.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Error: could not load whilecondition from recipe: " << this->name;
        return false;
      }
    } else if (level1tag=="body"){
      if (!this->body.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Error: could not load body from recipe: " << this->name;
        return false;
      }
    } else if (level1tag=="assignpost"){
      if (!this->assignpost.load(pLevel1Node)){
        FILE_LOG(logERROR) << "Error: could not load assignpost from recipe: " << this->name;
        return false;
      }
    } else
    {
      FILE_LOG(logERROR) << "Error: Unknown level 1 tag: " << level1tag;
      return false;
    }
    
  }
  return loadOkay;
}

/**
 ** Print contents of the recipe
 **/
void Recipe::print() {
	FILE_LOG(logDEBUG) << "* Recipe: " << this->name;
	FILE_LOG(logDEBUG) << "* Precondition";
	this->precondition.print();
	FILE_LOG(logDEBUG) << "* Whilecondition";
	this->whilecondition.print();
	FILE_LOG(logDEBUG) << "* Assignwhile";
	this->assignwhile.print();
	this->body.print();
	FILE_LOG(logDEBUG) << "* Assignpost";
	this->assignpost.print();
	FILE_LOG(logDEBUG) << "-----------------------------------------------";
}



/* add missing type and subtype to the formula atoms in goals, and assignposts
 * based on the share vars with preconditions */
bool Recipe::bindTypeAndSubtype() {
    bool res = true;
    for (vector<BodyElement>::iterator element_it = this->body.elements.begin();
         element_it!=this->body.elements.end(); element_it++) {
  	if (element_it->element_type == "goal") {
                /* fill missing type/subtype of formula atoms */
            if (!element_it->formula.bindTypeandSubtype(this->precondition,
                                                        this->name)) {
                res = false;
            }
        }
    }
    
    if (!this->assignpost.bindTypeandSubtype(this->precondition,
                                             this->name)) {
        res = false;
    }
    
    return res;
}
