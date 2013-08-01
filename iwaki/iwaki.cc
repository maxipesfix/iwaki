/*****************************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * FILE: iwaki.cc
 *
 * ABSTRACT: main methods
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
#include <algorithm>


#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
     


#include <math.h>

//using namespace std;
using std::string;
using std::iostream;



/********************************************/
/** Free functions*/
/********************************************/
void insertIntListSortedUnique(std::list<int> &aList, const int new_val) {
    std::list<int>::iterator val_it = aList.begin();
    while ((val_it != aList.end()) && (*val_it < new_val)) {
        val_it++;
    }
    if (val_it == aList.end()) {
                                                    /* append to the end */
        aList.insert(val_it, new_val);
    } else if (*val_it != new_val) {
                                                    /* insert before val_it */
        aList.insert(val_it, new_val);
    }
    
}


/********************************************/
/** Node methods */
/********************************************/

int Node::node_counter = 0;

void Node::print() {
    FILE_LOG(logDEBUG)  << "node_id:" << this->node_id;
    FILE_LOG(logDEBUG)  << "recipe_name" << this->recipe_name;
}

void Node::print(TLogLevel log_level) {
    FILE_LOG(log_level)  << "node_id:" << this->node_id;
    FILE_LOG(log_level)  << "recipe_name" << this->recipe_name;
}



void Node::setWhileBindings(tree<Node>::iterator root) {
    FILE_LOG(logDEBUG)  << "Setting while bindings" << endl;



}

/********************************************/
/** Plan tree methods */
/********************************************/
void PlanTree::print() {
        /* default log level */
    TLogLevel log_level = logDEBUG;
    
    FILE_LOG(log_level)  << "Plan tree:";
    tree<Node>::iterator sib2=this->tr.begin();
    tree<Node>::iterator end2=this->tr.end();
    while(sib2!=end2) {
        string indent = "";
        for(int i=0; i<this->tr.depth(sib2); ++i)
            indent += " ";
        FILE_LOG(log_level)  << indent << (*sib2).recipe_name;
        ++sib2;
    }
}


void PlanTree::print(TLogLevel log_level) {
    FILE_LOG(log_level)  << "Plan tree:";
    tree<Node>::iterator sib2=this->tr.begin();
    tree<Node>::iterator end2=this->tr.end();
    while(sib2!=end2) {
        string indent = "";
        for(int i=0; i<this->tr.depth(sib2); ++i)
            indent += " ";
        FILE_LOG(log_level)  << indent << (*sib2).recipe_name;
        ++sib2;
    }
}

/**
 * Counts number of instances of a child nodes corresponding to the
 * recipe childName
 * */
int PlanTree::nodeChildNameCount(tree<Node>::iterator &parentNode, string &childName) {
    int count = 0;
    tree<Node>::sibling_iterator sibNode=this->tr.begin(parentNode);
    while(sibNode != this->tr.end(parentNode)) {
        if (sibNode->recipe_name == childName) {
            count++;
        }
        sibNode++;
    }
    FILE_LOG(logDEBUG4)  << "Counter " << count << " instances of " <<  childName;
    return count;
}


tree<Node>::iterator PlanTree::findNodeGivenActionId(int action_id) {
    tree<Node>::iterator node=this->tr.begin();
    while(node != this->tr.end()) {
        if (node->ae_action_id == action_id) {
            return node;
        }
        else { node++;	}
    }

    return node;
}


/********************************************/
/** IM methods */
/** Loading and initializing */
/********************************************/
string InteractionManager::getRecipeDir()
{
    return (string)this->script_path + "/recipes/";
}

string InteractionManager::getActionDir()
{
    return (string)this->script_path + "/actions/";
}


/*
 *
 * Load recipe file specified in init file
 *
 * */
bool InteractionManager::loadRecipeFile(string recipe_filename)
{
    FILE_LOG(logDEBUG1)  << "Loading recipe file: " << recipe_filename;
    string full_recipe_filename = this->getRecipeDir() + recipe_filename;
    TiXmlDocument doc(full_recipe_filename.c_str());
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay ) {
        FILE_LOG(logERROR) << "Could not load recipe file " <<  recipe_filename.c_str() \
                             << ". Error=" << doc.ErrorDesc() << ". Exiting.";
    } else {
        TiXmlHandle hDoc(&doc);
        TiXmlElement* pElem;
        TiXmlHandle hRoot(0);
        string m_name;

            // block: Root name. Should be one or more action in BML
        pElem=hDoc.FirstChildElement().Element();
            // should always have a valid root but handle gracefully if it does not
        if (!pElem) return false;
        m_name=pElem->Value();

        for(; pElem; pElem=pElem->NextSiblingElement()) {
                //const TiXmlAttribute* name_node = pRecipeElem->attributeSet.Find("name");
            if (!pElem->Attribute("name")) {
                FILE_LOG(logWARNING) << "Recipe name missing in file: " << recipe_filename \
                                     << ", skipping";
                loadOkay=false;
            }
            else {
                string recipe_name = pElem->Attribute("name");
                Recipe recipe;
                if (!recipe.load(pElem)) {
                    FILE_LOG(logWARNING) << "Could not load recipe: " << recipe_name << \
                        " from file: " << recipe_filename << ", skipping";
                    loadOkay=false;
                }
                if (loadOkay) {this->recipes[recipe_name] = recipe;}
            }
        }

    }
    return loadOkay;
}

/*
 *
 * Load action file specified in init file (Basically identical to loading recipe file above)
 *
 * */

bool InteractionManager::loadActionFile(string recipe_filename)
{
    FILE_LOG(logDEBUG1)  << "Loading action file: " << recipe_filename;
    string full_recipe_filename = this->getActionDir() + recipe_filename;
    TiXmlDocument doc(full_recipe_filename.c_str());
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        FILE_LOG(logERROR) << "Could not load action file " <<  recipe_filename.c_str() \
                           << ". Error=" << doc.ErrorDesc() << ". Exiting.\n";
    } else
    {
        TiXmlHandle hDoc(&doc);
        TiXmlElement* pElem;
        TiXmlHandle hRoot(0);
        string m_name;

            // block: Root name. Should be one or more action in BML
        pElem=hDoc.FirstChildElement().Element();
            // should always have a valid root but handle gracefully if it does not
        if (!pElem) return false;
        m_name=pElem->Value();

        for(; pElem; pElem=pElem->NextSiblingElement()) {
                //const TiXmlAttribute* name_node = pRecipeElem->attributeSet.Find("name");
            if (!pElem->Attribute("name")) {
                FILE_LOG(logWARNING) << "Action name missing in file: " << recipe_filename \
                                     << ", skipping" << endl;
                loadOkay=false;
            }
            else {
                string action_name = pElem->Attribute("name");
                Action an_action;
                if (!an_action.load(pElem)) {
                    FILE_LOG(logWARNING) << "Could not load action block: " << action_name << \
                        " from file: " << recipe_filename << ", skipping" << endl;
                    loadOkay=false;
                }
                if (loadOkay) {this->actions[action_name] = an_action;
                    an_action.print(); /* debug */
                }
            }
        }

    }
    return loadOkay;
}

/** load default atoms */
bool InteractionManager::load_DefaultAtoms(string default_atoms_filename)
{
    string full_filename = this->script_path + "/" + default_atoms_filename;
    TiXmlDocument doc(full_filename.c_str());
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        FILE_LOG(logERROR) << "Could not load default atoms file:\n" << full_filename \
                           << "\nError=" << doc.ErrorDesc() << ". Exiting...\n";
    } 
    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);
    
        // block: Root name. 
    {
        pElem=hDoc.FirstChildElement().Element();
            // should always have a valid root but handle gracefully if it does
        if (!pElem) return false;
        string m_name=pElem->Value();
        if (m_name!="default_atoms") return false;
            // save this for later
        hRoot=TiXmlHandle(pElem);
    }
    
        /* call the formula loader */
    return this->default_atoms.load(pElem);
}
/**
 * Load init file and all the recipe files specified there
 **/
bool InteractionManager::load_init(string init_filename)
{
    string full_init_filename = this->script_path + "/" + init_filename;
    TiXmlDocument doc(full_init_filename.c_str());
    bool loadOkay = doc.LoadFile();

    if ( !loadOkay )
    {
        FILE_LOG(logERROR) << "Could not load init file:\n" << full_init_filename \
                           << "\nError=" << doc.ErrorDesc() << ". Exiting...\n";
    } else
    {
        TiXmlHandle hDoc(&doc);
        TiXmlElement* pElem;
        TiXmlHandle hRoot(0);

            // block: Root name. Should be iminit
        {
            pElem=hDoc.FirstChildElement().Element();
                // should always have a valid root but handle gracefully if it does
            if (!pElem) return false;
            string m_name=pElem->Value();
            if (m_name!="iminit") return false;
                // save this for later
            hRoot=TiXmlHandle(pElem);
        }


            // block: recipefiles
        {
            TiXmlElement* pFileElem =
                hRoot.FirstChild( "recipefiles" ).FirstChild().Element();

            for(; pFileElem; pFileElem=pFileElem->NextSiblingElement())
            {
                    //const char* pKey=pFileElem->Value();
                const char* pText=pFileElem->GetText();

                    //printf("pKey: %s pText: %s\n", pKey, pText);

                if (!this->loadRecipeFile((string)pText))
                {
                    FILE_LOG(logERROR) << "Could not load recipe file: "
                                       << pText << ", skipping" << endl;
                };
            }
        }

            // block: actionfiles
        {
            TiXmlElement* pFileElem =
                hRoot.FirstChild( "actionfiles" ).FirstChild().Element();

            for(; pFileElem; pFileElem=pFileElem->NextSiblingElement())
            {
                    //const char* pKey=pFileElem->Value();
                const char* pText=pFileElem->GetText();

                    //printf("pKey: %s pText: %s\n", pKey, pText);

                if (!this->loadActionFile((string)pText))
                {
                    FILE_LOG(logERROR) << "Could not load acton file: "
                                       << pText << ", skipping" << endl;
                };
            }
        }

            // block: triggerables
        this->triggerables_rec.clear(); // clear the current triggerables record
        {
            TiXmlElement* pTrigNode =
                hRoot.FirstChild( "triggerables" ).FirstChild().Element();
            for(; pTrigNode; pTrigNode=pTrigNode->NextSiblingElement())
            {
                string max_instances;
                string name = pTrigNode->Attribute("name");
                if (!pTrigNode->Attribute("max_instances")) {
                    max_instances = "any";
                } else {
                    max_instances = pTrigNode->Attribute("max_instances");
                }

                TriggerableRecord arec;
                arec.name = name;
                arec.max_instances = max_instances;

                    /* check if priority is specified */
                int priority = 0;
                if (pTrigNode->Attribute("priority")) {
                    priority = string_to_number(pTrigNode->Attribute("priority"));

                        /* if priority is specified,
                         * update priority in the recipe map */

                    map<string, Recipe>::iterator recipe_it =
                        this->recipes.find(name);
                    if (recipe_it == this->recipes.end()) {
                        FILE_LOG(logERROR) <<
                            "Error updating recipe priorities: recipe "
                                           << name <<
                            " is not found in IM recipe map";
                        return false;
                    } else {
                        recipe_it->second.priority = priority;
                    }
                } else {
                        /* if no priority specified, retrieve it from
                         * the recipe map (it's always there, even if default) */
                    map<string, Recipe>::iterator recipe_it =
                        this->recipes.find(name);
                    if (recipe_it == this->recipes.end()) {
                        FILE_LOG(logERROR) <<
                            "Error retrieving a priority for the recipe "
                                           << name <<
                            " --- not found in IM recipe map";
                        return false;
                    } else {
                         priority = recipe_it->second.priority;
                    }
                }

                    /* insert triggerable according to the priority order
                     * NOTE: inserting in the end of the order group,
                     * because guess what, some recipes already rely on their
                     * order in init file. Need those recipes re-written! */
                list<TriggerableRecord>::iterator trec_it =
                    this->triggerables_rec.begin();
                while ((trec_it != this->triggerables_rec.end()) &&
                       (this->recipes.find(trec_it->name)->second.priority >=
                        priority )) {
                    trec_it++;
                }
                this->triggerables_rec.insert(trec_it, arec);
                
                FILE_LOG(logDEBUG1) << "name: " << name
                                    << ", max_instances: " << max_instances;
            }

        }

            /* block: backchainables */
        TiXmlElement* pTrigNode =
            hRoot.FirstChild( "backchaineables" ).FirstChild().Element();
        for(; pTrigNode; pTrigNode=pTrigNode->NextSiblingElement())
        {
            string name = pTrigNode->Attribute("name");
            
                /* check if priority is specified */
            int priority = 0;
            if (pTrigNode->Attribute("priority")) {
                priority = string_to_number(pTrigNode->Attribute("priority"));
                
                    /* if priority is specified,
                     * update priority in the recipe map */
                
                map<string, Recipe>::iterator recipe_it =
                    this->recipes.find(name);
                if (recipe_it == this->recipes.end()) {
                    FILE_LOG(logERROR) <<
                        "Error updating recipe priorities: recipe "
                                       << name <<
                        " is not found in IM recipe map";
                    return false;
                } else {
                    recipe_it->second.priority = priority;
                }
            }   
        }

            /* fill in the list of recipe names */
        for (std::map<string, Recipe>::iterator recipe2_it =
                 this->recipes.begin(); recipe2_it != this->recipes.end();
             recipe2_it++) {
            this->recipe_names.push_back(recipe2_it->first);
        }
    }
    
    
    return loadOkay;
}

void InteractionManager::printRecipes() {
    FILE_LOG(logDEBUG) << "Recipes are:" << endl;
    for (std::map<string, Recipe>::iterator a_recipe = this->recipes.begin(); \
         a_recipe != this->recipes.end(); a_recipe++) {
        a_recipe->second.print();
    }
}

void InteractionManager::printTriggerables() {
    cout << "Triggerables are:" << endl;
    for (std::list<TriggerableRecord>::iterator a_trig = this->triggerables_rec.begin(); \
         a_trig != this->triggerables_rec.end(); a_trig++) {
        FILE_LOG(logDEBUG) << "name: " << a_trig->name << ", ";
        FILE_LOG(logDEBUG) << "max_instances: "<< a_trig->max_instances << endl;
    }
}

 /* set IM member vars */
void InteractionManager::setGlobals() {
    
        /* push globals atom from atom defaults to root bindings */
        /* get the conjunction (TODO: use only conjunction instead of the formula) */
    if (this->default_atoms.findAtom("type", "im", "subtype", "globals") == this->default_atoms.atoms.end()) {
        FILE_LOG(logERROR) << "IM error: Did not find the globals atom default";
        return;
    }
    Atom new_atom = *this->default_atoms.findAtom("type", "im", "subtype", "globals");
    this->pushAtomBinding(new_atom);
    FILE_LOG(logDEBUG) << "just pushed new IM globals atom to the root bindings.";
}

Conjunction* InteractionManager::getGlobalBindings() {  
    tree<Node>::iterator root = this->ptree.tr.begin();
    return &(root->bindings);
}

/***********************************************************
 * action methods that need to be defined above of actions
 * since actions don't know about conjunctions, for example
 * ******************************************************/

/* convert args to an atom
 * Used in datablock text value evaluations, since expression parser needs a conjunction of
 * bindings for reference resolution. This atom can be used to form such a conjunction */
void InteractionManager::makeAtomFromArgs(Atom &an_atom, Args &an_args) {
    for (std::list<ArgSlot>::iterator arg_it = an_args.begin(); arg_it != an_args.end(); 
         arg_it++) {
        VarSlot aVarSlot;
        aVarSlot.var = arg_it->name;
        aVarSlot.val = arg_it->value;
        an_atom.varslots.push_back(aVarSlot);
        FILE_LOG(logDEBUG4) << "arg's name: " << arg_it->name << ", arg's value: " << \
            arg_it->value;
    }
}

/* replace references to arguments in datablock text values */
void InteractionManager::evalActionDatablocks(Action &an_action) {

/* make a conjunction of bindings out of arguments
 * because that's the format the exparser accepts */
    Conjunction bindingsFromArgs;
    Atom atomFromArgs;
    this->makeAtomFromArgs(atomFromArgs, an_action.args);
    bindingsFromArgs.atoms.push_back(atomFromArgs);

        /* get global bindings */
        //tree<Node>::iterator root = this->ptree.tr.begin();
        //Conjunction& gBindings = root->bindings;
        /* just a stub for global bindings since action content are
         * filled only based on the bindings from args. Args are supposed
         * to be dereferenced based on local recipe and global im bindings earlier*/
    Conjunction gBindings; 
    
    ExpressionParser exparser(bindingsFromArgs);
    string unit="_NO_VALUE_";
    
        /* call exparser on for every text value of every datablock's element */
        /* iterate over datablocks of the action */
    for(std::list<Datablock>::iterator dblock_it = an_action.datablocks.begin();
        dblock_it != an_action.datablocks.end(); dblock_it++) {
            /* iterate over datablock's entries */
        for(std::map<string, string>::iterator dbentry_it = dblock_it->begin();
            dbentry_it != dblock_it->end(); dbentry_it++) {
            FILE_LOG(logDEBUG3) << "Reading datablock entry: " << dbentry_it->first << \
                " : " << dbentry_it->second;
            dbentry_it->second = exparser.eval("string", dbentry_it->second, unit);
            FILE_LOG(logDEBUG3) << "Updated datablock entry: " << dbentry_it->first << \
                " : " << dbentry_it->second;
        }
    }
}



/* binds action arg according to args spec in the body of the recipe and
 * current node's bindings */
void InteractionManager::bindActionArg(std::list<ArgSlot>::iterator &arg_it, Args &args,
                                       Conjunction &bindings) {
        /* find if the action argument is passed in body element args */
    std::list<ArgSlot>::iterator arg_it2 = args.begin();
    while ( arg_it2 != args.end() && (arg_it2->name != arg_it->name) ) {
        arg_it2++;
    }

        /* get global bindings */
        //Conjunction* gBindings = this->getGlobalBindings();
    ExpressionParser exparser(bindings);
    string unit="_NO_VALUE_";

    if ( arg_it2 != args.end() ) {
            /* found arg passed */
            /*TODO: parse value string as a in-string expression. For now consider it a value.*/
        arg_it->value = exparser.eval(arg_it->type, arg_it2->value, unit);
        arg_it->unit = unit;
    }
    else {
            /* arg is not passed, use default (can be in-string expression too) */
        FILE_LOG(logDEBUG3) << "Arg: " << arg_it->name <<
            "not passed in the recipe, trying to use default: " << arg_it->default_value;
        arg_it->value = exparser.eval(arg_it->type, arg_it->default_value, unit);
        arg_it->unit = unit;
    }

}

/* binds action args according to args spec in the body of the recipe and
 * current node's bindings */
void InteractionManager::bindActionArgs(Action &an_action, Args &args, Conjunction &bindings) {
    #ifdef DEBUG
    FILE_LOG(logDEBUG3) << "Binding action args. Args passed: ";
    args.print(logDEBUG3);
    #endif
    std::list<ArgSlot>::iterator arg_it = an_action.args.begin();
    while (arg_it!=an_action.args.end()) {
        this->bindActionArg(arg_it, args, bindings);
        arg_it++;
    }
}


/***********************************************************
 * utility methods that need to be defined as IM methods
 * because they may need to access to the IM global variables
 * ******************************************************/



/***********************************************
 * IM: methods for plan tree transformations
 * *********************************************/

/**
 * Attempt to push triggerables on the stack. Should be done after each
 * update to the state of the IM
 * */
bool InteractionManager::tryPushTriggerables() {
    bool res = true;
    for (std::list<TriggerableRecord>::iterator a_trig = this->triggerables_rec.begin(); \
         a_trig != this->triggerables_rec.end(); a_trig++) {
        this->tryPushTriggerable(*a_trig);
    }
    return res;
}

/**
 * Try push a triggerable recipe to the stack
 **/
bool InteractionManager::tryPushTriggerable(TriggerableRecord &a_trig) {
    bool res = true;

    FILE_LOG(logDEBUG4) << "Trying to push triggerable: " << a_trig.name;
	/** get top of the tree **/
    tree<Node>::iterator root = this->ptree.tr.begin();

    Conjunction new_bindings;

	/** If preconditions of the triggerable script are satisfied,
	 * push it to the top **/
    if (((a_trig.max_instances=="any")||
         (this->ptree.nodeChildNameCount(root, a_trig.name) <           \
          string_to_number((const std::string)a_trig.max_instances)))&&
        /* this line needs executed to assign the new_bindings */
        (this->checkPreconditionGivenRecipe(a_trig.name, root, new_bindings))) {

            /**
             * make a new tree node, assign the new bindings to it
             * **/

        map<string, Recipe>::iterator recipe_it = this->recipes.find(a_trig.name);
        if (recipe_it == this->recipes.end()) {
            FILE_LOG(logERROR) << "Error in IM: recipe " << a_trig.name << \
                " not found in IM recipe map";
            return false;
        }

        Node new_node(recipe_it->second);
        new_node.bindings = new_bindings;
        new_node.recipe_name = a_trig.name;
        
            /** bind whilecondition field of the new node
             * based on the precondition bindings (while_bindings are
             * supposed to be a
             * subset of precondition bindings).
             * */
        if (!this->bindWhilecondition(new_node)) {
            FILE_LOG(logERROR) <<
                "When pushing triggerables, failed to bind whilecondition for node: " <<
                a_trig.name;
            return false;
        }
        
            /** push node onto the top of the tree **/
        FILE_LOG(logDEBUG3) << "about to push a  new node on the top";
        if (root==NULL) {
            FILE_LOG(logERROR) << "root is nil.";
        }
        tree<Node>::iterator new_node_it = this->ptree.tr.append_child(root, new_node);
            /** execute the first body element **/
            /* uncurse edit. We want to execute the initial element right away in case
             * it's an assignment batch (that may block other triggerables from starting). */
        this->executeNodeAssignmentBlock(new_node_it);
    }
    return res;
}

/* pass formula and copy it for cases when we may need to instatiate vals of some of
 * "this" slots */
bool InteractionManager::checkPreconditionGivenFormula(Formula aprecond, \
                                                       tree<Node>::iterator &parent_node, \
                                                       Conjunction &new_bindings) {
    
    this->updateGlobalBindings();
    Conjunction *gBindings = this->getGlobalBindings();

        /* TODO: to allow cross referencing within same precondition,
         * For a given matching candidate between precond atoms and global atoms,
         * do first pass just to do the bindings to vars where val is not specified
         * NOTE that we disallow non-pure bindings, e.g. when both val and var are non empty
         * in preconditions, at least */

        /* binding pure bindings and evluation of slot vals should be done
         * as part of matching process -- for all possible match combinations */

    return aprecond.unify(*gBindings, new_bindings);
}

/** check if the recipe preconditions are satisfied,
 * perform binding of the recipe vars to the parent vars
 **/
bool InteractionManager::checkPreconditionGivenRecipe(string &recipe_name, \
                                                      tree<Node>::iterator &parent_node, \
                                                      Conjunction &new_bindings) {

	/** do the unification between recipe's preconditions and the
	 * parent's (root for now) bindings */

	/* find recipe */
    map<string, Recipe>::iterator recipe_it = this->recipes.find(recipe_name);
    if (recipe_it == this->recipes.end()) {
        FILE_LOG(logERROR) << "Error in IM: recipe not found in IM recipe map: " <<\
            recipe_name;
        return false;
    }
    FILE_LOG(logDEBUG4) << "Checking precondition of recipe: " << recipe_name;
	/** now do the unification **/
        /* TOCHECK: that evaluating preconds right in the recipe does not
         * leave a long lasting damage. TODO: see if we can avoid copying here
         * and use a reference to precondition instead. Most probably need a copy
         * since "this" val is copied from poscond to precond. */

    if (!checkPreconditionGivenFormula((recipe_it->second).precondition,
                                       parent_node, new_bindings)) {
            /** failed to unify */
        FILE_LOG(logDEBUG4) << "Preconditions failed for recipe: " << recipe_name;
        return false;
    }
    else {
        FILE_LOG(logDEBUG4) << "Preconditions succeeded for recipe: " << recipe_name;
        return true;
    }
}

/**
 *  Setting while condition bindings consists of instantiating "this" slot value
 *  according to var shared by preconditions (and hence node bindings) and
 *  whileconditions.
 *  Note that this means that whilecondition atoms are a subset of preconditions
 *  atoms (but atom slots do not have to be a subset).
 **/
bool InteractionManager::bindWhilecondition(Node &node) {

    
        /* basically all wee need is to copy whileconditions and set values of
         * "this" slots for whilecondition atoms */
         
	/* find recipe */
    map<string, Recipe>::iterator recipe_it = this->recipes.find(node.recipe_name);
    if (recipe_it == this->recipes.end()) {
        FILE_LOG(logERROR) << "Error in IM: recipe " << node.recipe_name << \
            " not found in IM recipe map";
        return false;
    }

        /* copy whilecondition to the node's whilecondition */
    
    node.whilecondition = (recipe_it->second).whilecondition;

        /* bind "this" slots of whilecondition */
    
    node.whilecondition.bindThis(node.bindings, node.recipe_name);


    
	/** TOBE REMOVED. now do the unification **/
        //if (!(recipe_it->second).whilecondition.unify(node.bindings, new_bindings)) {
        //  /** failed to unify */
        //return false;
        // }

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) << "Setting the following bound node whileconditions:";
    node.whilecondition.print(logDEBUG4);
    #endif
    return true; 
}


/*
 * See if Node has any atoms that are marked toBeDeleted==Now.
 * */
void InteractionManager::garbageCollectDeletedAtoms() {
    tree<Node>::iterator root = this->ptree.tr.begin();
    std::vector< Atom >::iterator atom_it=root->bindings.atoms.begin();
    while (atom_it != root->bindings.atoms.end()) {

            /* delete atoms only when they are not involved in any nodes, even
             * if toBeDelete=Now. In Now case, the involved nodes should have beel deleted
             * in CheckWhileconditions */
        if (((atom_it->toBeDeleted == Now)||(atom_it->toBeDeleted == WhenNotUsed))&&
            (atom_it->node_ids.empty())) {
            FILE_LOG(logDEBUG4) << "About to delete atom with this: " <<
                atom_it->readSlotVal("this");
            atom_it = root->bindings.atoms.erase(atom_it);
        } else {
                /* reset node_ids so that they are refreshed in the next tick.*/
            atom_it->node_ids.clear();
            atom_it++;
        }
    }
}


/** check if the recipe whileconditions are satisfied. node's whileconditions
 * are already bound by 'this' slot values to the respective global atoms.
 * Hence the check is just a completely "Locked" unification that skips the
 * binding pass, because whileconditions do not allow for introduction of
 * any new $-vars, for simplicity, for now.
 **/
bool InteractionManager::checkWhilecondition(tree<Node>::post_order_iterator &node, \
                                             tree<Node>::iterator &parent_node) {
    
	/* do the unification between recipe's whileconditions and the
	 * parent's bindings */
    if (!node->whilecondition.unifyWithoutBinding(parent_node->bindings, node->bindings)) {
        FILE_LOG(logDEBUG4) << "Failed whilecondition for node with recipe: "
                            << node->recipe_name;
        return false;
    } else {
        FILE_LOG(logDEBUG4) << "Succeeded whilecondition for node with recipe: "
                            << node->recipe_name;

        return true;
    }

}

/*
 * See if Node has any atoms that are marked toBeDeleted==Now. Returns true if there
 * are ghost atoms like this.
 * */
bool InteractionManager::checkForGhostAtoms(Node &node) {
    
    for(std::vector< Atom >::iterator atom_it=node.bindings.atoms.begin();             
        atom_it != node.bindings.atoms.end(); atom_it++) {

        if (atom_it->toBeDeleted == Now) {
            return true;
        }
    }
    return false;
}

/**
 * check for whileconditions and for presense of ghost atoms, i.e. atoms that
 * have toBeDeleted var set to Now.
 * Prune nodes with whileconds not satisfied and those with ghost atoms.
 * Edit 10 Oct 2011: added updateBinding for each node checked.
 * even though post-order travelsal guarantees children are checked before
 * parents, it is possible that parent needs to be pruned but a child's whilecondition
 * is true. in that case whole branch still needs to be pruned.
 * */
void InteractionManager::checkPruneWhileConditions() {
	/** iterate using post-order, children first */
    FILE_LOG(logDEBUG4)  << "Cheking for failing whilecondition nodes..." ;
    tree<Node>::post_order_iterator sib1=this->ptree.tr.begin_post();
    tree<Node>::iterator root = this->ptree.tr.begin();
    Conjunction *gBindings = this->getGlobalBindings();
    while(sib1 != this->ptree.tr.end_post()) {

            /* skip ROOT node. Things will go wrong if ROOT is included,
             * in particular when counting globla atom's node_ids*/
        if (sib1 == this->ptree.tr.begin()) {
            ++sib1;
            continue;
        }
            /* update local bindings, in case whilecondition has $-vars */

        #ifdef DEBUG
        FILE_LOG(logDEBUG4)  << "Node bindings for node with id: " << sib1->node_id <<
            ", recipe name: " << sib1->recipe_name;
        sib1->bindings.print(logDEBUG4);
        #endif
        
        sib1->bindings.updateBinding(*gBindings, sib1->node_id);
        
        if (!this->checkWhilecondition(sib1, root)) {
      	 	/* the returned iterator should be "safe" according to tree
                 * node erase method --
                 * it skips children in case there are any to be erased  */
            FILE_LOG(logDEBUG3)  << "Marking a node with failed whilecondition for removal: " \
                                 <<  sib1->recipe_name ;
                /* this is safe because erase method returns the next safe node for
                 * any kind of iterator (taking into account that children are erased too) */
            sib1->whileconditionFailed = true;
                //sib1 = this->ptree.pruneNode(sib1);
            ++sib1;
        } else if (this->checkForGhostAtoms(*sib1)) {
      	 	/* the returned iterator should be "safe" according to tree
                 * node erase method --
                 * it skips children in case there are any to be erased  */
            FILE_LOG(logDEBUG3)  << "Removing the node because it has ghost atoms. Node's recipe: " \
                                  <<  sib1->recipe_name ;
                 /* this is safe because erase method returns the next safe node for
                  * any kind of iterator (taking into account that children are erased too) */
            sib1->whileconditionFailed = true;
                //sib1 = this->ptree.pruneNode(sib1);
            ++sib1;
        } else {
            ++sib1;
        }
    }
}


void InteractionManager::printSettings() {
    FILE_LOG(logINFO)  << "IM is running at tick period: " << setprecision(20) <<
        this->timer_period_microsec/1000000.0 << " seconds,";
    FILE_LOG(logINFO)  << "   with init file: " << this->init_file_name;
    FILE_LOG(logINFO)  << "   with total of " << this->recipes.size() << " recipes,";
    FILE_LOG(logINFO)  << "   of which " << this->triggerables_rec.size() << " are triggerable.";
}

/*
 * Parse the list of user-specified backchainables and verify that such recipes
 * really exist.
 * */

std::list<string> parseBackchanablesFromUser(string recipe_names) {
    std::list<string> recipeList = splitIntoListRemoveWhitespace(recipe_names, ',');
        /* check that all the recipes in the list exist */
    FILE_LOG(logDEBUG4)  << "User-defined backchainables are: ";
    for (std::list<string>::const_iterator rname_it = recipeList.begin();
             rname_it != recipeList.end(); rname_it++) {
        FILE_LOG(logDEBUG4)  << *rname_it << ".";
    }
    return recipeList;
}

/* Given the goal, the list of candidates compute the backchanable candidates based
 * on the match between the goal atom and postconditions.
 * goalRecipeName is passed for logging only. */
void InteractionManager::findBackchanablesForAGoal(BodyElement &element, std::list<string> &candidateRecipes, string goalRecipeName) {

        /* things needed to be passed to unifyWithoutBinding */
    std::vector< Match > mapping;
    int matched_con_id;
    Conjunction empty_binding;
    
    for (std::list<string>::iterator
             recipeName_it = candidateRecipes.begin();
         recipeName_it != candidateRecipes.end(); recipeName_it++) {

        Recipe &candidateRecipe = this->recipes[*recipeName_it];
        if (candidateRecipe.assignpost.disjuncts.size() == 0) {
                /* empty postcondition, move on */
            continue;
        }
                    
        vector<Conjunction>::iterator conj_it =
            candidateRecipe.assignpost.disjuncts.begin();
                    
        if (element.formula.unifyWithoutBinding((*conj_it),
                                                    empty_binding,
                                                    mapping,
                                                    matched_con_id,
                                                    Complete)) {
                /* insert the recipe into backchainables list
                 * in the order of decreasing priority.
                 * NOTE: insert at the end of the priority group
                 * because, guess what, some recipes already
                 * rely on the order in the init file. Need those
                 * recipes rewritten! */
            std::list<string>::iterator bchain_it = element.backchainables.begin();
            while ((bchain_it !=  element.backchainables.end()) &&
                   (this->recipes.find(*bchain_it)->second.priority >=
                    candidateRecipe.priority)) {
                bchain_it++;
            }
            element.backchainables.insert(bchain_it,
                                                candidateRecipe.name);
                /* inserted just before the recipe of a lower priority
                 * (or in the end) */
            FILE_LOG(logDEBUG2) << "Postconditions of recipe "
                                << candidateRecipe.name << 
                " may potentially satisfy goal " <<
                element.name <<
                " of recipe " << goalRecipeName;
        }
    }

}

/* Find potential backchainable recipes for each goal, based on the match
 * between goal and postcondition. Only the list of recipes and their content
 * should be necessary for this operation (nothing else from the IM object) */
void InteractionManager::preprocessBackchainables() {

        /* iterate through recipes looking for goals */
    for (std::map<string, Recipe>::iterator recipe_it = this->recipes.begin();
         recipe_it != this->recipes.end(); recipe_it++) {
   
                                                /* iterate through body elements*/
        for (std::vector<BodyElement>::iterator
                 element_it = recipe_it->second.body.elements.begin();
             element_it != recipe_it->second.body.elements.end(); element_it++) {

            
            if (element_it->element_type == "goal") {
                if ((element_it->recipe_name == "any") ||
                    (element_it->recipe_name.empty())) {
                    
                    if (element_it->formula.disjuncts.size() == 0) {
                        FILE_LOG(logERROR) << "Recipe " << recipe_it->first
                                           << " has a goal with recipe_names set as 'any' "
                                           << "and the empty goal formula."
                                           << "This does not make much sense.";
                    }
                        /* if there are no restrictions on backchanable recipes
                         * iterate through all the recipes trying to find potentially
                         * matching postconditions */
                    findBackchanablesForAGoal(*element_it, this->recipe_names,
                                              recipe_it->first); 
                    
                } else {
                        /* pass the list of the candidate recipes from the
                         * goal's recipe_name attribute */
     
                    std::list<string> backchainablesFromUser =
                        parseBackchanablesFromUser(element_it->recipe_name);
                    
                    if (element_it->formula.disjuncts.size() == 0) {
                            /* if goal is empty, just copy user-defined backchainables to
                             * computed backchainables */
                        element_it->backchainables = backchainablesFromUser;  
                    } else {
                        findBackchanablesForAGoal(*element_it, backchainablesFromUser,
                                                  recipe_it->first);
                    }
                }

                    /* print the backchinables for this goal to the log file */
                FILE_LOG(logINFO) << "Backchainables for the recipe " <<
                    recipe_it->second.name << " are: ";
                for (std::list<string>::iterator recipe_name_it =
                         element_it -> backchainables.begin();
                     recipe_name_it != element_it -> backchainables.end();
                     recipe_name_it++) {
                    FILE_LOG(logINFO) << *recipe_name_it << ", ";
                }
            }
            
        }
    }
}


/*
 * Do such steps as:
 * - validation of precondition atoms against default atoms
 * - build lists of possible backchainable recipes for each goal
 * - build lists of recipes to possibly correspond for each slot of default atoms
 * */
bool InteractionManager::preprocess() {
        /* build list of possible backchainable recipes for each goal */
    this->preprocessBackchainables();
    
    return true;
}

/**
 * Initialize everything in IM:
 * - load init file
 * - set globals
 * - set up the plan tree
 * - set up the stack
 * */

bool InteractionManager::initialize() {
    bool res = true;

    srand((unsigned)time(NULL));    /* initialize random gen */
    
        /* if present command line init file name overrides the default */
    if (this->init_file_name.empty()) {
        this->init_file_name = IM_INIT_FILENAME;
    }


        /* check script path directory */
    if (this->script_path.empty()) {
        cerr << "IM error: script path is not set." << endl;
        exit(1);        
    }
    
    if (!this->load_init(this->init_file_name)) {
        cout << "IM error: Initialization of IM from the file failed." << endl;
        FILE_LOG(logERROR) << "Initialization of IM from the file failed.";
    	return false;
    }

            /*
             * Initialize history
             * */
    for (std::map<string, Recipe>::iterator recipe_it = this->recipes.begin(); \
         recipe_it != this->recipes.end(); recipe_it++) {

        RecipeHistory recipeHistory;
   
                                                /* iterate through body elements*/
        for (std::vector<BodyElement>::iterator element_it =
                 recipe_it->second.body.elements.begin();              \
             element_it != recipe_it->second.body.elements.end(); element_it++) {
            BodyElementHistory elementHistory;

            if (element_it->random) {
                                                              /* initialize vector */
                for (unsigned int i = 0; i < element_it->element_probs.size(); i++) {
                    elementHistory.outcome_age.push_back(MAX_ELEMENT_AGE);
                }
            }
            
            recipeHistory.bodyElementHistories.push_back(elementHistory);    
        }
        
        this->history.recipeHistories[recipe_it->second.name] = recipeHistory;
    }
    
        /* bind type and subtype of goals, assignposts according to preconds */
    for (std::map<string, Recipe>::iterator recipe_it = this->recipes.begin(); \
         recipe_it != this->recipes.end(); recipe_it++) {
        if (!recipe_it->second.bindTypeAndSubtype()) {
             FILE_LOG(logERROR)  <<
                 "Filling missing type/subtype slots failed for the recipe" <<
                 recipe_it->second.name;
        }
    }

        /* if present command line defaults file name overrides the default */
    if (this->default_atoms_file_name.empty()) {
        this->default_atoms_file_name = DEFAULT_ATOMS_FILENAME;
    }
    
        /** load the default atom slots **/
    if (!this->load_DefaultAtoms(this->default_atoms_file_name)) {
        FILE_LOG(logERROR) << "Loading default atoms from the file failed.";
    	return false;
    }
    
        /** preprocess **/
    if (!this->preprocess()) {
        FILE_LOG(logERROR) << "Preprocessing failed.";
        return false;
    }
    
        /** initalize plan tree**/
        /** create root node: it will hold IM-level variables as its bindings **/
    Node node;

    node.recipe_name = "ROOT";
    tree<Node>::iterator root;
    root = this->ptree.tr.begin();
    this->ptree.tr.insert(root, node);

        /** set globals. must to after plan tree is initalized since the globals are stored at
         * root node bindings **/
    this->setGlobals();

    initExpressionParsing();
    
        /** try push triggerables on the plan tree **/
        /* uncurse edit */
        //if (!this->tryPushTriggerables()) {
        //cout << "Pushing triggerables failed, for some reason.";
        //return false;
        //}

	/** print plan tree **/
    #ifdef DEBUG
    FILE_LOG(logINFO) << "Compiled with debug mode ON.";
    this->ptree.print(logDEBUG1);
    #endif

    this->printSettings();

    return res;
}

/******************************
 * Atom bindings transformations
 * *******************************/

/**
 * In the IM bindings replaces the the atom that matches type and subtype
 * with the new atom.
 * Usage: in a single-user environment, presence of the new user will
 * trigger replacement of the old user record.
 * */
void InteractionManager::replaceAtomBinding(Atom &new_atom)
{
    tree<Node>::iterator root = this->ptree.tr.begin();

    #ifdef DEBUG
    FILE_LOG(logDEBUG4)  <<  "Root bindings before replace:";
    root->bindings.print(logDEBUG4);
    #endif

    vector<Atom>::iterator matching_atom = \
        root->bindings.findAtom("type", new_atom.readSlotVal("type"), \
                                "subtype", new_atom.readSlotVal("subtype"));
    if (matching_atom != root->bindings.atoms.end()) {
            /** matching atom already is in bindings */
        root->bindings.atoms.erase(matching_atom);
    }
    root->bindings.atoms.push_back(new_atom);

    #ifdef DEBUG
    FILE_LOG(logDEBUG4)  <<  "Root bindings after replace:";
    root->bindings.print(logDEBUG4);
    #endif
    
	/* TODO: propagate changes down the tree */
	/** Propagation changes, part 1: check whileconditions */
        /* uncurse edit */
        //this->checkPruneWhileConditions();

}

void InteractionManager::pushAtomBinding(Atom &new_atom)
{
    tree<Node>::iterator root = this->ptree.tr.begin();

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings before push:";
    root->bindings.print(logDEBUG4);
    #endif

    if (new_atom.checkSyntax()) {
        if ((new_atom.readSlotVal("this")=="_NOT_FOUND_")||
            (new_atom.readSlotVal("this")=="_NO_VALUE_")) {
                /* fix missing "this" slot by generating an id */
            string new_this = "this_" + \
                new_atom.readSlotVal("subtype")+to_string(this->atom_counter);
            FILE_LOG(logDEBUG4) <<
                "Missing 'this' slot of atom, generating new value: " <<
                new_this;
            new_atom.setSlotVal("this", new_this);
            this->atom_counter++;
        }
        root->bindings.atoms.push_back(new_atom);

        #ifdef DEBUG
        FILE_LOG(logDEBUG4) <<  "Root bindings after push:";
        root->bindings.print(logDEBUG4);
        #endif
    } else {
        FILE_LOG(logERROR) <<  "Missing type and/or subtype slots of atom.";
    }


        /* TODO: propagate changes down the tree */
	/** Propagation changes, part 1: check whileconditions */
        /* uncurse edit */
        //this->checkPruneWhileConditions();

}

/**
 * Updates the global bindings stored in the root node. This function
 * does this non-destructively for the original globals atom: that is,
 * if some new varslots were created before, they should be kept intact.
 * */
void InteractionManager::updateGlobalBindings()
{
    FILE_LOG(logDEBUG4) << "Updating global bindings...";
        /* Create a new global atom */
    Atom new_atom;
    VarSlot v1,v2,v3,v4,v5,v6,v7,v8,v9,v10;

    v1.name = ("type");
    v1.val = ("im");

    v2.name = ("subtype");
    v2.val = ("globals");

    v3.name = ("time");
    v3.val = to_string(getSystemTimeMSec()/1000.0); /* store in seconds */
    v3.var = "_im_time"; /* cos lookup is done via varname.*/
    v3.type = "number";


    v4.name = ("year");
    v4.val = to_string(getCurrentYear()); /* year since 0 */
    v4.type = "number";

    v5.name = ("month");
    v5.val = to_string(getCurrentMonth()); /* month since January */
    v5.type = "number";

    v6.name = ("month_day");
    v6.val = to_string(getCurrentMonthdayInt()); /* days since 1st */
    v6.type = "number";

    v7.name = ("week_day");
    v7.val = to_string(getCurrentWeekdayInt()); /* days since Monday */
    v7.type = "number";

    v8.name = ("hour");
    v8.val = to_string(getCurrentHour()); /* hours since midnight */
    v8.type = "number";

    v9.name = ("minute");
    v9.val = to_string(getCurrentMinute()); /* minute 0-59 */
    v9.type = "number";
    
    v10.name = ("second");
    v10.val = to_string(getCurrentSecond()); /* second 0-59 */
    v10.type = "number";
    
    new_atom.varslots.push_back(v1);
    new_atom.varslots.push_back(v2);
    new_atom.varslots.push_back(v3);
    new_atom.varslots.push_back(v4);
    new_atom.varslots.push_back(v5);
    new_atom.varslots.push_back(v6);
    new_atom.varslots.push_back(v7);
    new_atom.varslots.push_back(v8);
    new_atom.varslots.push_back(v9);
    new_atom.varslots.push_back(v10);
    
    if  (!this->updateRootBindingAtomMatchingType(new_atom)) {
        this->pushAtomBinding(new_atom);
    }
    FILE_LOG(logDEBUG4) << "Finished updating global bindings.";
}



/**
 * Updates the bindings for the atom that matches 'this' slot in the
 * ptree root
 * */
bool InteractionManager::updateAtomBinding(Atom &new_atom)
{
    tree<Node>::iterator root = this->ptree.tr.begin();

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings before UpdateAtomBinding:";
    root->bindings.print(logDEBUG4);
    #endif
    
    vector<Atom>::iterator matching_atom = \
        root->bindings.findAtom("this", new_atom.readSlotVal("this"));
    if (matching_atom == root->bindings.atoms.end()) {
            /** no matching atom in the bindings */
        FILE_LOG(logDEBUG4)  << "IM Error: matching atom not found in the bindings";
        return false;
    }

    matching_atom->updateAtom(new_atom);

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings after UpdateAtomBinding:";
    root->bindings.print(logDEBUG4);
    #endif
    
	/* TODO: propagate changes down the tree */
	/** Propagation changes, part 1: check whileconditions */
        /* uncurse edit */
        // this->checkPruneWhileConditions();
    return true;
}

/**
 * Updates the bindings for the atom that matches type and subtype.
 * If no matching atom is found, does nothing.
 * */
bool InteractionManager::updateRootBindingAtomMatchingType(Atom &new_atom, bool create)
{
    tree<Node>::iterator root = this->ptree.tr.begin();

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings before updateRootBindingAtomMatchType:";
    root->bindings.print(logDEBUG4);
    #endif
    
    vector<Atom>::iterator matching_atom = \
        root->bindings.findAtom("type", new_atom.readSlotVal("type"),       \
                                "subtype", new_atom.readSlotVal("subtype"));
    if (matching_atom == root->bindings.atoms.end()) {
            /* no matching atom in the bindings */
        if (create) {
            FILE_LOG(logDEBUG3) << \
                "No matching atom found in the bindings, creating new atom";
                /* first, get a default atom of the require type and subtype,
                 * modify it's id to the new_atom's,
                 * push it on bindings, and then
                 * update bindings with the new_atom */
            vector<Atom>::iterator default_atom_it =                      \
                this->default_atoms.findAtom("type", new_atom.readSlotVal("type"), \
                                             "subtype", new_atom.readSlotVal("subtype"));
            if (default_atom_it ==  this->default_atoms.atoms.end()) {
                FILE_LOG(logERROR) <<                                   \
                    "No matching default atom found for type: " <<  new_atom.readSlotVal("type") \
                                   << " subtype: " << new_atom.readSlotVal("subtype");
                return false;
            }
                /* found default atom of the right type/subtype, modify id */
            Atom default_atom = *default_atom_it;
            this->pushAtomBinding(default_atom);
            this->updateRootBindingAtomMatchingTypeId(new_atom);
        } else {
            FILE_LOG(logDEBUG3) << "No matching atom found in the bindings, ignoring";
            return false;
        }
    } else {
        matching_atom->updateAtom(new_atom);
    }
    
    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings after updateRootBindingAtomMatchType:" ;
    root->bindings.print(logDEBUG4);
    #endif
    
	/* TODO: propagate changes down the tree */
	/** Propagation changes, part 1: check whileconditions */
        /* uncurse edit */
        //this->checkPruneWhileConditions();
    return true;
}



/**
 * Updates the bindings for the atom that matches type, subtype and id.
 * If no matching atom is found, creates a new atom.
 * */
bool InteractionManager::updateRootBindingAtomMatchingTypeId(Atom &new_atom)
{
    tree<Node>::iterator root = this->ptree.tr.begin();

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings before updateRootBindingAtomMatchTypeId:";
    root->bindings.print(logDEBUG4);
    #endif
    
    vector<Atom>::iterator matching_atom = \
        root->bindings.findAtom("type", new_atom.readSlotVal("type"),       \
                                "subtype", new_atom.readSlotVal("subtype"), \
                                "id",  new_atom.readSlotVal("id"));
    if (matching_atom == root->bindings.atoms.end()) {
            /* no matching atom in the bindings -- create a new atom */
        FILE_LOG(logDEBUG3) << \
            "No matching atom found in the bindings, creating new atom";
            /* first, get a default atom of the require type and subtype,
             * modify it's id to the new_atom's,
             * push it on bindings, and then
             * update bindings with the new_atom */
        vector<Atom>::iterator default_atom_it =                          \
            this->default_atoms.findAtom("type", new_atom.readSlotVal("type"), \
                                         "subtype", new_atom.readSlotVal("subtype"));
        if (default_atom_it ==  this->default_atoms.atoms.end()) {
            FILE_LOG(logERROR) <<                                       \
                "No matching default atom found for type: " <<  new_atom.readSlotVal("type") \
                               << " subtype: " << new_atom.readSlotVal("subtype");
            return false;
        }
            /* found default atom of the right type/subtype, modify id */
        Atom default_atom = *default_atom_it;
        default_atom.setSlotVal("id", new_atom.readSlotVal("id"));
        this->pushAtomBinding(default_atom);
        this->updateRootBindingAtomMatchingTypeId(new_atom);
    } else {
        matching_atom->updateAtom(new_atom);
    }
    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings after updateRootBindingAtomMatchTypeId:";
    root->bindings.print(logDEBUG4);
    #endif

	/* TODO: propagate changes down the tree */
	/** Propagation changes, part 1: check whileconditions */
        /* uncurse edit */
        //this->checkPruneWhileConditions();
    return true;
}

/**
 * Updates the bindings for the atom that matches any of specified slots,
 * just like findAtom method of Conjunction does.
 * If no matching atom is found, creates a new atom.
 * */
bool InteractionManager::updateRootBindingAtomMatchAnySlots(Atom &new_atom, string slot_name1)
{
    tree<Node>::iterator root = this->ptree.tr.begin();

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings before updateRootBindingAtomMatchAnySlots:";
    root->bindings.print(logDEBUG4);
    #endif
    
    vector<Atom>::iterator matching_atom = \
        root->bindings.findAtom("type", new_atom.readSlotVal("type"),       \
                                "subtype", new_atom.readSlotVal("subtype"), \
                                slot_name1, new_atom.readSlotVal(slot_name1));
    if (matching_atom == root->bindings.atoms.end()) {
            /* no matching atom in the bindings -- create a new atom */
        FILE_LOG(logDEBUG4) << \
            "No matching atom found in the bindings, creating new atom";
            /* first, get a default atom of the require type and subtype,
             * modify it's id to the new_atom's,
             * push it on bindings, and then
             * update bindings with the new_atom */
        vector<Atom>::iterator default_atom_it =                          \
            this->default_atoms.findAtom("type", new_atom.readSlotVal("type"), \
                                         "subtype", new_atom.readSlotVal("subtype"));
        if (default_atom_it ==  this->default_atoms.atoms.end()) {
            FILE_LOG(logERROR) <<                                       \
                "No matching default atom found for type: " <<          \
                new_atom.readSlotVal("type")                            \
                               << " subtype: " << new_atom.readSlotVal("subtype");
            return false;
        }
            /* found default atom of the right type/subtype, modify its slots */
        Atom default_atom = *default_atom_it;
        default_atom.setSlotVal(slot_name1, new_atom.readSlotVal(slot_name1));
        this->pushAtomBinding(default_atom);
            /* Call the same function again, this time the atom is for sure there,
             * so 2 calls is maximum in this recursion. Didn't use update_atom, since
             * we would have to find matching atom first -- similar overhead */
            /* TODO: use the way it's done is update...SlotList */
        this->updateRootBindingAtomMatchAnySlots(new_atom, slot_name1);
    } else {
        matching_atom->updateAtom(new_atom);
    }

    #ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings after updateRootBindingAtomMatchAnySlots:";
    root->bindings.print(logDEBUG4);
    #endif
    
	/* TODO: propagate changes down the tree */
	/** Propagation changes, part 1: check whileconditions */
        /* uncurse edit */
        //this->checkPruneWhileConditions();
    return true;
}



bool InteractionManager::updateRootBindingAtomMatchingSlotList(Atom &new_atom,
                                           std::list<string> &slotList) {

    Conjunction *gBindings_p = this->getGlobalBindings();

        /* find matching atom in global bindings */
    vector<Atom>::iterator matching_atom = gBindings_p->atoms.begin();
    
    while (matching_atom != gBindings_p->atoms.end()) {
            /* see if matching_atom matches the slotList */
        bool matched = true;
        for ( std::list<string>::iterator aSlot = slotList.begin();
              aSlot != slotList.end(); aSlot++) {
            if (matching_atom->readSlotVal(*aSlot) !=
                new_atom.readSlotVal(*aSlot)) {
                matched = false;
                break;
            }
        }

        if ( matched ) {break;}      
        matching_atom++;
    }

    if (matching_atom == gBindings_p->atoms.end()) {
            /* no matching atom in the bindings -- create a new atom */
        FILE_LOG(logDEBUG4) << \
            "No matching atom found in the bindings, creating new atom";
            /* first, get a default atom of the require type and subtype,
             * modify it's id to the new_atom's,
             * push it on bindings, and then
             * update bindings with the new_atom */
        vector<Atom>::iterator default_atom_it =                          \
            this->default_atoms.findAtom("type", new_atom.readSlotVal("type"), \
                                         "subtype", new_atom.readSlotVal("subtype"));
        if (default_atom_it ==  this->default_atoms.atoms.end()) {
            FILE_LOG(logERROR) <<                                       \
                "No matching default atom found for type: " <<          \
                new_atom.readSlotVal("type")                            \
                               << " subtype: " << new_atom.readSlotVal("subtype");
            return false;
        }
            /* found default atom of the right type/subtype, modify its slots */
        Atom default_atom = *default_atom_it;
        
        default_atom.updateAtom(new_atom);          /* it's a copy of the default atom
                                                     * so OK to pollute */
        this->pushAtomBinding(default_atom);
            /* Call the same function again, this time the atom is for sure there,
             * so 2 calls is maximum in this recursion. Didn't use update_atom, since
             * we would have to find matching atom first -- similar overhead */
            // shouldn't be necessary April 15, 2012
            //this->updateRootBindingAtomMatchAnySlots(new_atom, slot_name1);
    } else {
        matching_atom->updateAtom(new_atom);
    }

        //#ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings after updateRootBindingAtomMatchAnySlots:";
    gBindings_p->print(logDEBUG4);
        //#endif
    
    return true;
}

/*
 * A generic function for updating globals with an atom whose
 * matching mask is defined by slots with a field unique_match.
 * */
void InteractionManager::updateRootBindingAtomMatchingUniqueMask(Atom &new_atom) {

    Conjunction *gBindings_p = this->getGlobalBindings();

        /* find matching atom in global bindings */
    vector<Atom>::iterator matching_atom_it = gBindings_p->atoms.begin();
    
    while (matching_atom_it != gBindings_p->atoms.end()) {
            /* see if matching_atom matches the slotList */
        bool matched = true;
        for ( std::list<VarSlot>::iterator varslot_it = new_atom.varslots.begin();
              varslot_it != new_atom.varslots.end(); varslot_it++) {
            if ( !varslot_it->unique_mask ) { continue; }
            else if (( matching_atom_it->readSlotVal(varslot_it->name) == "_NOT_FOUND_")||
                     ( matching_atom_it->readSlotVal(varslot_it->name) != varslot_it->val )) {
                matched = false;
                break;
            }
        }
        if ( matched ) { break; }    
        matching_atom_it++;
    }

    if (matching_atom_it == gBindings_p->atoms.end()) {
            /* no matching atom in the bindings -- create a new atom */
        FILE_LOG(logDEBUG4) << \
            "No matching atom found in the bindings, creating new atom";
            /* first, get a default atom of the required type and subtype,
             * modify it's id to the new_atom's,
             * push it on bindings, and then
             * update bindings with the new_atom */
        vector<Atom>::iterator default_atom_it =
            this->default_atoms.findAtom("type", new_atom.readSlotVal("type"),
                                         "subtype", new_atom.readSlotVal("subtype"));
        if (default_atom_it ==  this->default_atoms.atoms.end()) {
            FILE_LOG(logERROR) << 
                "No matching default atom found for type: " <<
                new_atom.readSlotVal("type") <<
                " subtype: " << new_atom.readSlotVal("subtype");
            return; /* throw an exception perhaps */
        }
            /* found default atom of the right type/subtype, modify its slots */
        Atom default_atom = *default_atom_it;
        
        default_atom.updateAtom(new_atom);          /* it's a copy of the default atom
                                                     * so OK to pollute */
        this->pushAtomBinding(default_atom);
    } else {
            /* matching atom found in global bindings, update it */
        matching_atom_it->updateAtom(new_atom);
    }

        //#ifdef DEBUG
    FILE_LOG(logDEBUG4) <<  "Root bindings after updateRootBindingAtomMatchAnySlots:";
    gBindings_p->print(logDEBUG4);
        //#endif
}

/*
 * Process the input atom queue. The atoms on the queue should have unique_mask
 * fields of their arg_slots properly set up.
 * */

void InteractionManager::updateRootBindingFromInputQueue() {
    std::list<Atom>::iterator atom_it = this->input_queue.begin();
    
    while( atom_it != this->input_queue.end() ) {
        this->updateRootBindingAtomMatchingUniqueMask(*atom_it);
            /* remove the atom from the queue after the update
             * NOTE that for loop gave a segfault: need to inc iterator
             * before the corresponding atom is poped. */
        atom_it++;
        this->input_queue.pop_front();
    }
}


/**********************************************
 * Plan tree execution
 * ********************************************/


/*
 * Special case of control flow move decision when node's whilecondition has failed
 * at some point.
 * */
ControlFlowMove InteractionManager::decideFailedWhileconditionNodeControlFlowMove
(tree<Node>::iterator_base &node) {
    
    ControlFlowMove resultMove = UndefinedMove;

        /* if whilecondition has failed for this node, then it's doomed to be purged. Here
         * we will decide exactly how this should be done. */
    if ((node->ae_status == "completed")||(node->ae_status == "aborted")||
        (node->ae_status == "pending")||(node->ae_status == "_NO_VALUE_")) {
            /* safe to purge the node, after doing what was prescribed in
             * the recipe's ifWhileconditionFailed */
        if (node->ifWhileconditionFailed == "skip_to_assignpost") {
            resultMove = SkipToAssignpost;
        } else if (node->ifWhileconditionFailed == "skip_to_end") {
            resultMove = SkipToEnd;
        } else {
            FILE_LOG(logERROR) << "Unknown value of ifWhileconditionFailed: "
                               << node->ifWhileconditionFailed << ", for node with recipe name: "
                               << node->recipe_name;
            resultMove = UndefinedMove;
        }
    } else if (node->ae_status == "executing") {
            /* this is more tricky. depending on if_node_purged variable of the active BodyElement
             * we either abort, or wait, or abort and wait for confirmation. If active element is a goal
             * then these three possibilities are applied recursively to children. */
        BodyElement &anActiveElement = node->body.elements[node->active_element];
            /* node->active_element is not -1 here
             * since ae_status is executing */
        if (anActiveElement.element_type=="action") {
            if (anActiveElement.if_node_purged=="wait") {
                    /* just regular wait till completion,
                     * and then we get here again */
                resultMove = DoNothing;
            } else if (anActiveElement.if_node_purged=="abort") {
                    /* do as prescribed in ifWhileconditionFailed,
                     * rather than in bodyElements if_aborted,
                     * for whilecondition failure is a special
                     * situation */
                    
                    /* but first send abort command to the executors */
                this->dispatchActionCommandAbort(node->ae_action_id, anActiveElement.actor);
                    /* now do what ifWhileconditionFailed prescribed */
                if (node->ifWhileconditionFailed == "skip_to_assignpost") {
                    resultMove = SkipToAssignpost;
                } else if (node->ifWhileconditionFailed == "skip_to_end") {
                    resultMove = SkipToEnd;
                } else {
                    FILE_LOG(logERROR) << "Unknown value of ifWhileconditionFailed: "
                                       << node->ifWhileconditionFailed << ", for node with recipe name: "
                                       << node->recipe_name;
                    resultMove = UndefinedMove;
                }
            } else if (anActiveElement.if_node_purged=="abort_and_wait") {
                    /* send abort and do nothing, as
                     * we'll get back here when the
                     * action is aborted */
                    /* TODO: mark that abort action has been sent to avoid multiple abort action commands */
                this->dispatchActionCommandAbort(node->ae_action_id, anActiveElement.actor);
                resultMove = DoNothing;
            } else {
                FILE_LOG(logERROR) << "Unknown value of if_node_purged in element: "
                                   << node->active_element << ", of node with recipe name: "
                                   << node->recipe_name;
                resultMove = UndefinedMove;
            }

        } else if (anActiveElement.element_type=="goal") {
                /* TODO: to be implemented. for now, do nothing.*/
            resultMove = DoNothing;
        }
    }
    return resultMove;
}

    /** decide which body element to execute
     * This is the control flow decision, based on the status previous action
     * and the status-conditional arguments of action
     * TODO: all combinations of ae_statuses, conditional args **/
ControlFlowMove InteractionManager::decideNodeControlFlowMove(tree<Node>::iterator_base &node) {

    ControlFlowMove resultMove = UndefinedMove;

        /*
         * if whilecondition has failed for this node, then it's doomed to be purged. Here
         * we will decide exactly how this should be done.
         * */
    if (node->whileconditionFailed) {
        return decideFailedWhileconditionNodeControlFlowMove(node);
    }

        /*
         * OK, whilecondition is true for this node
         * */
        /* check if it's a brand new node body */
    if (node->active_element != -1) {
            /* get the previous active element */
        BodyElement &anActiveElement =                  \
            node->body.elements[node->active_element];

            /* TODO: all combinations of ae_status and conditional control flow spec */
        if (node->ae_status == "completed") {
            if (anActiveElement.if_completed == "skip_to_next") {
                if (node->active_element < ((int)node->body.elements.size()) - 1 ) {
                        /** have not executed every element in the body yet */
                    resultMove = SkipToNext;
                } else {
                    resultMove = SkipToAssignpost;
                }
            }
        }

        if (node->ae_status == "aborted") {
            if (anActiveElement.if_aborted == "skip_to_next") {
                if (node->active_element < ((int)node->body.elements.size()) - 1 ) {
                        /** have not executed every element in the body yet */
                    resultMove = SkipToNext;
                } else {
                    resultMove = SkipToAssignpost;
                }
            } else if (anActiveElement.if_aborted == "skip_to_end") {
                resultMove = SkipToEnd;
            }
        }

        if ((node->ae_status == "executing") || (node->ae_status == "pending")) {
            
            if (anActiveElement.isTimedOut()) {
            
                FILE_LOG(logDEBUG3) << "Timed out element with status: "
                                    << node->ae_status;
                FILE_LOG(logDEBUG3) << "if_timeout: " << anActiveElement.if_timeout;
            
                if (anActiveElement.if_timeout == "skip_to_next") {
                    if (node->active_element < ((int)node->body.elements.size()) - 1 ) {
                                                      /* have not executed every
                                                       * element in the body yet */
                        resultMove = SkipToNext;
                    } else {
                        resultMove = SkipToAssignpost;
                    }
                } else if (anActiveElement.if_timeout == "skip_to_end") {
                    resultMove = SkipToEnd;
                }
            } else {                                   /* if not timed out */
                resultMove = DoNothing;                /* keep on waiting  */
            }
        }
    } else {                                    /* in case of the brand new node body */
        if (node->active_element < ((int)node->body.elements.size()) - 1 ) {
                                                /* have at least one body element
                                                 * in the body to execute */
            resultMove = SkipToNext;
        } else {
            resultMove = SkipToAssignpost;      /* empty body, just execute assignpost */
        }
    }
    
    return resultMove;
}

    
void InteractionManager::executeNodeAssignmentBlock(tree<Node>::iterator_base &node) {

    if (node->recipe_name == "ROOT") {
        FILE_LOG(logDEBUG3) << "executeNodeAssignmentBlock skipped ROOT.";
        return;
    }
    
    FILE_LOG(logDEBUG3) << "Trying to execute a potential assignment block for the node: " <<
        node->recipe_name;


    while ((decideNodeControlFlowMove(node) == SkipToNext ) &&
            (node->body.elements[node->active_element + 1].element_type ==
             "assignment")) {
        node->ae_status = "executing";
        node->active_element++;
        this->action_counter++;
        BodyElement &element1 =                                         \
            node->body.elements[node->active_element];
        if (!this->executeAssignment(element1, node->bindings)) {
            FILE_LOG(logDEBUG3) << "Assignment failed.";
                /* TODO: handle failure of assignment (normally
                 * due to the _DEFUNCT_ local binding) */
            break; /* not sure what to do if assignment fails, for now just return */
        }
        node->ae_status = "completed";
    }
}

void InteractionManager::executeNode(tree<Node>::post_order_iterator &node) {

    if (node->recipe_name == "ROOT") {
        FILE_LOG(logDEBUG3) << "executeNode skipped ROOT.";
        return;
    }
    
    FILE_LOG(logDEBUG3) << "Executing node: " << node->recipe_name;

    ControlFlowMove aFlowMove = decideNodeControlFlowMove(node);

    if (aFlowMove == DoNothing) {
        return;
    } else if (aFlowMove == SkipToNext) {
        node->active_element++;
    } else if (aFlowMove == Repeat) {
                                            /* do not advance active element counter,
                                            * basically do nothing here */ 
    }
    
    if ((aFlowMove != SkipToAssignpost)&&(aFlowMove != SkipToEnd)) {
            /* basically if the flow move is to execute a body element */
        node->ae_action_id = this->action_counter;
        FILE_LOG(logDEBUG3) << "Executing body element: " << node->active_element << \
            " as action_id: " << node->ae_action_id;
            /** increment the global IM action counter */
        this->action_counter++;
        node->ae_status = "pending";

        BodyElement &element1 = \
            node->body.elements[node->active_element];
        if (element1.element_type=="action") {
            node->ae_status = "executing";
            if (!this->dispatchAction(element1, node->bindings,
                                      node->ae_action_id, node->recipe_name,
                                      node->active_element)) {
                FILE_LOG(logDEBUG1) << "Dispatch of action failed.";
                node->ae_status = "pending";
            } else {
                element1.dispatch_time = getSystemTimeMSec()/1000.0;
            }
        }
        else if (element1.element_type=="goal") {
            node->ae_status = "executing";
            string status;
            if (!this->tryBackchainOnGoal(element1, node, status)) {
                FILE_LOG(logDEBUG4) << "Backchaining on goal failed.";
                node->ae_status = "pending";
            } else {
                if (status == "completed") {
                    node->ae_status = "completed";
                        /* uncurse edit */
                        //this->executeNode(node);
                } 
            }
        }
        else if (element1.element_type=="assignment") {
            node->ae_status = "executing";
            if (!this->executeAssignment(element1, node->bindings)) {
                FILE_LOG(logDEBUG1) << "Assignment failed.";
                    /* TODO: handle failure of assignment (normally
                     * due to the _DEFUNCT_ local binding) */
            }
            node->ae_status = "completed";

                /* if the next body element is an assignment, executeNodeAssignmentBlock.
                 * Need executeAssignment above since node->active_element has been
                 * already advanced.
                 * */
            if ((decideNodeControlFlowMove(node) == SkipToNext ) &&
                (node->body.elements[node->active_element + 1].element_type ==
                 "assignment")) {
                this->executeNodeAssignmentBlock(node);
            }
            
                /** move to the next BodyElement right away*/
                /* uncurse edit */
                //this->executeNode(node);
        }
        else {
            FILE_LOG(logERROR) << "Executing body element type " << element1.element_type \
                               << " not implemented yet.";
        }
    }
    else if (aFlowMove == SkipToAssignpost) {
            /** assignpost */
        Formula &assignpost = \
            node->assignpost;
        FILE_LOG(logDEBUG3) << "Node's body completed, execute assignpost.";
        node->ae_status = "executing";
        if (!this->executeAssignPost(assignpost, node->bindings)) {
            FILE_LOG(logDEBUG1) << "Assignpost failed.";
        }
        node->ae_status = "completed";

            /** purge node*/
        FILE_LOG(logDEBUG3) << "Node's body completed, purging node: " << node->recipe_name;
        tree<Node>::iterator parent_node = this->ptree.tr.parent(node);
        this->ptree.print(logDEBUG3);
        this->ptree.tr.erase(node);
        this->ptree.print(logDEBUG3);
        if (parent_node != this->ptree.tr.begin()) {
            FILE_LOG(logDEBUG3) << "Execute the parent: " << parent_node->recipe_name;
                /* goal is presumably achieved now, the body element is completed */
            parent_node->ae_status = "completed";
                /* uncurse edit */
                //this->executeNode(parent_node);
        } else {
            FILE_LOG(logDEBUG3) << "Parent of erased node is ROOT (1). ";
        }
    } else if (aFlowMove == SkipToEnd) {
            /** purge node*/
        FILE_LOG(logDEBUG3) << "Node's body completed, purging node: " << node->recipe_name;
        tree<Node>::iterator parent_node = this->ptree.tr.parent(node);
        this->ptree.print(logDEBUG3);
        this->ptree.tr.erase(node);
        this->ptree.print(logDEBUG3);
        if (parent_node != this->ptree.tr.begin()) {
            FILE_LOG(logDEBUG3) << "Execute the parent: " << parent_node->recipe_name;
                /* goal is presumably achieved now, the body element is completed */
            parent_node->ae_status = "completed";
                /* uncurse edit */
                //this->executeNode(parent_node);
        } else {
            FILE_LOG(logDEBUG3) << "Parent of erased node is ROOT (2). ";
        }
    } else {
         FILE_LOG(logERROR) << "IM error: Unknown flow move: " << aFlowMove;
    }
}

bool InteractionManager::executeAssignment(BodyElement &element1, Conjunction &bindings) {
    FILE_LOG(logDEBUG3) << "Attempting to execute assignment: " << element1.name;
	/** get assignment formula and update root bindings with it
	 * using current node bindings
	 * for reference resolution */
    if (element1.formula.disjuncts.size()!=1) {
        FILE_LOG(logDEBUG3) << "IM error: assignment formula has " << \
            element1.formula.disjuncts.size() <<                        \
            " conjuncts, must have exactly 1.";
    }
    else {

            /* update global and local bindings */
        this->updateGlobalBindings();
        Conjunction *gBindings = this->getGlobalBindings();
        bindings.updateBinding(*gBindings);
        
            /* make a copy of the formula since we gonna evaluate atoms in place*/
        Formula aformula = element1.formula;
            /* iterate trough atoms of the single conjunction */
        vector<Conjunction>::iterator conj1 = aformula.disjuncts.begin();
        
        for(vector<Atom>::iterator
                atom1 = conj1->atoms.begin(); atom1!=conj1->atoms.end(); atom1++) {
                /* find the atom id according to the binding to slot_var*/
            string var_name = atom1->readSlotVar("this"); /** get atom id var_name */

            #ifdef DEBUG
            FILE_LOG(logDEBUG3) << "Requested to assign to atom with 'this': " << var_name;
            bindings.print(logDEBUG3);
            #endif
            
            string atom_id = bindings.readAtomVarVal(var_name);

            if (atom_id == "_DEFUNCT_") {
                    /* local atom binding is defunct since global bindings are not found
                     * Assignment fails. */
                return false;
            }
            
            FILE_LOG(logDEBUG3) << "Found in bindings atom to assign to, it's id: "
                                << atom_id;
                /* write atom_id into "this" slot of the assignment atom */
            atom1->setSlotVal("this", atom_id);

                /* WE DO NOT allow pure bindings in the assignment.
                 * Here, "this" slot of the assignment atom is of course
                 * already bound to the corresponding global atom via
                 * a preconditions atom that shares same var.*/
            
                /* evaluate value expressions in the new atom */
            atom1->evalSlotVals(bindings);
            
                /* update root bindings, not that all vals have been evaluated */
            if (!this->updateAtomBinding(*atom1)) {
                FILE_LOG(logERROR) << "IM error: failed to update root bindings.";
            }
                /* NOTE: we need to update the local bindings again, because the
                 * next assignment in the block may rely on the modified local vars.
                 * Of course, this can also invalidate preconditions, which is OK,
                 * since we only check them once when the atom is loaded. */
             bindings.updateBinding(*gBindings);
        }
    }
    return true;
}

/* work on a copy of the formula since we write "this" slot value in place. */

bool InteractionManager::executeAssignPost(Formula form1, Conjunction &bindings) {
    FILE_LOG(logDEBUG3) << "Attempting to execute assignpost. ";
	/** get assignment formula and update root bindings with it using current node bindings
	 * for reference resolution */
    if (form1.disjuncts.size()>1) { /* ok to have 0 atoms in assignpost? */
        FILE_LOG(logERROR) << "IM error: assignment formula has " << \
            form1.disjuncts.size() << " conjuncts, must have exactly 1.";
    } else if (form1.disjuncts.empty()) {
            /* empty assignpost */
        return true;
    } else {
            /** iterate trough atoms of the single conjunction */
        vector<Conjunction>::iterator conj1 = form1.disjuncts.begin();
        for(vector<Atom>::iterator atom1 = conj1->atoms.begin(); atom1!=conj1->atoms.end(); atom1++) {
                /** find the atom id according to the binding to slot_var*/
            string var_name = atom1->readSlotVar("this"); /** get atom id var_name */

            #ifdef DEBUG
            FILE_LOG(logDEBUG4) << "Read var_name: " << var_name;
            bindings.print(logDEBUG4);
            #endif

            string atom_id = bindings.readAtomVarVal(var_name);

            #ifdef DEBUG
            FILE_LOG(logDEBUG4) << "Read varval: " << atom_id;
            #endif
            
            atom1->setSlotVal("this", atom_id);
            if (!this->updateAtomBinding(*atom1)) {
                FILE_LOG(logERROR) << "IM error: failed to update root bindings.";
            }
        }
    }
    return true;
}


/** 
 * do the necessary active action index advancement 
 */

void InteractionManager::processActionCompletionStatus(ActionStatus &astat) {

        /* find the node that needs advancement. Such node may not exist anymore,
         * since the  */
    tree<Node>::iterator node = this->ptree.findNodeGivenActionId(astat.action_id);
    if (node != this->ptree.tr.end()) {

            /*
             * perform assignment of return_args using node's $-bindings */
        if (!this->processReturnArgs(astat.return_args, node->bindings)) {
            FILE_LOG(logERROR) << "Error assigning return args for action: " <<
                astat.action_id;
        }
            /* update action status in the tree accordingly */
        node->ae_status = astat.status;
    } 
    else {
        FILE_LOG(logDEBUG2) <<
            "Oops, the node for this action is not found, must have been deleted.";
    }	
    
}

/* processes action's return_args doing $-var binding lookup and updating
 * respective global bindings accordingly to returned values. Note that unlike
 * assignment operators, return args get assigned according to $-vars in ArgSlot
 * var slot.*/
bool InteractionManager::processReturnArgs(Args &returnArgs, Conjunction &bindings) {
    for (list<ArgSlot>::iterator arg_it = returnArgs.begin();
         arg_it != returnArgs.end(); arg_it++) {
        if ( (arg_it->var.empty()) || (arg_it->var == "_NO_VALUE_") ) {
            FILE_LOG(logERROR) << "Missing var in return_arg with name: " << arg_it->name;
            return false;
        } else {
                /* var is present, look it up in the bindings */
            Atom* lAtom_p = bindings.findAtomByVar(arg_it->var);
            if (!lAtom_p) {
                FILE_LOG(logERROR) << "Node bindings do not contain return_arg var: "
                                   << arg_it->var;
                return false;
            } else {
                    /* first, update local binding value so that we have a fresh one
                     * while we are at it anyways, before updateBindings is called */
                VarSlot* lSlot_p = lAtom_p->getSlotByVar(arg_it->var);
                    /* this slot must exist here */
                lSlot_p->val = arg_it->value;
                
                    /* update global bindings */ 
                string this_val = lAtom_p->readSlotVal("this");
                if (this_val == "_NOT_FOUND_") {
                    FILE_LOG(logERROR) <<
                        "Missing 'this' slot in node bindings with reuturn_arg var: "
                                       << arg_it->var;
                    return false;
                } else if (this_val == "_NO_VALUE_") {
                    FILE_LOG(logERROR) <<
                        "No value for 'this' slot in node bindings with reuturn_arg var: "
                                       << arg_it->var;
                    return false;
                } else {
                        /* valid this slot value found, update the corresponding global atom */
                    Conjunction *gBindings = this->getGlobalBindings();
                    vector<Atom>::iterator gAtom_it =           \
                        gBindings->findAtom("this", this_val);
                    gAtom_it->setSlotVal(lSlot_p->name, arg_it->value);
                }
            }
        }
        
    }
    
    return true;
}

/*********************
 * Backchaining
 * ********************/

bool InteractionManager::tryBackchainOnGoal(BodyElement &element1, tree<Node>::iterator_base &node, string &status) {
    FILE_LOG(logDEBUG4) << "Attempting to backchain on goal: " << element1.name;

        /* prepare the goal's formula by adding type and subtype to goal's atoms */
    
	/** check what recipe's post assignments satisfy the goal */    
    bool res = false;
        /* work on a copy of the goal formula*/
    Formula goalFormula = element1.formula;
        /* check if the goal is already true by binding "this" and
         * matching against globals (same as checking whilecondition).
         * If goal formula is empty, it is not satisfied, and
         * is satisfied by any of the user-defined backchainable recipes */
    goalFormula.bindThis(node->bindings, node->recipe_name);
    if ( (goalFormula.disjuncts.size() != 0)
         && goalFormula.unifyWithoutBinding(*(this->getGlobalBindings()),
                                            node->bindings) ) {
        FILE_LOG(logDEBUG3) << "Goal " << element1.name <<
            " is already satisfied by globals.";
        status = "completed";
        return true;
    }
        /* OK goal is not already true or goal is empty. Try to match against
         * assignposts of recipes. There are no missing type and subtype
         * slots, since recipe loading took care of that. */
    
    Formula goalFormula2 = element1.formula; /* get a fresh copy with empty "this" */

    if (true) {
            /* use backchaining preprocessing */
        for (std::list<string>::iterator recipe_name_it = element1.backchainables.begin(); \
             recipe_name_it != element1.backchainables.end(); recipe_name_it++) {
            if (this->tryBackchainOnGoalWithRecipe(this->recipes[*recipe_name_it],
                                                   element1.name, goalFormula,
                                                   goalFormula2, node)) {
                    /* stop matching as soon as any recipe succeedes */
                res=true;
                status = "executing";
                break;}
        }
        status = "pending";
        return res;
    } else {
            /* do not use backchaning preprocessing */
        for (std::map<string, Recipe>::iterator recipe_it = this->recipes.begin(); \
             recipe_it != this->recipes.end(); recipe_it++) {
            if (this->tryBackchainOnGoalWithRecipe(recipe_it->second,
                                                   element1.name, goalFormula,
                                                   goalFormula2, node)) {
                    /* stop matching as soon as any recipe succeedes */
                res=true;
                status = "executing";
                break;}
        }
        status = "pending";
        return res;
    }
}

/*
 * GoalFormula1 has "this" vals instatiated. GoalFormula2 does not. This is
 * to save some time since we need those "this" values but also
 * unification with postcondition needs "this" vals empty. A bit ugly, but well.
 * */


bool InteractionManager::tryBackchainOnGoalWithRecipe(Recipe &recipe, 
                                                      string &goalName,
                                                      Formula &goalFormula1,
                                                      Formula &goalFormula2,
                                                      tree<Node>::iterator_base &node) {
    FILE_LOG(logDEBUG4) << "Attempting to backchain on goal: " << goalName << \
        "with recipe: " << recipe.name;
	/*
         * check if post assignments satisfy the goal
         * */
        /* update global and local bindings */
    this->updateGlobalBindings();
    Conjunction *gBindings = this->getGlobalBindings();
    node->bindings.updateBinding(*gBindings);

        /*
         * Unify goal against assignpost. In other words, goal is like a precondition
         * that would be satisfied if assignpost were executed.
         * Amazingly syntax of assignment
         * allows to use it as a binding. Note that post assignment
         * should not use $-vars besides "this"
         * because this recipe's local bindings cannot yet be defined.
         * */

        /* assignpost is only one conjunction */
    vector<Conjunction>::iterator conj_it = recipe.assignpost.disjuncts.begin();
    if (conj_it == recipe.assignpost.disjuncts.end()) {
            /* empty assignpost satisfies only empty goals. For now
             * we disallow empty goals though, so, false. */
        return false;
    }

        /* TODO: tricky case of unification. nail it.*/
    
        /* goal formula does not have 'this' values initialized, _NO_VALUE_
         * but postassignment "this" vals should be the same, so,
         * "this" slots should always unify which is the desired functionality.
         *
         * However since the atoms will be unlocked (no "this" slot locking is
         * possible), we need to supply type and subtypes for both atoms).
         * */
    std::vector< Match > mapping;
    int matched_con_id;
    
        /* debugging output */
    #ifdef DEBUG
    FILE_LOG(logDEBUG4) << "========Goal formula:";
    goalFormula2.print(logDEBUG4);
    FILE_LOG(logDEBUG4) << "========End Goal formula.";
    FILE_LOG(logDEBUG4) << "========AssignPost conjunction:";
    conj_it->print(logDEBUG4);
    FILE_LOG(logDEBUG4) << "========End  AssignPost conjunction.";
    #endif
    
    if (!goalFormula2.unifyWithoutBinding((*conj_it),
                                          node->bindings,
                                          mapping,
                                          matched_con_id, Complete)) {
        FILE_LOG(logDEBUG4) << "Postconditions of recipe " << recipe.name << 
            " DO NOT satisfy goal " << goalName;
        return false;
    }

    #ifdef DEBUG
    FILE_LOG(logDEBUG3) << "Postconditions of recipe " << recipe.name << 
        " satisfy goal " << goalName;
    FILE_LOG(logDEBUG3) << "Matched conjunction id: " << matched_con_id;
    FILE_LOG(logDEBUG3) << "Mapping of the match:";
    for(std::vector< Match >::iterator aMatch_it=mapping.begin(); 
        aMatch_it != mapping.end(); aMatch_it++) {
        aMatch_it->print(logDEBUG3);
    }
    #endif
    
    	/** If preconditions of the recipe are satisfied,
	 * backchain after the current node **/
    Conjunction new_bindings;
    tree<Node>::iterator root = this->ptree.tr.begin();
    Formula precond = recipe.precondition; /* work on a copy */
    string new_var;
    string this_val;
    std::vector<Atom>::size_type i = 0;
    std::vector<Atom>::size_type id1 = 0;
       
        /* bind vals of some "this" slots of precond based on bindings of current node
         * vars shared with the goal, mapping and matched_con_id obtained above
         * and shared vars between the new recipe's assignpost and preconds. */

        /* Implementation: iterate through assignpost atoms,
         * find corresponding 'this' vals,
         * set them for precond. */
        /* get the right conjunction from goalFormula1. Use goalFormula1,
         * since it has instantiated "this" slots */
    
    Conjunction &matched_con = goalFormula1.disjuncts[matched_con_id];
        
    for (std::vector< Atom >::iterator atom_it = conj_it->atoms.begin();
         atom_it != conj_it->atoms.end(); atom_it++) {
        new_var = atom_it->readSlotVar("this");
        if ((new_var == "_NO_VALUE_")||(new_var == "_NOT_FOUND_")) {
            FILE_LOG(logERROR) <<
                "Not found a var for 'this' atom of assignpost with i = "
                               << i;
            return false;
        }
            /* find corresponding atom in goal, get its "this" val */
        bool found = false;
        for(std::vector< Match >::iterator aMatch_it=mapping.begin(); 
            aMatch_it != mapping.end(); aMatch_it++) {
            if (aMatch_it->id2 == i) {
                id1 = aMatch_it->id1;
                found = true;
                break;
            }
        }
        if (!found) {
            FILE_LOG(logERROR) << "Not found a match entry in mapping for\n"
                               << "assignpost atom with 'this' = " << new_var;
            return false;
        }
            /* got id1, read "this" var of id1-th atom from match_con_id of
             * the goal. */
        this_val = matched_con.atoms[id1].readSlotVal("this");
        if ((this_val == "_NO_VALUE_")||(this_val == "_NOT_FOUND_")) {
            FILE_LOG(logERROR) <<
                "Not found a val for 'this' atom of a goal";
            return false;
        }
            /* find an atom in precond of the new recipe with var==new_val */
        Atom* precAtom_p = precond.findAtomByVar(new_var);
        if (!precAtom_p) {
            FILE_LOG(logERROR) << "No assignpost var: " << new_var <<
                " found in preconditions of new recipe: " <<
                recipe.name;
            return false;
        }
            /* update that atom's "this" val to this_val */
        precAtom_p->setSlotVal("this", this_val);
        
            /* increment assignpost atom index */
        i++;
    }
    
    if (this->checkPreconditionGivenFormula(precond, root, new_bindings)) {
            /** make a new tree node, assign the new bindings to it**/
        FILE_LOG(logDEBUG3) << "Also preconditions of backchaining candidate recipe "
                            << recipe.name << " satisfy goal " << goalName;
        Node new_node(recipe);
        new_node.bindings = new_bindings;
        new_node.recipe_name = recipe.name;
        
            /** set up the whilecondition field of the new node
             * based on the local bindings (whilecondition atoms are
             * assumed to be a subset of precondition atoms because we
             * need 'this' values to be already locked).
             * */
        
        if (!this->bindWhilecondition(new_node)) {
            FILE_LOG(logERROR) << "When backchaining on goals for node: "
                               << node->recipe_name <<
                "\n           failed to bind whileconditions for node: " << recipe.name;
            return false;
        }
            /** push node onto the top of the tree **/
        FILE_LOG(logDEBUG3) << "about to backchain by adding a node";
        tree<Node>::iterator new_node_it = this->ptree.tr.append_child(node, new_node);
        this->ptree.print(logDEBUG3);
            /* uncurse edit reversed, to give a chance first element to get executed right away
             * in case it's an assignment that's gonna block other recipes from loading*/
        this->executeNodeAssignmentBlock(new_node_it);
        return true;
    }
    else {
        FILE_LOG(logDEBUG3) << "But preconditions of recipe " << recipe.name << \
            " DO NOT satisfy goal " << goalName;
        return false;
    }
}


void InteractionManager::tryBackchain() {
	/** iterate using post-order, children first, maybe doesn't matter */
    FILE_LOG(logDEBUG4) << "Trying to backchain on everything";
    for(tree<Node>::iterator node=this->ptree.tr.begin();  \
        node!=this->ptree.tr.end(); node++) {
        if (node->recipe_name == "ROOT") {
            FILE_LOG(logDEBUG4) << "Backchaning skipped root node";
            continue;
        }
        FILE_LOG(logDEBUG4) << "Checking the node to backchain from: " << node->recipe_name;
        FILE_LOG(logDEBUG4) << "node->ae_status: " << node->ae_status;
        if (node->ae_status == "pending") {
                /** probably a goal */
            FILE_LOG(logDEBUG4) << "Found pending ae_status for the node: " << node->recipe_name;

            map<string, Recipe>::iterator recipe_it = this->recipes.find(node->recipe_name);
            if (recipe_it == this->recipes.end()) {
                FILE_LOG(logERROR) << "Error in IM: recipe " << node->recipe_name << \
                    " not found in IM recipe map";
                return;
            }

            FILE_LOG(logDEBUG4) << "Pending active element: " << node->active_element << \
                " as action_id: " << node->ae_action_id;

            BodyElement &element1 = \
                (recipe_it->second).body.elements[node->active_element];
                /** see if the active element is a goal**/
            if (element1.element_type=="goal") {
                node->ae_status = "executing";
                string status;
                if (!this->tryBackchainOnGoal(element1, node, status)) {
                    FILE_LOG(logDEBUG4) << "Backchaining on goal failed.";
                    node->ae_status = "pending";
                } else if (status == "completed") {
                    node->ae_status = "completed";
                }
            }
        }
    }
}

/* iterate through the nodes, for now in a tree order and try executing next element. This
 * is for the update stage that doesn't do backchaining, hence we only check nodes
 * whose ae_status is completed or executing (the latter one is relevant when we want to check
 * if the action has timed out) */
void InteractionManager::tryExecuteNodes() {
	                                     /** iterate using post-order,
                                              * children first, maybe doesn't matter */
    FILE_LOG(logDEBUG4) << "Trying to execute every node";
    tree<Node>::post_order_iterator node=this->ptree.tr.begin_post();
    while (node!=this->ptree.tr.end_post()) {
        if (node->recipe_name == "ROOT") {
            FILE_LOG(logDEBUG4) << "tryExecuteNodes skipped ROOT.";
            node++;
            continue;
        }
        FILE_LOG(logDEBUG4) << "Checking the node to execute: " << node->recipe_name;
        FILE_LOG(logDEBUG4) << "node->ae_status: " << node->ae_status;
        tree<Node>::post_order_iterator next_node=node;
        next_node++;
            /* NOTE: the purpose of the following if condition is not clear. */
        if ( (node->ae_status == "completed") || (node->ae_status == "_NO_VALUE_") ||
             (node->ae_status == "executing") || (node->ae_status == "aborted")) {
                                             /* ready to execute an element */
            FILE_LOG(logDEBUG4) << "Found completed or executing status for the node: "
                                << node->recipe_name;
                                             /* execution may result in erasing a node.
                                              * only post_order iteration is safe,
                                              * unless we receive an interator to a safe
                                              * node from erase method of the tree.*/
            this->executeNode(node);  
        }
        node = next_node;
    }
}

/* perform all necessary state updates within a single time tick */
void InteractionManager::doTick() {
    double start_tick = getSystemTimeMSec()/1000.0;

    this->timing_dump_counter--;
    if (this->timing_dump_counter < 0) {
        this->timing_dump_counter =  this->timing_dump_period - 1;
        this->subcycle_timings.clear();
    }
    
    FILE_LOG(logDEBUG4) << "@@@@@@@@@@@@ start of an IM tick at system time: " <<
        setprecision(20) << start_tick;
    
    FILE_LOG(logDEBUG4) << "@@@@@ Global bindings at the start of the tick: ";
    this->getGlobalBindings()->print(logDEBUG4);

    FILE_LOG(logDEBUG4) << "@@@@@ Step 0: Process input atom queue.";
    double start = getSystemTimeMSec()/1000.0;
    this->updateRootBindingFromInputQueue();
    double end = getSystemTimeMSec()/1000.0;
    FILE_LOG(logDEBUG4) << "@@@@@ Ended step 0: Process input atom queue at: " <<
        setprecision(20) << getSystemTimeMSec()/1000.0;
    this->subcycle_timings["inputqueue"].push_back(end - start);
    
    FILE_LOG(logDEBUG4) << "@@@@@ Step 1: Check whileconditions.";
    start = getSystemTimeMSec()/1000.0;
    this->checkPruneWhileConditions();
    end = getSystemTimeMSec()/1000.0;
    FILE_LOG(logDEBUG4) << "@@@@@ Ended step 1: Check whileconditions at: " <<
        setprecision(20) << getSystemTimeMSec()/1000.0;
    this->subcycle_timings["whilecondition"].push_back(end - start);
    
        /* temporary test 1: backchain before triggerables to give it a priority */
    FILE_LOG(logDEBUG4) << "@@@@@ Step 1.5: Try backchain.";
    start = getSystemTimeMSec()/1000.0;
    this->tryBackchain();
    end = getSystemTimeMSec()/1000.0;
    FILE_LOG(logDEBUG4) << "@@@@@ Ended step 1.5: Try backchain at: " << setprecision(20) <<
        getSystemTimeMSec()/1000.0;
    this->subcycle_timings["backchain"].push_back(end - start);
    
    FILE_LOG(logDEBUG4) << "@@@@@ Step 2: Push triggerables.";
    start = getSystemTimeMSec()/1000.0;
    this->tryPushTriggerables();
    end = getSystemTimeMSec()/1000.0;
    FILE_LOG(logDEBUG4) << "@@@@@ Ended step 2: Push triggerables at: " <<
        setprecision(20) << getSystemTimeMSec()/1000.0;
    this->subcycle_timings["pushtriggerables"].push_back(end - start);
        /* temporary test 1.
           FILE_LOG(logDEBUG3) << "@@@@@ Step 3: Try backchain.";
           this->tryBackchain();
           FILE_LOG(logDEBUG3) << "@@@@@ Ended step 3: Try backchain at: " << setprecision(20) <<
           getSystemTimeMSec()/1000.0;
        */
    FILE_LOG(logDEBUG4) << "@@@@@ Step 4: Execute nodes.";
    start = getSystemTimeMSec()/1000.0;
    this->tryExecuteNodes();
    end = getSystemTimeMSec()/1000.0;
    FILE_LOG(logDEBUG4) << "@@@@@ Ended step 4: Execute nodes.";
    this->subcycle_timings["executenodes"].push_back(end - start);
     
    this->ptree.print(logDEBUG4);

    FILE_LOG(logDEBUG4) << "@@@@@ Step 5: Garbage collect deleted atoms.";
    start = getSystemTimeMSec()/1000.0;
    this->garbageCollectDeletedAtoms();
    end = getSystemTimeMSec()/1000.0;
    FILE_LOG(logDEBUG4) << "@@@@@ Ended step 5: Garbage collect deteled atoms.";
    this->subcycle_timings["garbagecollect"].push_back(end - start);
    
    this->ptree.print(logDEBUG4);

    FILE_LOG(logDEBUG4) << "@@@@@@@@@@@@ end of a tick at system time: " <<
        setprecision(20) << getSystemTimeMSec()/1000.0;
    
    double end_tick = getSystemTimeMSec()/1000.0;
    this->subcycle_timings["cycle"].push_back(end_tick - start_tick);
}

