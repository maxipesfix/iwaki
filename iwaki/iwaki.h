/*****************************************************************************
 * PROJECT: Iwaki Interaction Manager
 *
 * FILE: iwaki.h
 *
 * ABSTRACT: iwaki header file.
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

#ifndef IWAKI_H
#define IWAKI_H

#include "queueio.h"
#include "nlpw_utils.h"
#include "tinyxml.h"
#include "tree.hh"
#include <iostream>
#include <deque>


#define IM_INIT_FILENAME "initialize_im.xml"
#define DEFAULT_ATOMS_FILENAME "default_atoms.xml"

#define MAX_LOGFILE_SIZE 100000000
#define MAX_ELEMENT_AGE 1000

#define ABORT_ACTION_NAME "abort_action"

class Conjunction;
class InteractionManager;
class ExpressionParser;
class Matching;
class Match;
class History;

enum ControlFlowMove {UndefinedMove, DoNothing, Repeat, SkipToNext, SkipToAssignpost, SkipToEnd};
enum MatchType {Unlocked, Locked};
enum HowComplete {Partial, Complete};
enum WhenToDelete {NotYet, WhenNotUsed, Now};

void insertIntListSortedUnique(std::list<int> &aList, const int value);
void insertStringListSortedUnique(std::list<string> &aList, const string &value);

class TriggerableRecord {
  public:
	// default constructor
    TriggerableRecord() {}

  public:
    string name;
    string max_instances;
};

#ifdef USE_RE2
/* variable constraint binding */
class VarSlot {
  public:
    // default constructor
    VarSlot(): relation("equal"), val("_NO_VALUE_"), type("string"),
        unique_mask(false), re_p(NULL), updated(true) {}
    VarSlot(string name, string val): name(name), relation("equal"), val(val),
        type("string"), unique_mask(false), re_p(NULL), updated(true) {}
    VarSlot(string name, const char *val): name(name), relation("equal"),
        val((string)val), type("string"), unique_mask(false), re_p(NULL),
        updated(true) {}
    VarSlot(string name, int val): name(name), relation("="),
        val(to_string(val)), type("number"), unique_mask(false), re_p(NULL),
        updated(true) {}
    VarSlot(string name, double val): name(name), relation("="),
        val(to_string(val)), type("number"), unique_mask(false), re_p(NULL),
        updated(true) {}
    VarSlot(string name, bool val): name(name), relation("equal"),
        val(val ? "true" : "false"), type("string"), unique_mask(false),
        re_p(NULL), updated(true) {}
    VarSlot(string name, string val, bool unique_mask): name(name),
        relation("equal"), val(val), type("string"), unique_mask(unique_mask),
        re_p(NULL), updated(true) {}
    bool load(TiXmlElement* pElem);
    bool typeCheck();
    void print();
    void print(TLogLevel log_level);
    void printWithLabels();
    bool unify(VarSlot &varslot2, Conjunction &new_bindings, HowComplete howcomplete);
    bool evalRelation(string &val2, string &relation, string &type,
                      Conjunction &new_bindings, HowComplete howcomplete);
    bool evalStringRelation(string &val2, string &relation, string &type,
                            Conjunction &new_bindings, HowComplete howcomplete);
    bool evalNumberRelation(string &val2, string &relation, string &type,
                            Conjunction &new_bindings, HowComplete howcomplete);
    bool evalRERelation(string &val2, string &val1, string &relation, string &type,
                        Conjunction &new_bindings);
  public:
    string name;
    string relation;
    string val;
    string var;
    string type;
    bool unique_mask;
    list<string> enumerables; /* used in a default atom definition to
                         * specify the list of allowed values */
    list<string> inPrecondOfRecipes; /* a list of recipes for which this slot is
                                       * in the preconditions */
    RE2 *re_p;          /* pointer to a pre-compiled regular expression */
    bool updated;
};

#else

/* variable constraint binding */
class VarSlot {
  public:
        // default constructor
    VarSlot(): relation("equal"), val("_NO_VALUE_"), type("string"),
        unique_mask(false), updated(true) {}
    VarSlot(string name, string val): name(name), relation("equal"), val(val),
        type("string"), unique_mask(false), updated(true) {}
    VarSlot(string name, const char *val): name(name), relation("equal"),
        val((string)val), type("string"), unique_mask(false), updated(true) {}
    VarSlot(string name, int val): name(name), relation("="),
        val(to_string(val)), type("number"), unique_mask(false), updated(true) {}
    VarSlot(string name, double val): name(name), relation("="),
        val(to_string(val)), type("number"), unique_mask(false), updated(true) {}
    VarSlot(string name, bool val): name(name), relation("equal"),
        val(val ? "true" : "false"), type("string"), unique_mask(false), updated(true) {}
        VarSlot(string name, string val, bool unique_mask): name(name),
            relation("equal"), val(val), type("string"), unique_mask(unique_mask),
            updated(true) {}
    bool load(TiXmlElement* pElem);
    bool typeCheck();
    void print();
    void print(TLogLevel log_level);
    void printWithLabels();
    bool unify(VarSlot &varslot2, Conjunction &new_bindings, HowComplete howcomplete);
    bool evalRelation(string &val2, string &relation, string &type,
                      Conjunction &new_bindings, HowComplete howcomplete);
    bool evalStringRelation(string &val2, string &relation, string &type,
                            Conjunction &new_bindings, HowComplete howcomplete);
    bool evalNumberRelation(string &val2, string &relation, string &type,
                            Conjunction &new_bindings, HowComplete howcomplete);
  public:
    string name;
    string relation;
    string val;
    string var;
    string type;
    bool unique_mask;
    list<string> enumerables; /* used in a default atom definition to
                                    * specify the list of allowed values. Checked
                                    * against this spec in pre- post-conditions
                                    * and assignments
                                    * upon recipe loading, and during run-time */
    list<string> inPrecondOfRecipes; /* a list of recipes for which this slot is
                                       * in the preconditions */
    bool updated;
};
#endif

class Atom {
  public:
        // default constructor
    Atom(): quantifier("exist"), toBeDeleted(NotYet) {}
    bool load(TiXmlElement* pElem);
    bool typeCheck();
    void updateFromDefaults(Atom &defaults_atom);
    void updateFromDefaults(Conjunction &defaults_atoms);
    void updateDefaultsToRecipesMap(string &recipe_name,
                                    Atom &defaults_atom);
    void updateDefaultsToRecipesMap(string &recipe_name,
                                    Conjunction &default_atoms);
    void updateDefaultsToRecipesMap(string &recipe_name,
                                    string &varslot_name);
    void print();
    void print(TLogLevel log_level);
    void printWithLabels();
    string toBeDeleted2String();
    bool unify(Atom &atom2, Conjunction &new_bindings, HowComplete howcomplete); 
    bool hasVar(string var);
    VarSlot* getSlotByName(string slot_name);
    VarSlot* getSlotByVar(string slot_var);
    string readSlotVal(string slot_name);
    string readVarVal(string slot_var);
    string readSlotVar(string slot_name);
    list<string> &getSlotEnum(string slot_name);
    void setSlotVal(string slot_name, string slot_val);
    void setSlotVal(string slot_name, string slot_val, bool updated);
    void setSlotUniqueMask(string slot_name, bool mask_val);
    bool setSlotValByVar(string slot_var, string slot_val);
    void updateAtomValsOnly(Atom &atom2);
    void updateAtom(Atom &atom2);
    void evalSlotVals(Conjunction &bindings);
    void bindBindings(Atom &gAtom, Atom &atom_bindings);
    bool checkSyntax();
  public:
    std::list<VarSlot> varslots;
    string quantifier;
    WhenToDelete toBeDeleted;
    std::list<int> node_ids;
};

class Conjunction {
  public:
    // default constructor
    Conjunction() {
            //std::list<Atom> temp_list;
            //this->atoms = temp_list;
            //cout << "Atoms length in constructor is: " << this->atoms.size();
    }
    void print();
    void print(TLogLevel log_level);
    bool load(TiXmlElement* pElem);
    bool typeCheck();
    void updateFromDefaults(Conjunction &defaults_atoms);
    void updateDefaultsToRecipesMap(string &recipe_name,
                                    Conjunction &default_atoms);
    Atom* findAtomByVar(string var1);
    bool unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings,
                             std::vector< Match > &mapping, HowComplete howcomplete);
    bool unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings);
    bool unify(Conjunction &bindings2, Conjunction &new_bindings);
    bool unifyLockedAtoms(Conjunction &con2, Conjunction &new_bindings,
                          Matching &initMatching, HowComplete howcomplete);
    void updateBinding(Conjunction &gBindings);
    void updateBinding(Conjunction &gBindings, const int node_id);
    void bindBindings(Conjunction &gBindings, Conjunction &new_bindings, Matching &aMatching);

    vector<Atom>::iterator findAtom(string n1, string v1, string n2, string v2, \
                                  string n3, string v3);
    vector<Atom>::iterator findAtom(string n1, string v1, string n2, string v2);
    vector<Atom>::iterator findAtom(string n1, string v1);
    string readAtomVarVal(string slotvar); /** find atom with var and read its varslot value*/
  public:
    std::vector<Atom> atoms;
};

/* Formula is a disjunction of conjuncts */
class Formula {
  public:
        // default constructor
    Formula() {}

  public:
    bool load(TiXmlElement* pElem);
    void print();
    void print(TLogLevel log_level);
    bool typeCheck();
    void updateFromDefaults(Conjunction &defaults_atoms);
    void updateDefaultsToRecipesMap(string &recipe_name,
                                    Conjunction &default_atoms);
    bool bindTypeandSubtype(Formula precondition, string recipe_name);
    Atom* findAtomByVar(string var1);
    void bindThis(Conjunction &lBindings, string &recipe_name);
    bool unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings,
                             std::vector< Match > &mapping, int &matched_con_id,
                             HowComplete howcomplete);
    bool unifyWithoutBinding(Conjunction &con2, Conjunction &new_bindings);
    bool unify(Conjunction &parent_bindings, Conjunction &new_bindings);
    void evalSlotVals();
  public:
    std::vector<Conjunction> disjuncts;
};


class Match {
  public:
        // default constructor
    Match(): id1(0), id2(0), match_type(Unlocked) {}
        
  public:
        //another constructor
    Match(std::vector<Atom>::size_type id1,
          std::vector<Atom>::size_type id2): id1(id1), id2(id2) {}

  public:
    void print();
    void print(TLogLevel log_level);
  public:
    std::vector<Atom>::size_type id1;
    std::vector<Atom>::size_type id2;
    MatchType match_type;
};


/* A matching of two conjunctions */
class Matching {
  public:
        // default constructors
    Matching(): permute_id(-1), total_permutes(0),  combine_id(-1), total_combines(0), \
        j(0), x(0) {}
  public:
        // constructor with default indexes based on sizes
    Matching(std::vector<Atom>::size_type size1,
             std::vector<Atom>::size_type size2): permute_id(-1), total_permutes(0), \
        combine_id(-1), total_combines(0), j(0), x(0) {
        for ( std::vector<Atom>::size_type i=0; i < size1; i++) {
            ids1.push_back(i);
        }
        for ( std::vector<Atom>::size_type i=0; i < size2; i++) {
            ids2.push_back(i);
        }
    }
  public:
    bool init(Conjunction &con1, Conjunction &con2);
    bool addAtomToGroup(std::vector<Atom>::size_type id1, string &group_id, Conjunction &con2);
    string getAtomSignature(Atom &atom);
    bool next();
    bool nextPerm();
    void print();
    void print(TLogLevel log_level);
  public:
    /* group matchings based on some slot vals such as type and subtype */
    std::map< string, Matching > groups; 
    std::vector< std::vector<Atom>::size_type > ids1; /* available indexes to match */
    std::vector< std::vector<Atom>::size_type > ids2; /* available indexes to match */
    std::vector< Match > mapping;
    long permute_id;
    long total_permutes; /* total number of permutations */
    long combine_id;
    long total_combines; /* total number of eventual combinations */
    std::vector< std::vector<Atom>::size_type > subset_ids;
    std::vector< std::vector<Atom>::size_type > permute_ids;
    std::vector<Atom>::size_type j;
    std::vector<Atom>::size_type x;
};



/* Goal, action, recipe spec */
class BodyElement {
  public:
        // default constructor
    BodyElement(): forced(false),
        chosen_outcome(-1), unique_within(0), random(false),
        pXmlElement(NULL),
        timeout(DEFAULT_ACTION_TIMEOUT),
        dispatch_time(0.0)
        {
            /* action status-dependent default behaviors*/
        if_completed = "skip_to_next";
        if_aborted = "skip_to_end";
        if_failed = "skip_to_next";
        if_timeout = "skip_to_next";
            /* recipe status-dependent default behaviors */
        if_node_purged = "abort"; /* acceptable values are: abort, wait */
    }
    bool load(TiXmlElement* pElem);
    bool typeCheck();
    void updateFromDefaults(Conjunction &defaults_atoms);
    bool parseActionLevel2_preprocessRandom(TiXmlElement* pElem);
    void assignArgsToRandElements(Args &args);
    void assignReturnArgsToRandElements(Args &return_args);
    bool verifyProbabilities();
    BodyElement derandomize(string recipe_name, unsigned int active_element,
                            History &history);
    void print();
    bool isTimedOut();
  public:
    string element_type; /* assignment, action, goal*/
    string name; /* name if action */
    string recipe_name; /* recipe names or 'any' */
    bool forced; /* forced backchaining even if the goal is true */
    Formula formula; /* goal or assignment*/
    string actor; /* who does the action */
    string action_space; /* which namespace to search action name in */
    string initiator; /* initator of a recipe */
    Args args; /* arguments of an action */
    Args return_args; /* return_args of an action */
    std::vector< BodyElement > random_elements;
    std::vector< double > element_probs;
    int chosen_outcome;
    int unique_within; /* radius of the neighbourhood without repetitions */
    bool random; /* is there a random block inside? (reparse each execution with coin tosses) */
    TiXmlElement* pXmlElement; /* copy of the body element in case it's random */ 
    string if_completed;
    string if_aborted;
    string if_failed;
    string if_timeout;
    string if_node_purged;
    double timeout;
    double dispatch_time;
    string priority;
    std::list<string> backchainables; /* computed list of recipes that
                                       * can potentially
                                       * satisfy this goal element */
};


class Body {
  public:
    // default constructor
    Body() {}
    bool load(TiXmlElement* pElem);
    void print();
    bool typeCheck();
    void updateFromDefaults(Conjunction &defaults_atoms);
  public:
    string order;
    std::vector<BodyElement> elements;
};


class Recipe{

  public:
    //default constructor
    Recipe(): priority(0), ifWhileconditionFailed("skip_to_end"),
        precondUpdated(true) {}

    bool load(TiXmlElement* pElem);
    void print();
    bool typeCheck();
    bool bindTypeAndSubtype();
    void updateFromDefaults(Conjunction &defaults_atoms);
    void updateDefaultsToRecipePreconditionsMap(Conjunction &defaults_atoms);

  public:
    string name;
    int priority;
    Formula precondition;
    Formula whilecondition;
    Formula assignwhile;
    Formula assignpost;
    Body body;
    Conjunction bindings;
    string ifWhileconditionFailed;
    bool precondUpdated; /* flag indicating that the precondition atoms MAY have
                          * been updated since last check. */
};

/**
 * Node of a plan tree. Contains information on the state of the recipe
 * and parent/children pointers.
 * ae_status values are:
 * - pending, when a new node element is in the process of getting ready to be
 *         executed, which seems to be when either
 *         action dispatch is failed,
 *         or goal backchaining is failed,
 *         or when execute node started but before dispatch happened.
 * - completed, when action is returned
 * - executing, during the execution process
 * */
class Node{
  public:
        // default constructor
        /* node_id is the node counter */
    Node(): node_id(node_counter), active_element(-1), ae_action_id(-1),
        ae_status("_NO_VALUE_"), whileconditionFailed(false)  {
            //Conjunction temp_bindings;
            //this->bindings = temp_bindings;
            //this->while_bindings = temp_bindings;
        node_counter++;
    }

  public:
    Node(Recipe &recipe): node_id(node_counter), active_element(-1),
        ae_action_id(-1), ae_status("_NO_VALUE_"),
        bodyElementDescriptionsHaveBeenSet(false),
        whileconditionFailed(false) {
        body = recipe.body;
        assignpost = recipe.assignpost;
        ifWhileconditionFailed = recipe.ifWhileconditionFailed;
            //Conjunction temp_bindings;
            //this->bindings = temp_bindings;
            //this->while_bindings = temp_bindings;
        node_counter++;
    }

        void setWhileBindings(tree<Node>::iterator root);
        void print();
        void print(TLogLevel log_level);
  public:
        string recipe_name;
        int node_id;                            /* a unique node id */
        int active_element;                     /* index of active body element */
        int ae_action_id;                       /* unique action_id for an active
                                                 * element */
        string ae_status;                       /* status of the active element:
                                                 * pending, executing, completed */
        Conjunction bindings;
        Formula whilecondition;
        Body body;
        Formula assignpost;
        vector<string> statuses;               /* statuses of body elements */
        bool bodyElementDescriptionsHaveBeenSet;
        vector<string> element_descriptions;   /* descriptions of elements for textUI/
                                                * Should be updated to current bindings,
                                                * which may change as recipe is being
                                                * executed */
        bool whileconditionFailed;            /* true if  whilecondition has ever failed
                                                * for this node, or there are ghost atoms
                                                * in its precondition */
        string ifWhileconditionFailed;

  private:
        static int node_counter;
};

inline bool operator==(const Node &node1, const Node &node2) {
    return node1.node_id == node2.node_id;
};


/**
 * Plan is a tree of recipes and actions. This is a Plan Tree in
 * Collagene terminology.
 * */
class PlanTree{
  public:
        // default constructor
    PlanTree() { }
    
    void print();
    void print(TLogLevel log_level);
    int nodeChildNameCount(tree<Node>::iterator &parent, string &childName);
    tree<Node>::iterator findNodeGivenActionId(int action_id);
    tree<Node>::post_order_iterator pruneNode(tree<Node>::post_order_iterator &aNode);
  public:
    tree<Node> tr;
};

/**
 * Plan is a tree of recipes and actions. This is a Plan Tree in
 * Collagene terminology.
 * */
class FocusStack{
  public:
        // default constructor
    FocusStack() { }

  public:
    std::list<Node> nodes;
};


/**
 * RecipeHistory object includes all the structures that record recipe history
 * */
class BodyElementHistory {
  public:
        // default constructor
    BodyElementHistory() { }

  public:
    std::vector< int > outcome_age;  /* how many calls ago each outcome was chosen */
    std::deque < int > outcome_history;           /* the list of last outcomes */
};


/**
 * RecipeHistory object includes all the structures that record recipe history
 * */
class RecipeHistory {
  public:
        // default constructor
    RecipeHistory() { }

  public:
    std::vector<BodyElementHistory> bodyElementHistories;
};



/**
 * History object includes all the structures that record interaction history
 * */
class History {
  public:
        // default constructor
    History() { }

  public:
    std::map<std::string, RecipeHistory> recipeHistories;
};




class InteractionManager{
  public:
        // default constructor
    InteractionManager(): pending_answer_action_id(0), user_counter(0),
        atom_counter(0), action_counter(0), timing_dump_period(10) { }

  public:
    string getRecipeDir();
    string getActionDir();
        /** loading and initialization */
    bool load_init(string init_filename);
    bool load_DefaultAtoms(string default_atom_filename);
    bool loadRecipeFile(string recipe_filename);
    bool loadActionFile(string recipe_filename);
    bool typeCheckRecipes();
    void printRecipes();
    void printTriggerables();
    void setGlobals();

        /* Accessors */
    Conjunction* getGlobalBindings();
    
        /* action methods that need to be defined above of actions
         * since actions don't know about conjunctions, for example */
    void makeAtomFromArgs(Atom &an_atom, Args &an_args);
    void evalActionDatablocks(Action &an_action);
    void bindActionArg(std::list<ArgSlot>::iterator &arg_it, Args &args,
                       Conjunction &bindings);
    void bindActionArgs(Action &an_action, Args &args, Conjunction &bindings);

        /* utility methods that need to be defined as IM methods
         * because they may need to access to the IM global variables */


        /** IM: methods for plan tree transformations */
    bool tryPushTriggerables();
    bool tryPushTriggerable(TriggerableRecord &a_trig);

        /** preprocessing */
    void insertRecipeIntoPrioritySortedList(std::list<string> &recipe_list,
                                            Recipe &aRecipe);
    void findBackchainablesForAGoal(BodyElement &element, std::list<string>
                                    &candidateRecipes, string goalRecipeName);
    void preprocessDefaultsToPreconditionsMap();
    void preprocessBackchainables();
    bool preprocess();
    bool initialize();

    /** atom binding transformations */
    bool checkPreconditionGivenFormula(Formula aprecond, tree<Node>::iterator
                                       &parent,
                                       Conjunction &new_bindings);
    bool checkPreconditionGivenRecipe(string &name, tree<Node>::iterator &parent,
                                      Conjunction &new_bindings);
    void replaceAtomBinding(Atom &new_atom); /* completelet replace root of
                                              * ptree bindings for the atom*/
    void pushAtomBinding(Atom &new_atom);
    void updateGlobalBindings();
    bool updateAtomBinding(Atom &new_atom); /* update root of ptree bindings
                                             * for the atom*/
    bool updateRootBindingAtomMatchingType(Atom &new_atom, bool create = false);
    bool updateRootBindingAtomMatchingTypeId(Atom &new_atom);
    bool updateRootBindingAtomMatchAnySlots(Atom &new_atom, string slot_name1);
    bool updateRootBindingAtomMatchingSlotList(Atom &atom,
                                               std::list<string> &slotList);
    void updateRootBindingAtomMatchingUniqueMask(Atom &new_atom);
    void updateRootBindingFromInputQueue();
    bool checkWhilecondition(tree<Node>::post_order_iterator &node,
                             tree<Node>::iterator &parent);
    void checkPruneWhileConditions();
    bool bindWhilecondition(Node &node);
    /** plan tree execution */
    ControlFlowMove decideFailedWhileconditionNodeControlFlowMove
        (tree<Node>::iterator_base &node);
    ControlFlowMove decideNodeControlFlowMove(tree<Node>::iterator_base &node);
    void executeNodeAssignmentBlock(tree<Node>::iterator_base &node);
    void executeNode(tree<Node>::post_order_iterator &node);
    bool executeAssignment(BodyElement &element1, Conjunction &bindings);
    bool executeAssignPost(Formula form1, Conjunction &bindings);
    void processActionCompletionStatus(ActionStatus &astat);
    bool processReturnArgs(Args &returnArgs, Conjunction &bindings);
    void tryExecuteNodes();
    void garbageCollectDeletedAtoms();
    void doTick();
    /** backchaining */
    bool tryBackchainOnGoal(BodyElement &element1,
                            tree<Node>::iterator_base &node, string &status);
    bool tryBackchainOnGoalWithRecipe(Recipe &recipe, string &goalName,
                                      Formula &goalFormula1,
                                      Formula &goalFormula2,
                                      tree<Node>::iterator_base &node);
    void tryBackchain();
    /** task execution */
    bool dispatchAction(BodyElement &element1, Conjunction &bindings,
                        int action_id, string recipe_name, int active_element);
    bool dispatchActionCommandAbort(int action_id, string &actor);
    bool executeUserAction(BodyElement &element1, Conjunction &bindings,
                           int action_id);
    bool checkForGhostAtoms(Node &node);
    void printSettings();
    //void callbackInviteString(blackboardEntry<char*> &invite_str);

    /* mark recipe precondUpdated as updated. */

    void markRecipePrecondUpdated(list<string> &recipe_names);
    void givenAtomMarkRecipePrecondUpdated(Atom &atom);

  public:
    string init_file_name;
    string script_path;
    string default_atoms_file_name;
  	/* next 2 lines is a temporary crap to account for robocept_interact legacy*/
    int pending_answer_action_id;
    string pending_answer;
        /* counters */
    int user_counter;
    int atom_counter;
    int action_counter;
    /** collection of triggerables from recipes and recipes themselves **/
    std::list<TriggerableRecord> triggerables_rec;
    std::map<string, Recipe> recipes;
    std::map<string, Action> actions;
    std::list<string> recipe_names;
    /** current state of the IM **/
    PlanTree ptree;  		/** plan tree **/
    FocusStack fstack; 		/** focus stack **/
    Conjunction globals; 	/** global variables **/
    Conjunction default_atoms;  /** default slot fillers for atoms **/ 
    std::map<int, BodyElement> pending_user_actions;
    History history;            /** all history structures **/
    std::list<Atom> input_queue; /** queue of input Atoms **/
    std::list<Action> output_queue; /** queue of output Actions **/
    std::list<Node> new_nodes_for_ui; /** this is for the UI, such as TextUI **/
        /** params **/
    std::map<string, std::list<double> > subcycle_timings; /** timings of subcycles **/
    int timing_dump_counter;
    int timing_dump_period;
    double timer_period_microsec;
    bool optimizedPreconditionCheck; /* do selective precondition check based on updated
                                      * slota of global recipes or not */
};

#endif // IWAKI_H
