/*
ResourceProfile, Task and related classes for interfacing with the queue.
*/
#ifndef QUEUEIO_H
#define QUEUEIO_H

// maximum horizon for scheduling, ms
#ifndef Q_HORIZON
#define Q_HORIZON 100000
#endif

// scaling of reward w.r.t. time ticks
#ifndef SCALING
#define SCALING 1000
#endif

/* extra paddig for queue end time, 1 sec for now. This is the latest we expect to
 * schedule the task (relatively to requested_time) when the queue is free */
#ifndef Q_PADDING
#define Q_PADDING 1000
#endif

#include <iostream>
#include <utility>

// #include "console.h"
#include "tinyxml.h"
#include "log.h"
#include <string>
#include <map>
#include <list>


#include <boost/serialization/base_object.hpp> // May not need this.
#include <boost/serialization/utility.hpp> // Serialize pair.
#include <boost/serialization/list.hpp>
#include <boost/serialization/string.hpp>


/* moved to imcore.h 
   #define ACTION_DISPATCH_BB_VAR "action_dispatch"
   #define ACTION_STATUS_BB_VAR "action_status"

   #define ACTION_TIMING_BB_VAR "action_timing"
   #define ACTION_TIMING_RESPONSE_BB_VAR "action_timing_response"
   #define ACTION_TIMING_RESPONSE_FORMAT "{int, float}"
*/

#define DEFAULT_ACTION_TIMEOUT -1.0
#define DEFAULT_ACTION_PRIORITY "0"

/* force obsolete */
/*
  #define INVITE_HALA_STRING "invite_hala_string"
  #define INVITE_HALA_FORMAT "string"
  #define OUTPUT_HALA_STRING "output_hala_string"
  #define OUTPUT_HALA_FORMAT "string"
*/

double getSystemTimeMSec();

/* TimeYPair is a pair of values (time, val) for the reward profile descriptions */

typedef std::pair<unsigned long, double> TimeYPair;

class TimeYSegment {
  public:
    TimeYSegment(): left(std::make_pair(0,0.0)), right(std::make_pair(0,0.0)) { }

    void truncateSegUpto(unsigned long upto);
    double areaTo(unsigned long to);
    double areaFrom(unsigned long from);
    double areaFromTo(unsigned long from, unsigned long to);
    double area();
    double rewardAt(unsigned long at);
    unsigned long findFirstBelowAfter(unsigned long from, double reward);
  public:
    TimeYPair left;
    TimeYPair right;

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
      ar & left;
      ar & right;
    }
};

class TimeYSegmentRes : public TimeYSegment {
  public:
    //default constructor
    TimeYSegmentRes(): preemptive(true) { }
  public:
    bool preemptive; /* can be withdrawn from the task and used by someone else */
    std::string task_id;
  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
      ar & boost::serialization::base_object<TimeYSegment>(*this);
      ar & preemptive;
      ar & task_id;
    }
};

/*
Resource profile is a class describing a piecewise linear function of time
that defined a reward received from the resource under the present schedule
of tasks on the queue. For times when the resource is interruptable the value of
reward is finite, otherwise it's infinite positive.
*/
class ResourceProfile {
  public:
     //default constructor
    ResourceProfile() { }
    void makeProfile(unsigned long stime, unsigned long ftime);
    /*compute the end time of the last segment */
    unsigned long getDuration();
    unsigned long startTime();
    unsigned long findFirstBelowAfter(unsigned long stime1, double threshold);
    void padUpto(unsigned long upto);
    void padFrom(unsigned long from);
    void truncateRProfUpto(unsigned long upto);
    void print(std::string format_string);
    void shift(unsigned long shift);
    void addProfile(ResourceProfile *rp);
    std::list<TimeYSegmentRes>::iterator getSegmentContainingTime(unsigned long tpoint);
  public:
    std::list<TimeYSegmentRes> profile;
  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
      ar & profile;
    }
};


class ResourceProfileList {
  public:
     //default constructor
    ResourceProfileList() { }
    void print();
    void addProfiles(ResourceProfile *rp);
  public:
    std::list<ResourceProfile> profiles;
};


class Task {
  friend bool operator<=(const Task&, const Task&);

  public:
    Task(const std::string &the_name): name(the_name), requested_time(0),\
        peak_width(0), scheduled_time(0), reward_threshold(-1.0) { }
    // default constructor
        Task():  requested_time(0), peak_width(0), scheduled_time(0),\
            reward_threshold(-1.0)  { }

  public:
    std::string name; /* just for an information purposes */
    std::string requester;
    std::string id;
    std::map<std::string, ResourceProfile> profiles;
    unsigned long requested_time; /* absolute time in ms */
    unsigned long peak_width; /* width of the reward peak in ms*/
    unsigned long scheduled_time;
    double reward_threshold;

  public:
    int dispatch();

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
            // ar & boost::serialization::base_object<std::map<std::string, ResourceProfile> > profiles;
        ar & profiles;
        ar & name;
        ar & requester;
        ar & id;
    }
};

 /* This defines the partial(?) order on tasks*/

inline bool operator<=(const Task &lhTask, const Task &rhTask)
{
    // must be made a friend of Task
    // TODO: will access the Task network and figure out the order.
  return true;
}

/* this is a temp function to build a sample resource profile map */
std::map<std::string, ResourceProfile> makeTaskResourceProfiles(const std::string &task_name);

/***********************************************
 *
 * Actions
 *
 **********************************************/

class ArgSlot {
  public:
    // default constructor
    ArgSlot(): type("string") {}

  public:
    bool load(TiXmlElement* pElem);
    void print();
    void print(TLogLevel log_level);
    void printWithLabels();

  public:
    std::string name;
    std::string type;
    std::string default_value;
    std::string value;
    std::string unit;
    std::string var; /* used to reference $-var in return_args of an action */

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
      ar & name;
      ar & type;
      ar & default_value;
      ar & value;
      ar & unit;
      ar & var;
    }
};

class Args : public std::list<ArgSlot> {
  public:
    Args() {};

  public:
    bool load(TiXmlElement* pElem);
    void setArgByName(std::string arg_name, std::string arg_val);
    std::string getArgValue(std::string arg_name) const;
    bool trySetArgByName(std::string arg_name, std::string arg_val);
    void print();
    void print(TLogLevel log_level);

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object< std::list<ArgSlot> >(*this);
    }
};




class Datablock : public std::map<std::string, std::string> {
  public:
    Datablock() {};

  public:
    bool load(TiXmlElement* pElem);
    void print();
    void print(TLogLevel log_level);

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object< std::map<std::string, std::string> >(*this);
    }
};




class Datablocks : public std::list<Datablock> {
  public:
    Datablocks() {};

  public:
    void print();
    void print(TLogLevel log_level);

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object< std::list<Datablock> >(*this);
    }
};



class Action{

  public:
        //default constructor
    Action(): id(0), timeout(DEFAULT_ACTION_TIMEOUT) {}
  public:
    Action(int provided_id): id(provided_id), timeout(DEFAULT_ACTION_TIMEOUT), priority(DEFAULT_ACTION_PRIORITY) {}

  public:
    bool load(TiXmlElement* pElem);
    void print();
    void print(TLogLevel log_level);

  public:
    std::string name;
    std::string actor;
    int id;
    Args args;
    Args return_args;
    double timeout;
    std::string if_timeout;
    std::string priority;                             /* a priority class name interpreted
                                                       * by executors */
    
    Datablocks datablocks;

    Task task; /* the resource profile representation of the action
                * suitable for scheduling on the queue */

        /* TODO: The rest of the member variables come here
         * including the resource profile-like representations like segments
         * and constraints */

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
        ar & name;
        ar & actor;
        ar & args;
        ar & return_args;
        ar & datablocks;
        ar & id;
        ar & priority;
        ar & if_timeout;
        ar & timeout;
        ar & task;
    }
};


class ActionStatus {
  public:
    // default constructor
    ActionStatus() {}

  public:
    void print(TLogLevel log_level);

    
  public:
    int action_id;
    std::string status;
    std::string executor;
    Args return_args;

  private:
    friend class boost::serialization::access;
    template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
    {
      ar & action_id;
      ar & status;
      ar & executor;
      ar & return_args;
    }
};



#endif //QUEUEIO_H
