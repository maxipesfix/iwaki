#include <iostream>
#include "queueio.h"
#include <string>
//#include <Python.h>
#include <csignal>
#include <algorithm>
#include <vector>

#include <sys/time.h>
#include <unistd.h>

//#include "BoostIpc.h"

using namespace std;
using std::string;
using std::iostream;


/* Just gets current system time in milliseconds */
double getSystemTimeMSec(){

  struct timeval cur_time;
  double cur_time_msec, cur_time_msec_usec;

  /* get current time */
  gettimeofday(&cur_time, NULL);
  cur_time_msec = (double)cur_time.tv_sec;
  cur_time_msec_usec = (double)cur_time.tv_usec;
  cur_time_msec = (cur_time_msec*1000.0 + cur_time_msec_usec/1000.0) + 0.5;
  return cur_time_msec;
}


/*********** TimeYSegment methods *********/

void TimeYSegment::truncateSegUpto(unsigned long upto)
{
  if ((this->left.first < upto) && (this->right.first > upto))
  {
    this->left.second = this->left.second + \
        ((upto-this->left.first)/(this->right.first - this->left.first))*\
        (this->right.second - this->left.second);
    this->left.first = upto;
  }
  return;
}


double TimeYSegment::areaTo(unsigned long to)
{
  double area = 0.0;
  if ((this->left.first <= to) && (to <= this->right.first)) {
    area = (((double)(to - this->left.first))*\
        (this->left.second + this->right.second + \
        (this->left.second - this->right.second)*\
        ((double)(this->right.first - to))/\
        ((double)(this->right.first - this->left.first)))/2.0)/SCALING;
  }
  return area;
}


double TimeYSegment::areaFrom(unsigned long from)
{
  double area = 0.0;
  if ((this->left.first <= from) && (from <= this->right.first)) {
    area = (((double)(this->right.first - from))*\
        (2.0*this->right.second + \
        (this->left.second - this->right.second)*\
        ((double)(this->right.first - from))/\
        ((double)(this->right.first - this->left.first)))/2.0)/SCALING;
  }
  return area;
}

double TimeYSegment::area()
{
  double area = 0.0;
  area = (((double)(this->right.first - this->left.first))*\
      (this->left.second + this->right.second)/2.0)/SCALING;
  return area;
}


double TimeYSegment::areaFromTo(unsigned long from, unsigned long to)
{
  double area = 0.0;
  if ((this->left.first <= from) && (from <= this->right.first) &&\
       (this->left.first <= to) && (to <= this->right.first) )  {
    area = this->area() - this->areaTo(from) - this->areaFrom(to);
       }
       return area;
}

double TimeYSegment::rewardAt(unsigned long at)
{
  if ((this->left.first <= at) && (at <= this->right.first))  {
    return this->left.second + (this->right.second - this->left.second)*\
        ((double)(at - this->left.first))/\
        ((double)(this->right.first - this->left.first));
  } else
  {return 0.0;}
}

/* returns earliest time point within the segment satisfying the condition
of being greater or equal to from and reward below the threshold.
Negative threshold indicates that it should be ignored.
If none do, returns 0 */
unsigned long TimeYSegment::findFirstBelowAfter(unsigned long from, double threshold)
{
  if ((this->right.first <= from) ||
       ((this->left.second > threshold) && (this->right.second >= threshold)\
       && (threshold >=0 )))
  {
    return 0;
  }
  else if ((threshold <= 0) ||\
            ((this->left.second <= threshold) && (this->right.second <= threshold)))
    /* threshold can be ignored */
  {
    return max(this->left.first, from);
  }
  else if ((this->left.second > threshold) && (this->right.second < threshold))
  { /* reward is decreasing */
    unsigned long intercept = this->left.first + \
        (unsigned long)(((double)(this->right.first - this->left.first)) *\
        (this->left.second - threshold)/(this->left.second - this->right.second));
    return max(intercept,  from);
  }
  else if ((this->left.second < threshold) && (this->right.second > threshold))
  { /* reward is increasing */
    unsigned long intercept = this->left.first + \
        (unsigned long)(((double)(this->right.first - this->left.first)) *\
        (this->right.second - threshold)/(this->right.second - this->left.second));
    if (intercept > from)
    {return max(this->left.first, from);}
    else
    {return 0;}
  }
  else
  {
    cout << "Error in TimeYSegment::findFirstBelowAfter. This is empty condition." << endl;
    return 0;
  }
}



/*********** Resource profile methods ********/

void ResourceProfile::print(string format_string)
{
  if (this->profile.empty())
  {
    cout << "Empty." << endl;
  }
  else
  {
    unsigned long base_time = (this->profile.begin())->left.first;
    cout << "Base time: "<< base_time << endl;

    list<TimeYSegmentRes>::iterator iter0, iter = this->profile.begin();
    while (iter != this->profile.end())
    {
      /* row of rewards */
      int j=0;
      iter0=iter;
      while ((iter != this->profile.end())&&(j<10))
      {
        if (iter->left.second < 10.0) {cout << " ";}
        printf("%2.2f", iter->left.second);
        if (iter->right.second < 10.0) {cout << "  ";} else {cout << " ";}
        printf("%2.2f", iter->right.second);
        j++;
        iter++;
        if ((iter != this->profile.end())&&(j<10))
        {
          cout << "|";
        }
      }
      /* row of time values*/
      iter = iter0;
      j=0;
      cout << endl;
      while ((iter != this->profile.end())&&(j<10))
      {
        printf("%5lu", iter->left.first-base_time);
        if (iter->preemptive) {cout << " ";} else {cout << "*";}
        printf("%5lu", iter->right.first-base_time);
        j++;
        iter++;
        if ((iter != this->profile.end())&&(j<10))
        {
          cout << "|";
        }
      }
      /* row of task_id if needed */
      if (format_string.find("task_id") != string::npos)
      {
        iter = iter0;
        j=0;
        cout << endl;
        while ((iter != this->profile.end())&&(j<10))
        {
          if (iter->task_id.size() <= 11)
          {
            cout << iter->task_id << string((11 - iter->task_id.size()), ' ');
          } else
          {
            cout << iter->task_id.substr(0,11);
          }
          j++;
          iter++;
          if ((iter != this->profile.end())&&(j<10))
          {
            cout << "|";
          }
        }
      }
      cout << endl;
      cout << endl;
    }
  }
}

/*
  Returns the iterator to the first segment containing time point
*/
list<TimeYSegmentRes>::iterator ResourceProfile::getSegmentContainingTime(unsigned long tpoint)
{
  list<TimeYSegmentRes>::iterator iter=this->profile.begin();

  while ((iter != this->profile.end())&&(iter->left.first <= tpoint))
  {
    iter++;
  }
  if (iter != this->profile.begin()) {iter--;}
  /* got the candidate segment */
  if ((iter->left.first <= tpoint)&&(tpoint < iter->right.first)) /* semi-open segment */
  {
    return  iter;
  }
  else
  {
    return this->profile.end();
  }
}

void ResourceProfile::makeProfile(unsigned long stime, unsigned long ftime)
{
  TimeYSegmentRes seg1;
  seg1.preemptive = true;
  seg1.left = make_pair(stime, 0.0);
  seg1.right = make_pair(ftime, 0.0);
  this->profile.clear();
  this->profile.push_back(seg1);
}

/*
  Returns the time of the right edge of the last segment
*/
unsigned long ResourceProfile::getDuration()
{
  if (this->profile.empty())
  {
    return 0;
  }
  else
  {
    return (--this->profile.end())->right.first;
  }
}


void ResourceProfile::padUpto(unsigned long upto)
{
  if (this->getDuration() < upto)
  {
    TimeYSegmentRes seg1;

    seg1.preemptive = true; // everything is for grabs
    seg1.left = make_pair(this->getDuration() ,0.0);
    seg1.right = make_pair(upto, 0.0);
    this->profile.push_back(seg1);
  }
  return;
}

void ResourceProfile::padFrom(unsigned long from)
{
  if (this->startTime() > from)
  {
    TimeYSegmentRes seg1;

    seg1.preemptive = true; // everything is for grabs
    seg1.left = make_pair(from, 0.0);
    seg1.right = make_pair(this->startTime(), 0.0);
    this->profile.push_front(seg1);
  }
  return;
}



void ResourceProfile::truncateRProfUpto(unsigned long upto)
{
  if (this->getDuration() <= upto)
  {
    /* completely outdated, remove */
    this->profile.clear();
  }
  else
  {
    list<TimeYSegmentRes>::iterator iter = this->profile.begin();
    while ((iter != this->profile.end())&&(iter->right.first <= upto))
    {
      /* Outdated segments */
      list<TimeYSegmentRes>::iterator seg_to_remove = iter;
      iter++;
      this->profile.erase(seg_to_remove);
    }
    /* the next segment (if exists) has right edge above upto. if
    its left edge is equal to upto, leave it in peace. Otherwise,
    truncate the segment */
    if ((iter != this->profile.end())&&(iter->left.first < upto))
    {
      /* Truncate the segment */
      iter->truncateSegUpto(upto);
    }
  }
  return;
}

unsigned long ResourceProfile::startTime()
{
  if (!this->profile.empty())
  {
    return this->profile.begin()->left.first;
  }
  else
  {
    cout << "Error in ResourceProfile::startTime: Empty resource" << endl;
    return 0;
  }
}

/* find first time point after the tpoint and below threshold of reward.
   if threshold is specified negative, ignore the reward
   if profile is empty, return 0
   if profile is all blocked or ends before tpoint, return right end (duration)*/
unsigned long ResourceProfile::findFirstBelowAfter(unsigned long tpoint, double threshold)
{
  list<TimeYSegmentRes>::iterator iter=this->profile.begin();

  /* empty profile returns 0 */
  if (this->profile.empty()) {return 0;}
  /* Find the first segment that satisfies the condition */
  while ((iter != this->profile.end())&&\
          ((iter->right.first <= tpoint)||!iter->preemptive||\
          (iter->findFirstBelowAfter(tpoint, threshold)==0)))
  {
    iter++;
  }
  /* if the profile is no good (blocked or ends before tpoint -- return the right end */
  if (iter == this->profile.end())
  {
    iter--;
    unsigned long answer = iter->right.first;
    cout << "About to return end point: " << answer << endl;
    return answer;
  }
  else
  {
    cout << "About to return inner point: " << iter->findFirstBelowAfter(tpoint, threshold) << endl;
    return iter->findFirstBelowAfter(tpoint, threshold);
  }
}


void ResourceProfile::shift(unsigned long shift)
{
  for (list<TimeYSegmentRes>::iterator iter = this->profile.begin();\
       iter!= this->profile.end(); iter++)
  {
    iter->left.first += shift;
    iter->right.first += shift;
  }
  return;
}

/* add the shifted resource profile to this one. pad as necessary. */
void ResourceProfile::addProfile(ResourceProfile *rp2)
{
  if (rp2->profile.empty()) {return;}
  /* if this profile is empty, adding is just copying rp2*/
  if (this->profile.empty())
  {
    /* copy profile */
    this->profile.insert(this->profile.end(), rp2->profile.begin(), rp2->profile.end());
    return;
  }
  /* neither rp1 nor rp2 are empty */
  if (this->startTime() < rp2->startTime())
  {
    /* need to pad rp2 from the beginning */
    rp2->padFrom(this->startTime());
  }
  else if (this->startTime() > rp2->startTime())
  { /* need to pad this profile from the beginning */
    this->padFrom(rp2->startTime());
  }

  if (this->startTime() != rp2->startTime()) {cout << "Error: unaligned profiles" << endl;}
  /* profiles are aligned. No more excuses but to add! */
  list<TimeYSegmentRes>::iterator iter1 = this->profile.begin();
  list<TimeYSegmentRes>::iterator iter2 = rp2->profile.begin();
  ResourceProfile rp;
  unsigned long now = iter1->left.first;

  /* pad if necessary */
  if (this->getDuration() < rp2->getDuration())
  {
    this->padUpto(rp2->getDuration());
  }
  else if (this->getDuration() > rp2->getDuration())
  {
    rp2->padUpto(this->getDuration());
  }

  while (iter1 != this->profile.end())
  {
    TimeYSegmentRes seg1;
    seg1.left.first = now;
    seg1.left.second = iter1->rewardAt(now) + iter2->rewardAt(now);
    seg1.preemptive = (iter1->preemptive && iter2->preemptive);
    if (!iter2->task_id.empty())
    { /* use the second argument's task id for adding a new task
      resource to the queue resource, if it's non-empty id (for fillers etc) */
      seg1.task_id = iter2->task_id;
    } else
    {
      seg1.task_id = iter1->task_id;
    }

    /* find nearest to now end of segment */
    if ((iter1->right.first - now) < (iter2->right.first - now))
    {
      /* iter1's end is closer: advance iter1 */
      now = iter1->right.first;
      seg1.right.first = now;
      seg1.right.second = iter1->right.second + iter2->rewardAt(now);
      iter1++;
    }
    else if ((iter1->right.first - now) > (iter2->right.first - now))
    {
      /* iter2's end is closer: advance iter2 */
      now = iter2->right.first;
      seg1.right.first = now;
      seg1.right.second = iter2->right.second + iter1->rewardAt(now);
      iter2++;
    }
    else /* equidistant from the ends: advance both iter1 and iter2 */
    {
      now = iter2->right.first;
      seg1.right.first = now;
      seg1.right.second = iter2->right.second + iter1->right.second;
      iter1++;
      iter2++;
    }
    rp.profile.push_back(seg1);
  }
  /* replace this->profile with rp */
  this->profile.clear();
  this->profile.insert(this->profile.end(), rp.profile.begin(), rp.profile.end());

  return;
}

/********** ResourceProfileList methods *********/

void ResourceProfileList::print()
{
  for (list<ResourceProfile>::iterator iter = this->profiles.begin(); iter!=this->profiles.end(); iter++)
  {
    iter->print("");
  }
  return;
}

/* Add the profiles in the List object and save the sum as *rp resource profile */
void ResourceProfileList::addProfiles(ResourceProfile *rp)
{
  for (list<ResourceProfile>::iterator iter = this->profiles.begin(); iter!=this->profiles.end(); iter++)
  {
    rp->addProfile(&(*iter));
  }
  return;
}



/********** Task methods *********/

/* Sends a message to the requesting module that task is ready to be performed */
int Task::dispatch()
{
  cout << "Dispatched task id, name: " << id << ", " << name  << endl;
  return 0;
}

/* Makes a sample task resource profiles */
/* TODO: more generic version of this should parse a new task msg to queue and
 * populate the resource profiles for the new task
*/
std::map<string, ResourceProfile> makeTaskResourceProfiles(const std::string &task_name)
{
  std::map<string, ResourceProfile> resources;
  TimeYSegmentRes seg1, seg2;
  ResourceProfile voice, head_gaze, eye_gaze;

  /* make a couple of resource time segments */
  seg1.left = make_pair(0,1.0);
  seg1.right = make_pair(1000,1.0);
  seg1.preemptive = false;
  seg1.task_id = "utterance1";
  seg2.left =  make_pair(1000,1.0);
  seg2.right =  make_pair(2000,1.0);
  seg2.preemptive = false;
  seg2.task_id = "utterance1";

  if (task_name == "utterance") {
    /* push the segments into profiles */
    voice.profile.push_back(seg1);
    voice.profile.push_back(seg2);

    /* allow head and eye gaze to be preempted*/
    seg2.preemptive = true;
    head_gaze.profile.push_back(seg1);
    head_gaze.profile.push_back(seg2);

    eye_gaze.profile.push_back(seg1);
    eye_gaze.profile.push_back(seg2);

    resources["voice"] = voice;
    resources["head_gaze"] = head_gaze;
    resources["eye_gaze"] = eye_gaze;

  } else if (task_name == "nod") {
    /* push the segments into profiles */
    seg1.preemptive = false; /* for the nod, neither of gazes can be preempted */
    seg1.task_id = "nod1";
    head_gaze.profile.push_back(seg1);
    eye_gaze.profile.push_back(seg1);

    resources["head_gaze"] = head_gaze;
    resources["eye_gaze"] = eye_gaze;
  }

  return resources;
}



/********************************************/
/* ArgSlot methods */
/********************************************/
bool ArgSlot::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  //cout << "Processing slot." << endl;

  /* All varslots should have a name */
  if (!pElem->Attribute("name")) {
    FILE_LOG(logERROR) << "arg slot has no name attribute.";
    return false;
  }
  else {
    this->name = pElem->Attribute("name");
  }
  /* type of the variable */
  if (pElem->Attribute("type")) {
    this->type = pElem->Attribute("type");
  }
  /* default value */
  if (pElem->Attribute("default")) {
    this->default_value = pElem->Attribute("default");
  }
  /* value */
  if (pElem->Attribute("value")) {
    this->value = pElem->Attribute("value");
  }
    /* $-var name to bind return value */
  if (pElem->Attribute("var")) {
    this->var = pElem->Attribute("var");
  }
      /* units */
  if (pElem->Attribute("unit")) {
    this->unit = pElem->Attribute("unit");
  }
  // cout << "Slot's name:"<< this->name << ", rel:"<< this->relation
  //   << ", val:"<< this->val << ", var:" << this->var <<endl;
  return loadOkay;
}

void ArgSlot::print() {
    char buffer[100];
    sprintf(buffer, "          %-15.15s %-15.15s %-15.15s %-10.10s %-10.10s %-10.10s",
            this->name.c_str(), this->value.c_str(), this->var.c_str(),
            this->type.c_str(), this->unit.c_str(), this->default_value.c_str());
    FILE_LOG(logINFO) << (string)buffer;
}

void ArgSlot::print(TLogLevel log_level) {
    char buffer[100];
    sprintf(buffer, "          %-15.15s %-15.15s %-15.15s %-10.10s %-10.10s %-10.10s",
            this->name.c_str(), this->value.c_str(), this->var.c_str(),
            this->type.c_str(), this->unit.c_str(), this->default_value.c_str());
    FILE_LOG(log_level) << (string)buffer;
}


void ArgSlot::printWithLabels() {
    FILE_LOG(logINFO) << "            ArgSlot " <<     \
        "name: " << this->name << ", " <<                \
        "type: " << this->type << ", " <<                \
        "value: " << this->value << ", " <<              \
        "var: " << this->var << ", " <<              \
        "unit: " << this->unit << ", " <<                \
        "default: " << this->default_value;
}



/********************************************/
/* Args methods */
/********************************************/
bool Args::load(TiXmlElement* pElem)
{
  bool loadOkay = true;

  FILE_LOG(logDEBUG2) << "Processing args" << endl;

  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();

  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement()) {
      string level1tag = pLevel1Node->Value();
        //cout << "In body, level 1 tag: " << level1tag << endl;
      if (level1tag=="arg") {
          ArgSlot an_arg;
          if (!an_arg.load(pLevel1Node)){
              FILE_LOG(logERROR) << "Could not load arg.";
              return false;
          } else {
              this->push_back(an_arg);
          }
      }
      else  {
          FILE_LOG(logERROR) << "In args block, unknown level 1 tag: " << level1tag;
          return false;
      }
  }
  return loadOkay;
}


void Args::setArgByName(string arg_name, string arg_val) {
    bool found_slot = false;
    for(list<ArgSlot>::iterator argslot1 = this->begin();     \
        argslot1 != this->end(); argslot1++) {
        if (argslot1->name == arg_name) {
            argslot1->value = arg_val;
            found_slot = true;
            break;
        }
    }
    if (!found_slot) {
            /** slot name does not exist yet in this Args, add it */
        ArgSlot new_argslot;
        new_argslot.name = arg_name;
        new_argslot.value = arg_val;
        this->push_back(new_argslot);	
    }
}

/* version that does not add an argslot if it's not present already */

bool Args::trySetArgByName(string arg_name, string arg_val) {
    bool found_slot = false;
    for(list<ArgSlot>::iterator argslot1 = this->begin();     \
        argslot1 != this->end(); argslot1++) {
        if (argslot1->name == arg_name) {
            argslot1->value = arg_val;
            found_slot = true;
            break;
        }
    }
    if (!found_slot) {
        return false;
    } else {
        return true;
    }
}


string Args::getArgValue(string arg_name) const {

    for(list<ArgSlot>::const_iterator argslot1 = this->begin();     \
        argslot1 != this->end(); argslot1++) {
        if (argslot1->name == arg_name) {
            return argslot1->value;
        }
    }
    return "_NOT_FOUND_";
}

/**
 ** Print args
 **/
void Args::print() {
    FILE_LOG(logINFO) << "---args----";
    std::list<ArgSlot>::iterator arg_it = this->begin();
    while (arg_it!=this->end()) {
        arg_it->print();
        arg_it++;
    }
    FILE_LOG(logINFO) << "--end args--";
}


void Args::print(TLogLevel log_level) {
    FILE_LOG(log_level) << "---args----";
    std::list<ArgSlot>::iterator arg_it = this->begin();
    while (arg_it!=this->end()) {
        arg_it->print(log_level);
        arg_it++;
    }
    FILE_LOG(log_level) << "--end args--";
}



/********************************************/
/* Datablock methods */
/********************************************/
bool Datablock::load(TiXmlElement* pElem)
{
  bool loadOkay = true;

  FILE_LOG(logDEBUG2) << "Loading datablock" << endl;

  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();

  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement()) {
      string level1tag = pLevel1Node->Value();
        //cout << "In body, level 1 tag: " << level1tag << endl;
      if (level1tag!="") {
              /* TODO: get textElement*/
          const char* text_c = pLevel1Node->GetText();
          string text_value = text_c?text_c:""; /* cast to std::string */
          std::pair<std::string, std::string> dblockEntry = make_pair(level1tag,text_value);

          FILE_LOG(logDEBUG1) << "Loaded datablock entry: " << level1tag << " : " << text_value;
          this->insert(dblockEntry);

      }
      else  {
          FILE_LOG(logERROR) << "In datablock, empty level 1 tag: " << level1tag;
          return false;
      }
  }
  return loadOkay;
}


/**
 ** Print datablock (which is a map)
 **/
void Datablock::print() {
    FILE_LOG(logINFO) << "-----datablock------";
    std::map<std::string, std::string>::const_iterator dblock_it = this->begin();
    while (dblock_it!=this->end()) {
        FILE_LOG(logINFO) << dblock_it->first << " : " << dblock_it->second;
        dblock_it++;
    }
    FILE_LOG(logINFO) << "-end of a datablock-";
}

void Datablock::print(TLogLevel log_level) {
    FILE_LOG(log_level) << "-----datablock------";
    std::map<std::string, std::string>::const_iterator dblock_it = this->begin();
    while (dblock_it!=this->end()) {
        FILE_LOG(log_level) << dblock_it->first << " : " << dblock_it->second;
        dblock_it++;
    }
    FILE_LOG(log_level) << "-end of a datablock-";
}



/********************************************/
/* Datablocks methods */
/********************************************/


/**
 ** Print list of datablocks
 **/
void Datablocks::print() {
    FILE_LOG(logINFO) << "------datablocks----";
    std::list<Datablock>::iterator dblock_it = this->begin();
    while (dblock_it!=this->end()) {
        dblock_it->print();
        dblock_it++;
    }
    FILE_LOG(logINFO) << "--end of datablocks--";
}

void Datablocks::print(TLogLevel log_level) {
    FILE_LOG(log_level) << "-----datablocks----";
    std::list<Datablock>::iterator dblock_it = this->begin();
    while (dblock_it!=this->end()) {
        dblock_it->print(log_level);
        dblock_it++;
    }
    FILE_LOG(log_level) << "--end of datablocks--";
}



/********************************************/
/* Action methods */
/********************************************/
bool Action::load(TiXmlElement* pElem)
{
  bool loadOkay = true;
  if (!pElem->Attribute("name")) {
    FILE_LOG(logERROR)  << "Error: Name missing in action.";
    return false;
  }
  this->name = pElem->Attribute("name");

  if (pElem->Attribute("actor")) {
      this->actor = pElem->Attribute("actor");
  }

  if (pElem->Attribute("priority")) {
      this->priority = pElem->Attribute("priority");
  }

  FILE_LOG(logINFO)  << "Loading action: \'" << this->name << "\'";
  FILE_LOG(logINFO)  << "... which is to be executed by: " << this->actor;
  
  TiXmlHandle hRoot=TiXmlHandle(pElem);
  TiXmlElement* pLevel1Node=hRoot.FirstChildElement().Element();

  for(; pLevel1Node; pLevel1Node=pLevel1Node->NextSiblingElement()) {
    string level1tag = pLevel1Node->Value();
    FILE_LOG(logDEBUG2) << "Level 1 tag: " << level1tag << endl;

    if (level1tag=="roboml:args") {
        if (!this->args.load(pLevel1Node)){
            FILE_LOG(logERROR) << "Could not load args from action: " << this->name;
            return false;
        } 
    } else if (level1tag=="roboml:return_args") {
        if (!this->return_args.load(pLevel1Node)){
            FILE_LOG(logERROR) << "Could not load return_args from action: " << this->name;
            return false;
        }
    }
    else if (level1tag=="roboml:datablock") {
        Datablock aDblock;
        if (!aDblock.load(pLevel1Node)) {
            FILE_LOG(logERROR) << "Could not load datablock from action: " << this->name;
            return false;
        }
            /* add the new datablock to the list */
        this->datablocks.push_back(aDblock);
    } else {
        FILE_LOG(logERROR) << "Unknown level 1 tag: " << level1tag;
        return true;
    }

  }
  return loadOkay;
}


/**
 ** Print contents of the action
 **/
void Action::print() {
    	FILE_LOG(logINFO) << "----------------Action---------------------------";
	FILE_LOG(logINFO) << "* Action: " << this->name;
        FILE_LOG(logINFO) << "*    id: " << this->id;
        FILE_LOG(logINFO) << "*    actor: " << this->actor;
        FILE_LOG(logINFO) << "*    priority: " << this->priority;
	this->args.print();
        this->return_args.print();
        this->datablocks.print();
            //TODO task/constraints printing
	FILE_LOG(logINFO) << "------------------End of an Action---------------";
}

void Action::print(TLogLevel log_level) {
        FILE_LOG(log_level) << "----------------Action---------------------------";
	FILE_LOG(log_level) << "* Action: " << this->name;
        FILE_LOG(log_level) << "*    id: " << this->id;
        FILE_LOG(log_level) << "*    actor: " << this->actor;
        FILE_LOG(log_level) << "*    priority: " << this->priority;
	this->args.print(log_level);
        this->return_args.print(log_level);
        this->datablocks.print(log_level);
            //TODO task/constraints printing
	FILE_LOG(log_level) << "------------------End of an Action---------------";
}



/**
 ** Print contents of an ActionStatus
 **/
void ActionStatus::print(TLogLevel log_level) {
	FILE_LOG(log_level) << "* ActionStatus: " << this->status;
        FILE_LOG(log_level) << "*    action id: " << this->action_id;
        FILE_LOG(log_level) << "*     executor: " << this->executor;
	this->return_args.print(log_level);
}
