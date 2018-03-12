// Core header file containing definations
#define __CORE_H__
#include<bits/stdc++.h>
#include<sys/time.h>
#include<unistd.h>
#include<thread>
using namespace std;
enum pkt_type{CONTROL,NORMAL,ROUTING,MESSAGE_IN,MESSAGE_OUT,PKT_DROP,ACK};

// only CONTROL packets will have packet subtypes
enum pkt_subtype{ROLE_REQ,ROLE_REP,ROLE_END,LOAD_BROADCAST,NEW_THRESHOLD};

struct Packet{
  long long packet_id;
  pkt_type type;
  pkt_subtype subtype;
  string src;
  string dst;
  int size;
  void *data;
  // start time will be set by the element puttig packets on the link
  struct timeval start;
};


struct Controller{
  string own_name;
  int avg_service_rate;
  int current_load = 0;
  int base_threshold = 1000;                  // 1000 milli second
  int current_threshold = base_threshold;
  string linkname;                            // link name to which it is connected
  queue<Packet> q;                            // packets in queue to be processed
  int queue_size = 10000;
  map<string,int> switch_pkt_count;
};


struct Link{
  string own_name;
  double delay;          // in milli second
  double bandwith;       // in Mbps
  string src;
  string dst;
  queue<Packet> q;
  bool special;       // if src is any switch and dst is controller.
};


struct Switch{
  string own_name;
  queue<Packet> q;
  int queue_size;
  string master_controller_name;  // the controller controlling the switch.
  bool direct;                    // set to true if there is a direct link to controller.
  vector<Link> links;             // links that switch is connected to.
};


vector<Switch> switches;
vector<Link> links;
vector<Controller> controllers;



struct dist_vector{
  string dst;
  int hop_count;
  string linkname;
};

struct forwarding_table{
  string switchname;
  int version; // version of forwading_table being broadcasted.
  vector<dist_vector> row;
};

// data for different pkt_types
/* NORMAL => NULL */

/* ROUTING => vector<dist_vector>*/

/* MESSAGE_IN, MESSAGE_OUT => NULL */

/* CONTROL.ROLE_[REQ|REP|END] => NULL */
/* CONTROL.[LOAD_BROADCAST|NEW_THRESHOLD] => integer */


int getid(string str){
  int idx = 0;
  while(!isdigit(str[idx])) idx++;
  return atoi(str.c_str()+idx);
}


long time_diff(timeval t2, timeval t1){
  struct timeval diff;
  long d = t2.tv_sec-t1.tv_sec;
  if(t2.tv_usec < t1.tv_usec)
    return d*1000000+(t2.tv_usec - t1.tv_usec);
    else return (d-1)*1000000 + (t2.tv_usec+1000000-t1.tv_usec);
}
