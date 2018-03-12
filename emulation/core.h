/*Core header file containing definations
  of controller,switch and links
*/

#define __CORE_H__
#include<bits/stdc++.h>
#include<sys/time.h>
#include<unistd.h>
#include<thread>
using namespace std;
enum pkt_type{CONTROL,NORMAL,ROUTING,PACKET_IN,PACKET_OUT,PKT_DROP,ACK};

// only CONTROL packets will have packet subtypes
enum pkt_subtype{LOAD_MIRGRATION,ROLE_REQ,ROLE_REP,LOAD_BROADCAST,NEW_THRESHOLD};
/*ROLE_REQ sent by target(lowest_loaded) controller to switch.
  ROLE_REP sent by switch to target controller to information
  load migration successful.
*/

struct Packet{
  long long packetid;
  pkt_type type;
  pkt_subtype subtype;
  string src;
  string dst;
  int size;                              // pkt != 0 for NORMAL data packets
  void *data;
  // start time will be set by the element puttig packets on the link
  struct timeval start;
};


struct Controller{
  string own_name;
  int min_processing_time = 1000;             // processing_time in microsecond.
  int max_processing_time = 2000;
  int avg_processing_time = (min_processing_time+max_processing_time)/2;
  int current_load = 0;                       // avg_processing_time*q.size()
  int load_informed = 0;
  int allowed_load_deviation = 100;
  int base_threshold = 1000000;               // 1E6 microsecond
  int current_threshold = base_threshold;
  int linkid ;                                // link id through which it is connected
  queue<Packet> q;                            // packets in queue to be processed
  int max_queue_size = 10000;
  map<string,int> switch_pkt_count;           // count of packets in queue by different switches
  vector<int> load_collections;               // load information of other controllers.
  double alpha = 0.70;                        // LB if lowest_load < alpha*current_threshold
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
  int max_queue_size;
  int controllerid;              // the controller controlling the switch.
  bool direct;                    // set to true if there is a direct link to controller.
  int direct_controllerid;       // set to -1 if direct is false.
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
  int counter;
};

struct broadcast_data{
  int *data;
  int counter;
};

// data for different pkt_types
/* NORMAL => NULL */

/* ROUTING => forwarding_table */
/* LOAD_BROADCAST => broadcast_data, data = load*/
/* NEW_THRESHOLD  => broadcast_data, data = new_threshold*/
/* LOAD_MIRGRATION => int */

/* PACKET_IN, PACKET_OUT => NULL */

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

int random(int low, int high){
  int diff = high-low;
  int r = rand()%(diff+1);
  return low+r;
}
