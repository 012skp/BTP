/*Core header file containing definations
  of controller,switch and links
*/

#define __CORE_H__
#include<bits/stdc++.h>
#include<sys/time.h>
#include<unistd.h>
#include<thread>
#include<mutex>
using namespace std;

struct timeval emulation_start_time;
struct timeval latest_packet_seen;
mutex lps_lock;     // latest_packet_seen lock
mutex atomic_lock;  // to execute a set of inst. atomically.
bool emulation_done;
int max_delay;

struct dist_vector{
  string dst;
  int hop_count;
  int linkid;
};

struct dist_vector_table{
  string switchname;
  int version; // version of forwarding_table being broadcasted.
  vector<dist_vector> row;
};


enum pkt_type{CONTROL,NORMAL,ROUTING,PACKET_IN,PACKET_OUT,PKT_DROP,ACK};

// only CONTROL packets will have packet subtypes
enum pkt_subtype{LOAD_MIRGRATION,ROLE_REQ,ROLE_REP,LOAD_BROADCAST,NEW_THRESHOLD};
/*ROLE_REQ sent by target(lowest_loaded) controller to switch.
  ROLE_REP sent by switch to target controller to information
  load migration successful.
*/

struct dist_vector_table;

struct Packet{
  long long packetid  = 0;
  pkt_type type;
  pkt_subtype subtype;
  string src;
  string dst;
  int size;                              // pkt != 0 for NORMAL data packets
  void *data;
  // start time will be set by the element puttig packets on the link
  struct timeval start_time;
};


struct Controller{
  string own_name;
  int min_processing_time = 1000;             // processing_time in microsecond.
  int max_processing_time = 2000;
  int avg_processing_time = (min_processing_time+max_processing_time)/2;
  int current_load = 0;                       // avg_processing_time*q.size()
  int load_informed = 0;
  int allowed_load_deviation = 100;
  int base_threshold = 1000000;               // 1 second
  int current_threshold = base_threshold;
  int linkid ;                                // link id through which it is connected
  queue<Packet> q;                            // packets in queue to be processed
  mutex *qlock = NULL;
  int max_queue_size = 1000;
  map<string,int> switch_pkt_count;           // count of packets in queue by different switches
  mutex *switch_pkt_count_lock = NULL;
  vector<int> load_collections;               // load information of other controllers.
  mutex *lclock = NULL;
  double alpha = 0.70;                        // LB if lowest_load < alpha*current_threshold
  int max_load_gap;                               // max_load_gap allowed between CT and (heighest load > BT).
};


struct Link{
  string own_name;
  long long  delay = 2000;                    // in  microsecond
  long long bandwith;                         // in Kbps
  string src;
  string dst;
  queue<Packet> q;
  mutex *qlock = NULL;
  bool special = false;                               // if src is any switch and dst is controller.
  vector<pair<double,string> > packet_drop;
};


struct Switch{
  string own_name;
  queue<Packet> q;
  mutex *qlock = NULL;
  int max_queue_size = 50000;
  int controllerid;                   // the controller controlling the switch.
  vector<int> connected_links;

  // Initialise forwarding_table and
  // my dist_vector_table using connected_links

  map<string,int> forwarding_table;   // map of (dst,linkid)


  dist_vector_table my_dvt;
  mutex *my_dvt_lock = NULL;
  /*If there is any change in my_dvt
    update forwading_table and broadcast it.
  */
  vector<int> dvt_version;

  map<long long int, Packet> buffer;
  /*In case of flow table miss packets
    from processing thread is kept in this
    'buffer' and packet information is sent
    to controller. Controller replies what
    to do with the packet.
  */


};


vector<Switch> switches;
vector<Link> links;
vector<Controller> controllers;


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


// Returns time difference in microsecond.
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

// Returns time elapsed from starting of program.
double current_time(){
  struct timeval t;
  gettimeofday(&t,NULL);
  long usec = time_diff(t,emulation_start_time);
  double time_elapsed = (1.0*usec)/1000000;
  return time_elapsed;
}
