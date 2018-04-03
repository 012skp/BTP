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



#define mp make_pair
#define pb push_back


struct timeval emulation_start_time;

struct dist_vector{
  string dst;
  int hop_count;
  int linkid;
};

struct dist_vector_table{
  string src;
  int version; // version of forwarding_table being broadcasted.
  vector<dist_vector> row;
};


enum pkt_type{CONTROL,NORMAL,ROUTING,PACKET_IN,PACKET_OUT,PKT_DROP,ACK};

string TYPE(int i){
  if(i==CONTROL) return "CONTROL";
  if(i==NORMAL) return "NORMAL";
  if(i==ROUTING) return "ROUTING";
  if(i==PACKET_IN) return "PACKET_IN";
  if(i==PACKET_OUT) return "PACKET_OUT";
  return "";
}


// only CONTROL packets will have packet subtypes
enum pkt_subtype{LOAD_MIGRATION,LOAD_MIGRATION_ACK,ROLE_REQ,ROLE_REQ_ACK,LOAD_BROADCAST,NEW_THRESHOLD};
/*ROLE_REQ sent by target(lowest_loaded) controller to switch.
  ROLE_REP sent by switch to target controller to information
  load migration successful.
*/

string SUBTYPE(int i){
  if(i==LOAD_MIGRATION) return "LOAD_MIGRATION";
  if(i==ROLE_REQ) return "ROLE_REQ";
  if(i==ROLE_REQ_ACK) return "ROLE_REQ_ACK";
  if(i==LOAD_BROADCAST) return "LOAD_BROADCAST";
  if(i==NEW_THRESHOLD) return "NEW_THRESHOLD";
  if(i==LOAD_MIGRATION_ACK) return "LOAD_MIGRATION_ACK";
  return "";
}


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
  int avg_processing_time = 1000;             // avg processing time;

  int current_load = 0;                       // packets arriving per second.

  // count of pkts in per second.
  int pkt_count =0;
  mutex *pkt_count_lock = NULL;

  int load_informed = 0;
  int base_threshold = 800;                   // no of packets per second
  int allowed_load_deviation = 0.1*base_threshold;

  int current_threshold = base_threshold;
  mutex *current_threshold_lock = NULL;

  bool load_migration_in_process = false;
  struct timeval load_migrated_time;          // waiting time after load migration = 1 second.


  queue<Packet> q;                            // packets in queue to be processed
  mutex *qlock = NULL;
  int max_queue_size = 500;

  map<string,int> switch_pkt_count;           // count of packets in queue by different switches
  mutex *switch_pkt_count_lock = NULL;

  map<string,int> switch_load;

  vector<int> load_collections;               // load information of other controllers.
  mutex *lc_lock = NULL;

  double alpha = 0.70;                        // LB if lowest_load < alpha*current_threshold
  int max_load_gap = 0.20*base_threshold;     // max_load_gap allowed between CT and (heighest load > BT).


  vector<int> connected_links;

  map<string,int> forwarding_table;             // map of (dst,linkid)

  dist_vector_table my_dvt;
  mutex *my_dvt_lock = NULL;

  // Stores the current read dvt_version of all nodes.
  map<string,int> dvt_version;

  bool terminate = false;
};






struct Link{
  string own_name;
  long long  delay = 2000;                    // in  microsecond
  long long bandwith = 1024*1024;             // in Kbps
  string src;
  string dst;
  queue<Packet> q;
  mutex *qlock = NULL;
  bool special = false;                               // if src is any switch and dst is controller.
  vector<pair<double,string> > packet_drop;
  bool terminate = false;
};


struct Switch{
  string own_name = "";
  queue<Packet> q;
  mutex *qlock = NULL;
  int max_queue_size = 50000;
  int controllerid;                             // the controller controlling the switch.
  vector<int> connected_links;

  // Initialise forwarding_table and
  // my dist_vector_table using
  // connected_links
  map<string,int> forwarding_table;             // map of (dst,linkid)


  dist_vector_table my_dvt;
  mutex *my_dvt_lock = NULL;
  /*If there is any change in my_dvt
    update forwarding_table and broadcast it.
  */
  map<string,int> dvt_version;

  map<long long int, Packet> buffer;
  /*In case of flow table miss packets
    from processing thread is kept in this
    'buffer' and packet information is sent
    to controller. Controller replies what
    to do with the packet.
  */

  int pkt_gen_interval = 1000;                 //generate pkt_after every 1 milli second.
  mutex *pkt_gen_interval_lock = NULL;

  vector<double> pkt_gen_time;

  int flow_table_hit_percentage = 0;
  bool terminate = false;

};


vector<Switch> switches;
vector<Link> links;
vector<Controller> controllers;

vector<thread> th_c;
vector<thread> th_s;
vector<thread> th_l;
vector<thread> th_clb;
vector<thread> th_spg;


struct broadcast_data{
  int *data;
  int counter;
};

// data for different pkt_types
/* NORMAL => NULL */

/* ROUTING => forwarding_table */
/* LOAD_BROADCAST => broadcast_data, data = load*/
/* NEW_THRESHOLD  => broadcast_data, data = new_threshold*/
/* LOAD_MIGRATION => int */

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


void dvt_info(string);

void routing_table_info(string src){
  dvt_info(src);
  return ;
  printf("Forwading Table of  %s\n",src.c_str());
  map<string,int> rt;
  if(src[0] == 'c') rt = controllers[getid(src)].forwarding_table;
  else rt = controllers[getid(src)].forwarding_table;
  
}

void dvt_info(string src){
  dist_vector_table dvt;
  if(src[0] == 's') dvt = switches[getid(src)].my_dvt;
  else dvt = controllers[getid(src)].my_dvt;
  printf("Distance Vector Table for %s\n",src.c_str());
  for(int i=0;i<dvt.row.size();i++){
    dist_vector &dv = dvt.row[i];
    printf("dst = %s, linkid = %d, hop_count = %d\n",dv.dst.c_str(),dv.linkid,dv.hop_count);
  }
}


int get_hop_count(string src, string dst){
  dist_vector_table dvt;
  if(src[0] == 'c'){
    controllers[getid(src)].my_dvt_lock->lock();
    dvt = controllers[getid(src)].my_dvt;
    controllers[getid(src)].my_dvt_lock->unlock();
  }
  else{
    switches[getid(src)].my_dvt_lock->lock();
    dvt = switches[getid(src)].my_dvt;
    switches[getid(src)].my_dvt_lock->unlock();
  }

  for(int i=0;i<dvt.row.size();i++){
    dist_vector dv = dvt.row[i];
    if(dv.dst == dst) return dv.hop_count;
  }

  return -1;

}