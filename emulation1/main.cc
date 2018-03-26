
//#define PRINT
#define PRINT_PKT_DROP
#define PRINT_CONTROL

#include "core.h"
#include "controller.cc"
#include "link.cc"
#include "switch.cc"
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



void packet_drop_statistic(map<double,int> &pdtc);
void throughput_statistic();
void topology_builder(string);
void start_control();

bool terminate_main_thread = false;
bool LB_RUNNING = true;
bool PG_RUNNING = true;

// When SIGINT signal received, terminate all threads.
void sig_handler(int sig){
  // Set every element terminate to true.
  for(int i=0;i<controllers.size();i++) controllers[i].terminate = true;
  for(int i=0;i<switches.size();i++) switches[i].terminate = true;
  for(int i=0;i<links.size();i++) links[i].terminate = true;

  // Wait for them to terminate.
  for(int i=0;i<controllers.size();i++) th_c[i].join();
  if(LB_RUNNING) for(int i=0;i<controllers.size();i++) th_clb[i].join();
  for(int i=0;i<switches.size();i++) th_s[i].join();
  if(PG_RUNNING) for(int i=0;i<switches.size();i++) th_spg[i].join();
  for(int i=0;i<links.size();i++) th_l[i].join();

  terminate_main_thread = true;
}


int main(){
  // Register to receive SIGINT.
  signal(SIGINT, sig_handler);
  gettimeofday(&emulation_start_time,NULL);
  srand((int)time(NULL));

  topology_builder("topology1");


  // Initialise mutex lock.
  for(int i=0;i<controllers.size();i++){
    controllers[i].qlock  = new mutex();
    controllers[i].lc_lock = new mutex();
    controllers[i].switch_pkt_count_lock = new mutex();
    controllers[i].pkt_count_lock = new mutex();
    controllers[i].load_collections.resize(controllers.size());
  }
  for(int i=0;i<links.size();i++) links[i].qlock = new mutex();
  for(int i=0;i<switches.size();i++){
    switches[i].qlock = new mutex();
    switches[i].my_dvt_lock = new mutex();
    switches[i].pkt_gen_interval_lock = new mutex();
  }



  // Create threads.
  th_c.resize(controllers.size());
  th_clb.resize(controllers.size());
  th_s.resize(switches.size());
  th_spg.resize(switches.size());
  th_l.resize(links.size());




  // Set pkt_gen_interval for each switch.
  switches[0].pkt_gen_interval =
  switches[1].pkt_gen_interval =
  switches[2].pkt_gen_interval =
  switches[4].pkt_gen_interval = 1000000/175; 

  switches[3].pkt_gen_interval = 1000000/1;  
  switches[5].pkt_gen_interval =
  switches[6].pkt_gen_interval =
  switches[7].pkt_gen_interval =
  switches[8].pkt_gen_interval = 1000000/175; 



  // Start thread except packet_generator and load_balancer.
  for(int i=0;i<controllers.size();i++) th_c[i] = thread(thread_controller_processing,i);
  for(int i=0;i<switches.size();i++) th_s[i] = thread(thread_switch_processing,i);
  for(int i=0;i<links.size();i++) th_l[i] = thread(thread_link,i);

  sleep(1); // Let all threads start.

  // Initial routing_table_info
  printf("-----------------------------------------\n");
  for(int i=0;i<switches.size();i++){
    routing_table_info(i);
    //dvt_info(i);
  }
  printf("-----------------------------------------\n");


  // Triggering route computation.
  Packet p;
  p.type = ROUTING;
  p.src = "s0";
  p.data = (void*)&switches[0].my_dvt;
  links[2].q.push(p);
  links[7].q.push(p);

  // Let route computation finish.
  // Wait for it.
  sleep(2);
  printf("Final routing table at %lf.\n",current_time());
  printf("-----------------------------------------\n");
  for(int i=0;i<switches.size();i++) dvt_info(i);
  printf("-----------------------------------------\n");



  // Testing with single packet.


  p.src = "s0";
  p.dst = "s7";
  p.type = NORMAL;
  p.packetid = 342;
  p.data = NULL;
  switches[0].qlock->lock();
  switches[0].q.push(p);
  switches[0].qlock->unlock();






  // Start load_balancer and packet_generator threads.
  if(LB_RUNNING) for(int i=0;i<controllers.size();i++) th_clb[i] = thread(thread_controller_load_balancing,i);
  if(PG_RUNNING) for(int i=0;i<switches.size();i++) th_spg[i] = thread(thread_switch_pkt_generator,i);


  // Wait untill get termiate.

  while(!terminate_main_thread){
    start_control();
    usleep(1000);
  }
  throughput_statistic();
  return 0;
}


void start_control(){
    int cid,load;
    scanf("%d %d",&cid,&load);
    switches[cid].pkt_gen_interval_lock->lock();
    switches[cid].pkt_gen_interval = 1000000/load;
    switches[cid].pkt_gen_interval_lock->unlock();
}

void throughput_statistic(){
  // Records packet_generation_time for all switches.
  vector<double> pgt;
  for(int i=0;i<switches.size();i++){
    for(int j=0;j<switches[i].pkt_gen_time.size();j++){
      pgt.push_back(switches[i].pkt_gen_time[j]);
    }
  }
  sort(pgt.begin(),pgt.end());

  // convert time from second to microsecond.
  for(int i=0;i<pgt.size();i++) pgt[i]*=1000000;

  FILE *fp = fopen("throughput","w");

  int idx = 0;
  int current_time = 0;
  map<double,int> pkt_gen_time_cnt;
  while(idx<pgt.size()){
    int cnt = 0;
    // count of packets generated in a period of 1 second.
    while(idx < pgt.size() && pgt[idx] < current_time+1000000){idx++;cnt++;}
    pkt_gen_time_cnt[((double)(current_time))/1000000] = cnt;
    current_time += 1000000;
  }

  map<double,int> pkt_drop_time_cnt;
  packet_drop_statistic(pkt_drop_time_cnt);

  // Now for every packet drop in 5000 microsecond interval decrease count from packet generation.
  auto itr = pkt_drop_time_cnt.begin();
  while(itr!=pkt_drop_time_cnt.end()){
    double drop_time = itr->first;
    int drop_cnt = itr->second;
    if(pkt_gen_time_cnt.find(drop_time) != pkt_gen_time_cnt.end()) pkt_gen_time_cnt[drop_time]-=drop_cnt;
    itr++;
  }
  itr = pkt_gen_time_cnt.begin();
  while(itr!=pkt_gen_time_cnt.end()){
    fprintf(fp,"%lf %d\n",itr->first,itr->second);
    itr++;
  }
  fclose(fp);
}

void packet_drop_statistic(map<double,int> &pkt_drop_time_cnt){
  vector<pair<double,string> > packets_dropped;
  for(int i=0;i<links.size();i++){
    packets_dropped.insert(packets_dropped.end(),
      links[i].packet_drop.begin(),links[i].packet_drop.end());
  }
  sort(packets_dropped.begin(),packets_dropped.end());
  // count of packets drop in interval 1 second
  vector<pair<double,string> > &pd = packets_dropped;

  // convert time from seconds to microseconds.
  for(int i=0;i<pd.size();i++) pd[i].first *= 1000000;

  int idx = 0;
  int current_time = 0;
  FILE *fp = fopen("pkt_drop","w");
  printf("pkt_drop cnt = %ld\n",pd.size());
  while(idx<pd.size()){
    int cnt = 0;
    while(pd[idx].first < current_time+1000000 && idx < pd.size()){cnt++; idx++;}
    fprintf(fp,"%lf %d\n",((double)(current_time))/1000000,cnt);
    pkt_drop_time_cnt[((double)(current_time))/1000000] = cnt;
    current_time += 1000000;
  }
  fclose(fp);
}

void topology_builder(string filename){
  FILE *fp = fopen(filename.c_str(),"r");
  char line[50];
    int cc,sc,lc;
  // get controllers_count.
  fscanf(fp,"%[^\n]\n",line);
  sscanf(line,"controllers_count%d",&cc);
  // get switch_count.
  fscanf(fp,"%[^\n]\n",line);
  sscanf(line,"switches_count%d",&sc);
  // get links_count.
  fscanf(fp,"%[^\n]\n",line);
  sscanf(line,"links_count%d",&lc);




  for(int i=0;i<cc;i++){
    Controller c;
    controllers.push_back(c);
  }
  for(int i=0;i<sc;i++){
    Switch s;
    switches.push_back(s);
  }
  for(int i=0;i<lc;i++){
    Link l;
    links.push_back(l);
  }

  // set own_name for each elements.
  for(int i=0;i<controllers.size();i++) controllers[i].own_name = "c"+to_string(i);
  for(int i=0;i<switches.size();i++) switches[i].own_name = "s"+to_string(i);
  for(int i=0;i<links.size();i++) links[i].own_name = "l"+to_string(i);


  // set controllers of each switch;
  int temp = sc;
  while(temp--){
    memset(line,0,50);
    fscanf(fp,"%[^\n]\n",line);
    int sid,cid;
    sscanf(line,"controller(s%d,c%d)",&sid,&cid);
    switches[sid].controllerid = cid;
  }

  // Get the links...
  while(!feof(fp)){
    memset(line,0,50);
    fscanf(fp,"%[^\n]\n",line);
    int lid;
    char _src[10],_dst[10];
    memset(_src,0,10);
    memset(_dst,0,10);
    sscanf(line,"link(l%d,%[^,],%[^)])",&lid,_src,_dst);

    // set link's src and dst.
    links[lid].src = _src;
    links[lid].dst = _dst;

    if(_src[0] == 'c'){
      int cid;
      sscanf(_src,"c%d",&cid);
      controllers[cid].linkid = lid;
    }
    else if(_src[0] == 's'){

      int sid;
      sscanf(_src,"s%d",&sid);
      switches[sid].connected_links.push_back(lid);
    }
    else perror("can't happend\n");

    if(_dst[0] == 'c') links[lid].special = true;
  }
  fclose(fp);



  // Initialise forwarding_table for each switch.
  for(int i=0;i<switches.size();i++){
    for(int j=0;j<switches[i].connected_links.size();j++){
      int lid = switches[i].connected_links[j];
      switches[i].forwarding_table[links[lid].dst] = lid;
    }
  }

  // Initialise my_dvt for each switch.
  for(int sid=0;sid<switches.size();sid++){
    switches[sid].my_dvt.switchname = switches[sid].own_name;
    switches[sid].my_dvt.version = 1;

    // dvt_version received of other switches set to 0.
    switches[sid].dvt_version.resize(switches.size());
    for(int i=0;i<switches.size();i++) switches[sid].dvt_version[i] = 0;


    for(int j=0;j<switches[sid].connected_links.size();j++){
      int lid = switches[sid].connected_links[j];
      dist_vector dv;
      dv.dst = links[lid].dst;
      dv.linkid = lid;
      dv.hop_count = 1;
      switches[sid].my_dvt.row.push_back(dv);
    }
  }
}
