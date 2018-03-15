
#define PRINT

#include "core.h"
#include "controller.cc"
#include "link.cc"
#include "switch.cc"
using namespace std;



void packet_drop_statistic();

void topology_builder(string);



int main(){
  gettimeofday(&emulation_start_time,NULL);
  gettimeofday(&latest_packet_seen,NULL);
  emulation_done = false;
  // If no packet seen in network for 100 milli second
  // stop emulation.
  max_delay = 100000;
  srand((int)time(NULL));

  topology_builder("topology1");


  // Initialise mutex lock.
  for(int i=0;i<controllers.size();i++){
    controllers[i].qlock  = new mutex();
    controllers[i].lclock = new mutex();
    controllers[i].switch_pkt_count_lock = new mutex();
  }
  for(int i=0;i<links.size();i++) links[i].qlock = new mutex();
  for(int i=0;i<switches.size();i++){
    switches[i].qlock = new mutex();
    switches[i].my_dvt_lock = new mutex();
    switches[i].pkt_gen_interval_lock = new mutex();
  }



  // Create threads.
  thread th_c[controllers.size()];
  thread th_clb[controllers.size()];
  thread th_s[switches.size()];
  thread th_spg[switches.size()];
  thread th_l[links.size()];




  // Set pkt_gen_interval for each switch.
  int no_of_switch_per_controller = switches.size()/controllers.size();
  double pkt_serving_rate = 1.0/2000.0;
  double pkt_gen_rate  = pkt_serving_rate/no_of_switch_per_controller;
  int pkt_gen_interval = (int)(1.0/pkt_gen_rate);
  for(int i=0;i<switches.size();i++) switches[i].pkt_gen_interval = pkt_gen_interval;



  // Start thread except packet_generator.
  for(int i=0;i<controllers.size();i++) th_c[i] = thread(thread_controller_processing,i);
  for(int i=0;i<controllers.size();i++) th_clb[i] = thread(thread_controller_load_balancing,i);
  for(int i=0;i<switches.size();i++) th_s[i] = thread(thread_switch_processing,i);
  for(int i=0;i<links.size();i++) th_l[i] = thread(thread_link,i);



  sleep(1); // start all threads before.


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

  // Let routing table generation finish.
  // Wait for it.
  while(1){
    struct timeval t;
    gettimeofday(&t,NULL);
    lps_lock.lock();
    long td = time_diff(t,latest_packet_seen);
    lps_lock.unlock();
    if(td > max_delay) break;
    usleep(10000);
  }
  sleep(5);
  printf("Final routing table at %lf.\n",current_time());
  printf("-----------------------------------------\n");
  for(int i=0;i<switches.size();i++){
    routing_table_info(i);
    dvt_info(i);
  }
  printf("-----------------------------------------\n");


  // Testing with single packet.
  /*
  sleep(2);
  p.src = "s0";
  p.dst = "s1";
  p.type = NORMAL;
  p.packetid = 342;
  p.data = NULL;
  switches[0].qlock->lock();
  switches[0].q.push(p);
  switches[0].qlock->unlock();
  */



  // Start all packet_generator threads.
  //for(int i=0;i<switches.size();i++) th_spg[i] = thread(thread_switch_pkt_generator,i);

  sleep(2);
  emulation_done = true;
  // wait for threads...



  for(int i=0;i<controllers.size();i++) th_c[i].join();
  for(int i=0;i<controllers.size();i++) th_clb[i].join();
  for(int i=0;i<switches.size();i++) th_s[i].join();
  //for(int i=0;i<switches.size();i++) th_spg[i].join();
  for(int i=0;i<links.size();i++) th_l[i].join();

  packet_drop_statistic();
  return 0;
}


void start_control(){

}

void packet_drop_statistic(){
  vector<pair<double,string> > packets_dropped;
  for(int i=0;i<links.size();i++){
    packets_dropped.insert(packets_dropped.end(),
      links[i].packet_drop.begin(),links[i].packet_drop.end());
  }
  sort(packets_dropped.begin(),packets_dropped.end());
  //count of packets drop at every 1 milli seconds.
  vector<pair<double,string> > &pd = packets_dropped;
  for(int i=0;i<pd.size();i++) pd[i].first *= 1000000;
  int idx = 0;
  int ct = 0;
  FILE *fp = fopen("pkt_drop","w");
  printf("pkt_drop cnt = %ld\n",pd.size());
  while(idx<pd.size()){
    int cnt = 0;
    while(pd[idx].first < ct+5000 && idx < pd.size()){cnt++; idx++;}
    if(cnt) fprintf(fp,"%lf %d\n",((double)(ct))/1000000,cnt);

    ct += 5000;
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
