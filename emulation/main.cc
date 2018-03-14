
#define PRINT

#include "core.h"
#include "controller.cc"
#include "link.cc"
#include "switch.cc"
using namespace std;



void packet_drop_statistic();

void topology_builder();

void routing_table_info(int switchid);

void dvt_info(int switchid);


int main(){
  gettimeofday(&emulation_start_time,NULL);
  gettimeofday(&latest_packet_seen,NULL);
  emulation_done = false;
  max_delay = 1000000;
  srand((int)time(NULL));

  //topology_builder();
  Controller c;
  Link l;
  Switch s;
  controllers.push_back(c);
  links.push_back(l);
  links.push_back(l);
  switches.push_back(s);
  links[0].src = "s0";
  links[0].dst = "c0";
  links[1].src = "c0";
  links[1].dst = "s0";
  links[0].special = true;
  controllers[0].linkid = 1;

  links[0].qlock = new mutex();
  links[1].qlock = new mutex();
  controllers[0].qlock = new mutex();
  controllers[0].lclock = new mutex();
  controllers[0].switch_pkt_count_lock = new mutex();
  switches[0].qlock = new mutex();

  // Create threads for each elements.
  thread th_c0(thread_controller_processing,0);
  thread th_l0(thread_link,0);
  thread th_l1(thread_link,1);
  //thread th_s0(thread_switch_processing,0);


/*
  thread th_c[controllers.size()];
  thread th_s[switches.size()];
  thread th_l[links.size()];

  for(int i=0;i<controllers.size();i++) th_c[i] = thread(thread_controller_processing,i);
  for(int i=0;i<switches.size();i++) th_s[i] = thread(thread_switch_processing,i);
  for(int i=0;i<links.size();i++) th_l[i] = thread(thread_link,i);

*/

  usleep(1000); // start all threads before.

  printf("-----------------------------------------\n");
  /*
  for(int i=0;i<switches.size();i++){
    routing_table_info(i);
    dvt_info(i);
  }
  */
  usleep(1000);

  Packet p;

  p.src = "s0";
  p.dst = "c0";
  p.type = PACKET_IN;
  p.data = NULL;
  for(int i=0;i<10;i++){
    p.packetid = 23+i;
    gettimeofday(&p.start_time,NULL);
    links[0].q.push(p);
  }



  emulation_done = true;
  // wait for threads...

    th_c0.join();
    th_l0.join();
    th_l1.join();
    //th_s0.join();

/*
  for(int i=0;i<controllers.size();i++) th_c[i].join();
  for(int i=0;i<switches.size();i++) th_s[i].join();
  for(int i=0;i<links.size();i++) th_l[i].join();
*/
  packet_drop_statistic();
  return 0;
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

void topology_builder(){
  FILE *fp = fopen("topology","r");
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

void routing_table_info(int switchid){
  printf("Forwading Table of switch %s\n",switches[switchid].own_name.c_str());
  for(auto itr : switches[switchid].forwarding_table){
    printf("%s->l%d\n",itr.first.c_str(),itr.second);
  }
}

void dvt_info(int switchid){
  dist_vector_table &dvt = switches[switchid].my_dvt;
  printf("Distance Vector Table for switch %s\n",dvt.switchname.c_str());
  for(int i=0;i<dvt.row.size();i++){
    dist_vector &dv = dvt.row[i];
    printf("dst = %s, linkid = %d, hop_count = %d\n",dv.dst.c_str(),dv.linkid,dv.hop_count);
  }
}
