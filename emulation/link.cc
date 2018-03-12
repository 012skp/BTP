/*Link's code
  This is code of half duplex link.
  Need to have two such links to work as full duplex.
*/

#include<bits/stdc++.h>
#ifndef __CORE_H__
#include "core.h"
#endif
using namespace std;



void thread_link(int  linkid){
  Link &mylink = links[linkid]; // get the link info from global database;

  // get destination queue as 'dq'
  queue<Packet> *dq = NULL;
  int dq_max_size;
  // If special link, dst is a controller else switch.
  if(mylink.special){
    dq = &controllers[getid(mylink.dst)].q;
    dq_max_size = controllers[getid(mylink.dst)].max_queue_size;
  }
  else{
    dq = &switches[getid(mylink.dst)].q;
    dq_max_size = switches[getid(mylink.dst)].max_queue_size;
  }

  printf("mylink src = %s and dst = %s\n",mylink.src.c_str(),mylink.dst.c_str());
  queue<Packet> &myq = mylink.q;

  printf("test\n");
  while(1){
      if(myq.size()>0){
        printf("got packet id = %lld\n",myq.front().packet_id);
        // If dst element has buffer full.
        if(dq->size() >= dq_max_size){
          // Drop the packet silently.
          printf("Packet drop at %s\n",mylink.dst.c_str());
          myq.pop();
        }
        else{
          Packet top_packet = myq.front();
            // Packet should wait for propagation delay in link.
            // If now - packet.start >= delay of mylink put it on dst queue.
            struct timeval now;
            gettimeofday(&now,NULL);

            // returns time difference in microsecond.
            double diff = time_diff(now,top_packet.start)/1000;

            if(diff >= mylink.delay){
              dq->push(top_packet);
              myq.pop();

              // If this is a special link, link has to increase the
              // packet count for switch form which this packet has originated.
              if(mylink.special){
                Controller &c = controllers[getid(mylink.dst)];
                map<string,int> &switch_pkt_count = c.switch_pkt_count;

                // If this is the first packet in queue.
                if(switch_pkt_count.find(top_packet.src) == switch_pkt_count.end())
                  switch_pkt_count[top_packet.src] = 1;
                else switch_pkt_count[top_packet.src]++;
              }

            }
            else sleep(0); // still need to wait: propagation delay
            // context switch so that other threads get chance.
        }
      }
      else {
        //printf("link %s queue is empty\n",linkname.c_str());
        sleep(0); // context switch so that other threads get chance.
      }
  }
}
