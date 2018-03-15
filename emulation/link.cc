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
  mutex *dqlock = NULL;
  int dq_max_size;
  // If special link, dst is a controller else switch.
  if(mylink.special){
    dq = &controllers[getid(mylink.dst)].q;
    dq_max_size = controllers[getid(mylink.dst)].max_queue_size;
    dqlock = controllers[getid(mylink.dst)].qlock;

  }
  else{
    dq = &switches[getid(mylink.dst)].q;
    dq_max_size = switches[getid(mylink.dst)].max_queue_size;
    dqlock = switches[getid(mylink.dst)].qlock;
  }

  queue<Packet> &myq = mylink.q;
  mutex *myqlock = links[linkid].qlock;

  while(1){
      // Lock well before reading or writing.
      myqlock->lock();
      //printf("L[%d] got the mutex\n",linkid);
      int myqsize = myq.size();
      myqlock->unlock();
      if(myqsize > 0){

        myqlock->lock();
        #ifdef PRINT
        printf("L[%d] time:%lf => packetid = %lld, src = %s, dst = %s, type = %s\n",linkid,current_time(),
                  myq.front().packetid,myq.front().src.c_str(),myq.front().dst.c_str(),
                  TYPE(myq.front().type).c_str());
        #endif
        myqlock->unlock();

        int dqsize;
        dqlock->lock();
        dqsize = dq->size();
        dqlock->unlock();

        // If dst element has buffer full.
        if(dqsize>= dq_max_size){
          // Drop the packet silently.

          myqlock->lock();
          #ifdef PRINT_PKT_DROP
          printf("Packet %lld drop at %s\n",myq.front().packetid,mylink.dst.c_str());
          #endif
          mylink.packet_drop.push_back({current_time(),mylink.dst});
          myq.pop();
          myqlock->unlock();
        }
        else{
          myqlock->lock();
          Packet top_packet = myq.front();
          myqlock->unlock();
            // Packet should wait for propagation delay in link.
            // If now - packet.start_time >= delay of mylink put it on dst queue.
            struct timeval now;
            gettimeofday(&now,NULL);

            // returns time difference in microsecond.
            double diff = (double)time_diff(now,top_packet.start_time);
            int propagation_delay_time = mylink.delay > diff?mylink.delay-diff:0;

            if(propagation_delay_time){
              usleep(propagation_delay_time);
            }

              dqlock->lock();
              dq->push(top_packet);
              dqlock->unlock();


              myqlock->lock();
              myq.pop();
              myqlock->unlock();




              // If this is a special link, link has to increase the
              // packet count for switch form which this packet has originated.
              if(mylink.special){
                Controller &c = controllers[getid(mylink.dst)];
                map<string,int> &switch_pkt_count = c.switch_pkt_count;

                mutex *switch_pkt_count_lock = c.switch_pkt_count_lock;
                // If this is the first packet in queue.
                switch_pkt_count_lock->lock();
                if(switch_pkt_count.find(top_packet.src) == switch_pkt_count.end())
                  switch_pkt_count[top_packet.src] = 1;
                  else switch_pkt_count[top_packet.src]++;
                switch_pkt_count_lock->unlock();
              }



        }

      }
      else {
        usleep(1000); // context switch so that other threads get chance.
      }
  }
}
