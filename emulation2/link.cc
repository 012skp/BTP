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
  printf("Link %d is up\n",linkid);
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
      if(mylink.terminate) {
        printf("Link %d exited\n",linkid);
        break;
      }

      // Lock well before reading or writing.
      myqlock->lock();
      int myqsize = myq.size();
      myqlock->unlock();
      if(myqsize > 0){
        // Get top packet as fp.(front packet)
        myqlock->lock();
        Packet fp = myq.front();
        myqlock->unlock();

        #ifdef PRINT
        printf("L[%d] time:%lf => packetid = %lld, src = %s, dst = %s, type = %s\n",linkid,current_time(),
                  fp.packetid,fp.src.c_str(),fp.dst.c_str(),TYPE(fp.type).c_str());
        #endif

        #ifdef PRINT_CONTROL
        if(fp.type == CONTROL){
          printf("L[%d] time:%lf => packetid = %lld, src = %s, dst = %s, type = %s, subtype %s\n",
                    linkid,current_time(),
                    myq.front().packetid,myq.front().src.c_str(),myq.front().dst.c_str(),
                    TYPE(myq.front().type).c_str(),SUBTYPE(myq.front().subtype).c_str());
        }
        #endif


        int dqsize;
        dqlock->lock();
        dqsize = dq->size();
        dqlock->unlock();

        // If dst element has buffer full.
        if(dqsize>= dq_max_size){
          // Drop the packet silently.

          #ifdef PRINT_PKT_DROP
          printf("Packet %lld drop at %s\n",fp.packetid,mylink.dst.c_str());
          #endif

          mylink.packet_drop.push_back({current_time(),mylink.dst});

          myqlock->lock();
          myq.pop();
          myqlock->unlock();

        }
        else{
            // Front packet is top_packet.
            Packet top_packet = fp;

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
                //printf("time = %lf: at c%d pkt count of %s is %d\n",current_time(),getid(mylink.dst),
                          //top_packet.src.c_str(),switch_pkt_count[top_packet.src]);
                switch_pkt_count_lock->unlock();

                // Increase the pkt_count in controller.
                c.pkt_count_lock->lock();
                c.pkt_count++;
                c.pkt_count_lock->unlock();
              }



        }

      }
      else {
        usleep(mylink.delay); // context switch so that other threads get chance.
      }
  }
}
