// Switchs codes...
#include<bits/stdc++.h>
#ifndef __CORE_H__
#include "core.h"
#endif
using namespace std;

void thread_switch_processing(int switchid){
  printf("Switch %d, Processing Thread is up\n",switchid);
  Switch &mys = switches[switchid];
  queue<Packet> &myq = mys.q;
  mutex *myqlock = mys.qlock;
  while(1){
    if(mys.terminate){
      printf("Switch Processing Thread %d exited\n",switchid);
      break;
    }
    myqlock->lock();
    bool myqempty;
    myqempty = myq.empty();
    myqlock->unlock();
    if(!myqempty){

      myqlock->lock();
      Packet p = myq.front();
      myqlock->unlock();

      #ifdef PRINT
      printf("S[%d] time:%lf => packetid = %lld, src = %s, dst = %s, type = %s\n",switchid,current_time(),
            p.packetid,p.src.c_str(),p.dst.c_str(),TYPE(p.type).c_str());
      #endif
      Packet fp = p;

      #ifdef PRINT_CONTROL
      if(fp.type == CONTROL) {
        printf("S[%d] time:%lf => packetid = %lld, src = %s, dst = %s, type = %s, subtype %s\n",
                  switchid,current_time(),
                  myq.front().packetid,myq.front().src.c_str(),myq.front().dst.c_str(),
                  TYPE(myq.front().type).c_str(),SUBTYPE(myq.front().subtype).c_str());
      }
      #endif

      // If this is ROUTING packet.
      if(p.type == ROUTING){
        // received_dist_vector_table.

        // Lock both r_dvt and my_dvt,
        // otherwise may fall into deadlock.
        mutex *r_dvt_lock =NULL;
        if(p.src[0] == 's') r_dvt_lock = switches[getid(p.src)].my_dvt_lock;
        else r_dvt_lock = controllers[getid(p.src)].my_dvt_lock;
        mutex *my_dvt_lock = mys.my_dvt_lock;

        bool already_unlocked = false;


        //printf("S[%d] waiting to get my_dvt_lock & my_dvt_lock[%d] lock\n",switchid,getid(p.dst));
        r_dvt_lock->lock();
        my_dvt_lock->lock();
        //printf("S[%d] got the both lock\n",switchid);


        // TODO Improvement copy r_dvt and update instead of locking.
        dist_vector_table *r_dvt = (dist_vector_table*)p.data;
        assert(r_dvt);

        // If this version of dist_vector_table already updated ignore it.
        if(mys.dvt_version[p.src] >= r_dvt->version); // ignore
        else{
          // Compare and update my_dvt using this received dvt.
          bool updated = false;
          for(int i=0; i<r_dvt->row.size(); i++){
            // row is tuple containing {dst,hop_count,linkid}
            dist_vector &r_dv = r_dvt->row[i];
            string dst = r_dv.dst;
            if(dst == mys.own_name) continue;

            // search for dst in my_dvt.
            bool new_dv_entry = true;

            for(int j=0; j<mys.my_dvt.row.size();j++){
              dist_vector &my_dv = mys.my_dvt.row[j];
              if(my_dv.dst == dst){
                if(my_dv.hop_count > r_dv.hop_count+1){
                  // update my_dv.
                  my_dv.hop_count = r_dv.hop_count+1;
                  my_dv.linkid = mys.forwarding_table[p.src];
                  updated = true;
                }
                new_dv_entry = false;
                break;
              }
            }
            if(new_dv_entry){
              dist_vector new_dv;
              new_dv.dst = dst;
              new_dv.hop_count = r_dv.hop_count+1;
              new_dv.linkid = mys.forwarding_table[p.src];
              mys.my_dvt.row.push_back(new_dv);
              updated = true;
            }

          }


          if(updated){
            // If my_dvt has been updated, update the forwarding_table.
            for(int i=0;i<mys.my_dvt.row.size();i++){
              dist_vector &dv = mys.my_dvt.row[i];
              if(dv.dst == mys.own_name) continue;
              mys.forwarding_table[dv.dst] = dv.linkid;
            }

            #ifdef PRINT
            printf("Routing table updated\n");
            routing_table_info("s"+to_string(switchid));
            #endif

            // Broadcast new dvt.

            mys.my_dvt.version++;
            Packet np;
            np.src = mys.own_name;
            np.dst = "";
            np.type = ROUTING;
            np.data = (void*)&mys.my_dvt;

            r_dvt_lock->unlock();
            my_dvt_lock->unlock();
            already_unlocked = true;

            for(int i=0;i<mys.connected_links.size();i++){
              gettimeofday(&np.start_time,NULL);

              links[mys.connected_links[i]].qlock->lock();
              links[mys.connected_links[i]].q.push(np);
              links[mys.connected_links[i]].qlock->unlock();

            }

          }



        }

        if(!already_unlocked){
          r_dvt_lock->unlock();
          my_dvt_lock->unlock();
        }

        myqlock->lock();
        myq.pop();
        myqlock->unlock();
        continue;
      }


      // If packet is for this switch.
      if(p.dst == mys.own_name){
        // Check for packet types.
        if(p.type == NORMAL){
          // Consume it.
          // TODO Sending Ack implement it latter.
        }
        else if(p.type == PACKET_OUT){
          // Get the packet from buffer.
          if(mys.buffer.find(p.packetid) == mys.buffer.end());
          else{
            Packet np = mys.buffer[p.packetid];
            mys.buffer.erase(p.packetid);

            //TODO transmissin dealy not implemented.
            gettimeofday(&np.start_time,NULL);
            //printf("S[%d] waiting to get qlock\n",switchid);
            links[mys.forwarding_table[np.dst]].qlock->lock();
            //printf("S[%d] got the qlock\n",switchid);
            links[mys.forwarding_table[np.dst]].q.push(np);
            links[mys.forwarding_table[np.dst]].qlock->unlock();
          }
        }
        else if(p.type == CONTROL && p.subtype == ROLE_REQ){
          // Got a ROLE_REQ packet, change your controllerid to it.
          printf("S[%d] time:%lf => Changing controller from %d to %d\n",switchid,current_time(),
                                                    mys.controllerid,getid(p.src));
          int previous_controllerid = mys.controllerid;
          mys.controllerid = getid(p.src);

          // Send ROLE_REQ_ACK packet back to controller.
          Packet np;
          np.src = mys.own_name;
          np.dst = p.src;
          np.type = CONTROL;
          np.subtype = ROLE_REQ_ACK;
          np.data = new int(previous_controllerid);
          gettimeofday(&np.start_time,NULL);

          links[mys.forwarding_table[np.dst]].qlock->lock();
          links[mys.forwarding_table[np.dst]].q.push(np);
          links[mys.forwarding_table[np.dst]].qlock->unlock();
          printf("S[%d] time:%lf => Sending ROLE_REQ_ACK to %s\n",switchid,current_time(),
                                                    np.dst.c_str());
        }
        else printf("S[%d] time:%lf => Unknown Packet received packetid =%lld, src = %s\n",
                     switchid,current_time(),p.packetid,p.src.c_str());
      }
      else{
        // This packet isn't for me, simply forward it.
        // Check for entry in flow table.
        bool entry = rand()%100<mys.flow_table_hit_percentage?true:false;
        // Only pkts generated by this switch will have table miss.
        if(p.src != mys.own_name) entry = true;
        if(entry){
          // If flow table hit, forwad it.
          // TODO transmission delay not implemented.
          gettimeofday(&p.start_time,NULL);
          //printf("S[%d] waiting to get qlock\n",switchid);
          links[mys.forwarding_table[p.dst]].qlock->lock();
          //printf("S[%d] got the qlock\n",switchid);
          links[mys.forwarding_table[p.dst]].q.push(p);
          links[mys.forwarding_table[p.dst]].qlock->unlock();
        }
        else {
          /*In case of flow table miss packets
            from processing thread is kept seperate
            buffer and packet information is sent
            to controller. Controller replies what
            to do with the packet.
          */
          mys.buffer[p.packetid] = p;

          /* In case of PACKET_IN drop for PACKET_OUT drop
             This packet will always be there in buffer.
             So another thread will be there checking for
             time out, and resend the packet to controller.
             On reception of PACKET_IN message for this packet
             clear the correspoding buffer.
          */

          gettimeofday(&mys.buffer[p.packetid].start_time,NULL);

          // Now send the PACKET_IN message to controller.
          Packet np;
          np.src = mys.own_name;
          np.dst = "c" + to_string(mys.controllerid);
          np.type = PACKET_IN;
          np.packetid = p.packetid;
          gettimeofday(&np.start_time,NULL);
          //printf("S[%d] waiting to get qlock\n",switchid);
          links[mys.forwarding_table[np.dst]].qlock->lock();
          //printf("S[%d] got the qlock\n",switchid);
          links[mys.forwarding_table[np.dst]].q.push(np);
          links[mys.forwarding_table[np.dst]].qlock->unlock();

        }
      }
      // all processing done pop the packet.
      //printf("S[%d] waiting to get myqlock\n",switchid);
      myqlock->lock();
      //printf("S[%d] got qlock\n",switchid);
      myq.pop();
      myqlock->unlock();
    }
    // no packets in queue.
    else usleep(1000); // sleep for 1 milli second
  }
}


// pkt_generator follows poissions distribution 
// with mean waiting time pkt_gen_interval.
void thread_switch_pkt_generator(int switchid){
  printf("Switch %d Packet Generator Thread is up\n",switchid);
  Switch &mys = switches[switchid];
  int pkt_id = switchid*1000000;
  while(1){
    if(mys.terminate) {
      printf("Switch %d Packet Generator Thread exited\n",switchid);
      break;
    }
    Packet p;
    p.src = mys.own_name;
    int did = rand()%switches.size();
    while(did == switchid) did = rand()%switches.size();
    p.dst = "s" + to_string(did);
    p.type = NORMAL;
    p.packetid = pkt_id++;
    p.data = NULL;

    mys.qlock->lock();
    mys.q.push(p);
    mys.pkt_gen_time.push_back(current_time());
    mys.qlock->unlock();

    mys.pkt_gen_interval_lock->lock();
    int pgi = mys.pkt_gen_interval;
    mys.pkt_gen_interval_lock->unlock();

    std::default_random_engine gen(time(NULL));
    std::poisson_distribution<int> pd(pgi);
    usleep(pd(gen));
  }
}
