// Controller's Code

#ifndef __CORE_H__
#include "core.h"
#endif



void thread_controller_load_measurement(int controllerid){
    Controller &myc = controllers[controllerid];
    queue<Packet> &myq = myc.q;
    while(1){
      myc.current_load = myq.size()*myc.avg_processing_time;
      usleep(10000); // period = 10 milli second.
    }
}

void thread_controller_load_informating(int controllerid){
  Controller &myc = controllers[controllerid];
  Link &l = links[myc.linkid];
  queue<Packet> &lq = l.q;
  while(1){
    if(abs(myc.load_informed - myc.current_load) > myc.allowed_load_deviation){
      // Do load broadcasting...
      for(int i=0;i<controllers.size();i++){
        if(i==controllerid) continue;
        Packet p;
        p.src = "c"+ to_string(controllerid);
        p.dst = "c"+ to_string(i);
        p.type = CONTROL;
        p.subtype = LOAD_BROADCAST;
        // TODO add load value.
        lq.push(p);
      }
      myc.load_informed = myc.current_load;
    }
    else usleep(10000); // sleep for 10 milli second.
  }
}


void thread_controller_load_balancer(int controllerid){
  Controller &myc = controllers[controllerid];

  while(1){
      bool heaviest = true;
      for(int i=0;i<myc.load_collections.size();i++){
        if(i == controllerid) continue;
        if(myc.current_load < myc.load_collections[i])
          goto wait;   // is not heaviest loaded controller don't do anything
      }

      // This is the heaviest loaded controller do load_balancing.
      if(myc.current_load > myc.current_threshold){
            // Search for lightest loaded controller.
            int lowest_cid = -1;    // lowest loaded controllerid.
            int lowest_load = myc.current_load;
            for(int i=0;i<myc.load_collections.size();i++){
              if(myc.load_collections[i] < lowest_load){
                lowest_load = myc.load_collections[i];
                lowest_cid = i;
              }
            }

            assert(lowest_cid != -1);


            // If there exists enough gap between current_threshold and lowest_load.
            if(lowest_load <= myc.alpha*myc.current_threshold){


              // Search for switch that satisfies
              // load < (heaviest_load-lowest_load)/2 -----(1)
              int desired_switch_load = (myc.current_load-lowest_load)/2;

              auto itr = myc.switch_pkt_count.begin();
              string the_switch = "";
              int load = -1;

              // try to find first switch that satisfies (1)
              assert(myc.q.size()>0);
              while(itr != myc.switch_pkt_count.end()){
                if( ((itr->second*myc.current_load)/myc.q.size()) < desired_switch_load){
                  the_switch = itr->first;
                  load = (itr->second*myc.current_load)/myc.q.size();
                  break;
                }
                itr++;
              }
              assert(load != -1);
              itr++;

              // Now try to find switch with heighest load satifying (1).
              while(itr != myc.switch_pkt_count.end()){
                if( ((itr->second*myc.current_load)/myc.q.size()) < desired_switch_load &&
                    ((itr->second*myc.current_load)/myc.q.size()) > load){
                      load = ((itr->second*myc.current_load)/myc.q.size());
                      the_switch = itr->first;
                }
              }

              // Now we have heaviest loaded switch satisying (1).
              // Do switch migration. Send CONTROL Packet with LOAD_MIRGRATION subtype

              int *sid = new int(getid(the_switch));
              //  Make sure to free the memroy upon reception of packet.
              Packet p;
              p.src = "c" + to_string(controllerid);
              p.dst = "c" + to_string(lowest_cid);
              p.type = CONTROL;
              p.subtype = LOAD_MIRGRATION;
              p.data = (void*)sid;

              links[myc.linkid].q.push(p);

            }
            else {
              // Broadcast NEW_THRESHOLD...
              Packet  p;
              p.src = "c" + to_string(controllerid);
              p.type = CONTROL;
              p.subtype = NEW_THRESHOLD;
              broadcast_data *bd = (broadcast_data*)malloc(sizeof(broadcast_data));
              // Set current_load as new_threshold.
              bd->data = new int(myc.current_load);
              bd->counter = controllers.size()-1;
              p.data = (void*)bd;
              // TODO make sure to free the memory when counter reaches 0.
              for(int i=0;i<controllers.size();i++){
                if(i == controllerid) continue;
                p.dst = "c" + to_string(i);
                links[myc.linkid].q.push(p);
              }

              goto wait;
            }

      }
      wait:
        usleep(10000); // sleep for 10 milli second
  }
}



void thread_controller_processing(int controllerid){
  Controller &myc = controllers[controllerid];
  queue<Packet> &myq = myc.q;
  // Get packets from queue and process them.
  while(1){
    while(!myq.empty()){
      Packet p = myq.front();
      if(p.type == PACKET_IN){
          // processing_time = random(min_processing_time,max_processig_time).
          int processing_time = random(myc.min_processing_time, myc.max_processing_time);
          usleep(processing_time);

          // send PACKET_OUT message
          Packet np;
          np.type = PACKET_OUT;
          np.src = p.dst;
          np.dst = p.src;
          np.packetid = p.packetid;

          links[myc.linkid].q.push(np);
      }
      else if(p.type == CONTROL){
          // Handle LOAD_MIRGRATION
          if(p.subtype = LOAD_MIRGRATION){
            // Retrive the switch id.
            int sid = *(int*)p.data;
            if(p.data) free(p.data);

            Packet np;
            np.type = CONTROL;
            np.subtype = ROLE_REQ;
            np.src = "c" + to_string(controllerid);
            np.dst = "s" + to_string(sid);
            links[myc.linkid].q.push(np);
          }

          // Handle NEW_THRESHOLD
          if(p.subtype = NEW_THRESHOLD){
            // Retrive new_th value.
            int new_th = *(((broadcast_data*)p.data)->data);
            // Decrement the counter.
            ((broadcast_data*)p.data)->counter--;
            int counter = ((broadcast_data*)p.data)->counter;
            if(counter == 0){
              // Free new_th memory
              free(((broadcast_data*)p.data)->data);
              free(p.data);
            }

            // Set new_th.
            myc.current_threshold = new_th;
          }

          // Handle LOAD_BROADCAST
          if(p.subtype = LOAD_BROADCAST){

          }

          // Handle ROLE_REP
          if(p.subtype = ROLE_REP){
            // do nothing.
          }
      }
      else printf("Unknown pkt_type %d at c%d\n",p.type,controllerid);

      myq.pop();
    }
  }
}
