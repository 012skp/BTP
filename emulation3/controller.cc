// Controller's Code

#include<bits/stdc++.h>
#ifndef __CORE_H__
#include "core.h"
#endif
using namespace std;

void controller_load_informating(int controllerid);
void controller_load_balancing_decision_maker(int controllerid);


// Load Collection every 1000 microsecond.
void thread_controller_load_balancing(int controllerid){
    printf("Controller %d Load Balncing is up\n",controllerid);
    char filename[20];
    memset(filename,0,20);
    strcat(filename,"c");
    sprintf(filename+1,"%d",controllerid);
    strcat(filename,"_load");
    FILE *fp = fopen(filename,"w");
    Controller &myc = controllers[controllerid];
    queue<Packet> &myq = myc.q;
    mutex *myqlock = myc.qlock;
    while(1){
      if(myc.terminate){
        printf("Controller %d Load Balncing exited\n",controllerid);
        break;
      }
      myqlock->lock();
      int qsize = myq.size();
      myqlock->unlock();
      assert(qsize <= myc.max_queue_size);

      myc.pkt_count_lock->lock();
      myc.current_load = myc.pkt_count*10;
      myc.pkt_count = 0;
      myc.pkt_count_lock->unlock();

      //printf("C[%d] time:%lf => load = %d\n",controllerid,current_time(),myc.current_load);

      // Copy switches' load from switch_pkt_count
      // and reset the switch_pkt_count. Load_balancer
      // will use switch_load data to find the switch 
      // to migrate.
      myc.switch_pkt_count_lock->lock();
      auto itr = myc.switch_pkt_count.begin();
      while(itr!=myc.switch_pkt_count.end()){
        myc.switch_load[itr->first] = itr->second;
        itr->second = 0;
        itr++;
      }
      myc.switch_pkt_count_lock->unlock();



      controller_load_informating(controllerid);

      // No load_migration if load migration is already in process.
      if(myc.load_migration_in_process == false){
        // After load migration is acknowledged,
        // wait for 1 second.
        struct timeval t;
        gettimeofday(&t,NULL);
        long td = time_diff(t,myc.load_migrated_time);
        if(td >= 1000000)
          controller_load_balancing_decision_maker(controllerid);
      }

      #ifdef PRINT
      printf("C[%d] time:%lf => current_load = %d\n",controllerid, current_time(),myc.current_load);
      #endif

      fprintf(fp,"%lf %d\n",current_time(),myc.current_load);
      usleep(100000); // period 100 milli second.


    }
}


// TODO different links for different controllers.
void controller_load_informating(int controllerid){
  Controller &myc = controllers[controllerid];
  
  
  
  if(abs(myc.load_informed - myc.current_load) > myc.allowed_load_deviation){
    // Do load broadcasting...
    Packet p;
    p.src = "c"+ to_string(controllerid);
    p.type = CONTROL;
    p.subtype = LOAD_BROADCAST;
    broadcast_data *bd = (broadcast_data*)malloc(sizeof(broadcast_data));
    bd->data = new int(myc.current_load);
    bd->counter = controllers.size()-1;
    p.data = (void*)bd;

    for(int i=0;i<controllers.size();i++){
      if(i==controllerid) continue;
      p.dst = "c"+ to_string(i);
      gettimeofday(&p.start_time,NULL);

      // Forwarding table may get changed by processing thread
      // Hence lock it before reading.
      myc.my_dvt_lock->lock();
      int lid = myc.forwarding_table[p.dst];
      myc.my_dvt_lock->unlock();

      links[lid].qlock->lock();
      links[lid].q.push(p);
      links[lid].qlock->unlock();
    }
    myc.load_informed = myc.current_load;
  }
}


void controller_load_balancing_decision_maker(int controllerid){
  if(controllers.size() == 1) return;
  Controller &myc = controllers[controllerid];

      // Get load_collections lock before reading it.
      // Because it may be updated my thread_controller_processing.
      mutex *lc_lock = myc.lc_lock;
      lc_lock->lock();
      for(int i=0;i<myc.load_collections.size();i++){
        if(i == controllerid) continue;
        if(myc.current_load <= myc.load_collections[i]){
          lc_lock->unlock();
          return ;           // If not heaviest no job.
        }
      }
      lc_lock->unlock();


      // This is the heaviest loaded controller do load_balancing.
      if(myc.current_load > myc.current_threshold){
            #ifdef PRINT_CONTROL
            printf("C[%d] time:%lf => current_load = %d, current_threshold = %d\n",
                            controllerid, current_time(),myc.current_load,
                            myc.current_threshold);
            #endif

            // Search for lightest loaded controller.
            int lowest_cid = -1;    // lowest loaded controllerid.
            int lowest_load = myc.current_load;

            lc_lock->lock();
            for(int i=0;i<myc.load_collections.size();i++){
              if(i == controllerid) continue;
              if(myc.load_collections[i] < lowest_load){
                lowest_load = myc.load_collections[i];
                lowest_cid = i;
              }
            }
            lc_lock->unlock();
            assert(lowest_cid != -1);


            // If there exists enough gap between current_threshold and lowest_load.
            if(lowest_load <= myc.alpha*myc.current_threshold){
              // Search for switch that satisfies
              // load < (heaviest_load-lowest_load)/2 -----(1)
              int desired_switch_load = (myc.current_load-lowest_load)/2;


              
              // Iterate through switch's load and find the desired switch.
              auto itr = myc.switch_load.begin();
              string the_switch = "";
              int load = -1;

              // try to find first switch that satisfies (1)
              while(itr != myc.switch_load.end()){
                if(itr->second <= desired_switch_load){
                  the_switch = itr->first;
                  load = itr->second;
                  break;
                }
                itr++;
              }

              // If couldn't find desired switch retun.
              if(load == -1){
                #ifdef PRINT_CONTROL
                printf("C[%d] time:%lf => couldn't find desired switch\n",controllerid,current_time());
                #endif

                return;
              }

              itr++;

              // Now try to find switch with heighest load satifying (1).
              while(itr != myc.switch_load.end()){
                if( itr->second <= desired_switch_load && itr->second > load){
                      load = itr->second;
                      the_switch = itr->first;
                }
                itr++;
              }


              if(load == 0){
                #ifdef PRINT_CONTROL
                printf("C[%d] time:%lf => couldn't find desired switch\n",controllerid,current_time());
                #endif

                return;
              }


              // Now we have heaviest loaded switch satisying (1).
              // Do switch migration. Send CONTROL Packet with LOAD_MIGRATION subtype

              #ifdef PRINT_CONTROL
              printf("C[%d] time:%lf => controller selected = %d, switch selected =%d\n",
                              controllerid, current_time(),lowest_cid,getid(the_switch));
              #endif

              int *sid = new int(getid(the_switch));
              //  Make sure to free the memroy upon reception of packet.
              Packet p;
              p.src = "c" + to_string(controllerid);
              p.dst = "c" + to_string(lowest_cid);
              p.type = CONTROL;
              p.subtype = LOAD_MIGRATION;
              p.data = (void*)sid;
              gettimeofday(&p.start_time,NULL);


              myc.my_dvt_lock->lock();
              int lid = myc.forwarding_table[p.dst];
              myc.my_dvt_lock->unlock();

              links[lid].qlock->lock();
              links[lid].q.push(p);
              links[lid].qlock->unlock();

              // set load_migration_in_process true;
              myc.load_migration_in_process = true;
              #ifdef PRINT_CONTROL
              printf("C[%d] transferring load of switch %d to controller %d\n",
                                              controllerid,getid(the_switch),
                                              lowest_cid);
              #endif
            }
            else {
              // Broadcast NEW_THRESHOLD...
              #ifdef PRINT_CONTROL
              printf("C[%d] couldn't balance load, broadcasting new threshold\n",controllerid);
              #endif

              Packet  p;
              p.src = "c" + to_string(controllerid);
              p.type = CONTROL;
              p.subtype = NEW_THRESHOLD;
              broadcast_data *bd = (broadcast_data*)malloc(sizeof(broadcast_data));
              // Set current_load as new_threshold.
              bd->data = new int(myc.current_load);
              bd->counter = controllers.size()-1;
              p.data = (void*)bd;

              // make sure to free the memory when counter reaches 0.

              for(int i=0;i<controllers.size();i++){
                if(i == controllerid) continue;
                p.dst = "c" + to_string(i);
                gettimeofday(&p.start_time,NULL);

                myc.my_dvt_lock->lock();
                int lid = myc.forwarding_table[p.dst];
                myc.my_dvt_lock->unlock();

                links[lid].qlock->lock();
                links[lid].q.push(p);
                links[lid].qlock->unlock();

              }
              
              myc.current_threshold = myc.current_load;
            }

      }
      
      if(myc.current_load > myc.base_threshold){
        bool  heaviest_loaded = true;

        mutex *lc_lock = myc.lc_lock;
        lc_lock->lock();
        int heaviest_load = myc.current_load;
        for(int i=0;i<myc.load_collections.size();i++){
          if(i==controllerid) continue;
          if(myc.load_collections[i] > heaviest_load){
            heaviest_loaded = false;
            break;
          }
        }
        lc_lock->unlock();

        // If heaviest_loaded check load_gap.
        if(heaviest_loaded && myc.current_threshold-heaviest_load > myc.max_load_gap){
          // Do load adjustment.
          myc.current_threshold = heaviest_load;
          // Broadcast NEW_THRESHOLD equal to heaviest_load.
          Packet  p;
          p.src = "c" + to_string(controllerid);
          p.type = CONTROL;
          p.subtype = NEW_THRESHOLD;
          broadcast_data *bd = (broadcast_data*)malloc(sizeof(broadcast_data));
          // Set current_load or heaviest_load as new_threshold.
          bd->data = new int(myc.current_load);
          bd->counter = controllers.size()-1;
          p.data = (void*)bd;
          // make sure to free the memory when counter reaches 0.

          
          
          for(int i=0;i<controllers.size();i++){
            if(i == controllerid) continue;
            p.dst = "c" + to_string(i);
            gettimeofday(&p.start_time,NULL);

            myc.my_dvt_lock->lock();
            int lid = myc.forwarding_table[p.dst];
            myc.my_dvt_lock->unlock();

            links[lid].qlock->lock();
            links[lid].q.push(p);
            links[lid].qlock->unlock();
          }

        }

      }


}




void thread_controller_processing(int controllerid){
  printf("Controller %d Processing Thread is up\n",controllerid);
  Controller &myc = controllers[controllerid];
  queue<Packet> &myq = myc.q;
  mutex *myqlock = myc.qlock;
  // Get packets from queue and process them.
  while(1){
    if(myc.terminate){
      printf("Controller %d Processing Thread exited\n",controllerid);
      break;
    }
    bool myqempty;
    myqlock->lock();
    myqempty = myq.empty();
    myqlock->unlock();

    if(!myqempty){

      myqlock->lock();
      Packet p = myq.front();
      myqlock->unlock();

      #ifdef PRINT
      printf("C[%d] time:%lf => packetid = %lld, src = %s, dst = %s, type = %s\n",controllerid,current_time(),
                          p.packetid,p.src.c_str(),p.dst.c_str(),TYPE(p.type).c_str());
      #endif


            // If this is ROUTING packet.
      if(p.type == ROUTING){
        // received_dist_vector_table.

        mutex *r_dvt_lock =NULL;
        if(p.src[0] == 's') r_dvt_lock = switches[getid(p.src)].my_dvt_lock;
        else r_dvt_lock = controllers[getid(p.src)].my_dvt_lock;


       
        assert(p.data);

        // lock before u read
        r_dvt_lock->lock();
        dist_vector_table r_dvt = *(dist_vector_table*)p.data;
        r_dvt_lock->unlock();

        mutex *my_dvt_lock = myc.my_dvt_lock;
        my_dvt_lock->lock();
        bool already_unlocked = false;





        // If this version of dist_vector_table already updated ignore it.
        if(myc.dvt_version[p.src] >= r_dvt.version); // ignore
        else{
          // Compare and update my_dvt using this received dvt.
          bool updated = false;
          for(int i=0; i<r_dvt.row.size(); i++){
            // row is tuple containing {dst,hop_count,linkid}
            dist_vector &r_dv = r_dvt.row[i];
            string dst = r_dv.dst;
            if(dst == myc.own_name) continue;

            // search for dst in my_dvt.
            bool new_dv_entry = true;

            for(int j=0; j<myc.my_dvt.row.size();j++){
              dist_vector &my_dv = myc.my_dvt.row[j];
              if(my_dv.dst == dst){
                if(my_dv.hop_count > r_dv.hop_count+1){
                  // update my_dv.
                  my_dv.hop_count = r_dv.hop_count+1;
                  my_dv.linkid = myc.forwarding_table[p.src];
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
              new_dv.linkid = myc.forwarding_table[p.src];
              myc.my_dvt.row.push_back(new_dv);
              updated = true;
            }

          }


          if(updated){
            // If my_dvt has been updated, update the forwarding_table.
            for(int i=0;i<myc.my_dvt.row.size();i++){
              dist_vector &dv = myc.my_dvt.row[i];
              if(dv.dst == myc.own_name) continue;
              myc.forwarding_table[dv.dst] = dv.linkid;
            }

            #ifdef PRINT
            printf("Routing table updated\n");
            routing_table_info("s"+to_string(controllerid));
            #endif

            // Broadcast new dvt.
            /*
            mys.my_dvt.version++;
            Packet np;
            np.src = mys.own_name;
            np.dst = "";
            np.type = ROUTING;
            np.data = (void*)&mys.my_dvt;

            my_dvt_lock->unlock();
            already_unlocked = true;

            for(int i=0;i<mys.connected_links.size();i++){
              gettimeofday(&np.start_time,NULL);

              links[mys.connected_links[i]].qlock->lock();
              links[mys.connected_links[i]].q.push(np);
              links[mys.connected_links[i]].qlock->unlock();

            }
            */

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


      else if(p.type == PACKET_IN){
          // processing_time = random(min_processing_time,max_processig_time).
          int processing_time = myc.avg_processing_time;
          //random(myc.min_processing_time, myc.max_processing_time);
          usleep(processing_time);

          // send PACKET_OUT message
          Packet np;
          np.type = PACKET_OUT;
          np.src = p.dst;
          np.dst = p.src;
          np.packetid = p.packetid;
          gettimeofday(&np.start_time,NULL);
          links[myc.forwarding_table[np.dst]].qlock->lock();
          links[myc.forwarding_table[np.dst]].q.push(np);
          links[myc.forwarding_table[np.dst]].qlock->unlock();
      }
      else if(p.type == CONTROL){
          // Handle LOAD_MIGRATION
          if(p.subtype == LOAD_MIGRATION){
            // Retrive the switch id.
            int sid = *(int*)p.data;
            if(p.data) free(p.data);

            // Create a new packet with ROLE_REQ type and send it to the switch.
            Packet np;
            np.type = CONTROL;
            np.subtype = ROLE_REQ;
            np.src = "c" + to_string(controllerid);
            np.dst = "s" + to_string(sid);
            gettimeofday(&np.start_time,NULL);

            links[myc.forwarding_table[np.dst]].qlock->lock();
            links[myc.forwarding_table[np.dst]].q.push(np);
            links[myc.forwarding_table[np.dst]].qlock->unlock();
            #ifdef PRINT_CONTROL
            printf("C[%d] time:%lf => Sending ROLE_REQ packet to switch %d\n",controllerid,current_time(),
                                sid);
            #endif

          }
          else if(p.subtype == LOAD_MIGRATION_ACK){
            // Now load migration has completed.
            myc.load_migration_in_process = false;
            gettimeofday(&myc.load_migrated_time,NULL);
          }
          else if(p.subtype == ROLE_REQ_ACK){
            #ifdef PRINT_CONTROL
            printf("C[%d] time:%lf => received ROLE_REQ_ACK packet from switch %d\n",controllerid,current_time(),
                                getid(p.src));
            #endif

            // Send LOAD_MIGRATION_ACK to src controller.
            Packet np;
            int previous_controllerid = *(int*)p.data;
            if(p.data) free(p.data);
            np.type = CONTROL;
            np.subtype = LOAD_MIGRATION_ACK;
            np.dst = "c" + to_string(previous_controllerid);
            np.src =  myc.own_name;
            gettimeofday(&np.start_time,NULL);

            links[myc.forwarding_table[np.dst]].qlock->lock();
            links[myc.forwarding_table[np.dst]].q.push(np);
            links[myc.forwarding_table[np.dst]].qlock->unlock();
          }

          // Handle NEW_THRESHOLD
          else if(p.subtype == NEW_THRESHOLD){
            // Retrive new_threshold value.
            int new_th = *(((broadcast_data*)p.data)->data);

            #ifdef PRINT_CONTROL
            printf("C[%d] time:%lf => New threshold received %d\n",controllerid,current_time(),new_th);
            #endif

            // Decrement the counter.
            ((broadcast_data*)p.data)->counter--;
            int counter = ((broadcast_data*)p.data)->counter;
            if(counter <= 0){
              // Free new_th memory
              free(((broadcast_data*)p.data)->data);
              free(p.data);
            }

            // Set new_th.
            myc.current_threshold = new_th;
          }

          // Handle LOAD_BROADCAST
          else if(p.subtype == LOAD_BROADCAST){
            int load =  *(((broadcast_data*)p.data)->data); // controller's load
            int scid =    getid(p.src);                     // src controller's id
            myc.lc_lock->lock();
            myc.load_collections[scid] =  load;
            myc.lc_lock->unlock();

            // Decrement the counter.
            ((broadcast_data*)p.data)->counter--;
            int counter = ((broadcast_data*)p.data)->counter;
            if(counter == 0){
              // Free new_th memory
              free(((broadcast_data*)p.data)->data);
              free(p.data);
            }
          }

          else printf("Unknown pkt_subtype %s under CONTROL at c%d\n",SUBTYPE(p.subtype).c_str(),controllerid);
      }
      else printf("Unknown pkt_type %s at c%d\n",TYPE(p.type).c_str(),controllerid);

      mutex *switch_pkt_count_lock = myc.switch_pkt_count_lock;

      myqlock->lock();
      myq.pop();
      myqlock->unlock();



    }
    else{
      usleep(0);
    }


  }
}
