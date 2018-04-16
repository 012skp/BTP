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
      assert(qsize <= myc.max_queue_size+5);

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
        myc.switch_load[itr->first] = itr->second*10;
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
    bd->data = (void*)malloc(sizeof(int));
    *(int*)bd->data = myc.current_load;
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
      // Calculate beta values for all controllers.
      vector<pair<double,int> > beta_values(controllers.size());
      mutex *lc_lock = myc.lc_lock;
      lc_lock->lock();
      for(int i=0;i<myc.load_collections.size();i++){
        beta_values[i] =  mp((1.0*myc.load_collections[i])/(myc.alpha*controllers[i].base_threshold),i);
      }
      lc_lock->unlock();
      beta_values[controllerid] =  mp((1.0*myc.current_load)/(myc.base_threshold*myc.alpha),controllerid);

      sort(beta_values.begin(),beta_values.end());
      if(beta_values[controllers.size()-1].second != controllerid)
        return;

      // This is the heaviest loaded controller having max beta value do load_balancing.
      double current_threshold = myc.base_threshold*myc.alpha;
      if(myc.current_load > current_threshold or myc.alpha >  1){

            // Search for lightly loaded controller.
            // condition for a controller being lightly loaded 
            // is its beta_value should be less than gama

            int idx = 0;
            for(idx=0;idx<beta_values.size();idx++){
              if(beta_values[idx].first < myc.gama) 
                printf("C[%d] time:%lf =>  controller %d is lightly loaded %lf\n",
                    controllerid, current_time(),beta_values[idx].second,beta_values[idx].first);
              else break;
            }

            beta_values.erase(beta_values.begin()+idx,beta_values.end());
            // Now beta_value contains only lightly loaded controllers. 

            // Create a mapping of switch-controller pair;
            vector< pair<    pair<int,int>,pair<int,int>    > > mapping;

            auto itr = myc.switch_load.begin();
            while(itr != myc.switch_load.end()){
              int sid = getid(itr->first);
              int sload = itr->second;
              if(sload == 0){ itr++; continue;}

              for(int i=0;i<beta_values.size();i++){
                int cid = beta_values[i].second;
                double current_threshold = controllers[cid].base_threshold*myc.alpha;
                int cload = current_threshold*beta_values[i].first;

                if(sload < current_threshold-cload){
                  int hop_count_target = get_hop_count("s"+to_string(sid), "c"+to_string(cid));
                  int hop_count_current = get_hop_count("s"+to_string(sid),
                                          "c"+to_string(switches[sid].controllerid));
                  int hop_count_diff = hop_count_target-hop_count_current;
                  mapping.push_back(mp(mp(hop_count_diff,sload),mp(sid,cid))); 
                }

              }

              itr++;
            }
            if(mapping.size() > 0){
              #ifdef PRINT_CONTROL
              printf("Possible pairs are-\n");
              for(int i=0;i<mapping.size();i++){
                printf("C[%d] - S[%d] hcd = %d, load = %d\n",mapping[i].second.second,mapping[i].second.first,
                                            mapping[i].first.first,mapping[i].first.second);
              }
              #endif
              sort(mapping.begin(),mapping.end());
              // Select the pair with with minimum hop_count_diff.
              // if there exists two minimum hop_count_diff then select
              // the pair with max switch load;

              int hcd = mapping[0].first.first; // min hop_count_diff;
              idx = 0;
              while(mapping[idx].first.first == mapping[0].first.first) idx++;
              idx--;
              // Selected switch-controller pair is mapping[idx].
              int the_switch = mapping[idx].second.first;
              int the_controller = mapping[idx].second.second;

            

              

              printf("C[%d] time:%lf => current_load = %d, current_threshold = %d\n",
                            controllerid, current_time(),myc.current_load,
                            (int)current_threshold);

              printf("C[%d] time:%lf => controller selected = %d, switch selected =%d\n",
                              controllerid, current_time(),the_controller
                              ,the_switch);
              

              // Do switch migration. Send CONTROL Packet with LOAD_MIGRATION subtype

              int *sid = (int*)malloc(sizeof(int));
              *sid = the_switch;
              //  Make sure to free the memroy upon reception of packet.
              Packet p;
              p.src = "c" + to_string(controllerid);
              p.dst = "c" + to_string(the_controller);
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

            }
            else {



              bool LB = false;
              double new_alpha = (1.0*myc.current_load)/myc.base_threshold;
              if(new_alpha > myc.alpha){
                myc.alpha = new_alpha>=1?new_alpha:1;
                LB = true;
              }
              else if(new_alpha < myc.alpha && (myc.alpha-new_alpha) > myc.max_load_gap){
                myc.alpha = new_alpha;
                LB = true;
              }

              if(LB){

                printf("C[%d] time:%lf => current_load = %d, current_threshold = %d\n",
                            controllerid, current_time(),myc.current_load,
                            (int)current_threshold);
  
                printf("C[%d] Broadcasting new threshold alpha = %lf\n",controllerid,myc.alpha);


                // Broadcast NEW_THRESHOLD...
                Packet  p;
                p.src = "c" + to_string(controllerid);
                p.type = CONTROL;
                p.subtype = NEW_THRESHOLD;
                broadcast_data *bd = (broadcast_data*)malloc(sizeof(broadcast_data));
                // Set current_load as new_threshold.
                bd->data = (void*)malloc(sizeof(double));
                *(double*)bd->data = myc.alpha;
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
            double new_alpha = *( (double*) ((broadcast_data*)p.data)->data);

            #ifdef PRINT_CONTROL
            printf("C[%d] time:%lf => New alpha received %lf\n",controllerid,current_time(),new_alpha);
            #endif

            // Decrement the counter.
            ((broadcast_data*)p.data)->counter--;
            int counter = ((broadcast_data*)p.data)->counter;
            if(counter <= 0){
              free(p.data);
            }

            // Set new_th.
            myc.alpha = new_alpha;
          }

          // Handle LOAD_BROADCAST
          else if(p.subtype == LOAD_BROADCAST){
            int load =  *( (int*) ((broadcast_data*)p.data)->data); // controller's load
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
