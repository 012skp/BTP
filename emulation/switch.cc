// Switchs codes...

void thread_switch_processing(int switchid){
  printf("thrad switch %d\n",switchid);
  Switch &mys = switches[switchid];
  queue<Packet> &myq = mys.q;
  mutex *myqlock = mys.qlock;
  while(1){
    myqlock->lock();
    bool myqempty;
    myqlock->unlock();
    while(!myqempty){
      lps_lock.lock();
      gettimeofday(&latest_packet_seen,NULL);
      lps_lock.unlock();

      myqlock->lock();
      Packet &p = myq.front();
      myqlock->unlock();

      // If this is ROUTING packet.
      if(p.type = ROUTING){
        printf("received a ROUTING packet from %s version %d\n",p.src.c_str(),
            ((dist_vector_table*)p.data)->version);
        // received_dist_vector_table.

        // Lock both r_dvt and my_dvt,
        // otherwise may fall into deadlock.
        mutex *r_dvt_lock = switches[getid(p.src)].my_dvt_lock;
        mutex *my_dvt_lock = mys.my_dvt_lock;
        atomic_lock.lock();
        r_dvt_lock->lock();
        my_dvt_lock->lock();
        atomic_lock.unlock();

        dist_vector_table *r_dvt = (dist_vector_table*)p.data;
        assert(r_dvt);
        // If this version of dist_vector_table already updated ignore it.
        if(mys.dvt_version[getid(p.src)] >= r_dvt->version); // ignore
        else{
          // Compare and update my_dvt using this received dvt.
          bool updated = false;
          for(int i=0; i<r_dvt->row.size(); i++){
            // row is tuple containing {dst,hop_count,linkid}
            dist_vector &r_dv = r_dvt->row[i];
            string dst = r_dv.dst;
            // search for dst in my_dvt.
            bool new_dv_entry = true;

            for(int j=0; j<mys.my_dvt.row.size();j++){
              dist_vector &my_dv = mys.my_dvt.row[i];
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
            // If my_dvt has been updated update the forwarding_table.
            for(int i=0;i<mys.my_dvt.row.size();i++){
              dist_vector &dv = mys.my_dvt.row[i];
              mys.forwarding_table[dv.dst] = dv.linkid;
            }

            // Broadcast new dvt.

            mys.my_dvt.version++;
            Packet np;
            np.src = mys.own_name;
            np.dst = "";
            np.type = ROUTING;
            np.data = (dist_vector_table*)&mys.my_dvt;

            for(int i=0;i<mys.connected_links.size();i++){
              // don't send it to controller or the switch from which dvt was received.
              if(links[mys.connected_links[i]].dst[0] == 'c') continue;
              //if(links[mys.connected_links[i]].dst == p.src) continue;
              gettimeofday(&np.start_time,NULL);

              links[mys.connected_links[i]].qlock->lock();
              links[mys.connected_links[i]].q.push(np);
              links[mys.connected_links[i]].qlock->unlock();

            }

          }



        }

        atomic_lock.lock();
        r_dvt_lock->lock();
        my_dvt_lock->lock();
        atomic_lock.unlock();

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
          Packet np = mys.buffer[p.packetid];
          mys.buffer.erase(p.packetid);

          //TODO transmissin dealy not implemented.
          gettimeofday(&np.start_time,NULL);
          links[mys.forwarding_table[np.dst]].qlock->lock();
          links[mys.forwarding_table[np.dst]].q.push(np);
          links[mys.forwarding_table[np.dst]].qlock->unlock();
        }
      }
      else{
        // This packet isn't for me, simply forward it.
        // Check for entry in flow table.
        bool entry = rand()%10<7?true:false;
        if(entry){
          // If flow table hit, forwad it.
          // TODO transmission delay not implemented.
          gettimeofday(&p.start_time,NULL);
          links[mys.forwarding_table[p.dst]].qlock->lock();
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
          links[mys.forwarding_table[np.dst]].qlock->lock();
          links[mys.forwarding_table[np.dst]].q.push(np);
          links[mys.forwarding_table[np.dst]].qlock->unlock();

        }
      }
      // all processing done pop the packet.
      myq.pop();
    }
    // no packets in queue.
    if(emulation_done){
      struct timeval t;
      gettimeofday(&t,NULL);
      if(time_diff(t,latest_packet_seen)>max_delay) break;
    }
    usleep(1000); // sleep for 1 milli second
  }
}
