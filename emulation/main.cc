#include "core.h"
#include "controller.cc"
#include "link.cc"

using namespace std;


int main(){

  Link l0;
  Switch s0;
  Controller c0;

  string linkname = "l0";


  l0.src = "s0";
  l0.dst = "c0";
  l0.special = true;
  links.push_back(l0);
  switches.push_back(s0);
  controllers.push_back(c0);

  thread th1(thread_link, linkname);
  // random packet generator..
  for(int i=0;i>-10;i++){
    Packet p;
    p.packet_id = i;
    p.src = "s0";
    p.dst = "c0";
    links[0].q.push(p);
    usleep(0);

  }
  th1.join();


}
