#include<bits/stdc++.h>
#include<thread>
#include<mutex>
using namespace std;

struct C{
  mutex *m1 = NULL;
};

int main(){
  C c;
  vector<C> vC;
  vC.push_back(c);
  vC.push_back(c);
  vC.push_back(c);
  vC.push_back(c);
  vC[0].m1 = new mutex();

  return 0;
}
