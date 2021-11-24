#include <string>
#include <iostream>
#include <cstring>
using namespace std;
int main(){


    char data[256];
   double vel=20;
   char velc[sizeof(vel)];
    char del=',';
    snprintf(data,sizeof(vel),"%2.4f",vel);     
     snprintf(velc,sizeof(vel),"%2.4f",vel);
    strcat(data,&del); 
    strcat(data,velc);  
    cout<<data<<endl;
    return 0;
}