
#include "udp.cpp"

using namespace std;

#define MAXDATASIZE 256
int main(){
    char hola[] ="hola\0";
    char *ho=hola;
    int id=0;
    string ip="192.168.1.2";
    string port="4050";
    
   UDP prueba;
   
   prueba.initTalkerSocket(ip,port);
   while(true){
   prueba.SendTalkerSocket(ho,strlen(hola));
   sleep(1);
   }
    return 0;
    
}