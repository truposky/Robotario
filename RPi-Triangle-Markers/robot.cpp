#include "robot.hh"

void Robot::SetupRobotData(int a,string b, string c){
    ID=a;
    ip=b;
    port=c;

}

void Robot::SetupConection(int &id,string &IP,string &P){
    id=ID;
    IP=ip;
    P=port;
}