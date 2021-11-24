#ifndef ROBOT_HH
#define ROBOT_HH
#include "common.hh"

class udp
{
    private:
        char MYPORT[] = "4242";
        int MAXBUFLEN =255;
    public:
        char buf[MAXBUFLEN];
        int comRobot();
        voidf tokenize(const string s, char c, vector <string>& v);
        void operationSend();
        void *get_in_addr(struct sockaddr *sa);
        void SetupRobots();


}


#endif