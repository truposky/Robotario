#ifndef ROBOT_HH
#define ROBOT_HH

#include <string>
/* se crea un clase robot para poder manejar los diferentes parametros
de los robots y las instrucciones que se desean hacer*/
using namespace std;



class Robot
{
    private:
        int ID ;
        string ip;
        string port; 
        

    public:
        double radWheel=3.35;
        void SetupRobotData(int,string,string);
        void SetupConection(int& ,string& ,string&);
        //void rightWheel(wheel a);
        //void leftWheel(wheel b);
        void IMU();

};





class wheel
{
    friend class Robot;
    private:
        int N=20;//encoder resolution
        int R=6;//wheel radius
    public:
        double angularSpeed();
        double linearSpeed();

};

#endif