#ifndef ROBOT_HH
#define ROBOT_HH

#include <string>
/* se crea un clase robot para poder manejar los diferentes parametros
de los robots y las instrucciones que se desean hacer*/
using namespace std;

const float RAD_WHEEL=3.35;
const int WHEEL_RESOLUTION=20;

class Robot
{
    private:
        int ID ;
        string ip;
        string port; 

    public:
        void SetupRobotData(int,string,string);
        void SetupConection(int& ,string& ,string&);
        void linearVelocity();
	void angularVelocity();
        
        void IMU();

};

class Wheel: public Robot
{
	private:
		float radWheel=3.35;//cm
		float l=12.4;//cm

	public:
		void angularSpeed(float&,float&);
}
		float A=[2][2]={{L/(2*R),1/R, {-L/(2*R),1/R}};//inverse matrix for compute the angular velocities of every wheel
	public:
		void angularSpeed(float&,float&);
}



#endif
