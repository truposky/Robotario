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
class Wheel: public Robot
{
	private:
		float R=3.35;//cm
		const float L=12.4;//cm
class Wheel: public Robot
{
	private:
		float radWheel=3.35;//cm
		float l=12.4;//cm

	public:
		void angularSpeed(float&,float&);
}
		float A=[2][2]={{L/(2*R),1/L, {-L/(2*R),1/L}};//inverse matrix for compute the angular velocities of every wheel
	public:
		void angularSpeed(float&,float&);
}
Wheel(float&,float V,float W){

  	for(int i=0; i <2;i++){
		for(int j=0;j<2;j++){
			
		}
	}

}
