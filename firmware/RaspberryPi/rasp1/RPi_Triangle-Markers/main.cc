/*
** listener.c -- a datagram sockets "server" demo
*/


/*
 * Copyright (c) 2019 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include "common.hh"
#include "udp.cpp"
#include "misc.cc"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pthread.h> 
#include <time.h> 
#include <math.h> 
#include <fstream>
#include <bits/stdc++.h>
#include <termios.h>    // POSIX terminal control definitions
using namespace std;
using namespace tinyxml2;



namespace {
const char* about = "Pose estimation of ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0'}"
	;
}
#define MYPORT "4241"   // the port users will be connecting to
#define PORTBROADCAST "6868"
#define MAXBUFLEN 256
#define SAMPLINGTIME 20000 // in usec
#define MAXSINGNALLENGTH 4000
#define CENTER 320 //this is the setpoint for a distance between markers of 30 cm (in degree)
const float KP=0.01075;
// get sockaddr, IPv4 or IPv6:
 char buf[MAXDATASIZE];
string convertToString(char* a, int size)
{
    string s = a;
    return s;
}
//----prototipos-----//
void error(const char *msg)
{
    perror(msg);
    exit(-1);
}

void tokenize(const string s, char c,vector<string>& v);//split the string 
void operationSend();//allow the user choose an instruction for send to the robot
void SetupRobots();//copy the information in the xml file to save in the class robot.
int broadcastRasp();
void *robotMove(void *arg);
void *dataAruco(void *arg);
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
void SetupRobots();

enum {r1, r2, r3, r4,r5};
     //definition of robots
Robot robot1,robot2,robot3,robot4;//se define la clase para los distintos robots.
UDP comRobot1,comRobot2,comRobot3,comServer;//comunication udp

struct record_data//struct for share information between threads
{
    int id; //id of every robot
    double x, y , z;
    int n;  //counter for know how many robots there are
    
};

list<record_data> arucoInfo;//list for save the information of all the arucos
list<record_data>::iterator it;
list<record_data>::iterator it2;

pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_t _Move, _detectAruco; ;



int main(int argc,char **argv)
{
    

    SetupRobots();//prepare the network data of every robot
    record_data data;//struct for save in linked list

    //init aruco code----------
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 1;
    }

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");

    float marker_length_m = parser.get<float>("l");
    int wait_time = 10;
    if (marker_length_m<= 0) {
        std::cerr << "marker length must be a positive value in meter" 
                  << std::endl;
        return 1;
    }
    cv::String videoInput = "0";//se selecciona la entrada de la camara
    cv::VideoCapture in_video;
    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
            parser.printMessage();
            return 1;
        }
     char* end = nullptr;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        if (!end || end == videoInput.c_str()) {
            in_video.open(videoInput); // url
            in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
			in_video.set(cv::CAP_PROP_FPS,40);
        } else {
            in_video.open(source); // id
        }
    } else {
        in_video.open(0);
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }
    cv::Mat grayMat;
    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
   
    int frame_width=in_video.get(cv::CAP_PROP_FRAME_WIDTH); 
    int frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT);  

    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
   
    cv::Mat rotated_image;
    //std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    //std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
     cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(frame_width,frame_height),1);
    //---------------------------------end aruco code-----
    arucoInfo.clear();
    //aruco::DetectorParameters detectorParams;
    pthread_create(&_detectAruco,NULL,dataAruco,NULL);//create thread for store the values of the markers
    
    while (in_video.grab())
    {
        in_video.retrieve(image);
        
	//cv::Mat for_Rotation = cv::getRotationMatrix2D(cv::Point2f((image.cols-1) / 2, (image.rows-1) / 2), (180), 1);//affine transformation matrix for the 2D rotation//
	//    cv::warpAffine(image,rotated_image,for_Rotation,image.size());
        
	cvtColor(image,grayMat,cv::COLOR_BGR2GRAY);
        

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(grayMat, dictionary, corners, ids);
        
            
        while(!arucoInfo.empty()){//avoid fill list more than markers there are
            arucoInfo.pop_back();
        }
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs, tvecs);
            
            /*std::cout << "Translation: " << tvecs[0]
                << "\tRotation: " << rvecs[0] 
                << std::endl;
            */
            // Draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                //cv::aruco::drawAxis(image, camera_matrix, dist_coeffs,
                //        rvecs[i], tvecs[i], 0.1);
                        
                data.x=tvecs[i](0);
                //cout<<"datax "<<data.x<<endl;
                
                data.y=tvecs[i](1);
                data.z=tvecs[i](2);
               // cout<<"data y "<<data.y<<endl;
                data.id=ids.at(i);
                pthread_mutex_lock(&mutex_);
                arucoInfo.push_back(data);
                 int status = pthread_mutex_unlock (&mutex_);
            if (status != 0)
                exit(status);
                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x1: " << std::setw(8) << tvecs[0](0);
                            
                cv::putText(image, vector_to_marker.str(),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z1: " << std::setw(8) << tvecs[0](2);
                cv::putText(image, vector_to_marker.str(),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x2: " << std::setw(8) << tvecs[1](0);
                cv::putText(image, vector_to_marker.str(),
                            cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
                           

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z2: " << std::setw(8) << tvecs[1](2);
                cv::putText(image, vector_to_marker.str(),
                            cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
            }
            
        }
   	video<<image;       
    /*  imshow("Pose estimation", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;*/
    }

    in_video.release();
    video.release();
   //// destroyALLWindows();
    pthread_exit(NULL);
    return 0;
}

void SetupRobots()
{
    // Read the sample.xml file
    XMLDocument Robotdoc;
    Robotdoc.LoadFile( "robots_info.xml" );

    XMLNode* Robotarium =Robotdoc.FirstChild();
    XMLElement *robot=Robotarium->FirstChildElement("robot");
    int i=0;
    while(robot !=NULL)
    {
        
        XMLElement *robotChild=robot->FirstChildElement("ID");
        int ID;
        robotChild->QueryIntText(&ID);
        cout<<"ID:"<<ID<<endl;
    
         robotChild=robot->FirstChildElement("IP");
         const char* ip=robotChild->GetText();
         string ss=ip;
         cout<<"ip:"<<ip<<endl;

        robotChild=robot->FirstChildElement("PORT");
        const char* port=robotChild->GetText();
        string p=port;
        cout<<"puerto:"<<p<<endl;
      
        robot=robot->NextSiblingElement("robot"); 
        switch (i)
        {
            case 0:
                robot1.SetupRobotData(ID,ss,p);
                break;
            case 1:
                robot2.SetupRobotData(ID,ss,p);
                break;
            case 2:
                robot3.SetupRobotData(ID,ss,p);
                break;
             case 3:
                robot4.SetupRobotData(ID,ss,p);
                break;

        }       
        i++;   
    }
}



int broadcastRasp(){
    
    
    int out=0;
    int sockfd;
    struct addrinfo hints, *servinfo,*p;
    int rv;
    int numbytes;
    struct sockaddr_storage their_addr;
    socklen_t addr_len;
    memset (&hints,0,sizeof(hints));
    hints.ai_family=AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags=AI_PASSIVE;//use my ip
    if((rv=getaddrinfo(NULL,PORTBROADCAST,&hints,&servinfo)) !=0){
        return -1;
    }
    for(p=servinfo;p != NULL; p=p->ai_next){
        if((sockfd=socket(p->ai_family, p->ai_socktype,p->ai_protocol)) == -1){
            perror("listener: socket");
            continue;
        }
    
        if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1){
            close(sockfd);
            perror("listener : bind");
            continue;
        }
        
        break;
    }
    
    if(p == NULL){
        cout<<"listener: failed to bind socket"<<endl;
        return -1;
    }
    
    cout<<"waiting broadcast"<<endl;
    addr_len=sizeof(their_addr);
    
    if((numbytes = recvfrom(sockfd,buf,MAXBUFLEN,0,(struct sockaddr *)&their_addr, &addr_len)) == -1){
        perror("recvfrom");
        return -1;
    }
     operation_recv=( struct appdata*)&buf;
     if((numbytes< HEADER_LEN) || (numbytes != operation_recv->len+HEADER_LEN) )
        {
            
            cout<<"(servidor) unidad de datos incompleta :"<<numbytes<<endl;
        }
        else
        {
              // relaiza operacion solicitada por el cliente 

            switch (operation_recv->op){
                case OP_SALUDO:
                    //cout<<" contenido "<<operation_recv->data<<endl;
                break;
                case OP_MESSAGE_RECIVE:
                    
                    //cout<<" contenido "<<operation_recv->data<<endl;
                break;
                case OP_VEL_ROBOT:

                break;
                case OP_BROADCAST:
                    out=1;
                    
            }
        }

   
    
    freeaddrinfo(servinfo);
    
    close(sockfd);
    return out;
}



void *dataAruco(void *arg)
{//thread function
   
    struct logo_data{
        double td;
        double wheel_vel[2];
        vector<int> id;
        vector<double> x;
        vector<double> y;
        vector<double> z;
        string timeStamp;
	
    };
    
    char backD,backI;	

    int n=0;
    string ipServer="192.168.78.2",portServer="4052";
    ofstream logo("log.txt");
    logo.close();//file created

    list<logo_data> savelog;
    list<logo_data>::iterator iter;
    logo_data info;
    
   // robot1.SetupConection(id,ip,port);//for now only use 1 robot for communication
    //in this case the experiment needs the velocity 
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

   
    double td;
    int cont=0;
    int idAux;


    //comunication server 
        comServer.initTalkerSocket(ipServer,portServer);

    //-----------------------------SE CREA COMUNICACION USB------------------
    int arduino = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);
    /* Error Handling */
    if ( tcgetattr ( arduino, &tty ) != 0 ) {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }
    /* Save old tty parameters */
    tty_old = tty;
    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);
    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  1;                  // read doesn't block
    tty.c_cc[VTIME]  =  1;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    /* Make raw */
    cfmakeraw(&tty);


     
   

    while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected
    unsigned char buff[249];
    char resultString[120]; 
    
    while((n=broadcastRasp() )!= 1);//Waiting for broadcast
    
    cout<<"start"<<endl;  

    //pthread_create(&_Move,NULL,robotMove,(void*)arduino);//thread for move the robot
    //send operation, arduino will send data until we say to stop it
    operation_send.op=OP_VEL_ROBOT;
    operation_send.len = sizeof (operation_send.data);
    write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);
    //delay 
    //usleep(500);

    tcflush( arduino, TCIFLUSH );
       if ( tcsetattr ( arduino, TCSANOW, &tty ) != 0) {
         std::cout << "Error " << errno << " from tcsetattr" << std::endl;
       }
     int numbytes;   

     //main code for read variables
    while(n<MAXSINGNALLENGTH){
       
        gettimeofday(&tval_before,NULL);
        
        tcflush( arduino, TCIFLUSH );
        if ( tcsetattr ( arduino, TCSANOW, &tty ) != 0) {
         std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        }	

       // memset(buff,'\0',MAXBUFLEN);
       
       	td=(double)n*SAMPLINGTIME/1000; 
        
        int l=0;
	unsigned char d;
       
	numbytes=0;
/*	while(numbytes=read(arduino,&d,1) != -1){
		buff[i]=d;
		
		i++;
		if( d == '\0') break;
	}*/
	for(int i=0;i<18;i++){

		numbytes=read(arduino,&d,1);
		buff[i]=d;
		l++;
		
	}
			


	info.wheel_vel[0] = bytesToDouble(&buff[0]);
        info.wheel_vel[1] = bytesToDouble(&buff[8]);
	backD= buff[16];
	backI= buff[17];	
	if(backD =='a'){
		info.wheel_vel[0]=-1*info.wheel_vel[0];
	}

	if(backI=='a'){

		info.wheel_vel[1]=-1*info.wheel_vel[1];
	}
      // cout<<l<<","<<info.wheel_vel[0]<<","<<info.wheel_vel[1]<<endl;
        cont=0;
	
        
        if(arucoInfo.size()>0)
        {
            
            for(it=arucoInfo.begin();it !=arucoInfo.end();it++)
            {
                
                info.id.push_back((*it).id);
                info.x.push_back((*it).x);
                info.y.push_back((*it).y);
                info.z.push_back((*it).z);
                idAux=it->id;     
                cont++;     
                
            }
        }
        if(cont == 0){
            info.id.push_back(900);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
            info.id.push_back(900);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
        }
        else if(cont==1){

            if(idAux==2){
                idAux=900;
            }
            else{
                idAux=900;
            }
            
            info.id.push_back(idAux);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
        }
	    info.td=td;
	    info.timeStamp=to_string(tval_before.tv_usec);
    

        snprintf(resultString, sizeof(resultString), "Robot2:Current time : %ld s :%ld us\n",tval_before.tv_sec, tval_before.tv_usec);
	    //comServer.SendTalkerSocket(resultString,sizeof(resultString));
	    savelog.push_back(info);//save info in list
	    //cout<<"centro: "<<cx<<"z"<<z<<" count :"<<count<<","<<info.timeStamp<<endl;
	
	    //----------------------------------------------------
            //cout<<"enviar VEL"<<endl;
	    
	    info.id.erase(info.id.begin(),info.id.end());
	    info.x.erase(info.x.begin(),info.x.end());
	    info.y.erase(info.y.begin(),info.y.end());
	    info.z.erase(info.z.begin(),info.z.end());
        n++;
       
        gettimeofday(&tval_after,NULL);
        timersub(&tval_after,&tval_before,&tval_sample);
        
        if( tval_sample.tv_usec<0)
        {
            error("error time");
        }
        else if (tval_sample.tv_usec>SAMPLINGTIME)
        {
            //error("time of program greater than sample time");
            cout<<"tiempo de programa mayor "<<tval_sample.tv_usec<<endl;
        }
        else
        {
	    
            usleep((unsigned int)((suseconds_t)SAMPLINGTIME-tval_sample.tv_usec));
            
        }
        
        
       
        
    }
    //operation for stop robot
    //pthread_cancel(_Move);
     double velocity_robot[2];
    double angularWheel[2];
    double vel=0;
    double w=0;
    velocity_robot[0]=w;
    velocity_robot[1]=vel;
    robot1.angularWheelSpeed(angularWheel,velocity_robot);
    cout<<"v: "<<vel<<"w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
    doubleToBytes(angularWheel[0], &operation_send.data[0]);
    doubleToBytes(angularWheel[1], &operation_send.data[8]);
    operation_send.op=OP_MOVE_WHEEL;
    operation_send.len = sizeof (operation_send.data);
            
    write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);

    //----------------------------------------------------
    
    //save data on logo.txt
   
    logo.open("log.txt");
    logo<<"robot1"<<endl;
    logo<<"td(ms),wD,wI,id,x,y,z,id,x,y,z"<<endl;
    for(iter=savelog.begin();iter !=savelog.end();iter++)
    {
        string wheelVel1=to_string(iter->wheel_vel[0]);
        string wheelVel2=to_string(iter->wheel_vel[1]);
        string X,Y,Z;
        
        logo<<iter->td<<","<<wheelVel1<<","<<wheelVel2;
        for(int i=0; i<iter->id.size();i++){
            X=to_string(iter->x.at(i));
            Y=to_string(iter->y.at(i));
            Z=to_string(iter->z.at(i));
            logo<<","<<iter->id.at(i)<<","<<X<<","<<Y<<","<<Z;

        }
        logo<<endl;
    }
    cout<<"data save"<<endl; 
    operation_send.op=OP_STOP_SERIAL;
    operation_send.len = sizeof (operation_send.data);
    write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);
    close(arduino);
    pthread_exit(NULL);
    return NULL;
}



void *robotMove(void *arg){
    _Move = pthread_self();


    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

    double vel=0;//linear velocity of robot
    double auxVel=0,auxW=0;
    int idRobot;   
    double w=0;//angular velocity of robot
    bool correction =false,forward;
    int count=0;
    double velocity_robot[2];
    double angularWheel[2];
    double cx;
    double z;
    int arduino=(int)arg;
    vel=7*3.35;
    const int SAMPLE_TIME=700000;//us
    
    while(1){
        
        gettimeofday(&tval_before,NULL);
        
        cx=0;
        count=0;
        if(arucoInfo.size()>0)
            {
               
                for(it2=arucoInfo.begin();it2 !=arucoInfo.end();it2++){
                    idRobot=(*it2).id;              
                    cx += (*it2).x;
                    z=(*it2).z;
                    count++;
                }
                cx=cx/2*100;// times 100 becouse is in meters and we need in cm
            }
//	cout<<"count: "<<count<<","<<count<<endl;
        if(count > 1){
            
   /*             if(cx>5 || cx < -5){

                    w=-KP*cx;
		    vel=vel+2;
                }
                else{
                    w=0;
                }*/
                
                if(z <= 0.71){
                    
                    vel= -7.8*3.35;
                    //w=-w; 
                    forward=false;
                }
                else if( z>1.1 ){
                    vel=7.8*3.35;
                    forward=true;
                }
                
                if (correction){
                    if( forward){
                    
                        vel=7.8*3.35;
                        correction=false;
                    }
                    else if( !forward){
                        vel=-7.8*3.35;
                        correction=false;
                    }
                }

                if(cx>5 || cx < -5){

                    w=-KP*cx;
		    if(vel>0){
		    vel=vel+2;
		    }
		    else if(vel<0){
			    //w=-w;
			    vel=vel-2;
		    }
		 }
                else{
                    w=0;
                }
                
                
                if((vel != auxVel || w != auxW) || correction){
                        auxVel=vel;
                        auxW=w;
                        //compute angular Wheel
                        velocity_robot[0]=w;
                        velocity_robot[1]=vel;
                        robot1.angularWheelSpeed(angularWheel,velocity_robot);
                        doubleToBytes(angularWheel[0], &operation_send.data[0]);
                        doubleToBytes(angularWheel[1], &operation_send.data[8]);
                        //send angular Wheel
                        operation_send.op=OP_MOVE_WHEEL;
                        operation_send.len = sizeof (operation_send.data);
                    cout<<"normal ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
                        write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);

                }
	    }
	    else {
                correction =true;
              /* // vel=0;
               // w=0;	
                velocity_robot[0]=w;
            	velocity_robot[1]=vel;
            	robot1.angularWheelSpeed(angularWheel,velocity_robot);
            	doubleToBytes(angularWheel[0], &operation_send.data[0]);
            	doubleToBytes(angularWheel[1], &operation_send.data[8]);

            	operation_send.op=OP_MOVE_WHEEL;
            	operation_send.len = sizeof (operation_send.data);
                cout<<"parada1 "<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
            	write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);
		*/
	    	    usleep(390000);
		        count=0;
		
        
        
                for(it2=arucoInfo.begin();it2 !=arucoInfo.end();it2++)
                {
                   
                    idRobot=it2->id;              
                   
                    
                        count++;
                } 
		
                
                if(count !=2){
                    
                    vel=0;

                    if(idRobot ==1){

                        w=15.8*robot2.R/robot2.L;
                    }
                    else{
                        
                        w=-15.8*robot2.R/robot2.L;
                    }
            
                
           
          
                    velocity_robot[0]=w;
                    velocity_robot[1]=vel;
                    

                    robot1.angularWheelSpeed(angularWheel,velocity_robot);
                    velocity_robot[0]=w;
                    velocity_robot[1]=vel;
                    

                    robot1.angularWheelSpeed(angularWheel,velocity_robot);
                    doubleToBytes(angularWheel[0], &operation_send.data[0]);
                    doubleToBytes(angularWheel[1], &operation_send.data[8]);
                    operation_send.op=OP_MOVE_WHEEL;
                    operation_send.len = sizeof (operation_send.data);
                    
                    cout<<"giro correccion "<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
                    
                    write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);
                    usleep(170000);
                    
                    w=0;
                    vel=0;
                    velocity_robot[0]=w;
                    velocity_robot[1]=vel;
                    robot1.angularWheelSpeed(angularWheel,velocity_robot);
                    doubleToBytes(angularWheel[0], &operation_send.data[0]);
                    doubleToBytes(angularWheel[1], &operation_send.data[8]);
                    operation_send.op=OP_MOVE_WHEEL;
                    operation_send.len = sizeof (operation_send.data);
                    cout<<"parada2 "<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<"forward "<<forward<<endl;
                    write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);
                //    usleep(95000);
                }
	   	 
            }
            
            gettimeofday(&tval_after,NULL);
            timersub(&tval_after,&tval_before,&tval_sample);
        
            if( tval_sample.tv_usec<0)
       	    {
           	 error("error time");
       	    }
       	    else if (tval_sample.tv_usec>SAMPLE_TIME)
            {
           	 //error("time of program greater than sample time");
           	 cout<<"(movimiento)tiempo de programa mayor "<<tval_sample.tv_usec<<endl;
       	    }
            else
           {
	    
          	  usleep((unsigned int)((suseconds_t)SAMPLE_TIME-tval_sample.tv_usec));
            
       	    }
        
    
    }
}
