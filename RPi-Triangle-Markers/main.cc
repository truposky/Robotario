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
        "{v        |<none>| Custom video source, otherwise '0' }"
        ;
}



#define MYPORT "4242"   // the port users will be connecting to
#define MAXBUFLEN 256
#define SAMPLINGTIME 500000 // in usec
#define MAXSINGNALLENGTH 5
#define CENTER 320 //this is the setpoint for a distance between markers of 30 cm (in degree)
const float KP=0.0185;
// get sockaddr, IPv4 or IPv6:
 char buf[MAXDATASIZE];
string convertToString(char* a, int size)
{
    string s = a;
    return s;
}
//----prototipos-----//
int comRobot(int id,string ip,string port,int instruction);//used for send and recive instructions and data for every robot
void tokenize(const string s, char c,vector<string>& v);//split the string 
void concatenateChar(char c, char *word);//not used for now
void operationSend();//allow the user choose an instruction for send to the robot
void SetupRobots();//copy the information in the xml file to save in the class robot.
int broadCast();
void error(const char *msg)
{
    perror(msg);
    exit(-1);
}
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
void SetupRobots();
void SerialCommunication(int id,string ip,string port,int instructions);//serial communication for robot same as udp

enum {r1, r2, r3, r4,r5};
     //definition of robots
Robot robot1,robot2,robot3,robot4;//se define la clase para los distintos robots.
struct record_data//struct for share information between threads
{
    int id; //id of every robot
    int cx; //center of very marker
    int n;  //counter for know how many robots there are
    float degree;
};

list<record_data> arucoInfo;//list for save the information of all the arucos
list<record_data>::iterator it;


pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;

float PixeltoDegree(int cx){
    float degree=(cx+256.47368)/6.38772;
    return degree;
}

void *dataAruco(void *arg)
{//thread function

    struct logo_data{
        double td;
        double wheel_vel[2];
        vector<int> id;
        vector<float> degree;
    };
    int id;
    string ip,port;
    ofstream logo("logo.txt");
    logo.close();//file created

    list<logo_data> savelog;
    list<logo_data>::iterator iter;
    logo_data info;
    robot2.SetupConection(id,ip,port);//for now only use 1 robot for communication
    //in this case the experiment needs the velocity 
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

    int n=0;

    double velocity_robot[2];
    double angularWheel[2];
    int meanPoint=0,auxId=0;
    float auxDegree=0;
    double td;
    while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected

    while(n<MAXSINGNALLENGTH){
        
        gettimeofday(&tval_before,NULL);
        td=(double)n*0.5; 
        
      // comRobot(id,ip,port,OP_VEL_ROBOT);//request for the velocity of the robot
        SerialCommunication(id,ip,port,OP_VEL_ROBOT);
        info.wheel_vel[0] = bytesToDouble(&operation_recv->data[0]);
        info.wheel_vel[1] = bytesToDouble(&operation_recv->data[8]);
	
	
        info.td=td;
        int cont=0;
        auxDegree=0;
        if(arucoInfo.size()>0)
        {
            for(it=arucoInfo.begin();it !=arucoInfo.end();it++)
            {
                info.id.push_back(it->id);
                info.degree.push_back(it->degree);
                cout<<"loop:"<<it->id<<","<<it->degree<<endl;
                meanPoint+=it->cx;
                
                auxId=it->id;
                if(cont ==0)
                {
                    auxDegree=it->degree;
                }
                else if(cont ==1)
                {
                    auxDegree=auxDegree-it->degree;
                    if(auxDegree<0)auxDegree=-auxDegree;
                }
                cont++;
                
            }
        }
        
        meanPoint=meanPoint/2;
        //cout<<"meanpoint: "<<meanPoint<<",degree:"<<auxDegree<<endl;
    
        meanPoint=0;
        savelog.push_back(info);
        info.id.erase(info.id.begin(),info.id.end());
        info.degree.erase(info.degree.begin(),info.degree.end());
	
        //-----------------------move--------------------------
       // memset (operation_send.data, '\0',MAXDATASIZE-HEADER_LEN);//is necesary for measure the length, strlen needs \0
      /*  velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot1.angularWheelSpeed(angularWheel,velocity_robot);
        cout<<"w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //----------------------------------------------------
        comRobot(id,ip,port,OP_MOVE_WHEEL);*/
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
           
            usleep(SAMPLINGTIME-tval_sample.tv_usec);
            
        }
        
        
       
        
    }
  
   /* robot1.angularWheelSpeed(angularWheel,velocity_robot);
    snprintf(operation_send.data,sizeof(angularWheel[0]),"%f",angularWheel[0]);     
    snprintf(wc,sizeof(angularWheel[1]),"%f",angularWheel[1]);
    strcat(operation_send.data,&del); 
    strcat(operation_send.data,wc); 
    //comRobot(id,ip,port,OP_MOVE_WHEEL);
    */
    //save data on logo.txt
    cout<<"data save"<<endl;
    logo.open("logo.txt");
    for(iter=savelog.begin();iter !=savelog.end();iter++)
    {
        string wheelVel1=to_string(iter->wheel_vel[0]);
        string wheelVel2=to_string(iter->wheel_vel[1]);
        logo<<iter->td<<","<<wheelVel1<<","<<wheelVel2;
        for(int i=0; i<iter->id.size();i++){
            logo<<","<<iter->id.at(i)<<","<<iter->degree.at(i);
        }
        logo<<endl;
    }
    return NULL;
}
int main(int argc,char **argv)
{
    pthread_t detectAruco;

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

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    
    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
 // Default resolutions of the frame are obtained.The default resolutions are system dependent.

	  int frame_width = in_video.get(cv::CAP_PROP_FRAME_WIDTH);

    int frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT);
    //std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    //std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
     cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(frame_width,frame_height));
    //---------------------------------end aruco code-----
    arucoInfo.clear();
    //pthread_create(&detectAruco,NULL,dataAruco,NULL);
    broadCast();
    while (in_video.grab())
    {
        in_video.retrieve(image);
        image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
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
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);

                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x: " << std::setw(8) << tvecs[0](0);
                            
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
            }
            //mutex
           pthread_mutex_lock(&mutex_);
            
            while(!arucoInfo.empty()){//avoid fill list more than markers there are
                arucoInfo.pop_back();
            }
            for(int i=0; i < ids.size(); i++)
            {
                
                int cx1 =static_cast<int> ((corners.at(i).at(1).x - corners.at(i).at(0).x)/2 + corners.at(i).at(0).x);
                int cx2 =static_cast<int> ((corners.at(i).at(3).x  - corners.at(i).at(2).x )/2 + corners.at(i).at(2).x );
                int cam_center_posX = (cx1 + cx2)/2;
                
                data.cx=cam_center_posX;
                data.id=ids.at(i);
                data.degree=PixeltoDegree(data.cx);
                //cout<<data.id<<","<<data.degree<<endl;
                arucoInfo.push_back(data);
            }
           int status = pthread_mutex_unlock (&mutex_);
            if (status != 0)
             exit(status);
            
           // cout<<"data.cx: "<<data.cx<<endl;
        }
        else{
            arucoInfo.clear();
        }
        //end mutex
        video.write(image_copy);
      /*imshow("Pose estimation", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;*/
    }

    in_video.release();
    video.release();
    //destroyALLWindows();
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

int comRobot(int id,string ip,string port,int instruction){
    
    //se crea el socket y se establece la comunicación
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;
    struct sockaddr_storage robot_addr;
    socklen_t addr_len = sizeof robot_addr;


    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
   //hints.ai_flags = IPPROTO_UDP; 

    const char *ipRobot=ip.c_str();
    const char *portRobot=port.c_str();
    
    if ((rv = getaddrinfo(ipRobot, portRobot, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
    }
           // set timeout
    struct timeval timeout;
    timeout.tv_sec = 1;//sec
    timeout.tv_usec = 0;//microsecond
    
    // loop through all the results and make a socket
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
            p->ai_protocol)) == -1) {
            perror("talker: socket");
            continue;
        }
        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
            cout<<"fail"<<endl;//perror("setsockopt failed:");
        }
    break;
   
    }
    /*if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    error("setsockopt(SO_REUSEADDR) failed");*/

    if (p == NULL) {
        fprintf(stderr, "talker: failed to create socket\n");
        return 2;
    }
    memset (buf, '\0', MAXDATASIZE); /* Pone a cero el buffer inicialmente */
    //aqui se indica la operacion que se desea realizar
    operation_send.id=id;//se asigna el id del robot1
    operation_send.len = sizeof (operation_send.data);
    operation_send.op=instruction;

    if ((numbytes = sendto(sockfd,(char *) &operation_send, operation_send.len+HEADER_LEN, 0,p->ai_addr, p->ai_addrlen)) == -1)
    {
        perror("talker: sendto");
        exit(1);
    }
    if(operation_send.op != OP_MOVE_WHEEL)
    { //this condition is only for the raspberry experiment, to avoiud  a big wait time;
        if((numbytes=recvfrom(sockfd,buf,MAXBUFLEN-1,0,(struct sockaddr*)&robot_addr, &addr_len))==-1){
        
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
            }
       
      //  memset (buf, '\0', MAXDATASIZE);
        }
    }
   
    
    freeaddrinfo(servinfo);
    
    close(sockfd);
    
    return 0;
    


}

void SerialCommunication(int id,string ip,string port,int instruction){
    int numbytes;
    int USB = open( "/dev/ttyACM0", O_RDWR| O_NOCTTY );

    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);

    /* Error Handling */
    if ( tcgetattr ( USB, &tty ) != 0 ) {
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
    tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( USB, TCIFLUSH );
    if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
    std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }


    unsigned char cmd[] = "b";
    operation_send.id=id;//se asigna el id del robot1
   /* double wD=8,wI=8;
    doubleToBytes(wD, &operation_send.data[0]);
    doubleToBytes(wI, &operation_send.data[8]);*/
    operation_send.len = sizeof (operation_send.data);
    operation_send.op=instruction;
   

    write( USB,(char*) &operation_send, operation_send.len );
     if(operation_send.op != OP_MOVE_WHEEL)
    { //this condition is only for the raspberry experiment, to avoiud  a big wait time;
       numbytes= read(USB,(char*)buf,MAXBUFLEN);
        operation_recv=( struct appdata*)&buf;

        if((numbytes< HEADER_LEN) || (numbytes != operation_recv->len+HEADER_LEN) )
        {
            
            cout<<"(servidor) unidad de datos incompleta :"<<numbytes<<endl;
            cout<<"len: "<<operation_recv->len<<endl;
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
            }
       
      //  memset (buf, '\0', MAXDATASIZE);
        }
    }
}