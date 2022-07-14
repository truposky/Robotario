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
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write
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

#define PORTBROADCAST "6868"
#define MAXBUFLEN 256
#define SAMPLINGTIME 8000 // in usec
#define MAXSINGNALLENGTH 2000
#define CENTER 320 //this is the setpoint for a distance between markers of 30 cm (in degree)
const float KP=1.5;
// get sockaddr, IPv4 or IPv6:
 char buf[MAXDATASIZE];

void tokenize(const string s, char c,vector<string>& v);//split the string 
void operationSend();//allow the user choose an instruction for send to the robot
void SetupRobots();//copy the information in the xml file to save in the class robot.
int broadcastRasp();
int broadCast();
void *robotMove1(void *arg);
void *robotMove2(void *arg);
void *robotMove3(void *arg);
void *dataAruco(void *arg);
void SetupRobots();
vector<cv::Point3f> getCornersInCameraWorld(double side, cv::Vec3d rvec, cv::Vec3d tvec);

void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
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
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

   // assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}
struct record_data//struct for share information between threads
{
    int id; //id of every robot
    double x, y , z;
    double rx,ry,rz;
    int n;  //counter for know how many robots there are
    
};
list<record_data> arucoInfo;//list for save the information of all the arucos
list<record_data>::iterator it;//for save data
list<record_data>::iterator it2;//for move the robot.
list<record_data>::iterator it3;//for move the robot.
struct signalAlarm{
    bool finish=false;
};
signalAlarm end_thread1,end_thread2,end_thread3;
enum {r1, r2, r3, r4,r5};
     //definition of robots
Robot robot1,robot2,robot3,robot4;//se define la clase para los distintos robots.
UDP comRobot1,comRobot2,comRobot3,comServer;//comunication udp





pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;
pthread_t _Move,_Move2,_Move3, _detectAruco; ;



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
   // in_video.set(cv::CAP_PROP_FRAME_WIDTH,1920);
  //  in_video.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
  
    
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
          /*  in_video.open(videoInput); // url
            in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
			in_video.set(cv::CAP_PROP_FPS,20);*/
            
        } else {
            in_video.open(source); // id
            
        }
    } else {
        in_video.open(0);
    }
    in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'));
    //in_video.set(cv::CAP_PROP_FRAME_WIDTH,800);
    // in_video.set(cv::CAP_PROP_FRAME_HEIGHT,600);
     in_video.set(cv::CAP_PROP_FPS,20.0);
     in_video.set(cv::CAP_PROP_AUTOFOCUS,0);
     in_video.set(cv::CAP_PROP_SETTINGS,1);
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
    
    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    cv::Mat rotated_image;
    //std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    //std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
    // cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(frame_width,frame_height).0);
    //---------------------------------end aruco code-----
    arucoInfo.clear();
    //init listen socket for data of robots
    //aruco::DetectorParameters detectorParams;
    
    //broadCast();
   
    
    
     
     
    int frame_width = in_video.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT);
    cout<<frame_width<<endl;
    cout<<frame_height<<endl;
    cout<<in_video.get(cv::CAP_PROP_FPS)<<endl;;
    cout<<in_video.get(cv::CAP_PROP_FOURCC)<<endl;
    cv::Size S = cv::Size(800,600);
    cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),30, S,1);
   // cv::Size S = cv::Size((int) in_video.get(cv::CAP_PROP_FRAME_WIDTH),    // Acquire input size

     //             (int) in_video.get(cv::CAP_PROP_FRAME_HEIGHT));
    //int ex = static_cast<int>(in_video.get(cv::CAP_PROP_FOURCC));

     //outputVideo.open("outcpp.avi" ,ex, 30, S, true);
    
   // cv::Size frameSize(static_cast<int>(frame_width),static_cast<int>(frame_height));
    // Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
	 // cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30,frameSize ,0);
    //  cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 20, cv::Size(frame_width,frame_height));

    cv :: Mat Raux = cv::Mat::zeros(3,3,CV_64F);;
	double ang_roll = 0;
	double ang_pitch = 0;
	double ang_yaw = 0;
    double ang_yaw2 = 0;
	double r11 = 0;
    double r12 = 0;
    double r13 = 0;
	double r21 = 0;
    double r22 = 0;
    double r23 = 0;
	double r31 = 0;
	double r32 = 0;
	double r33 = 0;
    double angleError;
       // compute rot_mat
            cv::Mat rot_mat,rot_mat2;
    
   
     pthread_create(&_detectAruco,NULL,dataAruco,NULL);//create thread for store the values of the markers
     
    //main loop
    while (in_video.grab())
    {
        in_video.retrieve(image);
        
        cvtColor(image,grayMat,cv::COLOR_BGR2GRAY);
        //cv::resize(image,image,cv::Size(resized_width,resized_height));
        image.copyTo(image_copy);
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
            // cout<<"corners: "<<getCornersInCameraWorld(marker_length_m, rvecs[0],tvecs[0])<<endl;
            /*std::cout << "Translation: " << tvecs[0]
                << "\tRotation: " << rvecs[0] 
                << std::endl;
            */
            // Draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);
                        
                data.x=tvecs[i](0);
                
                data.y=tvecs[i](1);
                //cout<<ids.at(i)<<"datax "<<data.x<<","<<data.y<<endl;
                data.z=tvecs[i](2);
                cv::Rodrigues(rvecs[i], rot_mat);
                ang_yaw=atan2(rot_mat.at<double>(1,0),rot_mat.at<double>(0,0));
                //if(ang_yaw<0){ ang_yaw += 2*M_PI;}
               /* if(rot_mat.at<double>(0,0) < 0.00 && rot_mat.at<double>(1,0) > 0.00){
               // angleVector = atan2(y,-x)+M_PI;
                 ang_yaw=atan2(rot_mat.at<double>(1,0),-rot_mat.at<double>(0,0))+M_PI;
                
                }
                else  if(rot_mat.at<double>(0,0) < 0.00 && rot_mat.at<double>(1,0) < 0.00){
                    //angleVector=atan2(-y,-x)+M_PI;
                     ang_yaw=atan2(-rot_mat.at<double>(1,0),-rot_mat.at<double>(0,0))+M_PI;

                }
                else if(rot_mat.at<double>(0,0)*100 > 0.00 && rot_mat.at<double>(1,0)*100 < 0.00){
                    //angleVector=atan2(y,x)+ 2*M_PI;
                    cout<<"rot"<<rot_mat.at<double>(1,0)<<endl;
                    ang_yaw=atan2(-rot_mat.at<double>(1,0),rot_mat.at<double>(0,0))+2*M_PI;
                }
                else{
                    ang_yaw=atan2(rot_mat.at<double>(1,0),rot_mat.at<double>(0,0));
                }*/

                data.rx=rvecs[i](0);
                data.ry=rvecs[i](1);
                data.rz=ang_yaw;

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
          
            cv::Rodrigues(rvecs[0], rot_mat);
            cv::Rodrigues(rvecs[1], rot_mat2);
              // transpose of rot_mat for easy columns extraction
            cv::Mat rot_mat_t = rot_mat.t();
            r11=rot_mat.at<double>(0,0);
            r12=rot_mat.at<double>(0,1);
            r13=rot_mat.at<double>(0,2);
            r21=rot_mat.at<double>(1,0);
            r22=rot_mat.at<double>(1,1);
            r23=rot_mat.at<double>(1,2);
            r31=rot_mat.at<double>(2,0);
            r32=rot_mat.at<double>(2,1);
            r33=rot_mat.at<double>(2,2);
            ang_yaw=atan2(r21,r11);
            ang_yaw2=atan2(rot_mat2.at<double>(1,0),rot_mat2.at<double>(0,0)); 
          //  cout<<"angulo1: "<<ang_yaw<<endl;
           // cout<<"angulo2: "<<ang_yaw2<<endl;
            //cout<<"rot1: "<<rotationMatrixToEulerAngles(rot_mat)<<endl;
            // cout<<"rot2: "<<rotationMatrixToEulerAngles(rot_mat2)<<endl;
             /* cout<<"rx1: "<<rvecs[0](0)<<endl;
				cout<<"ry1: "<<rvecs[0](1)<<endl;
				cout<<"rz1: "<<rvecs[0](2)<<endl;

                cout<<"rx2: "<<rvecs[1](0)<<endl;
				cout<<"ry2: "<<rvecs[1](1)<<endl;
				cout<<"rz2: "<<rvecs[1](2)<<endl;
            angleError=rvecs[1](1)-rvecs[0](1);
                cout<<"Error: "<<angleError<<endl;*/
            
        }
         //outputVideo << image_copy;
        // cv::resize(image,image,cv::Size(1200,1600));
      
      video.write(image_copy);
      //video<<image;
      imshow("Pose estimation", image_copy);
        char key = (char)cv::waitKey(1);
        if (key == 27)
            break;
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
         const char* ipp=robotChild->GetText();
         string ss=ipp;
         cout<<"ip:"<<ipp<<endl;

        robotChild=robot->FirstChildElement("PORT");
        const char* portt=robotChild->GetText();
        string p=portt;
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
//crear clase udp
int broadCast(){

    //se crea el socket y se establece la comunicación
    int bcast_sock;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;

    


    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
   //hints.ai_flags = IPPROTO_UDP; 


    
    if ((rv = getaddrinfo("192.168.78.255", "6868", &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return -1;
    }
    int broadcastEnable=1;

    for(p = servinfo; p != NULL; p = p->ai_next) 
    {
        if ((bcast_sock = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) 
        {
            perror("talker: socket");
            continue;
        }
        if (setsockopt(bcast_sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))){
            cout<<"fail"<<endl;//perror("setsockopt failed:");
        }
    break;
   
    }

    if (p == NULL) {
        fprintf(stderr, "talker: failed to create socket\n");
        return 2;
    }
    memset (buf, '\0', MAXDATASIZE); /* Pone a cero el buffer inicialmente */
    //aqui se indica la operacion que se desea realizar
    operation_send.id=99;//se asigna el id del robot1
    operation_send.len = sizeof(operation_send.data);
    operation_send.op=OP_BROADCAST;

    if ((numbytes = sendto(bcast_sock,(char *) &operation_send, operation_send.len+HEADER_LEN, 0,p->ai_addr, p->ai_addrlen)) == -1)
    {
        perror("talker: sendto");
        exit(1);
    }
    return 0;
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
    sleep(2);
    struct logo_data{
        double td;
        double wheel_vel[2];
        vector<int> id;
        vector<double> x;
        vector<double> y;
        vector<double> z;
        
        string timeStamp;
	
    };
    
    int n=0,numbytes=0;
    ofstream logo("log.txt");
    ofstream regVel("vel.txt");
    logo.close();//file created
    regVel.open("vel.txt",ios::out);
    regVel.close();
    list<logo_data> savelog;
    list<logo_data>::iterator iter;
    logo_data info;
    string ip, port;
    int id;
    robot1.SetupConection(id,ip,port);
    comRobot1.initTalkerSocket(ip,port);
    robot2.SetupConection(id,ip,port);
    comRobot2.initTalkerSocket(ip,port);
    //cout<<"IDDDD"<<ip<<endl;
    robot3.SetupConection(id,ip,port);
    comRobot3.initTalkerSocket(ip,port);
    //in this case the experiment needs the velocity 
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

   
    double td;
    int cont=0;
    int idAux,idAux2,idAux3;


    //comunication server 
    comServer.initListenSocket(IP_SERVER,SERVERPORT);
    
    //comRobot2.initTalkerSocket(ip,port);
    //comRobot1.initTalkerSocket(ip,port);
 
    while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected
    char resultString[120];
    
    
   pthread_create(&_Move,NULL,robotMove1,NULL);
   pthread_create(&_Move2,NULL,robotMove2,NULL);//thread for move the robot
   pthread_create(&_Move3,NULL,robotMove3,NULL);
    //send operation, arduino will send data until we say to stop it
    info.wheel_vel[0]=0;
    info.wheel_vel[1]=0;
    int veces=0;
    cout<<"START"<<endl;
    //main code for read variables
    while(true){
       
        gettimeofday(&tval_before,NULL);
        regVel.open("vel.txt",ios::ate);
       
        
        td=(double)n*SAMPLINGTIME/1000; 
     /*   cout<<"envio"<<endl;

        operation_send.op=OP_VEL_ROBOT;
        operation_send.len = sizeof (operation_send.data);
        comRobot1.SendTalkerSocket((char*)&operation_send,operation_send.len);
        
        comServer.RecvListenSocket(buf,MAXDATASIZE,numbytes);
        operation_recv=( struct appdata*)&buf;
        info.wheel_vel[0] = bytesToDouble(&operation_recv->data[0]);
        if(operation_recv->data[16]=='a')
        {
            info.wheel_vel[0]=-info.wheel_vel[0];
        }
      
        info.wheel_vel[1] = bytesToDouble(&operation_recv->data[8]);
          if(operation_recv->data[17]=='a')
        {
            info.wheel_vel[1]=-info.wheel_vel[1];
        }


        operation_send.op=OP_VEL_ROBOT;
        operation_send.len = sizeof (operation_send.data);
        comRobot2.SendTalkerSocket((char*)&operation_send,operation_send.len);
        
        comServer.RecvListenSocket(buf,MAXDATASIZE,numbytes);
        operation_recv=( struct appdata*)&buf;
        info.wheel_vel[0] = bytesToDouble(&operation_recv->data[0]);
        if(bytesToShort(&operation_recv->data[16]))
        {
            info.wheel_vel[0]=-info.wheel_vel[0];
        }
      
        info.wheel_vel[1] = bytesToDouble(&operation_recv->data[8]);
        cout<<info.wheel_vel[1]<<endl;
          if(bytesToShort(&operation_recv->data[16]))
        {
            info.wheel_vel[1]=-info.wheel_vel[1];
        }
        

        operation_send.op=OP_VEL_ROBOT;
        operation_send.len = sizeof (operation_send.data);
        comRobot2.SendTalkerSocket((char*)&operation_send,operation_send.len);
        
        comServer.RecvListenSocket(buf,MAXDATASIZE,numbytes);
        operation_recv=( struct appdata*)&buf;
        int idd=operation_recv->id;
        cout<<"id :"<<idd<<endl;
        info.wheel_vel[0] = bytesToDouble(&operation_recv->data[0]);
        if(operation_recv->data[16]=='a')
        {
            info.wheel_vel[0]=-info.wheel_vel[0];
        }
      
        info.wheel_vel[1] = bytesToDouble(&operation_recv->data[8]);
          if(operation_recv->data[17]=='a')
        {
            info.wheel_vel[1]=-info.wheel_vel[1];
        }

        cout<<info.wheel_vel[0]<<","<<info.wheel_vel[1]<<endl;*/
        cont=0;
        if(arucoInfo.size()>0)
        {
            
            for(it=arucoInfo.begin();it !=arucoInfo.end();it++)
            {
                
                info.id.push_back((*it).id);
                info.x.push_back((*it).x);
                info.y.push_back((*it).y);
                info.z.push_back((*it).z);
                
                if(cont==0){
                    idAux=it->id;
                }
                else if(cont==1){
                    idAux2=it->id;
                }    
                else if(cont==2){
                    idAux3=it->id;
                }
                cont++;     
                
            }
        }
        if(cont==0){
            info.id.push_back(0);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
            info.id.push_back(2);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
            info.id.push_back(3);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
              info.id.push_back(20);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
        }
        if(cont == 1){
            if(idAux==20){
                idAux2=0;
                idAux3=2;
                idAux=3;
            }
            else if(idAux==0){
                idAux2=2;
                idAux3=3;
                idAux=20;
            }
            else if(idAux==2){
                idAux2=0;
                idAux3=3;
                idAux=20;
            }
             else if(idAux==3){
                idAux2=0;
                idAux3=2;
                idAux=20;
            }
            info.id.push_back(idAux2);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
            info.id.push_back(idAux3);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
            info.id.push_back(idAux);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
        }
        
        else if(cont==2){

            if((idAux==0 && idAux2==2) || (idAux==2 && idAux2==0)){

                idAux=3;
                idAux2=20;
            }
            else if((idAux==0 && idAux2==3) || (idAux==3 && idAux2==0)){

                idAux=2;
                idAux2=20;
            }
            else if ((idAux==0 && idAux2==20) ||(idAux==20 && idAux2==0)) {
                idAux=2;
                idAux2=3;
            }
            else if ((idAux==2 && idAux2==3)||(idAux==3 && idAux2==2)){
                idAux=0;
                idAux2=20;
            }
            else if((idAux==2 && idAux2==20)||(idAux==20 && idAux2==2)){
                idAux=0;
                idAux2=3;
            }
            else if((idAux==3 && idAux==20) || (idAux==3 && idAux==20)){
                idAux=0;
                idAux2=2;
            }
       
            info.id.push_back(idAux);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
            info.id.push_back(idAux2);
            info.x.push_back(-9999);
            info.y.push_back(-9999);
            info.z.push_back(-9999);
        }
        if(cont==3){
            info.id.push_back(99);
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
        regVel.close();
        n++;
        veces++;
        if(end_thread1.finish && end_thread2.finish && end_thread3.finish){
            break;
        }
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
  /*  pthread_cancel(_Move);
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
            
   // write( arduino,(char*) &operation_send, operation_send.len +HEADER_LEN);
    comRobot2.SendTalkerSocket((char*)&operation_send,operation_send.len);*/
    //----------------------------------------------------
    
    //save data on logo.txt
    cout<<"data save"<<endl;
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
    
   
    pthread_exit(NULL);
    return NULL;
}



void *robotMove1(void *arg){
   // _Move = pthread_self();
    //-----------setup comunicaction-------------------/
    string ip, port;
    int id;
    robot1.SetupConection(id,ip,port);
    comRobot1.initTalkerSocket(ip,port);
//-----------------------------------------------------/
    cout<<"IDDDD"<<ip<<endl;
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

    double vel=0;//linear velocity of robot
    double auxVel=0,auxW=0;
    int idRobot;   
    double w=0;//angular velocity of robot
    bool correction =false,forward,giro=false;
    int count=0;
    double velocity_robot[2];
    double angularWheel[2];
    double rx,ry=100,rz,x,y,z,x2,y2,z2;
    double angleVector,modulo;
    double cumError=0, auxcumError=0; //controlador integral.
    double err;
    const double ANGLE=-1.57;
    vel=8*3.35;
    const int SAMPLE_TIME=660000;//us
    
    while(true){
        gettimeofday(&tval_before,NULL);
        rx=0;
        ry=0;
        rz=0;
        count=0;
        if(arucoInfo.size()>0)
        {
               
                for(it2=arucoInfo.begin();it2 !=arucoInfo.end();it2++){
                    idRobot=it2->id;
                    if(idRobot==1){
                        x=it2->x;
                        y=it2->y;
                        rx=it2->rx;
                        ry=it2->ry;
                        rz=it2->rz;
                    }
                    else if(idRobot==20){
                        x2=it2->x;
                        y2=it2->y;
                        rx=it2->rx-rx;
                        ry=it2->ry-ry;
                    }
                    count++;
                }
               
        }
        x=x2-x;
        y=y2-y;
        cout<<"x:"<<x<<endl;
        cout<<"y:"<<y<<endl;
        /*if(x*100 < 0.00 && y*100 > 0.00){
        angleVector = atan2(y,-x) + M_PI;
        
        }
        else  if(x*100 < 0.00 && y*100 < 0.00){
            angleVector=atan2(-y,-x) + M_PI;
        }
        else if(x*100 > 0.00 && y*100 < 0.00){
             angleVector=atan2(-y,x) + 2*M_PI;
        }
        else{
            angleVector=atan2(y,x);
        }*/
         angleVector=atan2(y,x);
       // if(angleVector<0){ angleVector += 2*M_PI;}
        
        cout<<"angleVector: "<<angleVector<<endl;
        cout<<"rz"<<rz<<endl;
        err=rz-angleVector;
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
        cout<<err<<endl;
        cumError+=err;
        if(err<0 && cumError>0){
            cumError=0;
        }
        else if(err>0 && cumError<0){
            cumError=0;
        }
        if(cumError>0){
            if(cumError>14){
                cumError=14;
            }
        }
        else if(cumError<0){
            if(cumError<-14){
                cumError=-14;
            }
        }
        auxcumError=cumError;
        cout<<cumError<<endl;
        if(count>=2){
            if(err>0.65 || err<-0.65)
            {
                w=-(1.35*err+0.65*cumError*SAMPLE_TIME/1000000);
            }
            else{
                w=0;
                giro=true;
            }
        }
        else{
            w=0;
        }
        vel=0;
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot1.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
       // cout<<"normal ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot1.SendTalkerSocket((char*)&operation_send,operation_send.len);

        //cout<<"error "<<err<<endl;
        if(giro){
            break;
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
    do{
        gettimeofday(&tval_before,NULL);
        if(arucoInfo.size()>0)
        {
               
                for(it2=arucoInfo.begin();it2 !=arucoInfo.end();it2++){
                    idRobot=it2->id;
                    cout<<it2->id<<","<<x<<endl;
                   if(idRobot==1){
                        x=it2->x;
                        y=it2->y;
                        rx=it2->rx;
                        ry=it2->ry;
                        rz=it2->rz;
                    }
                    else if(idRobot==20){
                        x2=it2->x;
                        y2=it2->y;
                        rx=it2->rx-rx;
                        ry=it2->ry-ry;
                    }
                    count++;
                }
               
        }
        if(count>=2){
            x=x2-x;
            y=y2-y;
            //cout<<"x: "<<x<<endl;
            //cout<<"y: "<<y<<endl;
        
            /*if(x*100 < 0.00 && y*100 > 0.00){
            angleVector = atan2(y,-x)+M_PI;
            
            }
            else  if(x*100 < 0.00 && y*100 < 0.00){
                angleVector=atan2(-y,-x)+M_PI;
            }
            else if(x*100 > 0.00 && y*100 < 0.00){
                angleVector=atan2(-y,x)+ 2*M_PI;
            }
            else{
                angleVector=atan2(y,x);
            }*/
             angleVector=atan2(y,x);
             
            // if(angleVector<0){ angleVector += 2*M_PI;}
            modulo=sqrt((x*x)+(y*y));
        
            cout<<"modulo1: "<<modulo<<endl;
            err=angleVector-rz;
            if(err>M_PI){
            err=err-2*M_PI;
            }
            else if(err<-M_PI){
                err=err+2*M_PI;
            }
            cumError+=err;
            if(err<0 && cumError>0){
                cumError=0;
            }
            else if(err>0 && cumError<0){
                cumError=0;
            }
            if(cumError>0){
                if(cumError>10){
                    cumError=10;
                }
            }
            else if(cumError<0){
                if(cumError<-10){
                    cumError=-10;
                }
            }
        
            
            if(count>=2 && modulo>0.25){
                vel=8.2*3.35;
                if(err>0.55 || err<-0.55)
                {
                    w=-(0.55*err+0.25*cumError*SAMPLE_TIME/1000000);
                }
                else{
                    w=0;
                }
            }
            else{
                w=0;
                vel=0;
            }
        }
        
        
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot1.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
        cout<<"normal1 ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot1.SendTalkerSocket((char*)&operation_send,operation_send.len);

        cout<<"error "<<err<<endl;
     
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
    while(modulo>0.25);
    cout<<"FINISH ROBOT 1"<<endl;
    end_thread1.finish=true;
    pthread_exit(NULL);

}
void *robotMove2(void *arg){
    //_Move = pthread_self();
    //-----------setup comunicaction-------------------/
    string ip, port;
    int id;
    
//-----------------------------------------------------/
    
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

    double vel=0;//linear velocity of robot
    double auxVel=0,auxW=0;
    int idRobot;   
    double w=0;//angular velocity of robot
    bool correction =false,forward,giro=false;
    int count=0;
    double velocity_robot[2];
    double angularWheel[2];
    double rx,ry=100,rz,x,y,z,x2,y2,z2;
    double angleVector,modulo;
    double cumError=0, auxcumError=0; //controlador integral.
    double err;
    const double ANGLE=-1.57;
    vel=8*3.35;
    const int SAMPLE_TIME=660000;//us
    
    while(true){
        gettimeofday(&tval_before,NULL);
        rx=0;
        ry=0;
        rz=0;
        count=0;
        //cout<<"hola"<<port<<endl;
        if(arucoInfo.size()>0)
        {
               
                for(it3=arucoInfo.begin();it3 !=arucoInfo.end();it3++){
                    idRobot=it3->id;
                    cout<<idRobot<<","<<x<<","<<y<<endl;
                    if(idRobot==2){
                        x=it3->x;
                        y=it3->y;
                        rx=it3->rx;
                        ry=it3->ry;
                        rz=it3->rz;
                    }
                    else if(idRobot==20){
                        x2=it3->x;
                        y2=it3->y;
                        rx=it3->rx-rx;
                        ry=it3->ry-ry;
                    }
                    count++;
                }
               
        }
        x=x2-x;
        y=y2-y;
        cout<<"x:"<<x<<endl;
        cout<<"y:"<<y<<endl;
        /*if(x*100 < 0.00 && y*100 > 0.00){
        angleVector = atan2(y,-x)+M_PI;
        
        }
        else  if(x*100 < 0.00 && y*100 < 0.00){
            angleVector=atan2(-y,-x)+M_PI;
        }
        else if(x*100 > 0.00 && y*100 < 0.00){
             angleVector=atan2(-y,x) + 2*M_PI;
        }
        else{
            angleVector=atan2(y,x);
        }*/
        angleVector=atan2(y,x);
        //if(angleVector<0){ angleVector += 2*M_PI;}
        
        cout<<"angleVector2: "<<angleVector<<endl;
        cout<<"rz"<<rz<<endl;
        err=angleVector-rz;
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
        cumError+=err;
        if(err<0 && cumError>0){
            cumError=0;
        }
        else if(err>0 && cumError<0){
            cumError=0;
        }
        if(cumError>0){
            if(cumError>10){
                cumError=10;
            }
        }
        else if(cumError<0){
            if(cumError<-10){
                cumError=-10;
            }
        }
        auxcumError=cumError;
        //cout<<cumError<<endl;
        if(count>=2){
            if(err>0.55 || err<-0.55)
            {
                w=-(1.1*err+0.45*cumError*SAMPLE_TIME/1000000);
            }
            else{
                w=0;
                giro=true;
            }
        }
        else{
            w=0;
        }
        vel=0;
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot2.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
        //cout<<"normal ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot2.SendTalkerSocket((char*)&operation_send,operation_send.len);

        cout<<"error "<<err<<endl;
        if(giro){
            break;
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
    do{
        gettimeofday(&tval_before,NULL);
        if(arucoInfo.size()>0)
        {
               
                for(it3=arucoInfo.begin();it3 !=arucoInfo.end();it3++){
                    idRobot=it3->id;
                    cout<<it3->id<<","<<it3->x<<endl;
                    if(idRobot==2){
                        x=it3->x;
                        y=it3->y;
                        rx=it3->rx;
                        ry=it3->ry;
                        rz=it3->rz;
                    }
                    else if(idRobot==20){
                        x2=it3->x;
                        y2=it3->y;
                        rx=it3->rx-rx;
                        ry=it3->ry-ry;
                    }
                    count++;
                }
               
        }
        if(count>=2){
            x=x2-x;
            y=y2-y;
            //cout<<"x: "<<x<<endl;
            //cout<<"y: "<<y<<endl;
        
            /*if(x*100 < 0.00 && y*100 > 0.00){
            angleVector = atan2(y,-x)+M_PI;
            
            }
            else  if(x*100 < 0.00 && y*100 < 0.00){
                angleVector=atan2(-y,-x)+M_PI;
            }
            else if(x*100 > 0.00 && y*100 < 0.00){
                angleVector=atan2(-y,x)+ 2*M_PI;
            }
            else{
                angleVector=atan2(y,x);
            }*/
             angleVector=atan2(y,x);
             //if(angleVector<0){ angleVector += 2*M_PI;}
            modulo=sqrt((x*x)+(y*y));
        
        //cout<<"modulo: "<<modulo<<endl;
        err=angleVector-rz;
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
            cumError+=err;
            if(err<0 && cumError>0){
                cumError=0;
            }
            else if(err>0 && cumError<0){
                cumError=0;
            }
            if(cumError>0){
                if(cumError>10){
                    cumError=10;
                }
            }
            else if(cumError<0){
                if(cumError<-10){
                    cumError=-10;
                }
            }
        
            
            if(count>=2 && modulo>0.25){
                vel=8.2*3.35;
                if(err>0.49 || err<-0.49)
                {
                    w=-(0.5*err+0.2*cumError*SAMPLE_TIME/1000000);
                }
                else{
                    w=0;
                }
            }
            else{
                w=0;
                vel=0;
            }
        }
        
        
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot2.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
        cout<<"normal2 ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot2.SendTalkerSocket((char*)&operation_send,operation_send.len);

        cout<<"error "<<err<<endl;
     
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
    while(modulo>0.25);
    cout<<"FINISH ROBOT 2"<<endl;
    end_thread2.finish=true;
    pthread_exit(NULL);

}ç
void *robotMove3(void *arg){
    //_Move = pthread_self();
    //-----------setup comunicaction-------------------/
    string ip, port;
    int id;
    
//-----------------------------------------------------/

    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

    double vel=0;//linear velocity of robot
    double auxVel=0,auxW=0;
    int idRobot;   
    double w=0;//angular velocity of robot
    bool correction =false,forward,giro=false;
    int count=0;
    double velocity_robot[2];
    double angularWheel[2];
    double rx,ry=100,rz,x,y,z,x2,y2,z2;
    double angleVector,modulo;
    double cumError=0, auxcumError=0; //controlador integral.
    double err;
    const double ANGLE=-1.57;
    vel=8*3.35;
    const int SAMPLE_TIME=660000;//us
    
    while(true){
        gettimeofday(&tval_before,NULL);
        rx=0;
        ry=0;
        rz=0;
        count=0;
        cout<<"hola3"<<port<<endl;
        if(arucoInfo.size()>0)
        {
               
                for(it3=arucoInfo.begin();it3 !=arucoInfo.end();it3++){
                    idRobot=it3->id;
                    cout<<idRobot<<","<<x<<endl;
                    if(idRobot==3){
                        x=it3->x;
                        y=it3->y;
                        rx=it3->rx;
                        ry=it3->ry;
                        rz=it3->rz;
                    }
                    else if(idRobot==20){
                        x2=it3->x;
                        y2=it3->y;
                        rx=it3->rx-rx;
                        ry=it3->ry-ry;
                    }
                    count++;
                }
               
        }
        x=x2-x;
        y=y2-y;
        //cout<<"x:"<<x<<endl;
        //cout<<"y:"<<y<<endl;
       /*if(x*100 < 0.00 && y*100 > 0.00){
        angleVector = atan2(y,-x)+M_PI;
        
        }
        else  if(x*100 < 0.00 && y*100 < 0.00){
            angleVector=atan2(-y,-x)+M_PI;
        }
        else if(x*100 > 0.00 && y*100 < 0.00){
             angleVector=atan2(-y,x)+ 2*M_PI;
        }
        else{
            angleVector=atan2(y,x);
        }*/
        angleVector=atan2(y,x);
        //if(angleVector<0){ angleVector += 2*M_PI;}
        
        cout<<"angleVector3: "<<angleVector<<endl;
        //cout<<"rz"<<rz<<endl;
        err=angleVector-rz;
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
        cumError+=err;
        if(err<0 && cumError>0){
            cumError=0;
        }
        else if(err>0 && cumError<0){
            cumError=0;
        }
        if(cumError>0){
            if(cumError>10){
                cumError=10;
            }
        }
        else if(cumError<0){
            if(cumError<-10){
                cumError=-10;
            }
        }
        auxcumError=cumError;
        //cout<<cumError<<endl;
        if(count>=2){
            if(err>0.55 || err<-0.55)
            {
                w=-(1.2*err+0.6*cumError*SAMPLE_TIME/1000000);
            }
            else{
                w=0;
                giro=true;
            }
        }
        else{
            w=0;
        }
        vel=0;
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot3.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
        //cout<<"normal ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot3.SendTalkerSocket((char*)&operation_send,operation_send.len);

        cout<<"error "<<err<<endl;
        if(giro){
            break;
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
    do{
        gettimeofday(&tval_before,NULL);
        if(arucoInfo.size()>0)
        {
               
                for(it3=arucoInfo.begin();it3 !=arucoInfo.end();it3++){
                    idRobot=it3->id;
                    cout<<it3->id<<","<<it3->x<<endl;
                    if(idRobot==3){
                        x=it3->x;
                        y=it3->y;
                        rx=it3->rx;
                        ry=it3->ry;
                        rz=it3->rz;
                    }
                    else if(idRobot==20){
                        x2=it3->x;
                        y2=it3->y;
                        rx=it3->rx-rx;
                        ry=it3->ry-ry;
                    }
                    count++;
                }
               
        }
        if(count>=2){
            x=x2-x;
            y=y2-y;
            cout<<"x: "<<x<<endl;
            cout<<"y: "<<y<<endl;
        
        /*if(x*100 < 0.00 && y*100 > 0.00){
        angleVector = atan2(y,-x)+M_PI;
        
        }
        else  if(x*100 < 0.00 && y*100 < 0.00){
            angleVector=atan2(-y,-x)+M_PI;
        }
        else if(x*100 > 0.00 && y*100 < 0.00){
             angleVector=atan2(-y,x)+ 2*M_PI;
        }
        else{
            angleVector=atan2(y,x);
        }*/

        angleVector=atan2(y,x);
       // if(angleVector<0){ angleVector += 2*M_PI;}
        modulo=sqrt((x*x)+(y*y));
        
        //cout<<"modulo: "<<modulo<<endl;
        err=angleVector-rz;
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
            cumError+=err;
            if(err<0 && cumError>0){
                cumError=0;
            }
            else if(err>0 && cumError<0){
                cumError=0;
            }
            if(cumError>0){
                if(cumError>10){
                    cumError=10;
                }
            }
            else if(cumError<0){
                if(cumError<-10){
                    cumError=-10;
                }
            }
        
            
            if(count>=2 && modulo>0.25){
                vel=8.2*3.35;
                if(err>0.49 || err<-0.49)
                {
                    w=-(0.65*err+0.3*cumError*SAMPLE_TIME/1000000);
                }
                else{
                    w=0;
                }
            }
            else{
                w=0;
                vel=0;
            }
        }
        
        
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot2.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
        cout<<"normal ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot3.SendTalkerSocket((char*)&operation_send,operation_send.len);

        cout<<"error "<<err<<endl;
     
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
    while(modulo>0.25);
    cout<<"FINISH ROBOT 3"<<endl;
    end_thread3.finish=true;
    pthread_exit(NULL);

}




void *robotMove(void *arg){
   // _Move = pthread_self();
    //-----------setup comunicaction-------------------/
    string ip, port;
    int id;
    robot1.SetupConection(id,ip,port);
    comRobot1.initTalkerSocket(ip,port);
//-----------------------------------------------------/
    cout<<"IDDDD"<<ip<<endl;
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;

    double vel=0;//linear velocity of robot
    double auxVel=0,auxW=0;
    int idRobot;   
    double w=0;//angular velocity of robot
    bool correction =false,forward,giro=false;
    int count=0;
    double velocity_robot[2];
    double angularWheel[2];
    double rx,ry=100,rz,x,y,z,x2,y2,z2;
    double angleVector,modulo;
    double cumError=0, auxcumError=0; //controlador integral.
    double err;
    const double ANGLE=-1.57;
    vel=8*3.35;
    const int SAMPLE_TIME=660000;//us
    
    while(true){
        gettimeofday(&tval_before,NULL);
        rx=0;
        ry=0;
        rz=0;
        count=0;
        if(arucoInfo.size()>0)
        {
               
                for(it2=arucoInfo.begin();it2 !=arucoInfo.end();it2++){
                    idRobot=it2->id;
                    if(idRobot==1){
                        x=it2->x;
                        y=it2->y;
                        rx=it2->rx;
                        ry=it2->ry;
                        rz=it2->rz;
                    }
                    else if(idRobot==20){
                        x2=it2->x;
                        y2=it2->y;
                        rx=it2->rx-rx;
                        ry=it2->ry-ry;
                    }
                    count++;
                }
               
        }
        x=x2-x;
        y=y2-y;
        cout<<"x:"<<x<<endl;
        cout<<"y:"<<y<<endl;
        /*if(x*100 < 0.00 && y*100 > 0.00){
        angleVector = atan2(y,-x) + M_PI;
        
        }
        else  if(x*100 < 0.00 && y*100 < 0.00){
            angleVector=atan2(-y,-x) + M_PI;
        }
        else if(x*100 > 0.00 && y*100 < 0.00){
             angleVector=atan2(-y,x) + 2*M_PI;
        }
        else{
            angleVector=atan2(y,x);
        }*/
         angleVector=atan2(y,x);
       // if(angleVector<0){ angleVector += 2*M_PI;}
        
        cout<<"angleVector: "<<angleVector<<endl;
        cout<<"rz"<<rz<<endl;
        err=rz-angleVector;
        if(err>M_PI){
            err=err-2*M_PI;
        }
        else if(err<-M_PI){
            err=err+2*M_PI;
        }
        cout<<err<<endl;
        cumError+=err;
        if(err<0 && cumError>0){
            cumError=0;
        }
        else if(err>0 && cumError<0){
            cumError=0;
        }
        if(cumError>0){
            if(cumError>14){
                cumError=14;
            }
        }
        else if(cumError<0){
            if(cumError<-14){
                cumError=-14;
            }
        }
        auxcumError=cumError;
        cout<<cumError<<endl;
        if(count>=2){
            if(err>0.65 || err<-0.65)
            {
                w=-(1.35*err+0.65*cumError*SAMPLE_TIME/1000000);
            }
            else{
                w=0;
                giro=true;
            }
        }
        else{
            w=0;
        }
        vel=0;
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot1.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
       // cout<<"normal ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot1.SendTalkerSocket((char*)&operation_send,operation_send.len);

        //cout<<"error "<<err<<endl;
        if(giro){
            break;
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
    do{
        gettimeofday(&tval_before,NULL);
        if(arucoInfo.size()>0)
        {
               
                for(it2=arucoInfo.begin();it2 !=arucoInfo.end();it2++){
                    idRobot=it2->id;
                    cout<<it2->id<<","<<x<<endl;
                   if(idRobot==1){
                        x=it2->x;
                        y=it2->y;
                        rx=it2->rx;
                        ry=it2->ry;
                        rz=it2->rz;
                    }
                    else if(idRobot==20){
                        x2=it2->x;
                        y2=it2->y;
                        rx=it2->rx-rx;
                        ry=it2->ry-ry;
                    }
                    count++;
                }
               
        }
        if(count>=2){
            x=x2-x;
            y=y2-y;
            //cout<<"x: "<<x<<endl;
            //cout<<"y: "<<y<<endl;
        
            /*if(x*100 < 0.00 && y*100 > 0.00){
            angleVector = atan2(y,-x)+M_PI;
            
            }
            else  if(x*100 < 0.00 && y*100 < 0.00){
                angleVector=atan2(-y,-x)+M_PI;
            }
            else if(x*100 > 0.00 && y*100 < 0.00){
                angleVector=atan2(-y,x)+ 2*M_PI;
            }
            else{
                angleVector=atan2(y,x);
            }*/
             angleVector=atan2(y,x);
             
            // if(angleVector<0){ angleVector += 2*M_PI;}
            modulo=sqrt((x*x)+(y*y));
        
            cout<<"modulo1: "<<modulo<<endl;
            err=angleVector-rz;
            if(err>M_PI){
            err=err-2*M_PI;
            }
            else if(err<-M_PI){
                err=err+2*M_PI;
            }
            cumError+=err;
            if(err<0 && cumError>0){
                cumError=0;
            }
            else if(err>0 && cumError<0){
                cumError=0;
            }
            if(cumError>0){
                if(cumError>10){
                    cumError=10;
                }
            }
            else if(cumError<0){
                if(cumError<-10){
                    cumError=-10;
                }
            }
        
            
            if(count>=2 && modulo>0.25){
                vel=8.2*3.35;
                if(err>0.55 || err<-0.55)
                {
                    w=-(0.55*err+0.25*cumError*SAMPLE_TIME/1000000);
                }
                else{
                    w=0;
                }
            }
            else{
                w=0;
                vel=0;
            }
        }
        
        
        velocity_robot[0]=w;
        velocity_robot[1]=vel;
        robot1.angularWheelSpeed(angularWheel,velocity_robot);
        doubleToBytes(angularWheel[0], &operation_send.data[0]);
        doubleToBytes(angularWheel[1], &operation_send.data[8]);
        //send angular Wheel
        operation_send.op=OP_MOVE_WHEEL;
        operation_send.len = sizeof (operation_send.data);
        cout<<"normal1 ;"<<"v: "<<vel<<" w: "<<w<<","<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        comRobot1.SendTalkerSocket((char*)&operation_send,operation_send.len);

        cout<<"error "<<err<<endl;
     
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
    while(modulo>0.25);
    cout<<"FINISH ROBOT 1"<<endl;
    end_thread1.finish=true;
    pthread_exit(NULL);

}
vector<cv::Point3f> getCornersInCameraWorld(double side, cv::Vec3d rvec, cv::Vec3d tvec){

     double half_side = side/2;


     // compute rot_mat
     cv::Mat rot_mat

;
     cv::Rodrigues(rvec, rot_mat);

     // transpose of rot_mat for easy columns extraction
     cv::Mat rot_mat_t = rot_mat.t();

     // the two E-O and F-O vectors
     double * tmp = rot_mat_t.ptr<double>(0);
     cv::Point3f camWorldE(tmp[0]*half_side,
                       tmp[1]*half_side,
                       tmp[2]*half_side);

     tmp = rot_mat_t.ptr<double>(1);
     cv::Point3f camWorldF(tmp[0]*half_side,
                       tmp[1]*half_side,
                       tmp[2]*half_side);

     // convert tvec to point
     cv::Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

     // return vector:
     vector<cv::Point3f> ret(4,tvec_3f);

     ret[0] +=  camWorldE + camWorldF;
     ret[1] += -camWorldE + camWorldF;
     ret[2] += -camWorldE - camWorldF;
     ret[3] +=  camWorldE - camWorldF;

     return ret;
}
