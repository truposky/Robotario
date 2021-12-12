 //robot2
#include <WiFiNINA.h>
#include "common.h"
#include <math.h>
#include "MeanFilterLib.h"
#include <Arduino_LSM6DS3.h>
#include <string>
#include <vector>


using namespace std;
//WiFi variable setup
//char ssid[] = "Robotarium";     // your network SSID (name)
//char pass[] = "robotarium";    // your network password (use for WPA, or use as key for WEP)
char ssid[] = "MiFibra-4300";     // your network SSID (name)
char pass[] = "SzreaH22";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
//UDP variables setup
/*-----------ip setup-----------------*/
IPAddress ip_arduino1(192,168,1,5);
IPAddress ip_server(192,168,1,2);
WiFiUDP Udp;
unsigned int localPort = 4243;      // local port to listen on
char packetBuffer[256]; //buffer to hold incoming packet

//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------
MeanFilter<long> meanFilterD(5);
MeanFilter<long> meanFilterI(5);
//Time variables
unsigned long previous_timer;
unsigned long timer=10000;


//operation variables
struct appdata operation_send;
struct appdata *server_operation;

//prototypes//
void printWifiStatus() ;
void op_saludo();
void op_message();
void op_moveWheel();
void op_StopWheel();
void motorSetup();
void moveForward(const int pinMotor[3], int speed);
void moveBackward(const int pinMotor[3], int speed);
void fullStop(const int pinMotor[3]);
void moveWheel(int pwm,double w, const int pinMotor[3],bool back);
void isrD();
void isrI();
int pidD(double wD);
int pidI(double wI);
double ajusteGyroscope(double z);
void feedForwardD();
void feedForwardI();
void tokenize(const string s, char c,vector<string>& v);

void setup() {
 
  motorSetup();
  //interrupciones para contar pulsos de encoder
  pinMode(encoderD,INPUT_PULLUP);
  pinMode(encoderI,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderI), isrI, RISING);//prepara la entrada del encoder como interrupcion
  attachInterrupt(digitalPinToInterrupt(encoderD), isrD, RISING);
  //se prepara la IMU para poder ser leida
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }
  //se empieza con los motores parados
  fullStop( pinMotorD);
  fullStop( pinMotorI);

  //comunicacion por puerto serie
  Serial.begin(9600);
 /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
  //---se prepara la comunicacion UDP------//
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  WiFi.config(ip_arduino1);
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);//se prepara el puerto para escuchar
}
  
long int n=0;
void loop() {
      currentTime=micros();
      
      double fD,fI;
      int ajusteD,ajusteI;
      timeStopD=millis();
      timeStopI=millis();
      //condition for know when the wheel is stoped
      //se define un contador de tiempo para comprobar que las reudas estan paradas
      deltaTimeStopD=timeStopD-timeAfterDebounceD;
      deltaTimeStopI=timeStopI-timeAfterDebounceI;
     
      meanFilterD.AddValue(deltaTimeD);
      meanFilterI.AddValue(deltaTimeI);
      
          if(deltaTimeStopD>=200){
            fD=0;
          }
          else{
            fD=(double)1/(meanFilterD.GetFiltered()*N)*1000000;
          }
          if(deltaTimeStopI>=200){
            fI=0;
          }
          else{
            fI=(double)1/(meanFilterI.GetFiltered()*N)*1000000;
          }
     if(n ==20){     
             //condicion para que no supere linealidad y se sature.
             //es un filtro para que no de valores ridiculos
          if(fD<18/2/3.14)
          {
            wD=2*3.14*fD;
           
          }
          if(fI<18/2/3.14 )
          {
             wI=2*3.14*fI;
          }
          //fin filtro
          if(wD !=0 && setpointWD !=0){
            
             ajusteD=pidD(wD);
             if(setpointWD != 0){ 
             PWM_D=PWM_D+ajusteD;
              if(PWM_D>MAXPWM){
               PWM_D=MAXPWM;
             }
            else if(PWM_D<=MINPWM){
               PWM_D=MINPWM;
             }
            }
          }
          if(wI !=0 && setpointWI !=0){
           ajusteI=pidI(wI); 
          
             if(setpointWI !=0){
               PWM_I=PWM_I+ajusteI;
               if(PWM_I>MAXPWM){
                 PWM_I=MAXPWM;
               }
              else if(PWM_I<=MINPWM){
                 PWM_I=MINPWM;
               }
             }
          }
         
         moveWheel(PWM_I,setpointWI,pinMotorI,backI);
         moveWheel(PWM_D,setpointWD,pinMotorD,backD);
     Serial.print(PWM_D);
     Serial.print(",");
     Serial.print(PWM_I);
     Serial.print(",");
     Serial.print(wD);
     Serial.print(",");
     Serial.println(wI);
     }
       // start the UDP on port 4243
      // if there's data available, read a packet
      memset (packetBuffer,'\0',MAXDATASIZE);
      int packetSize = Udp.parsePacket();
      if (packetSize) 
      {//se comprueba si se recibe un paquete
        
        IPAddress remoteIp = Udp.remoteIP();//proviene del servidor u ordenador central
        int numbytes = Udp.read((byte*)packetBuffer, MAXDATASIZE); //se guardan los datos en el buffer
        server_operation= (struct appdata*)&packetBuffer;
        //se comprueba longitud
        if((numbytes <  HEADER_LEN) || (numbytes != server_operation->len + HEADER_LEN))
        {
            Serial.print("(arduino1) unidad de datos recibida de manera incompleta \n");
            
        }
        else
        {
            switch (server_operation->op)
            {
                case OP_SALUDO:
                    op_saludo();
                    break;
                case OP_MOVE_WHEEL:
                    op_moveWheel();
                    wD=setpointWD;
                    wI=setpointWI;
                    break;
                case OP_STOP_WHEEL:
        
                     break;
                case OP_VEL_ROBOT:
                    op_vel_robot();
                    break;
                default:
        
                     break;
              
            }
        }
      }
     
     n++;
     n=n*(n<=20);
     timeAfter=micros();
    elapsedTime=(double)(timeAfter-currentTime);
    if(elapsedTime<SAMPLINGTIME)
    {
      delayMicroseconds(SAMPLINGTIME-elapsedTime);
    }
    else if(elapsedTime>SAMPLINGTIME){
      Serial.print(elapsedTime);
      Serial.print(",");
      Serial.println("error");
    }
  
  
}

void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void op_saludo()
{
    operation_send.op=OP_SALUDO;
    operation_send.id=ID;
    strcpy(operation_send.data,"hola Soy arduino1");
    operation_send.len = strlen (operation_send.data);  /* len */
    Udp.beginPacket(ip_server,Udp.remotePort());
    Udp.write((byte*)&operation_send,operation_send.len+HEADER_LEN);
    Udp.endPacket();
}
void op_message()
{

    
  
}
void op_moveWheel()
{
   
   // send a reply, to the IP address and port that sent us the packet we received
  strcpy(operation_send.data,"mensaje recibido");
  operation_send.op=OP_MESSAGE_RECIVE;
  operation_send.id=ID;
  operation_send.len = strlen (operation_send.data);  //len 
  Udp.beginPacket(ip_server,Udp.remotePort());
  Udp.write((byte*)&operation_send,operation_send.len+HEADER_LEN);
  Udp.endPacket();
  string data=server_operation->data;
  char del =',';
  vector<string> vel;
  tokenize(data, del,vel);
  setpointWD=stod(vel[0]);
  setpointWI=stod(vel[1]);
  
  if(setpointWD<0)
  {
    setPointGWD=setpointWD;
    setpointWD=setpointWD*(-1);
    backD=true;
  }
  else if(setpointWD>0)
  {
    backD=false;
  }
  if(setpointWI<0)
  {
    setPointGWI=setpointWI;
    setpointWI=setpointWI*(-1);
    backI=true;
  }
  else if(setpointWI>0)
  {
    backI=false;
  }
  //if feedforward detect a problem with discontinuitiy on velocity robot send an alert message
  feedForwardD();
  feedForwardI();

  moveWheel(PWM_I,setpointWI,pinMotorI,backI);
    moveWheel(PWM_D,setpointWD,pinMotorD,backD);
  for(int i=0;i<200;i++){
    meanFilterD.AddValue(deltaTimeD);
    meanFilterI.AddValue(deltaTimeI);
  }
}
void op_StopWheel(){
  
}

void op_vel_robot(){
  operation_send.op=OP_VEL_ROBOT;
  operation_send.id=ID;
  char del=',';
  sprintf(operation_send.data,"%lf",wD);
  sprintf(operation_send.data+sizeof(wD),"%c",del);
  sprintf(operation_send.data+sizeof(wD)+sizeof(del),"%lf",wI);
  operation_send.len = strlen (operation_send.data);  /* len */
  Udp.beginPacket(ip_server,Udp.remotePort());
  Udp.write((byte*)&operation_send,operation_send.len+HEADER_LEN);
  Udp.endPacket();
 }



 void motorSetup()
 {

   pinMode(pinIN1, OUTPUT);
   pinMode(pinIN2, OUTPUT);
   pinMode(pinENA, OUTPUT);
   pinMode(pinIN3, OUTPUT);
   pinMode(pinIN4, OUTPUT);
   pinMode(pinENB, OUTPUT);
}

void moveForward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], HIGH);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], speed);
}
void moveBackward(const int pinMotor[3], int speed)
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], HIGH);

   analogWrite(pinMotor[0], speed);
}
void fullStop(const int pinMotor[3])
{
   digitalWrite(pinMotor[1], LOW);
   digitalWrite(pinMotor[2], LOW);

   analogWrite(pinMotor[0], 0);
}
void moveWheel(int pwm,double w, const int pinMotor[3],bool back)
{
  
  if(pwm==0 || w==0){
    fullStop(pinMotor);
  }
  else
  {
    if(back)
    {
      moveBackward(pinMotor,pwm);
    }
    else if(!back)
    {
      moveForward(pinMotor,pwm);
    }
  }
  //se espera un tiempo antes de cambiar  PWM
  //no se usa delay opara evitar interferir con las interruociones.
  
}

void isrD()
{
  
  timeBeforeDebounceD=millis();//tiempo para evira rebotes
  deltaDebounceD=timeBeforeDebounceD-timeAfterDebounceD;// tiempo que ha pasdo entre interrupcion he interrupcion
  if(deltaDebounceD>TIMEDEBOUNCE)
  {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.
    
    startTimeD=micros();
    encoder_countD++;
    if(encoder_countD==3){
    deltaTimeD=startTimeD-timeAfterD;
    encoder_countD=0;
    }
    
    timeAfterD=micros();
  }
  timeAfterDebounceD=millis();
  
}

void isrI()
{
    
    timeBeforeDebounceI=millis();//tiempo para evira rebotes
    deltaDebounceI=timeBeforeDebounceI-timeAfterDebounceI;// tiempo que ha pasdo entre interrupcion he interrupcion
    if(deltaDebounceI>TIMEDEBOUNCE)
    {//condicion para evitar rebotes
      //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.
     
        startTimeI=micros();
        encoder_countI++;//se cuenta los pasos de encoder
        if(encoder_countI==3){
        
        deltaTimeI=startTimeI-timeAfterI;
        encoder_countI=0;
        }
      
        timeAfterI=micros();
    }
    timeAfterDebounceI=millis();  
  
}
int pidD(double wD)
{
  int outputD=0;
  
  errorD = setpointWD - wD;
  if(errorD>=0.30|| errorD<(-0.30))
  {
    cumErrorD += errorD * SAMPLINGTIME/1000;                      // calcular la integral del error
    if(lastErrorD>0 && errorD<0)cumErrorD=errorD;
    if(lastErrorD<0 && errorD>0)cumErrorD=errorD;
    if(cumErrorD>15|| cumErrorD<-20) cumErrorD=0;                         //se resetea el error acumulativo
    rateErrorD = (errorD - lastErrorD) / SAMPLINGTIME*1000;         // calcular la derivada del error
    outputD =static_cast<int> (round(0.04*errorD +0.038*cumErrorD + 0*rateErrorD ));     // calcular la salida del PID     0.0025*cumErrorD
    lastErrorD = errorD;                                      // almacenar error anterior
    Serial.println(errorD);
  }
  
  return outputD;
}


int pidI(double wI)
{
  int outputI;
  errorI = setpointWI - wI;   
  
              
  if(errorI>=0.3 || errorI<(-0.3))
  {
    cumErrorI += errorI * (SAMPLINGTIME/1000);  //pasado un tiempo se tiene que borrrar cumerror                    // calcular la integral del error
    
    if(lastErrorI>0 && errorI<0){
    cumErrorI=errorI;
    }
    if(lastErrorI<0 && errorI>0){
    cumErrorI=errorI;
    }
    if(cumErrorI>15||cumErrorI<-20) cumErrorI=0;     //se resetea el error acumulativo 
    rateErrorI = (errorI - lastErrorI) /(SAMPLINGTIME*1000);         // calcular la derivada del error
    
    outputI = static_cast<int> (round(0.04*errorI  + 0.036*cumErrorI + 0*rateErrorI));     // calcular la salida del PID 
    lastErrorI = errorI; 
  
  }
  else{
    outputI=0;
  }
  return outputI;
  
}


void feedForwardD()
{
    if(setpointWD==0){
      PWM_D=0;
    }
    else if(setpointWD<MINSETPOINT){
      setpointWD=6;
      PWM_D=95;
    }
    else if(setpointWD < LIM_LINEAL){
      PWM_D=round((setpointWD+0.0894785)/0.0793996);
    }
    if(PWM_D<94){
      PWM_D=94;
      strcpy(operation_send.data,"mensaje recibido, alerta pwm en discontinuidad");
    }
    else if(PWM_D>MAXPWM){
      PWM_D=MAXPWM;
    }
   
}
void feedForwardI()
{    if(setpointWI==0){
      PWM_I=0;
    }  
    else if(setpointWI<MINSETPOINT){
      setpointWI=6;
      PWM_I=98;
    }
    else if(setpointWI < LIM_LINEAL){
     PWM_I=round((setpointWI-0.199148)/0.07553);
    }
    if(PWM_I<94){
      PWM_I=94;
      strcpy(operation_send.data,"mensaje recibido, alerta pwm en discontinuidad");
    }
    else if(PWM_I>MAXPWM){
      PWM_I=MAXPWM;
    }
    
}

void tokenize(const string s, char c,vector<string>& v)//sirve para separa la entrada string.
{
   string::size_type i = 0;
   string::size_type j = s.find(c);

   while (j != string::npos) 
   {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}
