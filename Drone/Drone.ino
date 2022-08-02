#include <helper_3dmath.h>
#include <MPU9250.h>
#include <VirtualWire.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
#include "math.h"

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}
// Initial time
long int ti,dt;
volatile bool intFlag=false;

int IN_M11=0,IN_M12=2,IN_M21=4,IN_M22=3,IN_M31=8,IN_M32=7,IN_M41=12,IN_M42=13,EN_M1=5,EN_M2=6,EN_M3=9,EN_M4=10;
int PIN_RADIO=11;
int buzzer=1;
int LED=7;
MPU9250 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

struct Position{
  float x;
  float y;
  float z;
};

struct rotations{
  float tangage;
  float roulis;
  float lacet;
};

rotations Rotations_C,Rotations_R,Rref;
Position position_C,position_R;

float max_tangage=30,max_roulis=30,max_lacet=180;
float Ref_tangage=0,Ref_roulis=0,Ref_lacet=0;
float min_tangage=-30,min_roulis=-30,min_lacet=0;
float X,Y,Z,ref_x=0,ref_y=0;
float Err_x,Err_y,Err_z;
float tangage=0,roulis=0,lacet=0;
float Err_tangage,Err_roulis,Err_lacet; //Err_x= x-capteur(x);
float tab_angle[70][6];//{X,Y,altitude,tangage,roulis,lacet}

long temp=millis();
boolean state=HIGH;
int lecture=0,lecture_test=0;
long delai_lecture=500;
long temp_lecture;
String message;
int element=0;

void setup() {
    // Arduino initializations
  Wire.begin();
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  
  // Store initial time
  ti=millis();
  dt=millis();

  getMotion(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  position_R=calculatePosition(ax, ay, az);
  position_C=calculatePosition(ax, ay, az);
        
  Rref.tangage=Ref_tangage;
  Rref.roulis=Ref_roulis;
  Rref.lacet=Ref_lacet;
  
  pinMode(IN_M11,OUTPUT);
  pinMode(IN_M21,OUTPUT);
  pinMode(IN_M31,OUTPUT);
  pinMode(IN_M41,OUTPUT); 
  pinMode(IN_M12,OUTPUT);
  pinMode(IN_M22,OUTPUT);
  pinMode(IN_M32,OUTPUT);
  pinMode(IN_M42,OUTPUT);
  pinMode(EN_M1,OUTPUT);
  pinMode(EN_M2,OUTPUT);
  pinMode(EN_M3,OUTPUT);
  pinMode(EN_M4,OUTPUT);
  pinMode(buzzer,OUTPUT);
  
  digitalWrite(LED,HIGH);
  tone(buzzer,220);
  delay(2000);
  noTone(buzzer);
  
  digitalWrite(LED,LOW);
  digitalWrite(IN_M11,LOW);
  digitalWrite(IN_M12,LOW);
  digitalWrite(IN_M21,LOW);
  digitalWrite(IN_M22,LOW);
  digitalWrite(IN_M31,LOW);
  digitalWrite(IN_M32,LOW);
  digitalWrite(IN_M41,LOW);
  digitalWrite(IN_M42,LOW);
    // Initialise the IO and ISR
  vw_set_rx_pin(PIN_RADIO);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);   // Bits per sec

  vw_rx_start();       // Start the receiver PLL running


        int  i;
        for(i=0;i<70;i++){
          tab_angle[i][0]=1000;
          tab_angle[i][1]=1000;
          tab_angle[i][2]=1000;
          tab_angle[i][3]=1000;
          tab_angle[i][4]=1000;
          tab_angle[i][5]=1000;
          tab_angle[i][6]=1000;
      }
}

  int16_t accx_p=500,daccx;
  int16_t accy_p=500,daccy;
  int16_t accz_p=1400,daccz;

// Counter
long int cpt=0;

int vitesse=50;// 0-255
int mode=0; //  0,mode normal, commande manuel ;
           //  1,mode auto,repetion de taches;
           //  2, mode enregistrement;

int Vinc=100;
void getMotion(int16_t ax1,int16_t ay1,int16_t az1, int16_t gx1,int16_t gy1,int16_t gz1,int16_t mx1,int16_t my1,int16_t mz1){
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  ax1 =-(Buf[0]<<8 | Buf[1]);
  ay1 =-(Buf[2]<<8 | Buf[3]);
  az1 = Buf[4]<<8 | Buf[5];
  
  // Gyroscope
  gx1=-(Buf[8]<<8 | Buf[9]);
  gy1=-(Buf[10]<<8 | Buf[11]);
  gz1=Buf[12]<<8 | Buf[13];
  // _____________________
  // :::  Magnetometer ::: 
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data
  mx1=0;
  my1=0;  
  Mag[7];  
  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
  // Magnetometer
  mx1=-(Mag[3]<<8 | Mag[2]);
  my1=-(Mag[1]<<8 | Mag[0]);    
  mz1=-(Mag[5]<<8 | Mag[4]);
  // Magnetometer
  mx1=mx1-200;
  my1=my1-70;
  mz1=mz1+700;
}
        
void loop(){
  if(mode==0){
    tone(buzzer,220);
    delay(500);
    noTone(buzzer);
    normal_loop();
  }
  if(mode==1){
    tone(buzzer,220);
    delay(500);
    noTone(buzzer);
    auto_loop();
  }
  if(mode==2){
    tone(buzzer,220);
    delay(500);
    noTone(buzzer);
    recorder_loop();
  }
}
void normal_loop(){
  while(mode==0){
    message=receiveMessage();
    
    if(message=="AU"){
      mode=1;
    } 
    if(message=="RE"){
      mode=2;
    } 
    
  if(message=="UP")//si gaz
  position_C.z+=5;
  else if(message=="DW")//si frein
  position_C.z-=5;
  else if(message=="AV")//si avance
  position_C.x+=5;
  else if(message=="AR")//si retour
  position_C.x-=5;
  else if(message=="GA")//si gauche
  position_C.y+=5;
  else if(message=="DR")//si droite
  position_C.y-=5;
  else if(message=="HO")//si rotaion à gauche
  lacet-=5;
  else if(message=="AH")//si rotaion à droite
  lacet+=5;
  else MotorWriter(vitesse,vitesse,vitesse,vitesse,0,0,true); // stationnaire
  /*
    if(tangage>max_tangage)
    tangage=max_tangage;
    
    if(tangage<min_tangage)
    tangage=min_tangage;
      
    if(roulis>max_roulis)
    roulis=max_roulis;
  
    if(roulis<min_roulis)
    roulis=min_roulis; 
    */ 

    if(lacet>max_lacet)
    lacet=max_lacet;
    
    if(lacet<min_lacet)
    lacet=min_lacet;    

           
        getMotion(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
         Rotations_R=calculateRotations(ax, ay, az,mx, my, mz);
        position_R=calculatePosition(ax, ay, az);
        Rotations_C.lacet=lacet;

        Err_x=position_C.x-position_R.x;
        Err_y=position_C.y-position_R.y;
        Err_z=position_C.z-position_R.z;
        Err_tangage=tangage-Rotations_R.tangage;//M1-M3%M2-M4
        Err_roulis=roulis-Rotations_R.roulis;//M1-M2%M3-M4
        Err_lacet=lacet-Rotations_R.lacet;//M1-M4

        Correct_x(Err_x,vitesse);
        Correct_y(Err_y,vitesse);
        Correct_z(Err_z,vitesse);
        Correct_tangage(Err_tangage,vitesse,Vinc);
        Correct_roulis(Err_roulis,vitesse,Vinc);
        Correct_lacet(Err_lacet,vitesse,Vinc);

  }
}
void auto_loop(){
 int i=0;
  while(mode==1){
    message=receiveMessage();
    
    if(message=="RE"){
      mode=2;
    } 
    if(message=="NO"){
      mode=0;
    } 
    if(millis()-temp>500){
      state=!state;
      temp=millis();
    }
    
    if(tab_angle[0][0]!=1000){
  
        getMotion(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        
        Rotations_R=calculateRotations(ax, ay, az,mx, my, mz);
        Rotations_C.tangage=tab_angle[lecture][3];
        Rotations_C.lacet=tab_angle[lecture][5];
        Rotations_C.roulis=tab_angle[lecture][4];
        position_R=calculatePosition(Rotations_C.tangage,Rotations_C.roulis,Rotations_C.lacet);
        position_C=calculatePosition(tab_angle[lecture][0],tab_angle[lecture][1],tab_angle[lecture][2]);

        Err_x=position_C.x-position_R.x;
        Err_y=position_C.y-position_R.y;
        Err_z=position_C.z-position_R.z;
        Err_tangage=Rotations_C.tangage-Rotations_R.tangage;//M1-M3%M2-M4
        Err_roulis=Rotations_C.roulis-Rotations_R.roulis;//M1-M2%M3-M4
        Err_lacet=Rotations_C.lacet-Rotations_R.lacet;//M1-M4

        if(abs(Err_x)<0.3 && abs(Err_y)<0.3 && abs(Err_y)<0.3 && abs(Err_tangage)<1 && abs(Err_roulis)<1 && abs(Err_lacet)<1){
          lecture ++;
          if(tab_angle[lecture][0]==1000)
            lecture=0;
        }
         else{
          Correct_x(Err_x,vitesse);
          Correct_y(Err_y,vitesse);
          Correct_z(Err_z,vitesse);
          Correct_tangage(Err_tangage,vitesse,Vinc);
          Correct_roulis(Err_roulis,vitesse,Vinc);
          Correct_lacet(Err_lacet,vitesse,Vinc);
        }
    }
  }
}

void recorder_loop(){
    element=0;
    int compt1=0,compt2=0;
  while(mode==2){
    message=receiveMessage();
    
    if(message=="AU"){
      mode=1;
    } 
    if(message=="NO"){
      mode=0;
    } 
   if(message=="UP")//si gaz
  Z+=5;
  else if(message=="DW")//si frein
  Z-=5;
  else if(message=="AV")//si avance
  X+=5;
  else if(message=="AR")//si retour
  X-=5;
  else if(message=="GA")//si gauche
  Y+=5;
  else if(message=="DR")//si droite
  Y-=5;
  else if(message=="HO")//si rotaion à gauche
  lacet-=5;
  else if(message=="AH")//si rotaion à droite
  lacet+=5;
  else MotorWriter(vitesse,vitesse,vitesse,vitesse,0,0,true); // stationnaire
  /*
    if(tangage>max_tangage)
    tangage=max_tangage;
    
    if(tangage<min_tangage)
    tangage=min_tangage;
      
    if(roulis>max_roulis)
    roulis=max_roulis;
  
    if(roulis<min_roulis)
    roulis=min_roulis; 
    */ 

    if(lacet>max_lacet)
    lacet=max_lacet;
    
    if(lacet<min_lacet)
    lacet=min_lacet;    

           
        getMotion(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
         Rotations_R=calculateRotations(ax, ay, az,mx, my, mz);
        position_R=calculatePosition(Rotations_R.tangage,Rotations_C.roulis,Rotations_C.lacet);
        Rotations_C.tangage=tangage;
        Rotations_C.lacet=lacet;
        Rotations_C.roulis=roulis;
        position_C=calculatePosition(Rotations_C.tangage,Rotations_C.roulis,Rotations_C.lacet);

        Err_x=position_C.x-position_R.x;
        Err_y=position_C.y-position_R.y;
        Err_z=position_C.z-position_R.z;
        Err_tangage=tangage-Rotations_R.tangage;//M1-M3%M2-M4
        Err_roulis=roulis-Rotations_R.roulis;//M1-M2%M3-M4
        Err_lacet=lacet-Rotations_R.lacet;//M1-M4

        Correct_x(Err_x,vitesse);
        Correct_y(Err_y,vitesse);
        Correct_z(Err_z,vitesse);
        Correct_tangage(Err_tangage,vitesse,Vinc);
        Correct_roulis(Err_roulis,vitesse,Vinc);
        Correct_lacet(Err_lacet,vitesse,Vinc);

  
       long tmp=millis();
   if(message=="RE"){
   
    tone(buzzer,420);
     if(element==0){
        int  i;
        for(i=0;i<50;i++){
          tab_angle[i][0]=1000;
          tab_angle[i][1]=1000;
          tab_angle[i][2]=1000;
          tab_angle[i][3]=1000;
          tab_angle[i][4]=1000;
          tab_angle[i][5]=1000;
        }
      }
      
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
         Rotations_R=calculateRotations(ax, ay, az,mx, my, mz);
        position_R=calculatePosition(Rotations_R.tangage,Rotations_C.roulis,Rotations_C.lacet);
      
      tab_angle[element][0]=position_R.x;
      tab_angle[element][1]=position_R.y;
      tab_angle[element][2]=position_R.z;
      tab_angle[element][3]=Rotations_R.tangage;
      tab_angle[element][4]=Rotations_R.roulis;
      tab_angle[element][5]=Rotations_R.lacet;
      element++;
    }
    
    if(millis()-tmp>500){
    noTone(buzzer);
    tmp=millis();
    }
  }
}

rotations calculateRotations(int16_t accx,int16_t accy,int16_t accz,int16_t mgx,int16_t mgy,int16_t mgz){
  rotations Rotations;
  Rotations.tangage=180*atan2(accx,accz)/M_PI;
//  Rotations.roulis=180*atan(accy/sqrt((accx*accx)+(accz*accz)))/M_PI;
  Rotations.roulis=180*atan2(accy,accz)/M_PI;
 Rotations.lacet=180*atan2(Rotations.roulis,Rotations.tangage)/M_PI;
  float yh=(mgy*cos(Rotations.roulis))-(mgz*sin(Rotations.roulis));
  float xh=(mgx*cos(Rotations.tangage))-(mgz*sin(Rotations.roulis)*sin(Rotations.tangage))+(mgz*cos(Rotations.roulis)*sin(Rotations.tangage));
  //Rotations.lacet=(180*atan(yh/xh)/M_PI);
  
  Rotations.tangage-=tangage_ref;
  Rotations.roulis-=roulis_ref;
  Rotations.lacet-=lacet_ref;
/*
  Rotations.lacet=Rotations.lacet/57.3;
  Rotations.tangage=Rotations.tangage/57.3;
  Rotations.roulis=Rotations.roulis/57.3;
*/
  return Rotations;
}
Vitesse {
  float x;
  float y;
  float z;
};
Vitesse V;

Position calculatePosition(int16_t accx,int16_t accy,int16_t accz){
  //Acc=sqrt(Accx*Accx+Accz*Accz+Accy*Accy);
  Position Pi;
  daccx=accx-accx_p;
  daccy=accy-accy_p;
  daccz=accz-accz_p; 
  if(sqrt(daccx*daccx+daccz*daccz+daccy*daccy)<50){
    daccx=0;
    daccy=0;
    daccz=0;
  }
   // acc en mm/s2
  dt=millis()-dt;
  float  dts;
  dts=(float)dt/1000; //dt en secondes 
  
  V.x =(float)daccx*dts; //V en mm/s
  V.y =(float)daccy*dts;
  V.z =(float)daccz*dts;
  
  Pi.x = Pi.x + dts*(V.x-Vx); //x en mm
  Pi.y = Pi.y + dts*(V.y-Vy);
  Pi.z = Pi.z + dts*(V.z-Vz);
  dt=millis();

  Vx=V.x;
  Vy=V.y;
  Vz=V.z;
  
  accx_p=accx;
  accy_p=accy;
  accz_p=accz;
}

String receiveMessage(){

    String message="";
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
  int i;

        digitalWrite(LED, HIGH); // Flash a light to show received good message
  // Message with a good checksum received, dump it.
  
  for (i = 0; i < buflen; i++)
  {
      message+=(char)buf[i];
   }
   digitalWrite(LED, LOW);
    }
  return message;
}
void MotorWriter(int vitesse_1,int vitesse_2,int vitesse_3,int vitesse_4,int dx,int dy,bool st){
  if(st){
    digitalWrite(IN_M11,LOW);
    digitalWrite(IN_M31,HIGH);
    digitalWrite(IN_M21,HIGH);
    digitalWrite(IN_M41,LOW);
    digitalWrite(IN_M12,HIGH);
    digitalWrite(IN_M32,LOW);
    digitalWrite(IN_M22,LOW);
    digitalWrite(IN_M42,HIGH);
  }
  if (dx==1){
    digitalWrite(IN_M11,LOW);
    digitalWrite(IN_M31,LOW);
    digitalWrite(IN_M21,HIGH);
    digitalWrite(IN_M41,HIGH);
    digitalWrite(IN_M12,HIGH);
    digitalWrite(IN_M32,HIGH);
    digitalWrite(IN_M22,LOW);
    digitalWrite(IN_M42,LOW);
  }
  if (dx==2){
    digitalWrite(IN_M11,HIGH);
    digitalWrite(IN_M31,HIGH);
    digitalWrite(IN_M21,LOW);
    digitalWrite(IN_M41,LOW);
    digitalWrite(IN_M12,LOW);
    digitalWrite(IN_M32,LOW);
    digitalWrite(IN_M22,HIGH);
    digitalWrite(IN_M42,HIGH);
  }
  
  if (dy==1){
    digitalWrite(IN_M11,HIGH);
    digitalWrite(IN_M21,HIGH);
    digitalWrite(IN_M31,LOW);
    digitalWrite(IN_M41,LOW);
    digitalWrite(IN_M12,LOW);
    digitalWrite(IN_M22,LOW);
    digitalWrite(IN_M32,HIGH);
    digitalWrite(IN_M42,HIGH);
  }
  if (dy==2){
    digitalWrite(IN_M11,LOW);
    digitalWrite(IN_M21,LOW);
    digitalWrite(IN_M31,HIGH);
    digitalWrite(IN_M41,HIGH);
    digitalWrite(IN_M12,HIGH);
    digitalWrite(IN_M22,HIGH);
    digitalWrite(IN_M32,LOW);
    digitalWrite(IN_M42,LOW);
  }
  digitalWrite(EN_M1,vitesse_1);
  digitalWrite(EN_M2,vitesse_2);
  digitalWrite(EN_M3,vitesse_3);
  digitalWrite(EN_M4,vitesse_4);
  
}
void Correct_lacet(float Err,int vitesse_c,int Vinc){
  int V=vitesse_c;
  int V2=Vinc-5;
  
  if(V2<0)
  V2=0;
  if(V1>255)
  V1=255;
  
    Vinc=V2;
    if(abs(Err)>1){
      if(Err<0){
        MotorWriter(V2,V,V,V2,0,0,false);
       }
      else 
        MotorWriter(V,V2,V2,V,0,0,false); 
    }
    else{
      Vinc=V;
    }
}
void Correct_tangage(float Err,int vitesse_c,int Vinc){
  int V=vitesse_c;
  int V2=Vinc-5;
  
  if(V2<0)
  V2=0;
  if(V1>255)
  V1=255;
  
    Vinc=V2;
    if(abs(Err)>1){
      if(Err<0){
        MotorWriter(V2,V,V2,V,0,0,false);
       }
      else 
        MotorWriter(V,V2,V,V2,0,0,false); 
    }
    else{
      Vinc=V;
    }
}
void Correct_roulis(float Err,int vitesse_c,int Vinc){
  int V=vitesse_c;
  int V2=Vinc-5;
  
  if(V2<0)
  V2=0;
  if(V1>255)
  V1=255;
  
    Vinc=V2;
    if(abs(Err)>1){
      if(Err<0){
        MotorWriter(V2,V2,V,V,0,0,false);
       }
      else 
        MotorWriter(V,V,V2,V2,0,0,false); 
    }
    else{
      Vinc=V;
    }
}
void Correct_x(float Err,int vitesse_c){
 int V=vitesse_c;
    if(abs(Err)>0.3){
      if(Err<0){
        MotorWriter(V,V,V,V,1,0,false);//on recule sur x; 
      }
      else {
        MotorWriter(V,V,V,V,2,0,false);//on avance sur x; 
        }
    }
}
void Correct_y(float Err,int vitesse_c){
 int V=vitesse_c;
    if(abs(Err)>0.3){
      if(Err<0){
        MotorWriter(V,V,V,V,0,1,false); // on recule sur y 
      }
      else {
        MotorWriter(V,V,V,V,0,2,false); // on avance sur y 
      }
    }
}
void Correct_z(float Err,int vitesse_c){
  int V1=vitesse_c+5;
  int V2=vitesse_c-5;
  
  if(V2<0)
  V2=0;
  if(V1>255)
  V1=255;
  
    if(abs(Err)>0.3){
      if(Err<0){
        MotorWriter(V2,V2,V2,V2,0,0,false); // on descend
        vitesse=V2;
        Vinc=V2;        
      }
      else {
        MotorWriter(V1,V1,V1,V1,0,0,false); // on monte
        vitesse=V1;
        Vinc=V1;
      }
    }
}
void Stationnaire(Position Pref,Position P,rotations Rref, rotations R,int vitesse){
  Err_z=Pref.z-P.z;
  Err_y=Pref.y-P.y;
  Err_x=Pref.x-P.x;
  
  Err_tangage=Rref.tangage-R.tangage;
  Err_roulis=Rref.roulis-R.roulis;
  Err_lacet=Rref.lacet-R.lacet;
  
  Correct_x(Err_x,vitesse);
  Correct_y(Err_y,vitesse);
  Correct_z(Err_z,vitesse);
  Correct_tangage(Err_tangage,vitesse,Vinc);
  Correct_roulis(Err_roulis,vitesse,Vinc);
  Correct_lacet(Err_lacet,vitesse,Vinc);
}
