
#include <SoftwareSerial.h>
#include <VirtualWire.h>

SoftwareSerial mySerial(2,3);//Rx,Tx
const int led_pin = 13;
const int transmit_pin = 9;
const int transmit_en_pin = 3;

int NORMAL=5,RECORD=6,AUTO=7;
int gaz=0,lacet=1,tangage=2,roulis=3;
int BOUTON_R=10;
int BOUTON_L=8;
int limite_H=800,limite_B=200;

void setup()
{
    // Initialise the IO and ISR
    vw_set_tx_pin(transmit_pin);
    vw_set_ptt_pin(transmit_en_pin);
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);       // Bits per sec
    pinMode(led_pin, OUTPUT);
    pinMode(BOUTON_R, INPUT);
    pinMode(BOUTON_L, INPUT);
    pinMode(NORMAL, INPUT);
    pinMode(RECORD, INPUT);
    pinMode(AUTO, INPUT);

    mySerial.begin(9600);
}

char msg[2];
void loop()
{
  if(digitalRead(NORMAL)==HIGH){
    msg[0]='N';
    msg[1]='0';
    sendMessage();
  }
  if(digitalRead(AUTO)==HIGH){
    msg[0]='A';
    msg[1]='U';
    sendMessage();
  }
  if(digitalRead(RECORD)==HIGH){
    msg[0]='R';
    msg[1]='E';
    sendMessage();
  }
  if(digitalRead(BOUTON_R)==LOW){
    msg[0]='B';
    msg[1]='R';
    sendMessage();
  }
  if(digitalRead(BOUTON_L)==LOW){
    msg[0]='B';
    msg[1]='L';
    sendMessage();
  }
  if(analogRead(gaz)>limite_H){
    msg[0]='U';
    msg[1]='P';
    sendMessage();
  }
  if(analogRead(gaz)<limite_B){
    msg[0]='D';
    msg[1]='W';
    sendMessage();
  }
  if(analogRead(tangage)>limite_H){
    msg[0]='A';
    msg[1]='V';
    sendMessage();
  }
  if(analogRead(tangage)<limite_B){
    msg[0]='A';
    msg[1]='R';
    sendMessage();
  }
  if(analogRead(roulis)>limite_H){
    msg[0]='G';
    msg[1]='A';
    sendMessage();
  }
  if(analogRead(roulis)<limite_B){
    msg[0]='D';
    msg[1]='R';
    sendMessage();
  }
  if(analogRead(lacet)>limite_H){
    msg[0]='H';
    msg[1]='O';
    sendMessage();
  }
  if(analogRead(lacet)<limite_B){
    msg[0]='A';
    msg[1]='H';
    sendMessage();
  }

}
void sendMessage(){
  digitalWrite(led_pin, HIGH); // Flash a light to show transmitting
  vw_send((uint8_t *)msg, 2);
  mySerial.println(msg);
  vw_wait_tx(); // Wait until the whole message is gone
  digitalWrite(led_pin, LOW);
  delay(200);
}
