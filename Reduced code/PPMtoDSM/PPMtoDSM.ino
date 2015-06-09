// ServoDecodeTest
#include <math.h>
#include <ServoDecode.h>
#define NODEBUG // Change this to #define DEBUG if needed

#define MAX_DSM2_CHANNELS  6   // Max number of DSM Channels transmitted
#define DSM2_HEADER_LEN 2      // DSM2 Protocol header lenght
#define BIND_PIN 4             // Pin used to bind
#define PPM_OK_LED 13          // Pin used for PPM Ok signal LED
#define BINDING_LED 11         // Pin used for Binding in process LED

byte header[DSM2_HEADER_LEN];
byte dsmchannel[MAX_DSM2_CHANNELS*2]={0x00,0xaa,0x05,0xff,0x09,0xff,0x0d,0xff,0x13,0x54,0x14,0xaa};


void setup()			  // run once, when the sketch starts
{
  delay(1000);
  #ifdef DEBUG
  Serial.begin(9600);
  #else
  Serial.begin(125000);
  #endif
  ServoDecode.begin();
  pinMode(PPM_OK_LED,OUTPUT);
  pinMode(BINDING_LED,OUTPUT);
  pinMode(BIND_PIN,INPUT);
  digitalWrite(BIND_PIN, HIGH);

  //setFailsafe()  can set a pulse width for a channel if the Tx signal is lost.
  //By default all channels not explicitly set will hold the last good signal

  ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse  width
  delay(1000);

  while(digitalRead(BIND_PIN)==LOW){
    header[0]=0x80;
    header[1]=0;
    digitalWrite(BINDING_LED,HIGH);
    send_dsm2();
    #ifdef DEBUG
    Serial.println("BINDING Mode");
    #endif
  }
  digitalWrite(BINDING_LED,LOW);

  while(ServoDecode.getState()!= READY_state){
    delay(100);
  }

  header[0]=0;
  header[1]=0;

  send_dsm2();

}


void loop()
{

  if( ServoDecode.getState()!= READY_state) {
    digitalWrite(PPM_OK_LED,LOW);
  }
  else {
    int idx,pulse;
    int chanmap[]={3,1,2,4,5,6};  //Channel Mapping Table

    for ( int i=0; i<MAX_DSM2_CHANNELS; i++) {
      idx=(i*2);
      pulse=ServoDecode.GetChannelPulseWidth(chanmap[i])-1000;
      dsmchannel[idx]=(i<<2)+(pulse>>8);
      dsmchannel[idx+1]=(byte) pulse;
    }
    digitalWrite(PPM_OK_LED,HIGH);
  }
  send_dsm2();

}

void send_dsm2(){

  #ifndef DEBUG
    Serial.write(header,DSM2_HEADER_LEN);
    Serial.write(dsmchannel,MAX_DSM2_CHANNELS*2);
  #else
    Serial.print(header[0],HEX);Serial.print("\t");
    Serial.print(header[1],HEX);Serial.print("\t");
    for(int e=0;e<MAX_DSM2_CHANNELS*2;e++){
      Serial.print(dsmchannel[e],HEX);
      Serial.print("\t");
    }
    Serial.println(" ");
  #endif

  #ifdef DEBUG
    delay(2000);
  #else
    delay(20);
  #endif
}


