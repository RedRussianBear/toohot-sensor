#include <SPI.h>
#include <WiFi101.h>

//Pin constants for rangefinder
#define trigPin 4
#define echoPin 8
//Pin constants for infrared thermometer
#define IR_CLK 2 //this is interrupt driven, pin 2 or 3
#define IR_DATA 9 //data pin

WiFiClient client;

//Wireless hotspot connection details
char ssid[] = "Ji Hyuk's iPhone"; 
char pass[] = "mxpd867!";
int status = WL_IDLE_STATUS;

//IRT deserializer vars
volatile int nbits = 0;
volatile byte hexbyte = 0;
volatile unsigned char message[4];
volatile int nbytes = 0;
volatile int message_waiting = 0;

unsigned long last_time = 0;

//Processed sensor vars
float temp= 99.00;
float qtemp;
float ambient;
long dist;
boolean light = false;
boolean sent = false;

int irSetup = 1;
int n = 0;

float latestReading = 0.0;

void setupRangeFinder() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void setupTempSensor() {
  pinMode(IR_CLK, INPUT);
  pinMode(IR_DATA, INPUT);
  pinMode(13, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(IR_CLK), readBit, FALLING); //0 -> pin2, 1 -> pin3
}

void temp_output() {
   //updateTempSensor();
   float tmp = getFreshTemp(); //gets the temperature reading
   float amb = getFreshAmb(); //gets the ambient temperature  
   float ftmp = (tmp*1.8) + 32;
   float famb = (amb*1.8) + 32;
   Serial.print("IR: ");
   Serial.print(tmp);
   Serial.print(" C");
   Serial.print('\t');
   Serial.print(ftmp);
   Serial.print(" F\t\t");
   Serial.print("AMB: ");
   Serial.print(amb);
   Serial.print(" C");
   Serial.print('\t');
   Serial.print(famb);
   Serial.println(" F");
 
}

void dist_output() {
  Serial.print(dist);
  Serial.println(" cm");
}

void updateRangeFinder() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  dist = distance;
}

void updateTempSensor() {
  if (message_waiting == 1) {
    last_time = millis();
    if (message[0] == 0x4c) { //from zytemp
      int t = message[1]<<8 | message[2];
      temp = t/16.0 -273.15;
    } 
    else if (message[0] == 0x66) {
      int t = message[1]<<8 | message[2];
      ambient = t/16.0 -273.15;
    }
    message_waiting = 0;
  }

  //update every second
  if (millis() - last_time > 1000) {
    nbits = 0;
    nbytes = 0;
    hexbyte = 0;
    message_waiting = 0;
    last_time = millis();
  }
  
}

// Interupt routine for handling IR sensor clock trailing edge
void readBit() {
  int val = digitalRead(IR_DATA);  //read the data from the IR module
  if(!light) digitalWrite(13, HIGH);   // set the LED on
  else digitalWrite(13, LOW);
  light = !light;  //change light state when a bit was read in
  nbits++;
  int bit = (val == HIGH) ? 1 : 0; 
  hexbyte = (hexbyte << 1) | bit; //compound bytes so that we creat a 8-bit key
  if (nbits == 8) { 
    if (hexbyte == 0xd) {
      nbytes = 0;
      message_waiting = 1;
    } 
    else if (message_waiting == 0) {
      if (nbytes < 4) {
        message[nbytes] = hexbyte;
      }
      nbytes++;
    }
    hexbyte = 0;
    nbits = 0;
  }
}

float getFreshTemp() { 
  return temp;
}

float getFreshAmb() {
  return ambient; 
}

void sendData() {
    //Establish connection
    while(!client.connected()){
      client.stop();
      delay(10);
      client.connect("54.208.8.50", 80);
    }

    //Send data
    Serial.print("Sending ");
    String PostData="{\"temp\":" + String(qtemp) + ",\"sensorID\":1}"; 
    Serial.println(PostData);
    client.println("POST /api/temps HTTP/1.1");
    client.println("Host: 54.208.8.50");
    client.println("User-Agent: Arduino/1.0");
    client.println("Connection: close");
    client.println("Content-Type: application/json;");
    client.print("Content-Length: ");
    client.println(PostData.length());
    client.println();
    client.println(PostData);

    // Waits for server response
    while(!client.available()){
      delay(1); 
    }
  
    // Read response from server
    while(client.available()) {
      char c = client.read();
      Serial.print(c);
    }
  
    // Wait for server to terminate
    while(client.connected()){
      Serial.println("Waiting for server to terminate"); 
    }
  
    // Disconnect
    client.stop();
}

void setup() {
   Serial.begin(9600);


   while (status != WL_CONNECTED) {
    Serial.println("Attempting to connect to WPA network...");
    status = WiFi.begin(ssid, pass);
    delay(4000);
   }
   
   setupTempSensor(); //start temperature
   setupRangeFinder();
   client.setTimeout(500);

   
}

void loop() {
  updateTempSensor();
  updateRangeFinder();
  temp_output();
  dist_output();
  //if(sent && dist > 55) sent = false;
  if(!sent && dist <= 55){ 
    sent = true;
    n = 0;
    qtemp = (temp + 0.12*dist);
  }
  else if(sent) {
    n++;
    if((temp + 0.12*dist) > qtemp && dist <= 55) qtemp = (temp + 0.12*dist);
    if(n == 3) {
      sent = false;
      sendData();
    }
  }
  delay(100);
}



