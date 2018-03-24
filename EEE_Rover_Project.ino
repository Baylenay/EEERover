//***EEE Rover Project***
// Created by Thomas Bayley on 23/01/2018 for team Tennessine
// Modified on 27/01/2018 by Thomas Bayley
// Brief: So far a function has been written to convert input values from a joystick to motor voltages.
// This has been achieved by imagining the joystick coordinate system on a complex plane [x, y] where x
// is real and y is imaginary. Therefore, given certain values for x and y the argument can be obtained.
// This allows us to describe the motor voltages as function of {abs(x, y), arg(x, y)}. Consider the motor
// a (left motor - see fig.1). When the argument, -PI < arg(x, y) < PI, of x and y is 0, x can take any +ve
// non-zero value and y must be 0. When the argument of x and y is PI/2, x is 0 and y must be any +ve non-zero 
// value. If we go find the values at all arguements round the phasor diagram we find that a as a function
// of the argument of x, y is equal to: a = {1+cos[arg(x, y)]}/2.For it is just PI out of phase. So, the
// equation for b is: b = {1+cos[arg(x, y)-PI]}/2. But this only gives us the motor voltage based on the
// difference x and y. We also would like to consider the magnitude of x and y. So we take the square root of
// the sum of the squares of x and y. We then multiply a and b by this to give:
// a = sqrt(x^2+y^2)*{1+cos[arg(x, y)]}/2
// and
// b = sqrt(x^2+y^2)*{1+cos[arg(x, y)-PI]}/2
//
// Also, the outline of function has been created but not finished called Check_Mat() which will check to see
// which materials have been identified.
//
//  _           _
// | |         | |
// |A|----o----|B|
// |_|         |_|
//
// fig.1: shows the wheels of the rover with A being the left motor and B being the right wheel motor
//
//
//Modified on 10/03/2018 by Thomas Bayley
//More functions have been created. In order to connect to wifi a function which uses content from the WiFiUdp library
//example has been created called printWifiStatus. To send and recieve data two functions have been created Get_Packet
//and Send_Packet. A function called get_freq has been made which uses example code from the FreqCounter library and is
//called in the Check_Mat function which has been updated. Most is now complete. However, we still need to define how to
//code the information we are going to send in the Send_Packet function.

#include <WiFiLink.h>
#include <WiFiUdp.h>
#include <FreqCounter.h>
#define PI 3.14159

//**Constant Variables**:
const int A = A2; // These are the motor pins and fig.1 shows which
const int B = A3; // pins correspond to which wheels on the rover
const int dir = A4;
const int radio_freq_c = 5; // radio carrier frequency will be fed to this pin
                            // - must be pin 5 since FreqCounter fn only works on this pin
const int radio_freq_i = 6; // radio information frequency will be fed to this pin
const int infrared_freq = 7; // frequency can be one of two
const int mag_field = 8; // input signal is high or low
const int ultrasonic = 9; // input signal is high or low

//**Variables**:
bool Gab;
bool Nuc;
bool Dur;
bool Bro;
bool Che;
bool Yea;
int reset; // this is in case not all properties of a material are detetected at once
           // if a property is detected it will be stored until the reset which will occur
           // when another drum needs to be measured. While reset is 0 store value of properties.
           // if it goes to 1 then assume no properties detected.
double dir_x; // A value between 0-1023 corresponding to the voltage at X
double dir_y; // A value between 0-1023 corresponding to the voltage at Y
int dir_ab;    // tells the user which direction the motor a and b are running: forwards = 1, backwards = 0
double a;     // amplitude of voltage accross left motor
double b;     // amplitude of voltage accross right motor
double k = 1; // scaling factor can be changed to correct voltage at a and b
int hfrq_dev = 5000; // this is the maximum frequency deviation from the true signal (high frequency signals)
int lfrq_dev = 50; // this is the maximum frequency deviation from the true signal (low frequency signals)
int mag_field_state = 0;
int ultrasonic_state = 0;
int mag_field_flag = 0;
int ultrasonic_flag = 0;
int radio_67_flag = 0;
int radio_103_flag = 0;
int infrared_421_flag = 0;
int infrared_607_flag = 0;

int status = WL_IDLE_STATUS;
char ssid[] = "EEERover"; //  The network SSID (name)
char pass[] = "exhibition";    // The network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // The network key Index number (needed only for WEP)
unsigned int localPort = 2390;      // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP Udp;

//**Declaring Functions**:
long int get_freq();
char Get_Packet();
void Send_Packet(char out_packet);
void printWifiStatus();
void Check_Mat();                   // function identifies which materials have been detected
double arg(double x, double y);     // function returns the argument of the complex number: x + iy
void Motor_Mag(double X, double Y); // function calculates values of a and b


//**Writing Functions**:

long int get_freq(){
  long int frq;
  FreqCounter::f_comp= 8;             // Set compensation to 12
  FreqCounter::start(1400);            // Start counting with gatetime of 100ms
  while (FreqCounter::f_ready == 0)         // wait until counter ready
  frq = FreqCounter::f_freq;            // read result
  return frq;
}

char Get_Packet() {
  int packetSize = Udp.parsePacket();
  if (packetSize){
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    return packetBuffer;
  }
  return;
}

void Send_Packet(char out_packet) {
  IPAddress remoteIp = Udp.remoteIP();
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(out_packet);
  Udp.endPacket();
  return;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void Check_Mat() {
  // code goes here - set bool values to true or false depending on whether or not material has been detected
  // There are 6 materials to be detected: Gaborite, Nucinkisite, Durranium, Brookesite, Cheungtium and Yeatmanine
  // these will be abbreviated to: Gab, Nuc, Dur, Bro, Che and Yea
  // Each material will be defined as a bool so that if detected its variable will become 'true' and if undetected will remain 'false'
   // check properties of materials:
   mag_field_state = digitalRead(mag_field);
   if (mag_field_state == HIGH){
     mag_field_flag = 1;
   }
   ultrasonic_state = digitalRead(ultrasonic);
   if (ultrasonic_state == HIGH){
     ultrasonic_flag = 1;
   }
   if (-hfrq_dev < get_freq()-67000 < hfrq_dev){
     radio_67_flag = 1;
   }
   if (-hfrq_dev < get_freq()-103000 < hfrq_dev){
     radio_103_flag = 1;
   }
  // Each material will be defined as a bool so that if detected its variable will become 'true' and if undetected will remain 'false'
   // once all infromation has been gathered try to determine which material
   if(radio_67_flag = 1){
    if(ultrasonic_flag = 1){ 
      Gab = true;
    }
    else{
      Nuc = true;
    }
   }
   if(radio_103_flag = 1){
    if(mag_field_flag = 1){ 
      Dur = true;
    }
    else{
      Bro = true;
    }
   }
   else{
    if(ultrasonic_flag = 1){ 
      Yea = true;
    }
    else{
      Che = true;
    }
   }
   
}

double arg(double x, double y){
  double theta = atan(abs(y/x));
  if (x>0) {  // if x is +ve
    if(y>0){  // if y is +ve
      dir_ab = 1;
      return theta;
    }
    if (y<0){ // if y is -ve
      dir_ab = 0;
      return -theta;
    }
    else {    // if y = 0
      dir_ab = 1;
      return 0;
    }
  }
  if(x<0)  {  // if x is -ve
    if(y>0){  // if y is +ve
      dir_ab = 1;
      return PI-theta;
    }
    if (y<0){ // if y is -ve
      dir_ab = 0;
      return theta-PI;
    }
    else {    // if y = 0
      dir_ab = 1;
      return -PI;
    }
  }
  else{ // if x = 0
    if(y>0){  // if y is +ve
      dir_ab = 1;
      return PI/2;
    }
    if (y<0){ // if y is -ve
      dir_ab = 0;
      return -PI/2;
    }
    else {    //if y = 0
      dir_ab = 1;
      return 0;
    }
  }
}

void Motor_Mag(double X, double Y) {
  X = k*(X-512); // Since when at centre joystick is centred at middle analogRead pin will read half the maximum of 1024.
  Y = k*(Y-512); // So will read 512 at the pin. So to remove this bias such that at rest dir_x and dir_y are 0 we must subtract 512.
  a = (sqrt(pow(X, 2)+pow(Y, 2)))*(1+cos(arg(X, Y)))/2;
  b = (sqrt(pow(X, 2)+pow(Y, 2)))*(1+cos(arg(X, Y)-PI))/2;
  if(dir_ab == 1){
    digitalWrite(dir, HIGH);
  }
  else{
    digitalWrite(dir, LOW);
  }
  analogWrite(A, a);
  analogWrite(B, b);
  return;
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Check if communication with wifi module has been established
  if (WiFi.status() == WL_NO_WIFI_MODULE_COMM) {
    Serial.println("Communication with WiFi module not established.");
    while (true); // don't continue:
  }

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Chan/ge this line if using open or WEP network:
    status = WiFi.begin(ssid,pass);

    // wait 10 seconds for connection:
    digitalWrite(13, HIGH);
    delay(10000);
    digitalWrite(13, LOW);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
  

  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  
  
}

void loop() {
  if(Get_Packet() == "reset"){ // This is just exmaple. Real code name will be different (maybe just 1 character)
    int mag_field_flag = 0;
    int ultrasonic_flag = 0;
    int radio_67_flag = 0;
    int radio_103_flag = 0;
    int infrared_421_flag = 0;
    int infrared_607_flag = 0;  
  }

  else{ // code must be coordinates. Therefore first decode packet buffer first
    // set dir_x and dir_y = to first and second half of PacketBuffer here
    Motor_Mag(dir_x, dir_y);
  }

  Check_Mat();
}
