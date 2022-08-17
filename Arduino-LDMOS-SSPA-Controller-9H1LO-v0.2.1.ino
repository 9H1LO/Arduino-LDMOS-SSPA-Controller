/* 
 *  
 *  Arduino LDMOS SSPA Controller by 9H1LO
 *  
 *  Much of the code routines based on code by S21RC - Fazlay Rabby  https://github.com/s21rc
 *  
 *  Icom CI-V code based on projects from ON7EQ & VE1ZAC  https://www.qsl.net/on7eq/en/   http://www.ve1zac.com/
 *  
 *  This software is released under “Commons Clause” License Condition v1.0  https://commonsclause.com
 *  
 *  You may modify it as you wish and re-distribute as you wish for NO COMMERCIAL OR PROFIT USE - I hope we dont see sellers trying to make money off this like we saw with the IC705 M5Stack display 
 *  
 *  Please always mention Callsigns above as contributors to this code
 *  
 *  This is a "hobby" code and we will not be held responsable for it destroying you Radio, SSPA, LDMOS devices, mental health etc etc, use at your own risk
 *  
 *  
*/

#include <EEPROMex.h>     ////https://github.com/thijse/Arduino-EEPROMEx
#include "EasyNextionLibrary.h" /// https://github.com/Seithan/EasyNextionLibrary
#include <ShiftRegister74HC595.h> /// https://github.com/Simsso/ShiftRegister74HC595
#include "math.h" 
#include <SoftwareSerial.h>
#include "bands.h"

SoftwareSerial Serial1(2, 11); // RX, TX

// To remove the debugging output just comment this define
#define DEBUG
#define TEMPDEBUG
#define CIVDEBUG

#define VCC A0
#define ID A1
#define INTAKE_TEMP A2
#define EXHAUST_TEMP A3
#define HEATSINK_TEMP A4
#define PTT A5
#define PWR_REF A6
#define PWR_FWD A7

// d2 -- civ rx
// d3 -- free for now...maybe alarm buzzer later ???
// d4 - mux data
// d5 - mux latch
// d5 - mux clock
#define BIAS_OFF 7
#define LED_PO 8
#define LED_I 9
#define LED_SWR 10
// d11 -- used for uart debug tx (serial1)
#define input2 12
#define antenna2 13


int alarm_maxTemp = 35;
int alarm_minVolt = 25;
int alarm_maxVolt = 60;
int alarm_maxCurrent = 10;


bool hlVolt = false;

///CIV stuff
int unsigned long QRG = 0; 
int unsigned long QRGcomp = 0; 
byte BAND =(0);                                 // the actual band we are on
byte oldBAND = (0);
int BANDRLY;
// some control variables
int i;
int incoming;


/* =======  Tempareture NTC sensor and Tempareture vars   ======== */
static const uint8_t NTC_pins[] = {A2, A3, A4};
#define nominal_resistance 10000       //Nominal resistance at 25⁰C
#define nominal_temeprature 25   // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5    // Number of samples
#define beta 3950  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 5100   //Value of  resistor used for the voltage divider
int samples = 0;   //array to store the samples

unsigned long lastTempRequest = 0;
int  TempdelayInMillis = 500;  ///// temperature sensor reading internal is ms

float tempC1 = 0.0 ;
float tempC2 = 0.0 ;
float tempC3 = 0.0 ;
int ItempC1 = 0;
int ItempC2 = 0;
int ItempC3 = 0;


//////// 
EasyNex myNex(Serial);

// create a global shift register object
// parameters: <number of shift registers> (data pin, clock pin, latch pin)
ShiftRegister74HC595<1> sr(4, 6, 5);

bool PTT_status = false;
int antenna_relay_mem = 0;
int input2_relay_mem = 0;
int band_relay_mem = 0;
int Nrelay;
unsigned long lastVRequest = 0;
unsigned long lastIRequest = 0;
float Vin = 0.00;
float Vout = 0.00;

int graph_Watt = 0;
int graph_Temp = 0;
int graph_Swr = 0;

int operate = 0;
int civ_auto = 0;

int graph_maxWatt = 600; //Scale from 0 to 600, set as per your SSPA board
int graph_maxTemp = 60; //Scale from 0 to 55, set as per your PA board.
int graph_maxSwr = 2; // Scale of the SWR for display
int VdelayInMillis = 1000; // refresh V display every N mili sec, set as per your liking
int IdelayInMillis = 100; // refresh I display every N mili sec, set as per your liking
float Res_1 = 150000.00; //Set R1 of voltage devider (VR VM pot, center to VDD) (you can use 150K 1% tol resistor)
float Res_2 = 10000.00 ; //Set R2 of voltage devider (VR VM pot, center to GND) (you can use 10K 1% tol resistor)
// Calibration factors for SWR/PO
int calibrP = 58;                // Assume 3.3v = 1000w in 50R. CalibrP = (3.3 / 5.0 x 1024 + Vdiode)x(3.3 / 5.0 x 1024 + Vdiode) / 1000w
int Vdiode = 60;                  // if we assume FWD voltage diode = 0,3v : Vdiode = 0,3 v / 5.0 v  x 1024
int T_pepHOLD = 600;          // msec pep hold time before return


/* ======== power and swr related variable ======== */

long lastTpep = 0;                                   // update PEP display


unsigned long   power_fwd = 0;              // power forward (watts)
unsigned long   power_ref = 0;              // power reflected (watts)
unsigned int   power_fwd_max = 0;           // power forward max (for peak hold)
unsigned int   power_ref_max = 0;           // power reflected max (for peak hold)

float SWR = 0;                            // SWR
float power_ratio = 0;                         // Power ratio P forward / P refl
int swr_display = 0;

bool error_i_status = false;
bool error_swr_status = false;
bool error_po_status = false;
bool error_temp_status = false;
bool error_v_status = false;

bool antenna2_status = false;


/* === Monitor and show PTT status on display === */
void onair() {
  int PTT_en = digitalRead(PTT);
  if (operate ==1){
  if (PTT_en == 0 && PTT_status != true) {
    myNex.writeNum("e1.val", 1);
    PTT_status = HIGH;
    bias_pa(1);
    // LCDband_disable();
  }
  else if (PTT_en == 1 && PTT_status != false) {

    myNex.writeNum("e1.val", 0);
    PTT_status = LOW;
    bias_pa(0);
    //LCDband_enable();
  }
}
}


//standby-oper
void trigger8(){
  if(PTT_status != HIGH){
  if (operate != true) {
    operate = 1;
    myNex.writeStr("bt1.txt", "OPER");
    myNex.writeNum("bt1.pco", 65535); 
  }else{
    operate = 0;
    myNex.writeStr("bt1.txt", "STBY");
    myNex.writeNum("bt1.pco", 33808); 
  }
}
}


void bias_pa(int bias) {
  digitalWrite(BIAS_OFF, bias);
}


/* === Monitor and show over current error on display === */
void error_i() {
  int error_i_en = digitalRead(LED_I);
  //error_i_en = 0; //test
  if (error_i_en == LOW && error_i_status == false) {

    myNex.writeNum("e4.val", 1);
    error_i_status = true;
  }
  else if (error_i_en == HIGH && error_i_status == true) {

    myNex.writeNum("e4.val", 0);
    error_i_status = false;
  }
}
/* === Monitor and show over SWR error on display === */
void error_swr() {
  int error_swr_en = digitalRead(LED_SWR);
  //error_swr_en = 0; //test
  if (error_swr_en == LOW && error_swr_status == false) {
    //hmiSend("e3.val=",1);
    myNex.writeNum("e3.val", 1);
    error_swr_status = true;
  }
  else if (error_swr_en == HIGH && error_swr_status == true) {
    //hmiSend("e3.val=",0);
    myNex.writeNum("e3.val", 0);
    error_swr_status = false;
  }
}
/* === Monitor and show over Power error on display === */
void error_po() {
  int error_po_en = digitalRead(LED_PO);
  // error_po_en = 0; //test ****************************************
  if (error_po_en == LOW && error_po_status == false) {

    myNex.writeNum("e2.val", 1);
    error_po_status = true;
  }
  else if (error_po_en == HIGH  && error_po_status == true) {
    myNex.writeNum("e2.val", 0);
    error_po_status = false;
  }
}
/* === Volt meter Function === */
void read_volt() {
  float vcc;
  int v1_val = 0;
  int V = 0;
  v1_val = analogRead(VCC);
  Vout = (v1_val * 5.00) / 1024.00;
  Vin = Vout / (Res_2 / (Res_1 + Res_2));
  V = ((Vin * 10) + 0.5);

  if ( V <= (alarm_minVolt*10) || V >= (alarm_maxVolt*10) ){
  hlVolt = true;
  }else{
     hlVolt = false;
  }


 if ( !error_v_status && hlVolt) {
    alarm(1);
    myNex.writeNum("e5.val", 1);
    error_v_status = true;
  }

  if (error_v_status && !hlVolt) {
    myNex.writeNum("e5.val", 0);
    alarm(0);
    error_v_status = false;
  }


  
  if (millis() - lastVRequest >= VdelayInMillis)
  {
    myNex.writeNum("Cv.val", V);
    myNex.writeNum("Gv.val", (V/10));
    lastVRequest = millis();
  }
}

/* === Over temp protection and show/clear temp error=== */
void error_temp(bool highTemp) {
  if (error_temp_status == false && highTemp) {
    alarm(1);
    myNex.writeNum("e6.val", 1);
    error_temp_status = true;
  }

  if (error_temp_status == true && !highTemp) {
    myNex.writeNum("e6.val", 0);
    alarm(0);
    error_temp_status = false;
  }

}
///// shared alarm
void alarm(bool alarm){
      bias_pa(0);
      operate = 0;
      PTT_status = 0 ;
  if (alarm){
    myNex.writeStr("bt1.txt", "ALRM");
    myNex.writeNum("bt1.pco", 63488); 
    myNex.writeNum("e1.val", 0);
  }else{
    myNex.writeStr("bt1.txt", "STBY");
    myNex.writeNum("bt1.pco", 33808); 
}
}

/* === Tempareturn Monitor  === */
void read_temp(){
  float temp[] = {0,0,0};
 if (millis() - lastTempRequest >= TempdelayInMillis){
  for (int x = 0; x < 3; x++){
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead((NTC_pins[x]));
    //delay(10);
  }
  average = 0;
  average = samples / samplingrate;
  average = 1023 / average - 1;
  average = Rref / average;
  float temperature;
  temperature = average / nominal_resistance;     // (R/Ro)
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= beta;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C
  temp[x] = temperature;
 }

  tempC1 = temp[0];
  tempC2 = temp[1];
  tempC3 = temp[2];
  ItempC1 = tempC1*10;
  ItempC2 = tempC2*10;
  ItempC3 = tempC3*10;

#ifdef TEMPDEBUG
   Serial1.print("Temperatures -- 1: " );
   Serial1.print(tempC1);
   Serial1.print("  2: " );
   Serial1.print(tempC2);
   Serial1.print("  3: " );
   Serial1.println(tempC3);
#endif

  lastTempRequest = millis(); 
  }

  
  if(tempC3>=(alarm_maxTemp-1)){
    error_temp(1);
//   setPwmDuty(0);
  }

  
  if(tempC3<=(alarm_maxTemp-5) && error_temp_status){
     error_temp(0);
   }

  
    myNex.writeNum("Ct1.val", ItempC1);
    myNex.writeNum("Ct2.val", ItempC2);
    myNex.writeNum("Ct3.val", ItempC3);
  
  float graph_limit = (graph_maxTemp/100.00);
  graph_Temp = (tempC3 / graph_limit);
  myNex.writeNum("Gt.val", graph_Temp);
 // int tempAverage = ((ItempC1+ItempC2+ItempC3)/3)/10;
 // myNex.writeNum("Gt.val", tempC3);
//  fanspeed();
  }



/* === Monitor current from protection board  === */
void read_ID() {
  int I;
  float Is;
  float Idv;
  float ref_vdd = (5.00 / 1024);

  Idv = ( analogRead(ID) * ref_vdd );
  Is = (Idv * 13000);
  Is = (Is / 4482);
  I = ((Is * 10) + 0.5);
  //  I = 123; //test value


  if (millis() - lastIRequest >= IdelayInMillis)
  {
    myNex.writeNum("Ca.val", I);
    myNex.writeNum("Gi.val", I);
    lastIRequest = millis();
  }

}
/* === SWR/Po calculation Function === */
void read_power() {
  power_fwd = analogRead(PWR_FWD);
  power_ref = analogRead(PWR_REF);

  if (power_fwd > 5) {        // only correct for diode voltage when more than zero
    power_fwd = (power_fwd + Vdiode) * (power_fwd + Vdiode) / calibrP;
  }

  if (power_ref > 5) {            // only correct for diode voltage when more than zero
    power_ref = (power_ref + Vdiode) * (power_ref + Vdiode) / calibrP;
  }

  // detect SWR error / load mismatch
  power_ratio = power_fwd / power_ref;           // calculate ratio with raw data
  SWR = abs ((1 + sqrt(power_ratio)) / (1 - sqrt(power_ratio))) ;

  // hold peak

  if (power_fwd >= power_fwd_max) {
    lastTpep = millis();
    power_fwd_max = power_fwd;
  }

  if (millis() > (lastTpep + T_pepHOLD)) { // clear the peak after hold time
    power_fwd_max = power_fwd;
  }


  swr_display = (SWR * 10) + 0.5;   // Float x 10 and convert to int with rounding

  if (swr_display < 10) {   // SWR cannot be lower than 1.0
    swr_display = 10 ;
  }

  //power_fwd_max is power output
  // swr_display: swr value x 10
}
/* === SWR/Po display Function === */
void display_power() {
  // power_fwd_max = 432; //Test value *********************************

  myNex.writeNum("Cw.val", power_fwd_max);

  float graph_limit_watt = (graph_maxWatt / 100.00);
  graph_Watt = (power_fwd_max / graph_limit_watt);
  // graph_Watt = 80; //Test value ***************************
  // swr_display = 15; //Test value ***************************
  myNex.writeNum("Gw.val", graph_Watt);
  myNex.writeNum("Cs.val", swr_display);

  float graph_limit_swr = ((graph_maxSwr - 1) / 100.00);
  graph_Swr = ((swr_display / 10) - 1) / (graph_limit_swr); // calculate from 1 to max swr.
  //graph_Swr = 15; //Test value *******************************
  myNex.writeNum("Gs.val", graph_Swr);

}
//Antenna2 relay
void trigger9() {
  if (PTT_status != HIGH) {
    digitalWrite(antenna2, LOW); //relay OFF
    myNex.writeNum("ant1.val", 1);
    myNex.writeNum("ant2.val", 0);
    EEPROM.writeInt(0, 0);
  }
}
void trigger10() {
  if (PTT_status != HIGH) {
    digitalWrite(antenna2, HIGH); //relay ON
    myNex.writeNum("ant2.val", 1);
    myNex.writeNum("ant1.val", 0);
    EEPROM.writeInt(0, 1);
  }
}
//Input 2 relay
void trigger11() {
  if (PTT_status != HIGH) {
    digitalWrite(input2, LOW);
    myNex.writeNum("rad1.val", 1);
    myNex.writeNum("rad2.val", 0);
    EEPROM.writeInt(10, 0);
    //Serial.println("Set-eeprom1-0");
  }
}
void trigger12() {
  if (PTT_status != HIGH) {
    digitalWrite(input2, HIGH);
    myNex.writeNum("rad1.val", 0);
    myNex.writeNum("rad2.val", 1);
    EEPROM.writeInt(10, 1);
    //Serial.println("Set-eeprom1-1");

  }
}
//// band from display touch
void trigger0() {
  set_band_relay(0);
}
void trigger1() {
  set_band_relay(1);
}
void trigger2() {
  set_band_relay(2);
}
void trigger3() {
  set_band_relay(3);
}
void trigger4() {
  set_band_relay(4);
}
void trigger5() {
  set_band_relay(5);
}
void trigger6() {
  set_band_relay(6);
}
void trigger7() {         /// CIV ENABLE 
  if (civ_auto != true) {
    civ_auto = 1;
    myNex.writeStr("b7.txt", "AUTO");
    myNex.writeNum("b7.pco", 65535); 
  }else{
    civ_auto = 0;
    myNex.writeStr("b7.txt", "MAN");
    myNex.writeNum("b7.pco", 33808); 
  }
     EEPROM.writeInt(15, civ_auto);
  
}


void set_band_relay(int Nrelay) {
  if (PTT_status != HIGH) {
    sr.setAllLow(); // set all pins LOW
    sr.set(Nrelay, HIGH);
    String buttonid1 = "b";
    String buttonid2 = ".val";
    buttonid1.concat(Nrelay);
    buttonid1.concat(buttonid2);
    myNex.writeNum(buttonid1, 1);
    EEPROM.writeInt(20, Nrelay);
  }
}
void setup() {
  //delay (1000); //wait for display initialization
  myNex.begin(115200);  // start Nextion Display
  Serial1.begin(19200);  // debug on TX...CIV on RX

  pinMode(antenna2, OUTPUT);
  pinMode(input2, OUTPUT);
  pinMode(PTT, INPUT);
  pinMode(BIAS_OFF, OUTPUT);
  pinMode(PTT, INPUT);
  pinMode(LED_PO, INPUT);
  pinMode(LED_I, INPUT);
  pinMode(LED_SWR, INPUT);
  pinMode(VCC, INPUT);


  /* ==== Temp sensor setup ==== */
  lastTempRequest = millis(); 

  
  //// restore relay state from eeprom
  antenna_relay_mem = EEPROM.readInt(0);
  if (antenna_relay_mem == 1) {
    digitalWrite(antenna2, HIGH); //relay ON
    myNex.writeNum("ant2.val", 1);
  } else {
    digitalWrite(antenna2, LOW); //relay OFF
    myNex.writeNum("ant1.val", 1);
  }

  //// restore relay state from eeprom
  input2_relay_mem = EEPROM.readInt(10);
  if (input2_relay_mem) {
    digitalWrite(input2, HIGH); //relay ON
    myNex.writeNum("rad2.val", 1);
  } else {
    digitalWrite(input2, LOW); //relay OFF
    myNex.writeNum("rad1.val", 1);
  }

 //// restore civ state from eeprom
  civ_auto = EEPROM.readInt(15);
 if (civ_auto) {
    civ_auto = 1;
    myNex.writeStr("b7.txt", "AUTO");
    myNex.writeNum("b7.pco", 65535); 
  }else{
    civ_auto = 0;
    myNex.writeStr("b7.txt", "MAN");
    myNex.writeNum("b7.pco", 33808); 
  }

  


  /// filters
  sr.setAllLow(); // set all pins LOW
  band_relay_mem = EEPROM.readInt(20);
  set_band_relay(band_relay_mem);


  #ifdef DEBUG
     Serial1.print("EEPROM --  ");
     Serial1.print("Ant Rly (0): ");
     Serial1.print(antenna_relay_mem);
     Serial1.print(" - In Rly (10): ");
     Serial1.print(input2_relay_mem);
     Serial1.print(" - CIV (15): ");
     Serial1.print(civ_auto);     
     Serial1.print(" Band Rly (20): ");
     Serial1.println(band_relay_mem);
  #endif

  

}
void loop() {
  myNex.NextionListen();
  onair(); //Check PTT status
  read_volt(); // Read V
  read_ID(); //Read I
  read_power(); //Read Po and SWR
  display_power(); // Display power
  error_i(); // Check Error: I
  error_swr(); // Check Error: SWR
  error_po(); // Check Error: Po
  read_temp(); // Read Temp
  icom_civ();
}
// civ stuff
// Assemble a message from the ICOM rig.
// If it is a frequency message, return the frequency
// This keeps track of the current frequency and only
// returns a non-zero value when the frequency has changed
unsigned char buffget[16];
int idx = 0;
unsigned long lastfreq = 0;
unsigned long get_frequency(void){
  unsigned char c;
  unsigned long t_QRG,MHZ;
  unsigned int KHZ,HZ;

  // If there's no input character return
  if(Serial1.available() <= 0){
    return(0);
  }
  // If we haven't found an end of message, reset and try again.
  if(idx >= 15) {
    idx = 0;
      #ifdef CIVDEBUG
      Serial1.println("RESET");
      #endif
    return(0);
  }
  // Read the character
  c = buffget[idx] = Serial1.read();
  // Look for the 0xFE that starts the message
  if(idx == 0) {
    if(c == 0xFE)idx++;
    return(0);
  }
  // We've found the 0xFE, now look for the 0xFD at the end
  if(c != 0xFD) {
    idx++;
    return(0);
  }
#ifdef CIVDEBUG
for(int j=0;j<=idx;j++) {
  if(buffget[j] < 10)Serial1.print("0");
  Serial1.print(buffget[j],HEX);
}
Serial1.println("");
#endif

  idx = 0;
  // There is a complete message in buffer get
  // If it is not a frequency message, ignore it
  if ((buffget[4] != 0) && (buffget[4] != 3) &&  (buffget[4] != 5)) return(0);


  // decode the frequency and return it
  MHZ = buffget[8];

  MHZ = MHZ - (((MHZ/16) * 6));              // Transform bytes ICOM CAT 
  if(MHZ >= 100) return(0);


  KHZ = buffget[7];
  KHZ = KHZ - (((KHZ/16) * 6));              // Transform bytes ICOM CAT 
  if(KHZ >= 100) return(0);


  HZ = buffget[6]; 
  HZ = HZ - (((HZ/16) * 6));                 // Transform bytes ICOM CAT 
  if(HZ >= 100) return(0);


  t_QRG = ((MHZ * 10000) + (KHZ * 100) + (HZ * 1)); // QRG variable stores frequency in MMkkkH  format

  if(t_QRG == lastfreq)return(0);
  lastfreq = t_QRG;
  return(t_QRG);

}

void icom_civ(){
  if (civ_auto){

  
  // Frequency change message from IC706
  // FEFE00580000350507FD
  

  if((QRG = get_frequency()) == 0)return;


  // Which band ?

  QRGcomp = QRG / 10;
  BAND = 99;                // default band = will generate error
  if ((QRGcomp < High4)   and (QRGcomp > Low4))  {   BAND = 6;  }
  if ((QRGcomp < High6)   and (QRGcomp > Low6))  {   BAND = 6;  }
  if ((QRGcomp < High10)  and (QRGcomp > Low10)) {   BAND = 5; }
  if ((QRGcomp < High12)  and (QRGcomp > Low12)) {   BAND = 5;  }
  if ((QRGcomp < High15)  and (QRGcomp > Low15)) {   BAND = 4;  }
  if ((QRGcomp < High17)  and (QRGcomp > Low17)) {   BAND = 4;  }
  if ((QRGcomp < High20)  and (QRGcomp > Low20)) {   BAND = 3;  }
  if ((QRGcomp < High30)  and (QRGcomp > Low30)) {   BAND = 3;  }
  if ((QRGcomp < High40)  and (QRGcomp > Low40)) {   BAND = 2;  }
  if ((QRGcomp < High60)  and (QRGcomp > Low60)) {   BAND = 2;  }
  if ((QRGcomp < High80)  and (QRGcomp > Low80)) {   BAND = 1;  }
  if ((QRGcomp < High160) and (QRGcomp > Low160)){   BAND = 0;  }
  if (BAND == 99) {
    operate = 0;
    myNex.writeStr("b7.txt", "OOB!");
    myNex.writeNum("b7.pco", 63488);
    myNex.writeNum("bt1.pco", 63488); 
    }else{
    operate = 1;
    myNex.writeStr("b7.txt", "AUTO");
    myNex.writeNum("b7.pco", 65535); 
    myNex.writeNum("bt1.pco", 65535); 
    }

     
#ifdef DEBUG
Serial1.print("BAND: ");
Serial1.println(BAND);
#endif

  if (BAND == oldBAND) return;  // no bandchange

  ///// Band is changed ! /////
  set_band_relay(BAND);

    String buttonid1 = "b";
    String buttonid2 = ".val";
    buttonid1.concat(oldBAND);
    buttonid1.concat(buttonid2);
    myNex.writeNum(buttonid1, 0);
    

  oldBAND = BAND; 

}

}





void getTemp(){
  for (int x = 0; x < 3; x++){
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead((NTC_pins[x]));
    //delay(10);
  }
  average = 0;
  average = samples / samplingrate;
  average = 1023 / average - 1;
  average = Rref / average;
  float temperature;
  temperature = average / nominal_resistance;     // (R/Ro)
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= beta;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C

  #ifdef TEMPDEBUG
  Serial1.print("Temperature ");
  Serial1.print("pin: ");
  Serial1.print(NTC_pins[x]);
  Serial1.print("  ");
  Serial1.println(temperature);
  #endif
  }
  #ifdef TEMPDEBUG
  Serial1.println("-------------");
  #endif
}
