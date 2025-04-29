
#include <PZEM004Tv30.h>
#define RXD2 16 
#define TXD2 17
 
PZEM004Tv30 pzem_r(Serial2, RXD2, TXD2);
 
float vr;
float ir;
float freq;
float pf_r;
float energy;
float power;
 
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}
 
void loop() {
    baca_pzem();
    Serial.print("Volt: ");Serial.print(vr,2  );Serial.print("V, ");
    Serial.print("curr: ");Serial.print(ir,3);Serial.print("A, ");
    Serial.print("pf: ");Serial.print(pf_r);Serial.println("%, ");
    Serial.print("Power: ");Serial.print(power);Serial.print("W, ");
    Serial.print("Energy: ");Serial.print(energy,3);Serial.print("kWh, ");
    Serial.print("freq: ");Serial.print(freq,1);Serial.println("Hz, ");
    Serial.println();
    delay(2000);
}
 
void baca_pzem(){
    vr = pzem_r.voltage();
    ir = pzem_r.current();
    freq = pzem_r.frequency();
    pf_r = pzem_r.pf();
    power = pzem_r.power();
    energy = pzem_r.energy();
}