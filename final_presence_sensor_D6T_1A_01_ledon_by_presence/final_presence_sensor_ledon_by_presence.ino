//Switch on by means of presence sensor D6T-1A-01.  Application to turn on the light in a corridor or bathroom.
//Sensor is connected via I2C with and arudino Wifi 1010 and switched with a standard relay 5vdc//

/* includes */
#include <Wire.h>

/* defines */
#define D6T_ADDR 0x0A  // for I2C 7bit address
#define D6T_CMD 0x4C  // for D6T-44L-06/06H, D6T-8L-09/09H, for D6T-1A-01/02
#define N_ROW 1
#define N_PIXEL 1
#define N_READ ((N_PIXEL + 1) * 2 + 1)

uint8_t rbuf[N_READ];
double ptat;
double pix_data[N_PIXEL];
double actual_temp;
const int readings_size= 5;
int readings_counter = 0;
double sensor_readings_A[readings_size];
double base_average_temp = -1000;


uint8_t calc_crc(uint8_t data) {
    int index;
    uint8_t temp;
    for (index = 0; index < 8; index++) {
        temp = data;
        data <<= 1;
        if (temp & 0x80) {data ^= 0x07;}
    }
    return data;
}

/** <!-- D6T_checkPEC {{{ 1--> D6T PEC(Packet Error Check) calculation.
 * calculate the data sequence,
 * from an I2C Read client address (8bit) to thermal data end.
 */
bool D6T_checkPEC(uint8_t buf[], int n) {
    int i;
    uint8_t crc = calc_crc((D6T_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    bool ret = crc != buf[n];
    if (ret) {
        Serial.print("PEC check failed:");
        Serial.print(crc, HEX);
        Serial.print("(cal) vs ");
        Serial.print(buf[n], HEX);
        Serial.println("(get)");
    }
    return ret;
}

/** <!-- conv8us_s16_le {{{1 --> convert a 16bit data from the byte stream.
 */
int16_t conv8us_s16_le(uint8_t* buf, int n) {
    uint16_t ret;
    ret = (uint16_t)buf[n];
    ret += ((uint16_t)buf[n + 1]) << 8;
    return (int16_t)ret;   // and convert negative.
}

/** <!-- setup {{{1 -->
 * 1. Initialize 
       - initialize a Serial port for output.
	   - initialize I2C.
 */
 
void setup() {
  pinMode(6,OUTPUT);
  digitalWrite(6, LOW);
  Serial.begin(115200);  // Serial baudrate = 115200bps
  Wire.begin();  // i2c master
	delay(5000); //wait 5 sec before starting mesurements
  
}

/** <!-- loop - Thermal sensor {{{1 -->
 * 2. read data.
 */
void loop() {
  int i = 0;
	int16_t itemp = 0;
	// Read data via I2C
	// I2C buffer of "Arduino MKR" is 256 buffer. (It is enough)
    memset(rbuf, 0, N_READ);
    Wire.beginTransmission(D6T_ADDR);  // I2C slave address
    Wire.write(D6T_CMD);               // D6T register
    Wire.endTransmission();            
    Wire.requestFrom(D6T_ADDR, N_READ);
    while (Wire.available()) {
        rbuf[i++] = Wire.read();
    }
    D6T_checkPEC(rbuf, N_READ - 1);

    //Convert to temperature data (degC)
	for (i = 0; i < N_PIXEL; i++) {
		itemp = conv8us_s16_le(rbuf, 2 + 2*i);
		pix_data[i] = (double)itemp / 10.0;
  
  delay(500);	
	}
    
    //Output results

    Serial.print(" current room Temperature: ");
    actual_temp = pix_data[0]; 
    if (readings_counter == readings_size){ //initialize sum of measurements
    double sum = 0;
    for (i=0; i<readings_size; i++){
      sum = sum + sensor_readings_A[i]; //sum of the measurements 
    }
    double average = sum / readings_size; //calculate the average temperature

    if (base_average_temp == -1000){
      base_average_temp = average; //first measurement after booting dummy value -1000

    }else{ //standard measurement after booting
    double diff = average - base_average_temp; //actual average minus base average by booting
    Serial.print("temperature difference is: ");
    Serial.print(diff);
    if (diff < - 0.5){ //condition if room temperature decrease on 0.5 points, no human presence on the room
      if(digitalRead(6== HIGH)){
        digitalWrite(6, LOW);
        Serial.println("led off");  // port A6  
      }
     base_average_temp = average; //update base_average 
    }else if(diff > 0.5){
     if(digitalRead(6== LOW)){
        digitalWrite(6, HIGH);
        Serial.println("led on ");
      base_average_temp = average; //update base_average with my actual temperature 
    }
    }
    }

    Serial.print("average is: ");
    Serial.print(average);
    readings_counter= 0;

    }else{//fullfill the array til counter = size
    sensor_readings_A [readings_counter] = actual_temp;
    readings_counter += 1;

    }
    //Serial.print(pix_data[0]);
    Serial.print(actual_temp);
	//}	
    Serial.println(" [degC]");
    delay(500);

}

