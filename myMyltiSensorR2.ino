//NO for MYSBOOTLOADER  #define MY_OTA_FIRMWARE_FEATURE
#define MY_RADIO_NRF24 // Enable and select radio type attached 
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_BAUD_RATE 9600
//#define MY_DEBUG 

#include <MySensors.h>
#include <Wire.h>
#include <SPI.h>
#include <BME280I2C.h>


#define CHILD_ID_TEMP1 1
#define CHILD_ID_MOISTURE 2
#define CHILD_ID_TEMP2 11
#define CHILD_ID_HUM 12
#define CHILD_ID_PRE 13
#define CHILD_ID_VOLTS 250
#define CHILD_ID_VOLTS2 251

#define ANALOGSENSORS_VCC_PIN  A1 //Pover for analog sensors

#define BATTERYSENSORPIN A0
#define MOISTUREPIN A3
#define THERMISTORPIN A2
#define LEDPIN 8


#define SERIESRESISTOR 9930 //R of second resistor 10000
#define THERMISTORNOMINAL 10000 //R of termoresistor at 25C
#define TEMPERATURENOMINAL 25 //temp. at nominal resistance
#define BCOEFFICIENT 4050 // betta coeff. of termoresistor (usualy 3000-4000)
#define NUMSAMPLES 3

BME280I2C bme;
MyMessage msgHum1(CHILD_ID_MOISTURE, V_HUM);
MyMessage msgTemp1(CHILD_ID_TEMP1, V_TEMP);
MyMessage msgHum2(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp2(CHILD_ID_TEMP2, V_TEMP);
MyMessage msgPress(CHILD_ID_PRE, V_PRESSURE);
MyMessage msgVolts(CHILD_ID_VOLTS, V_VOLTAGE);
MyMessage msgVolts2(CHILD_ID_VOLTS2, V_VOLTAGE);
MyMessage msgDiagnostic(254, V_CUSTOM);


//const long InternalReferenceVoltage = 1080; //1062;  // Adjust this value to your board's specific internal BG voltage

bool metric = true;
unsigned long SLEEP_TIME;
unsigned long scale_constant;
float temp1cor, temp2cor;

int preMoisture = 111;
int prebatteryPcnt; //Ground
float preTemperature1, preTemperature2, preHumidity, prePressure;


void presentation() {
	String myVals;

	SLEEP_TIME = (unsigned long)(loadState(7)) * 10000;
	scale_constant = ((long)(loadState(2) * 256L) + loadState(1)) * 256 + loadState(0); //112530L;
	temp1cor = (float)(loadState(5) - 128) / 10;
	temp2cor = (float)(loadState(6) - 128) / 10;
	myVals = "1|" + String(scale_constant) + "|" + String(temp1cor, 1) + "|" + String(temp2cor, 1) + "|" + SLEEP_TIME;
	sendSketchInfo("MyltiSensor 1MHz", "0.8");
	present(CHILD_ID_TEMP1, S_TEMP, "Ground");
	present(CHILD_ID_MOISTURE, S_HUM, "Ground");
	present(CHILD_ID_TEMP2, S_TEMP, "Air");
	present(CHILD_ID_HUM, S_HUM, "Air");
	present(CHILD_ID_PRE, S_BARO, "Air");
	present(CHILD_ID_VOLTS, S_MULTIMETER, "Battery");
	present(CHILD_ID_VOLTS2, S_MULTIMETER, "Battery2");
	present(254, S_CUSTOM, myVals.c_str());
	metric = getControllerConfig().isMetric;
}

void setup(){
	pinMode(ANALOGSENSORS_VCC_PIN, OUTPUT); //Pover for analog sensors
	pinMode(LEDPIN, OUTPUT);
	while (!bme.begin()) {
	#ifdef MY_DEBUG
		Serial.println("Could not find BME280I2C sensor!");
	#endif
		wait(1000);
	}
	digitalWrite(LEDPIN, HIGH);
	wait(10000);
	digitalWrite(LEDPIN, LOW);
#ifdef MY_DEBUG
	Serial.println("--------------------------------");
	Serial.print("SLEEP_TIME1 ");
	Serial.println(SLEEP_TIME);
	Serial.print("scale_constant ");
	Serial.println(scale_constant);
	Serial.print("temp1cor ");
	Serial.println(temp1cor);
	Serial.print("temp2cor ");
	Serial.println(temp2cor);
	Serial.println("--------------------------------");
#endif
}

void loop()
{
	int Moisture, batteryPcnt; //Ground
	float Temperature1, Temperature2(NAN), Humidity(NAN), Pressure(NAN);
	int volts, volts2;

	volts = readVcc();
	batteryPcnt = (min(volts, 310) - 180) * 100 / 130;  //180 - min Volts; 130 - delta
	send(msgVolts.set((float)volts/100, 2));
	volts2= volts*analog_average(BATTERYSENSORPIN)/1023;
	send(msgVolts2.set((float)volts2/100, 2));
	digitalWrite(ANALOGSENSORS_VCC_PIN, HIGH);
	digitalWrite(LEDPIN, HIGH);
	wait(500); //1000
	digitalWrite(LEDPIN, LOW);
	Temperature1 = readTermoRez()+ temp1cor;
	Moisture=readMoisture();
	digitalWrite(ANALOGSENSORS_VCC_PIN, LOW);

	digitalWrite(LEDPIN, HIGH);
	wait(500);
	digitalWrite(LEDPIN, LOW);
	bme.read(Pressure, Temperature2, Humidity, metric, true);
	wait(10);
	bme.read(Pressure, Temperature2, Humidity, metric, true);
	Temperature2 += temp2cor;

	digitalWrite(LEDPIN, HIGH);
//	if (Temperature1 != preTemperature1){
		send(msgTemp1.set(Temperature1, 1));
		preTemperature1= Temperature1;
//	}
//	if (Moisture != preMoisture) {
		send(msgHum1.set(Moisture));
		preMoisture=Moisture;
//	}
//	if (Temperature2 != preTemperature2) {
		send(msgTemp2.set(Temperature2, 1));
		preTemperature2 = Temperature2;
//	}
//	if (Humidity != preHumidity) {
		send(msgHum2.set(Humidity, 0));
		preHumidity = Humidity;
//	}
//	if (Pressure != prePressure) {
		send(msgPress.set(Pressure, 0));
		prePressure = Pressure;
//	}
//	if (batteryPcnt != prebatteryPcnt) {
		sendBatteryLevel(batteryPcnt);
		prebatteryPcnt = batteryPcnt;
//	}
	digitalWrite(LEDPIN, LOW);
	#ifdef MY_DEBUG
	Serial.println("--------------------------------");
	Serial.print("Air Temperature1 ");
	Serial.print(Temperature1);
	Serial.println(" *C");
	Serial.print("Air Moisture ");
	Serial.print(Moisture);
	Serial.println("%");
	Serial.print("BME280 Temperature = ");
	Serial.print(Temperature2);
	Serial.println(" *C");
	Serial.print("BME280 Pressure = ");
	Serial.print(Pressure);
	Serial.println(" hPa");
	Serial.print("BME280 Humidity = ");
	Serial.print(Humidity);
	Serial.println("%");
	Serial.print("Volts = ");
	Serial.println(volts);
	Serial.print("Volts2 = ");
	Serial.println(volts2);
	Serial.print("batteryPcnt = ");
	Serial.print(batteryPcnt);
	Serial.println("%");
	Serial.println("--------------------------------");
	#endif
	smartSleep(SLEEP_TIME);
}
void receive(const MyMessage &message) {
	int newsleep;
	switch (message.type) {
		case V_VAR1: //Voltage
			scale_constant = message.getLong();
			saveState(0, (byte)(scale_constant & 0xFF));
			saveState(1, (byte)((scale_constant >> 8) & 0xFF));
			saveState(2, (byte)((scale_constant >> 16) & 0xFF));
			saveState(3, (byte)((scale_constant >> 24) & 0xFF));
			send(msgDiagnostic.set((uint32_t)scale_constant));
			break;
		case V_VAR2: //temp1 
			temp1cor = message.getFloat();
			saveState(5, (int)(temp1cor * 10 - 128));
			send(msgDiagnostic.set(temp1cor, 2));
			break;
		case V_VAR3: //temp2
			temp2cor = message.getFloat();
			saveState(6, (int)(temp2cor * 10 - 128));
			send(msgDiagnostic.set(temp2cor, 2));
			break;
		case V_VAR4: //V_VAR3 * 10000 - SLEEP_TIME, mls
			newsleep = message.getInt();
			SLEEP_TIME = (unsigned long)newsleep * 1000;
			saveState(7, (int)(newsleep / 10));
			send(msgDiagnostic.set((uint32_t)SLEEP_TIME));
			break;
		default:
			break;
	}
}

float readTermoRez() {
	float average;
	float steinhart;
	average = 1023 / analog_average(THERMISTORPIN)-1;
	average = SERIESRESISTOR / average; // R of termoresistor
	steinhart = average / THERMISTORNOMINAL; // (R/Ro)
	steinhart = log(steinhart); // ln(R/Ro)
	steinhart /= BCOEFFICIENT;
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
	steinhart = 1.0 / steinhart;
	steinhart -= 273.15; // from K to C
	return steinhart;
}
int readMoisture() {
	return(int)(analog_average(MOISTUREPIN)*100/1023);
}

float analog_average(uint8_t sensor) {
	uint8_t i;
	float average;
	int samples[NUMSAMPLES];

	analogReference(DEFAULT);
	for (i = 0; i < NUMSAMPLES; i++) {
		samples[i] = analogRead(sensor);
		//Serial.println(analogRead(sensor));
		wait(100);
	}
	for (i = 0; i< NUMSAMPLES; i++) {
		average += samples[i];
	}
	average = average / NUMSAMPLES;
	return average;
}

int readVcc() {
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring
	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both
	long result = (high << 8) | low;
	result = scale_constant / result; // Calculate Vcc (in mV/10); 112530 = 1.1*1023*10	//result = scale_constant / result; // Calculate Vcc (in mV); 
	return (int)result; // Vcc in millivolts
}