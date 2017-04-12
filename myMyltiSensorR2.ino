//#define MY_OTA_FIRMWARE_FEATURE
#define MY_RADIO_NRF24 // Enable and select radio type attached 
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_BAUD_RATE 4800
#define MY_DEBUG 

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

#define ANALOGSENSORS_VCC_PIN  A1 //Pover for analog sensors

#define BATTERYSENSORPIN A0
#define MOISTUREPIN A3
#define THERMISTORPIN A2


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
MyMessage msgDiagnostic(254, V_CUSTOM);


const long InternalReferenceVoltage = 1080; //1062;  // Adjust this value to your board's specific internal BG voltage

bool metric = true;
unsigned long SLEEP_TIME;

int preMoisture = 111;
int prebatteryPcnt; //Ground
float preTemperature1, preTemperature2, preHumidity, prePressure;


void presentation() {
	sendSketchInfo("MyltiSensor 1MHz", "0.4");
	present(CHILD_ID_TEMP1, S_TEMP, "Ground");
	present(CHILD_ID_MOISTURE, S_HUM, "Ground");
	present(CHILD_ID_TEMP2, S_TEMP, "Air");
	present(CHILD_ID_HUM, S_HUM, "Air");
	present(CHILD_ID_PRE, S_BARO, "Air");
	present(CHILD_ID_VOLTS, S_MULTIMETER, "Battery");
	present(254, S_CUSTOM, "Diagnostic");
	metric = getControllerConfig().isMetric;
}

void setup(){
	SLEEP_TIME = (unsigned long)(loadState(7)) * 10000;

	pinMode(ANALOGSENSORS_VCC_PIN, OUTPUT); //Pover for analog sensors
	

	while (!bme.begin()) {
		Serial.println("Could not find BME280I2C sensor!");
		delay(1000);
	}
#ifdef MY_DEBUG
	Serial.println("--------------------------------");
	Serial.print("SLEEP_TIME ");
	Serial.println(SLEEP_TIME);
#endif
}

void loop()
{
	int Moisture, batteryPcnt; //Ground
	float Temperature1, Temperature2(NAN), Humidity(NAN), Pressure(NAN);
	int volts;


	//volts = getBandgap(); 
	volts = readVcc();
	send(msgVolts.set(volts, 2));
	volts = min(volts, 310); // 310 - max Volts
	batteryPcnt = (volts - 180) * 100 / 130;  //180 - min Volts; 130 - delta

	digitalWrite(ANALOGSENSORS_VCC_PIN, HIGH);
	wait(1000);
	Temperature1 = readTermoRez();
	Moisture=readMoisture();
	//digitalWrite(ANALOGSENSORS_VCC_PIN, LOW);
	bme.read(Pressure, Temperature2, Humidity, metric, true);
	wait(10);
	bme.read(Pressure, Temperature2, Humidity, metric, true);

	if (Temperature1 != preTemperature1){
		send(msgTemp1.set(Temperature1, 1));
		preTemperature1= Temperature1;
	}
	if (Moisture != preMoisture) {
		send(msgHum1.set(Moisture));
		preMoisture=Moisture;
	}
	if (Temperature2 != preTemperature2) {
		send(msgTemp2.set(Temperature2, 1));
		preTemperature2 = Temperature2;
	}
	if (Humidity != preHumidity) {
		send(msgHum2.set(Humidity, 0));
		preHumidity = Humidity;
	}
	if (Pressure != prePressure) {
		send(msgPress.set(Pressure, 0));
		prePressure = Pressure;
	}
//	if (batteryPcnt != prebatteryPcnt) {
		sendBatteryLevel(batteryPcnt);
		prebatteryPcnt = batteryPcnt;
//	}
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
int getBandgap() {
	// REFS0 : Selects AVcc external reference
	// MUX3 MUX2 MUX1 : Selects 1.1V (VBG)  
	ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1);
	ADCSRA |= bit(ADSC);  // start conversion
	while (ADCSRA & bit(ADSC))
	{
	}  // wait for conversion to complete
	int results = (((InternalReferenceVoltage * 1024) / ADC) + 5) / 10;
	return results;
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

	result = 110264L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*100
	//result = scale_constant / result; // Calculate Vcc (in mV); 
	return (int)result; // Vcc in millivolts
	

}