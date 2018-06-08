#define TINY_GSM_MODEM_SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

#define FONA_RX 3 //connect to FONA RX
#define FONA_TX 4 //connect to FONA TX
#define SerialMon Serial
SoftwareSerial SerialAT = SoftwareSerial(FONA_TX, FONA_RX); //initialize software serial

/////IBM bluemix Data START
#define ORG "<>"
#define DEVICE_TYPE "arduino"
#define DEVICE_ID "arduinoBoardGPS"
#define TOKEN "<>"


typedef struct GPSData {

String Lat;
String Lon;
String Date;
String Time;

}GPSData ;

GPSData gpsData;

const  char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
const  char topic[] = "iot-2/evt/status/fmt/json";
const  char authMethod[] = "use-token-auth";
char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;
//END

int SLEEP_MINUTES = 1; //Sleep time

/////GPRS Data START
const char apn[]  = "internet.vodafone.gr";
const char user[] = "";
const char pass[] = "";
//END

////General Data START
unsigned long ATtimeOut = 10000; // How long we will give an AT command to comeplete
//END

/////Libraies init START
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
//END

long lastReconnectAttempt = 0;

void setup() 
{
	SerialMon.begin(9600);
	delay(10);
	SerialAT.begin(9600);

	setupInternet();
	GPSon();
	setupgps();

	delay(3000);
	// MQTT Broker setup
	mqtt.setServer(server, 1883);
}

void loop() 
{
	if(getLocation()) { //If getLocation gets TRUE, print the rest!
		//Print Lat/Lon Values
		mqtt.loop();
		mqqtConnectionEngine();
		mqtt.loop();

		SerialMon.print(gpsData.Lat,gpsData.Lon,gpsData.Date,gpsData.Lon);

		flushFONA();
		SerialMon.print("End of getLocation\n");

		for(int i = 1; i < SLEEP_MINUTES; i++) {
			delay(30000);
			SerialMon.print("Minute:");
			SerialMon.print(i);
			SerialMon.print("\n");
		}

		SerialMon.print("inizializing..wait 10 seconds..\n");
		delay(10000);

	} else {
		delay(5000);
	}
}

boolean mqttConnect() 
{
	SerialMon.print("Connecting to ");
	SerialMon.print(server);
	if (!mqtt.connect(clientId,authMethod,token)) {
		SerialMon.println(" fail");
		return false;
	}

	SerialMon.println(" OK");
	return mqtt.connected();
}

void publishPayLoad()
{
	SerialMon.print("Sending payload: ");

	if (mqtt.publish(topic, (char*) buildJson().c_str())) {
		SerialMon.println("Publish ok");
	} else {
		SerialMon.println("Publish failed");
	}
}

String buildJson() 
{
	const size_t bufferSize = JSON_ARRAY_SIZE(2);
	DynamicJsonBuffer jsonBuffer(bufferSize);
	JsonObject& root = jsonBuffer.createObject();

	root["Latitude"]=gpsData.Lat;
	root["Longitude"]=gpsData.Lon;
	String jsonStr;
	root.printTo(jsonStr);

	return jsonStr;
}

void mqqtConnectionEngine()
{
	if (mqtt.connected()) {
		mqtt.loop();
		publishPayLoad();
		mqtt.loop();
		delay(5000);

	} else {
		// Reconnect every 10 seconds
		unsigned long t = millis();

		if (t - lastReconnectAttempt > 10000L) {
			lastReconnectAttempt = t;
			if (mqttConnect()) {
				lastReconnectAttempt = 0;
			}
		}
	}
}

void setupInternet()
{
	SerialMon.println("Initializing modem...");
	modem.restart();

	String modemInfo = modem.getModemInfo();
	SerialMon.print("Modem: ");
	SerialMon.println(modemInfo);

	SerialMon.print("Waiting for network...");
	if (!modem.waitForNetwork()) {
		SerialMon.println(" fail");
		while (true);
	}
	SerialMon.println(" OK");
	SerialMon.print("Connecting to ");
	SerialMon.print(apn);
	if (!modem.gprsConnect(apn, user, pass)) {
		SerialMon.println(" fail");
		while (true);
	}
	SerialMon.println(" OK");
}

//GPS FUNCTIONS
void setupgps()
{
	String ans;
	SerialMon.print("get localization ");
	if(sendATCommand("AT+CGNSSEQ=RMC", ans)){ //setup for GPS for fona 808
		SerialMon.print(ans + "\n");
	}
}

boolean getLocation() 
{
	String ans;
	SerialMon.print("get localization ");
	if(sendATCommand("AT+CGNSINF", ans)){ //turns off GPS for fona 808
		SerialMon.print(ans + "\n");

		if(ans.startsWith("+CGNSINF: 1,1,")) {
			SerialMon.print("Got Location\n"); //+CGNSINF: 1,1,20161222121541.000,41.931233,2.245903,530.100,0.04,269.5,1,,1.0,
            		gpsData.Date = ans.substring(14, 22);
			SerialMon.print("Date:" + Date + "\n");
            		gpsData.Time = ans.substring(22,28);
			SerialMon.print("Time:" + Time + "\n");
             		gpsData.Lat = ans.substring(33, 42);
			SerialMon.print("Lat:" + Lat + "\n");
            		gpsData.Lon = ans.substring(43, 51);
			SerialMon.print("Lon:" + Lon + "\n");
			
			return 1;

		} else {  //If the response of the device is does not start with +CGNSINF: 1,1,... then do nothing
			SerialMon.print("------------------debugger start-----------------\n");
			SerialMon.print("LOCATION NOT YET ACQUIRED \n");
			SerialMon.print(ans + "\n");
			SerialMon.print("------------------debugger stop-----------------\n");

			return 0;
		}
	} else { // if sendATCommand failed
		SerialMon.print("AT command failed: " + ans + "\n");
		return 0;
	}
}

void GPSon()
{
	String ans;
	SerialMon.print("TRYING TO TURN ON GPS ");
	if(sendATCommand("AT+CGNSPWR=1", ans)){ //Power GPS (1 - ON, 0 - OFF)
		SerialMon.print(ans + "\n");
	}
}

void GPSoff() 
{
	String ans;
	SerialMon.print("Turn off GPS ");
	if(sendATCommand("AT+CGNSPWR=0", ans)){ //Power GPS (1 - ON, 0 - OFF)
		SerialMon.print(ans + "\n");
	}
}

/*
 * Function to communicate with the
 * SIM808 module through AT commands
*/

boolean sendATCommand(String Command, String& ans)
{
	int complete = 0;
	char c;
	String content;
	unsigned long commandClock = millis();

	ans = "";
	SerialAT.println(Command);
	while(!complete && commandClock <= millis() + ATtimeOut) {
		while(!SerialAT.available() && commandClock <= millis()+ATtimeOut);
			while(SerialAT.available()) {
				c = SerialAT.read();
				if(c == 0x0A || c == 0x0D);
		}
		Serial.print("command: '" + String(Command) + "', response: '" + content + "'\n"); //Debug
		ans = content;
		complete = 1;
	}
	if (complete ==1) return 1;
	else return 0;
}

/*
 * Flush everything from the serial
 * and print it on the console
*/

void flushFONA()
{
	char inChar;
	while (SerialAT.available()){
		inChar = SerialAT.read();
		Serial.write(inChar);
		delay(20);
	}
}
