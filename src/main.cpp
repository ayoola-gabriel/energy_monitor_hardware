#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <DHT.h>
#include <PZEM004Tv30.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPSupabase.h>
#include <vector>
#include "time.h"
#include <preferences.h>
#include <NimBLEDevice.h>
#include <ESPSupabaseRealtime.h>

// --- Pin definitions (per README) ---
const int PIN_PIR = 19;      // D19 - PIR Sensor in
const int PIN_DHT = 18;      // D18 - DHT11 data (placeholder)
const int PIN_LDR = 36;      // D36 - LDR (ADC)
const int PIN_ACS712 = 39;   // analog pin for ACS712 (set in hardware)
const int PIN_BTN_BULB = 5;  // D5 - Button Switch (Bulb Control)
const int PIN_BTN_FN = 17;   // D17 - Function Button
const int PIN_BTN_AC = 16;   // D16 - Button Switch (AC control)
const int PIN_LED_BULB = 4;  // D4 - LED (Bulb on)
const int PIN_LED_AC = 2;    // D2 - LED (AC on)

const int PIN_BULB_CTRL = 33; // D33 - Bulb control pin
const int PIN_AC_CTRL = 12;   // D12 - AC control pin

const int PZEM_RX_PIN = 13; 
const int PZEM_TX_PIN = 14; 
const int BULB_RX_PIN = 27; // D27 - PZEM RX for bulb
const int BULB_TX_PIN = 26; // D26 - PZEM TX for bulb
// UART for PZEM004T (uses Serial2): RX=D13, TX=D14 per README


struct __attribute__((__packed__)) TelemetryData {
    float ac_current=0.0;
    float ac_power=0.0;
    float ac_energy=0.0;
    float power_factor=0.0;
    float bulb_current=0.0;
    float bulb_power=0.0;
    float bulb_energy=0.0;
    float voltage=0.0;
    float total_energy=0.0;
    float frequency=0.0;
    float temperature=0.0;
    float humidity=0.0;
    uint16_t ambient_light=0;
    bool occupancy=false;
};

struct __attribute__((__packed__)) SettingsData {
    uint16_t ldr_threshold = 1600;
    uint16_t ldr_hysteresis = 100;
    float temp_threshold = 25.0;
    float temp_hysteresis = 2.0;
};

struct __attribute__((__packed__)) DeviceState {
	bool acOn = false;
	bool bulbOn = false;
	bool autoMode = false;
	bool predictionMode = false;
};

struct __attribute__((__packed__)) WifiCredentials {
    char ssid[32];
    char password[64];
};

uint32_t lastDay = 0;

// Global instances to hold current states
TelemetryData liveData;
SettingsData deviceSettings;
DeviceState deviceState;
WifiCredentials wifiCredentials;

// Supabase / WiFi settings (fill these in)
const char* WIFI_SSID = "ngm_media";
const char* WIFI_PASS = "media2023";
const char* SUPABASE_URL = "https://bnqmrkhzhinyhhzoevkf.supabase.co"; // no trailing slash
const char* SUPABASE_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImJucW1ya2h6aGlueWhoem9ldmtmIiwicm9sZSI6ImFub24iLCJpYXQiOjE3ODE0NzYxNzUsImV4cCI6MjA5NzA1MjE3NX0.VMCNEglHJeavrc98ClbshmgaaqQJ7aUitJS35PKOxiU";
const char* SUPABASE_TABLE = "measurements"; // target table name

LiquidCrystal_PCF8574 lcd(0x3F); // I2C address for LCD (adjust if needed)

Supabase db;
SupabaseRealtime realtime;

Preferences preferences;

// Timers
unsigned long lastSecond = 0;
unsigned long lastMinute = 0;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600; // Adjust for your timezone (e.g., 3600 for GMT+1)
const int   daylightOffset_sec = 0;

#define LDR_THRESHOLD_DEFAULT 1600
#define TEMP_THRESHOLD_DEFAULT 25.0
#define LDR_HYSTERESIS_DEFAULT 100
#define TEMP_HYSTERESIS_DEFAULT 2.0

#define DEFAULT_SSID "ngm_media"
#define DEFAULT_PASSWORD "media2023"

// Measurement accumulators
float energy_accumulator_wh = 0.0; // accumulate watt-hours or similar per minute
int samples_this_minute = 0;
float bulbEnergy = 0.0; // separate accumulator for bulb energy to track against thresholds
float acEnergy = 0.0; // separate accumulator for AC energy to track against thresholds
float bulbEnergyThreshold = 1000.0; // example threshold in Wh for notifications
float acEnergyThreshold = 3500.0; // example threshold in Wh for notifications

bool switchingCompleted = false;
uint32_t lastACStateChange = 0;
uint32_t lastBulbStateChange = 0;
uint32_t bulbChangeHysteresis = 10; // seconds to wait before allowing another bulb toggle in auto mode	
uint32_t acSwitchDelay = 180; // seconds to wait before allowing another AC toggle in auto mode
uint32_t bulbSwitchDelay = 120; // seconds to wait before allowing another bulb toggle in auto mode


// Display rotation
unsigned long lastDisplayRotate = 0;
int displayIndex = 0; // which screen to show

// Button state tracking (debounced)
bool bulbOn = false;
bool acOn = false;
int lastBtnBulbState = HIGH;
int lastBtnAcState = HIGH;

bool fnBtnPressed = false;
unsigned long lastDebounceBulb = 0;
unsigned long lastDebounceAc = 0;
const unsigned long DEBOUNCE_MS = 50;

// Preferences (persisted)
String prefWifiName = "";
String prefWifiPass = "";

// DHT
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

// PZEM instance using Serial2
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem_bulb(Serial1, BULB_RX_PIN, BULB_TX_PIN);

// Constants
const float ACS712_SENSITIVITY = 0.066; // V/A for 30A model
const float ADC_REF = 3.3; // ADC reference (ESP32) - adjust if different
const int ADC_MAX = 4095;
const float NOMINAL_VOLTAGE = 0.0; // used to compute power for ACS712-measured loads

// Calibrated resting output (mV) of ACS712 (Vcc/2 when no current). Populated at setup().
long acs712_offset_mv = 0;

bool deviceConnected = false;

//BLE services and characterisitics UUID
#define SERVICE_UUID "f9a04633-c3e8-48ed-8cf4-cb04dd04cd41" //ble service
#define TELEMETRY_CHAR_UUID "33fd7248-5c86-457c-a094-b07571706577" //test characterisitic uuid
#define SETTING_CHAR_UUID "d2b4a7f8-5c86-4e2e-9f42-6f37ab995703"
#define WIFI_CHAR_UUID "6310fdb2-62e4-4d4d-8f36-b7bd0f58bcf9"
#define STATE_CHAR_UUID "33fd7249-5c86-457c-a094-b07571706577"

// Global BLE Characteristic Pointers
NimBLECharacteristic* pTelemetryChar = nullptr;
NimBLECharacteristic* pSettingsChar = nullptr;
NimBLECharacteristic* pWifiChar = nullptr;
NimBLECharacteristic* pStateChar = nullptr;

bool setupWiFi() {
	Serial.println("Connecting to WiFi...");
	WiFi.persistent(false); // don't store credentials in flash (we'll manage it ourselves)
	
	// Prefer stored credentials if available
	if (prefWifiName.length() > 0) {
		Serial.print("Using pref WiFi: "); Serial.print(prefWifiName); Serial.print(" / "); Serial.println(prefWifiPass);
		WiFi.begin(prefWifiName.c_str(), prefWifiPass.c_str());
	} else {
		WiFi.begin(WIFI_SSID, WIFI_PASS);
	}
	unsigned long start = millis();
	int i = 0;
	lcd.clear();
	while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
		Serial.print(".");	
		i++;
		lcd.setCursor(0, 0);
		(i % 5 == 0) ? lcd.print("Connecting...") : lcd.print("Connecting.. ");
		delay(200);
	}
	if(WiFi.status() == WL_CONNECTED) {
		lcd.clear();
		lcd.print("WiFi Connected");
		Serial.println("\nWiFi connected.");
		Serial.print("IP address: "); Serial.println(WiFi.localIP());
		delay(1000);
		return true;
	} else {
		Serial.println("\nWiFi connection failed.");
		lcd.clear();
		lcd.print("WiFi Failed");
		delay(1000);
		return false;
	}
}

bool reconnectWiFi() {
	if (WiFi.status() == WL_CONNECTED) return true;
	Serial.println("Reconnecting to WiFi...");
	lcd.clear();
	lcd.print("Reconnecting WiFi");
	WiFi.disconnect();
	delay(1000);
	return setupWiFi();
}

void setupHardware() {
	pinMode(PIN_PIR, INPUT);
	pinMode(PIN_BTN_BULB, INPUT_PULLUP);
	pinMode(PIN_BTN_FN, INPUT_PULLUP);
	pinMode(PIN_BTN_AC, INPUT_PULLUP);
	pinMode(PIN_LED_BULB, OUTPUT);
	pinMode(PIN_LED_AC, OUTPUT);
	pinMode(PIN_BULB_CTRL, OUTPUT);
	pinMode(PIN_AC_CTRL, OUTPUT);
	pinMode(PIN_LDR, INPUT);
	pinMode(PIN_ACS712, INPUT);

	// LittleFS
	// if (!LittleFS.begin()) {
		// failed to mount
	// }

	// UART2 for PZEM: RX/TX
	Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
	Serial1.begin(9600, SERIAL_8N1, BULB_RX_PIN, BULB_TX_PIN);

	Wire.begin();
	Wire.beginTransmission(0x3F);
  	int err = Wire.endTransmission();
  	Serial.print("Error: ");
  	Serial.print(err);

    if (err == 0) {
    Serial.println(": LCD found.");
    lcd.begin(16, 2);  // initialize the lcd
	lcd.setBacklight(255);
	lcd.print("Welcome");
  } else {
    Serial.println(": LCD not found.");
	return;
  }  // if

  delay(2000);
	//Initialize PIR sensor by reading it once (some PIRs need this to stabilize)

	dht.begin();

	// configure ADC attenuation for better voltage range
#if defined(ARDUINO_ARCH_ESP32)
    analogRead(PIN_LDR); // dummy read to initialize ADC
	analogRead(PIN_ACS712); // dummy read to initialize ADC
	analogSetPinAttenuation(PIN_LDR, ADC_11db);
	analogSetPinAttenuation(PIN_ACS712, ADC_11db);
#endif

	// Ensure outputs start OFF
	digitalWrite(PIN_BULB_CTRL, LOW);
	digitalWrite(PIN_AC_CTRL, LOW);
	digitalWrite(PIN_LED_BULB, LOW);
	digitalWrite(PIN_LED_AC, LOW);
}

void setupMotionSensor(){
	lcd.clear();
	for(int i=0; i<20; i++) {
		lcd.setCursor(0, 0);
		(i%2 == 0) ? lcd.print("Initializing...") : lcd.print("Initializing.. ");
		delay(1000);
	}
}

void readSensors() {
	// PIR
	// lastPIR = digitalRead(PIN_PIR) == HIGH;
	liveData.occupancy = digitalRead(PIN_PIR) == HIGH;

	// LDR
	liveData.ambient_light = 4096 - analogRead(PIN_LDR);
	// int ldr_raw = 4096 - analogRead(PIN_LDR);
	// lastLDR = ldr_raw;

	// DHT11
	float t = dht.readTemperature();
	float h = dht.readHumidity();
	if (!isnan(t)) liveData.temperature = t;
	if (!isnan(h)) liveData.humidity = h;

	// PZEM (AC unit) reading via library
	float v = pzem.voltage();
	if (v > 0) liveData.voltage = v;
	float iac = pzem.current();
	if (iac >= 0) liveData.ac_current = iac;
	float pac = pzem.power();
	if (pac >= 0) liveData.ac_power = pac;
	float energy = pzem.energy();
	if (energy >= 0) liveData.ac_energy = energy;
	float frequency = pzem.frequency();
	if(frequency >= 0) liveData.frequency = frequency;
    float pf = pzem.pf();
	if(pf >= 0) liveData.power_factor = pf;

	// Accumulate AC energy for this second
	energy_accumulator_wh += (liveData.ac_power / 3600.0);
	// acEnergy = energy;
	
	// PZEM (Bulb) reading via library
	float ibulb = pzem_bulb.current();
	if (ibulb >= 0) liveData.bulb_current = ibulb;
	float pbulb = pzem_bulb.power();
	if (pbulb >= 0) liveData.bulb_power = pbulb;
	float ebulb = pzem_bulb.energy();
	if (ebulb >= 0) liveData.bulb_energy = ebulb;


	// Accumulate energy: power (W) * 1 second = W*s; convert to Wh => /3600
	energy_accumulator_wh += (liveData.bulb_power / 3600.0);
	// liveData.bulb_energy += (liveData.bulb_power / 3600.0);
	liveData.total_energy = energy_accumulator_wh;
	samples_this_minute++;
}

// Preferences persistence using LittleFS + ArduinoJson
void saveDeviceSettings() {
	// Device settings preferences
	JsonDocument doc;
	doc["ldr_threshold"] = deviceSettings.ldr_threshold;
	doc["ldr_hysteresis"] = deviceSettings.ldr_hysteresis;
	doc["temp_threshold"] = deviceSettings.temp_threshold;
	doc["temp_hysteresis"] = deviceSettings.temp_hysteresis;

	File f = LittleFS.open("/prefs.json", FILE_WRITE);
	if (!f) {
		Serial.println("Failed to open prefs file for writing");
		return;
	}
	serializeJson(doc, f);
	f.close();
	Serial.println("Device Settings saved");
}

void saveStateSettings(){
	//Device State preferences
	JsonDocument state;
	state["ac_on"] = deviceState.acOn;
	state["bulb_on"] = deviceState.bulbOn;
	state["auto_mode"] = deviceState.autoMode;
	state["predict_mode"] = deviceState.predictionMode;

	File s = LittleFS.open("/states.json", FILE_WRITE);
	if (!s) {
		Serial.println("Failed to open prefs file for writing");
		return;
	}
	serializeJson(state, s);
	s.close();
	Serial.println("States saved");
}

void saveWiFiCredentials(){
	JsonDocument wifiPref;
	wifiPref["ssid"] = String(wifiCredentials.ssid);
	wifiPref["pass"] = String(wifiCredentials.password);

	File j = LittleFS.open("/wifi.json", FILE_WRITE);
	if (!j) {
		Serial.println("Failed to open prefs file for writing");
		return;
	}

	size_t written = serializeJson(wifiPref, j);
	j.close();

	if (written == 0) {
		Serial.println("Failed to write WiFi prefs to LittleFS");
		return;
	}

	Serial.println("WiFi Settings Saved");
}

void loadPreferences() {
	//load from ./prefs.json
	bool prefsExist = LittleFS.exists("/prefs.json");

	if (prefsExist)
	{
		File f = LittleFS.open("/prefs.json", FILE_READ);
		if (!f)
		{
			Serial.println("Failed to open prefs file");
		}
		else
		{
			JsonDocument doc;
			DeserializationError err = deserializeJson(doc, f);
			f.close();
			if (err)
			{
				Serial.print("Failed to parse prefs: ");
				Serial.println(err.c_str());
			}
			else
			{
				Serial.println("Loaded preferences from prefs.json");
				deviceSettings.ldr_threshold = doc["ldr_threshold"];
				deviceSettings.ldr_hysteresis = doc["ldr_hysteresis"];
				deviceSettings.temp_threshold = doc["temp_threshold"];
				deviceSettings.temp_hysteresis = doc["temp_hysteresis"];
			}
		}
	}
	else
	{
		Serial.println("No prefs file, using defaults");
		deviceSettings.ldr_threshold = LDR_THRESHOLD_DEFAULT;
		deviceSettings.ldr_hysteresis = LDR_HYSTERESIS_DEFAULT;
		deviceSettings.temp_threshold = TEMP_THRESHOLD_DEFAULT;
		deviceSettings.temp_hysteresis = TEMP_HYSTERESIS_DEFAULT;
	}

	//load from ./states.json
	bool statesExist = LittleFS.exists("/states.json");

	if(statesExist){
	File s = LittleFS.open("/states.json", FILE_READ);
	if (!s) {
		Serial.println("Failed to open states file");
	} else {
	JsonDocument state;
	DeserializationError e = deserializeJson(state, s);
	s.close();
	if (e) {
		Serial.print("Failed to parse states: "); Serial.println(e.c_str());
		return;
	}
	Serial.println("Loaded states from states.json");
	deviceState.acOn = state["ac_on"];
	deviceState.bulbOn = state["bulb_on"];
	deviceState.autoMode = state["auto_mode"];
	deviceState.predictionMode = state["predict_mode"];
}} else {
	Serial.println("No states file, using defaults");
	deviceState.acOn = false;
	deviceState.bulbOn = false;
	deviceState.autoMode = false;
	deviceState.predictionMode = false;
}

	//load WiFi credentials from ./wifi.json
	bool wifiExist = LittleFS.exists("/wifi.json");	

	if(!wifiExist) {
		Serial.println("No WiFi creds file, using defaults");
		prefWifiName = String(DEFAULT_SSID);
		prefWifiPass = String(DEFAULT_PASSWORD);
	} else {
	File j = LittleFS.open("/wifi.json", FILE_READ);
	if (!j) {
		Serial.println("Failed to open WiFi creds file");
	} else {
	JsonDocument wifiPref;
	DeserializationError error = deserializeJson(wifiPref, j);
	j.close();
	if (error) {
		Serial.print("Failed to parse WiFi creds: "); Serial.println(error.c_str());
		return;
	}
	Serial.println("Loaded WiFi creds from wifi.json");
	strncpy(wifiCredentials.ssid, wifiPref["ssid"].as<const char*>() ?: "", sizeof(wifiCredentials.ssid));
	strncpy(wifiCredentials.password, wifiPref["pass"].as<const char*>() ?: "", sizeof(wifiCredentials.password));

	prefWifiName = String(wifiCredentials.ssid);
	prefWifiPass = String(wifiCredentials.password);
}}

	// Print loaded preference values for debugging
	Serial.println("Loaded preferences:");
	Serial.print("ldr_threshold: "); Serial.println(deviceSettings.ldr_threshold);
	Serial.print("ldr Hysteresis: "); Serial.println(deviceSettings.ldr_hysteresis);
	Serial.print("temp_threshold: "); Serial.println(deviceSettings.temp_threshold);
	Serial.print("temp hysteresis: "); Serial.println(deviceSettings.temp_hysteresis);
	Serial.print("WiFi SSID: "); Serial.println(wifiCredentials.ssid);
	Serial.print("WiFi Pass: "); Serial.println(wifiCredentials.password);
	Serial.print("AC On: "); Serial.println(deviceState.acOn? "true":"false");
	Serial.print("Bulb On: "); Serial.println(deviceState.bulbOn? "true":"false");
	Serial.print("Auto Mode: "); Serial.println(deviceState.autoMode? "true":"false");
	Serial.print("Prediction Mode: "); Serial.println(deviceState.predictionMode? "true":"false");

	// Apply loaded states to outputs
	digitalWrite(PIN_BULB_CTRL, deviceState.bulbOn ? HIGH : LOW);
	digitalWrite(PIN_LED_BULB, deviceState.bulbOn ? HIGH : LOW);
	digitalWrite(PIN_AC_CTRL, deviceState.acOn ? HIGH : LOW);
	digitalWrite(PIN_LED_AC, deviceState.acOn ? HIGH : LOW);

	Serial.println("Preferences loaded");
}


// Server callback class to monitor connection events
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
        deviceConnected = true;
        Serial.print("BLE Client Connected: ");
		Serial.println(connInfo.getAddress().toString().c_str());
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        Serial.printf("BLE Client Disconnected. Reason code: %d\n", reason);
        Serial.println("Restarting Advertising...");
        
        // Restart advertising so nRF Connect can find it again
        NimBLEDevice::getAdvertising()->start();
    }
};

//Callback handlers for writing Settings
class SettingsCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) {
        std::string value = pChar->getValue();
        if (value.length() == sizeof(SettingsData)) {
            memcpy(&deviceSettings, value.data(), sizeof(SettingsData));
            
            // Apply your threshold updates here
            Serial.println("--- New Settings Received ---");
            Serial.printf("LDR Threshold: %u\n", deviceSettings.ldr_threshold);
            Serial.printf("LDR Hysteresis: %u\n", deviceSettings.ldr_hysteresis);
            Serial.printf("Temp Threshold: %.2f\n", deviceSettings.temp_threshold);
            Serial.printf("Temp Hysteresis: %.2f\n", deviceSettings.temp_hysteresis);
		
			// save to preferences
			saveDeviceSettings();
		}
    }

	//create onRead callback to send current settings to client when it requests
	void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) {
		Serial.println("Settings read requested by client");
		pChar->setValue((uint8_t*)&deviceSettings, sizeof(SettingsData));
	}
};

// Callbacks for writing and receiving device states
class StatesCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) {
		Serial.println("--- New Device State Received ---");
        std::string value = pChar->getValue();
        if (value.length() == sizeof(DeviceState)) {
            memcpy(&deviceState, value.data(), sizeof(DeviceState));
            
            // Apply your threshold updates here
			Serial.printf("AC On: %s\n", deviceState.acOn ? "true" : "false");
			Serial.printf("Bulb On: %s\n", deviceState.bulbOn ? "true" : "false");
			Serial.printf("Auto Mode: %s\n", deviceState.autoMode ? "true" : "false");
			Serial.printf("Prediction Mode: %s\n", deviceState.predictionMode ? "true" : "false");

			// Apply the new states to the hardware
			digitalWrite(PIN_BULB_CTRL, deviceState.bulbOn ? HIGH : LOW);
			digitalWrite(PIN_AC_CTRL, deviceState.acOn ? HIGH : LOW);
			digitalWrite(PIN_LED_BULB, deviceState.bulbOn ? HIGH : LOW);
			digitalWrite(PIN_LED_AC, deviceState.acOn ? HIGH : LOW);
			// savePreferences();
			saveStateSettings();
        }
    }

	//create onRead callback to send current states to client when it requests
	void onRead(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) {
		Serial.println("State read requested by client");
		pChar->setValue((uint8_t*)&deviceState, sizeof(DeviceState));
	}
};

class WifiCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) {
        std::string value = pChar->getValue();
        Serial.printf("Received WiFi struct: %s\n", value.c_str());
		if (value.length() == sizeof(WifiCredentials)) {
			memcpy(&wifiCredentials, value.data(), sizeof(WifiCredentials));
			
			// Apply your threshold updates here
			Serial.printf("New WiFi SSID: %s\n", wifiCredentials.ssid);
			Serial.printf("New WiFi Password: %s\n", wifiCredentials.password);

			//load into prefWifiName
			prefWifiName = String(wifiCredentials.ssid);
			prefWifiPass = String(wifiCredentials.password);
			
			// Save to preferences
			saveWiFiCredentials();
		}
    }
};

void uploadStatesToSupabase() {
	if (WiFi.status() != WL_CONNECTED) return;

	JsonDocument stDoc;
	stDoc["bulb"] = deviceState.bulbOn;
	stDoc["ac"] = deviceState.acOn;
	stDoc["auto"] = deviceState.autoMode;
	stDoc["predict"] = deviceState.predictionMode;
	String stPayload;
	Serial.print("Sending state to Supabase: ");
	serializeJson(stDoc, stPayload);
	Serial.println(stPayload);
	int code = db.insert("states", stPayload, false);
	Serial.print("States insert code: "); Serial.println(code);
	db.urlQuery_reset();
}

// Handle bulb and AC button presses with simple debounce and toggle behavior.
void handleButtons() {
	// Read buttons
	int readingBulb = digitalRead(PIN_BTN_BULB);
	int readingAc = digitalRead(PIN_BTN_AC);
	int readingFn = digitalRead(PIN_BTN_FN);

	bool changed = false;

	// Debounce bulb
	if (readingBulb != lastBtnBulbState) {
		lastDebounceBulb = millis();
		lastBtnBulbState = readingBulb;
	} else if ((millis() - lastDebounceBulb) > DEBOUNCE_MS) {
		static int stableBulbState = HIGH;
		if (readingBulb != stableBulbState) {
			stableBulbState = readingBulb;
			if (stableBulbState == LOW) { // pressed
				deviceState.bulbOn = !deviceState.bulbOn;
				digitalWrite(PIN_BULB_CTRL, deviceState.bulbOn ? HIGH : LOW);
				digitalWrite(PIN_LED_BULB, deviceState.bulbOn ? HIGH : LOW);
				Serial.print("Bulb toggled: "); Serial.println(deviceState.bulbOn);
				changed = true;
			}
		}
	}

	// Debounce AC
	if (readingAc != lastBtnAcState) {
		lastDebounceAc = millis();
		lastBtnAcState = readingAc;
	} else if ((millis() - lastDebounceAc) > DEBOUNCE_MS) {
		static int stableAcState = HIGH;
		if (readingAc != stableAcState) {
			stableAcState = readingAc;
			if (stableAcState == LOW) { // pressed
				deviceState.acOn = !deviceState.acOn;
				digitalWrite(PIN_AC_CTRL, deviceState.acOn ? HIGH : LOW);
				digitalWrite(PIN_LED_AC, deviceState.acOn ? HIGH : LOW);
				Serial.print("AC toggled: "); Serial.println(deviceState.acOn);
				changed = true;
			}
		}
	}
	
	// Handle function button (e.g., toggle auto mode)
	if (readingFn == LOW) {
		deviceState.autoMode = !deviceState.autoMode;
		Serial.print("Auto mode toggled: ");
		Serial.println(deviceState.autoMode);
		changed = true;
		delay(300); // simple debounce for function button
		fnBtnPressed = true;
		digitalWrite(PIN_LED_BULB, LOW);
		digitalWrite(PIN_LED_AC, LOW);
		for (int i = 0; i < 3; i++)
		{
			digitalWrite(PIN_LED_BULB, HIGH);
			digitalWrite(PIN_LED_AC, HIGH);
			delay(300);
			digitalWrite(PIN_LED_BULB, LOW);
			digitalWrite(PIN_LED_AC, LOW);
		}
	}

	// Only send state when it changes to avoid blocking on every loop
	static bool lastSentBulb = false;
	static bool lastSentAc = false;
	if (changed && (deviceState.bulbOn != lastSentBulb || deviceState.acOn != lastSentAc || fnBtnPressed)) {
		lastSentBulb = deviceState.bulbOn;
		lastSentAc = deviceState.acOn;
		fnBtnPressed = false;

		uploadStatesToSupabase();
		// persist the changed state
		saveStateSettings();
	}
}

String printLocalTime(){
  struct tm timeinfo;
  char buffer[64];
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return String();
  }

  if(timeinfo.tm_mday != lastDay){
	Serial.print("New day detected: "); Serial.println(timeinfo.tm_mday);
	// Reset daily accumulators
	pzem_bulb.resetEnergy();
	pzem.resetEnergy();
	lastDay = timeinfo.tm_mday;
	preferences.putInt("lastDay", lastDay);
  }

  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S+00:01", &timeinfo);
//   Serial.print("Current time: ");
//   Serial.println(buffer);
  return String(buffer);
}

void uploadToSupabase(float energy_min_wh) {
	if (WiFi.status() != WL_CONNECTED) return;

	HTTPClient http;
	String url = String(SUPABASE_URL) + "/rest/v1/" + SUPABASE_TABLE;

	// Build JSON payload
	String payload = "{";
	payload += "\"timestamp\":\"" + printLocalTime() + "\",";
	payload += "\"voltage_ac\":" + String(liveData.voltage,2) + ",";
	payload += "\"current_ac\":" + String(liveData.ac_current,2) + ",";
	payload += "\"power_ac\":" + String(liveData.ac_power,2) + ",";
	payload += "\"energy_ac\":" + String(liveData.ac_energy,2) + ",";
	payload += "\"frequency_ac\":" + String(liveData.frequency,2) + ",";
	payload += "\"pf_ac\":" + String(liveData.power_factor,2) + ",";
	payload += "\"current_bulb\":" + String(liveData.bulb_current,2) + ",";
	payload += "\"power_bulb\":" + String(liveData.bulb_power,2) + ",";
	payload += "\"energy_min_wh\":" + String(liveData.total_energy,2) + ",";
	payload += "\"ldr\":" + String(liveData.ambient_light) + ",";
	payload += "\"pir\":" + String(liveData.occupancy ? "true" : "false") + ",";
	payload += "\"temperature\":" + String(liveData.temperature,2) + ",";
	payload += "\"humidity\":" + String(liveData.humidity,2) + ",";
	payload += "\"samples\":" + String(samples_this_minute) + ",";
	payload += "\"bulb_state\":" + String(deviceState.bulbOn ? "true" : "false") + ",";
	payload += "\"ac_state\":" + String(deviceState.acOn ? "true" : "false");
	payload += "}";

	Serial.print("Uploading to Supabase: "); Serial.println(payload);

	int code = db.insert(SUPABASE_TABLE, payload, false);
	Serial.print("Supabase insert code: "); Serial.println(code);

  	db.urlQuery_reset();
}


void writeLCD(char* label, float current, float power) {
	lcd.clear();
	int l = strlen(label);
	int xpos = (16 - l) / 2; // center label
	lcd.setCursor(xpos, 0);
	lcd.print(label);
	lcd.setCursor(0, 1);
	lcd.print(current, 2);
	lcd.print("A");
	lcd.setCursor(12, 1);
	lcd.print(power, 0);
	lcd.print("W");
}

// Overload for AC display with more parameters
void writeLCD(float voltage, float pf, float freq, float energy) {
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print(voltage,1); lcd.print("V ");
	lcd.setCursor(10,0);
	lcd.print(freq,1); lcd.print("Hz");
	lcd.setCursor(0,1);
	if(energy < 1000) {
		lcd.print(energy,1); lcd.print("Wh");
	} else {
		lcd.print(energy/1000.0,1); lcd.print("kWh");
	}
	lcd.setCursor(10,1);
	lcd.print("PF:");
	lcd.print(pf,1);
}

//overload for environment display
void writeLCD(float temp, float hum,  int ldr, bool pir) {
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("T:"); lcd.print(temp,1); lcd.print((char)223); lcd.print("C");
	lcd.setCursor(9,0);
	lcd.print("H:"); lcd.print(hum,0); lcd.print("%");
	lcd.setCursor(0,1);
	lcd.print("L:"); lcd.print(ldr);
	lcd.setCursor(9,1);
	lcd.print("PIR:"); lcd.print(pir ? "Yes" : "No");
}

void rotateDisplay() {
	unsigned long now = millis();
	if (now - lastDisplayRotate < 3000) return;

	lastDisplayRotate = now;
	displayIndex = (displayIndex + 1) % 5;

	lcd.clear();
	switch (displayIndex) {
		case 0:
			writeLCD((char*)"Bulb", liveData.bulb_current, liveData.bulb_power);
			break;
		case 1:
			writeLCD((char*)"Air Conditioner", liveData.ac_current, liveData.ac_power);
			break;
		case 2:
			writeLCD(liveData.voltage, liveData.power_factor, liveData.frequency, liveData.total_energy);
			break;
		case 3:
			writeLCD(liveData.temperature, liveData.humidity, liveData.ambient_light, liveData.occupancy);
			break;
		case 4:
			lcd.clear();
			lcd.setCursor(0,0);
			lcd.print("WiFi");
			lcd.setCursor(0,1);
			lcd.print(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
	}
}

bool lastBulbState = false;
bool lastAcState = false;
void switchLoad(bool isBulb, bool on) {
	if (isBulb) {
		deviceState.bulbOn = on;
		digitalWrite(PIN_BULB_CTRL, deviceState.bulbOn ? HIGH : LOW);
		digitalWrite(PIN_LED_BULB, deviceState.bulbOn ? HIGH : LOW);
	} else {
		deviceState.acOn = on;
		digitalWrite(PIN_AC_CTRL, deviceState.acOn ? HIGH : LOW);
		digitalWrite(PIN_LED_AC, deviceState.acOn ? HIGH : LOW);
	}
	

	if(lastBulbState != deviceState.bulbOn || lastAcState != deviceState.acOn) {
		lastBulbState = deviceState.bulbOn;
		lastAcState = deviceState.acOn;
		saveStateSettings();
		uploadStatesToSupabase();
	}
}

void bleSetup() {
	Serial.println("Starting BLE");

	NimBLEDevice::init("Smart_Energy_Hub");

	// NimBLEDDevice::setMTU(247);

	NimBLEServer *pServer = NimBLEDevice::createServer();

	pServer->setCallbacks(new MyServerCallbacks());

	NimBLEService *pService = pServer->createService(SERVICE_UUID);

	pTelemetryChar = pService->createCharacteristic(
		TELEMETRY_CHAR_UUID, 
		NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
	);

	pSettingsChar = pService->createCharacteristic(
		SETTING_CHAR_UUID, 
		NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
	);

	pSettingsChar->setCallbacks(new SettingsCallbacks());

	pWifiChar = pService->createCharacteristic(
		WIFI_CHAR_UUID, 
		NIMBLE_PROPERTY::WRITE
	);

	pWifiChar->setCallbacks(new WifiCallbacks());

	pStateChar = pService->createCharacteristic(
		STATE_CHAR_UUID, 
		NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
	);
	pStateChar->setCallbacks(new StatesCallbacks());



	pServer->start();

	NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName("EneryMonitorESP32");
	pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->enableScanResponse(true);
    pAdvertising->start();

	Serial.println("NimBLE Server is successfully advertising.");
}

void updateBLETelemetry() {
	pTelemetryChar->setValue((uint8_t*)&liveData, sizeof(liveData));
	pTelemetryChar->notify();
}

void handleSupabaseChanges(String result){
	Serial.println("Realtime change received: ");
			//supabase realtime
				// Parse the payload to extract the new state values
				JsonDocument doc;
				DeserializationError err = deserializeJson(doc, result);
				if (err) {
					Serial.print("Failed to parse realtime payload: ");
					Serial.println(err.c_str());
					return;
				}

				// Serial.print("Realtime payload: ");
				// serializeJson(doc, Serial);
				// Serial.println();

				bool newBulbState = doc["record"]["bulb"];
				bool newAcState = doc["record"]["ac"];
				bool newAutoMode = doc["record"]["auto"];
				bool newPredictMode = doc["record"]["predict"];

				// Update device state if it has changed
				if (deviceState.bulbOn != newBulbState) {
					deviceState.bulbOn = newBulbState;
					digitalWrite(PIN_BULB_CTRL, deviceState.bulbOn ? HIGH : LOW);
					digitalWrite(PIN_LED_BULB, deviceState.bulbOn ? HIGH : LOW);
					Serial.print("Bulb state updated from Realtime: ");
					Serial.println(deviceState.bulbOn);
				}
				if (deviceState.acOn != newAcState) {
					deviceState.acOn = newAcState;
					digitalWrite(PIN_AC_CTRL, deviceState.acOn ? HIGH : LOW);
					digitalWrite(PIN_LED_AC, deviceState.acOn ? HIGH : LOW);
					Serial.print("AC state updated from Realtime: ");
					Serial.println(deviceState.acOn);
				}
				if (deviceState.autoMode != newAutoMode) {
					deviceState.autoMode = newAutoMode;
					Serial.print("Auto mode updated from Realtime: ");
					Serial.println(deviceState.autoMode);
				}
				if (deviceState.predictionMode != newPredictMode) {
					deviceState.predictionMode = newPredictMode;
					Serial.print("Prediction mode updated from Realtime: ");
					Serial.println(deviceState.predictionMode);
				}
	}

void setup() {
	Serial.begin(115200);

	setupHardware();

	bleSetup();

	// Mount filesystem and load preferences
	if(!LittleFS.begin(true)) {
  		Serial.println("LittleFS Mount Failed, formatting...");
  		LittleFS.format();
  		if(!LittleFS.begin()) {
    		Serial.println("Failed to initialize LittleFS");
    		return;
  		}
	}
	Serial.println("LittleFS mounted successfully");

	//Factory reset if button is held for 5 seconds
	if(digitalRead(PIN_BTN_FN) == LOW) {
		delay(3000);
		if(digitalRead(PIN_BTN_FN) == LOW) {
			Serial.println("Factory reset initiated...");
			lcd.clear();
			lcd.print("Factory Reset");
			LittleFS.remove("/prefs.json");
			LittleFS.remove("/states.json");
			LittleFS.remove("/wifi.json");
			delay(1000);
			preferences.begin("hour-prefs", false);
			preferences.clear();
			Serial.println("LittleFS formatted.");
			delay(5000);
			lcd.clear();
			lcd.print("Restarting...");
			delay(2000);
			lcd.setBacklight(0);
			delay(3000);
			ESP.restart();
		}
	}

	setupMotionSensor();

    loadPreferences();

	// bleSetup();

	bool s = setupWiFi();

	if(!s) {
		lcd.clear();
		lcd.setCursor(0,0);
		lcd.print("Retrying...");
		delay(2000);
		setupWiFi();
	}

	preferences.begin("hour-prefs", false);
	delay(100);	
	lastDay =preferences.getInt("lastDay", 0);
	Serial.print("Last day loaded from preferences: "); Serial.println(lastDay);
	energy_accumulator_wh = preferences.getFloat("energy", 0.0);
	Serial.print("Energy accumulator loaded from preferences: "); Serial.println(energy_accumulator_wh);

	if(s) {
		db.begin(SUPABASE_URL, SUPABASE_KEY);
		configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
		printLocalTime();

		realtime.begin(SUPABASE_URL, SUPABASE_KEY, handleSupabaseChanges);
		realtime.addChangesListener("states", "INSERT", "public", "");
		realtime.listen();
	}

	lastSecond = millis();
	lastMinute = millis();
}

void loop() {
	unsigned long now = millis();

	// Check buttons every loop to keep UI responsive
	handleButtons();

	realtime.loop();

	// Every second: read sensors
	if (now - lastSecond >= 1000) {
		lastSecond += 1000;
		readSensors();
		rotateDisplay();
		
	}

	// Every minute: aggregate and upload (or store when offline)
	if (now - lastMinute >= 60000) {
		reconnectWiFi();

		printLocalTime();

		lastMinute += 60000;

		// Compute total energy used in the last minute (placeholder)
		float energy_min_wh = liveData.total_energy; // placeholder
        if(deviceConnected){
			updateBLETelemetry();
		} else if (WiFi.status() == WL_CONNECTED) {
			uploadToSupabase(energy_min_wh);
		} 

		// Reset accumulators for next minute
		preferences.putFloat("energy", energy_accumulator_wh);
		samples_this_minute = 0;
	}

	// Auto control logic (example: turn on bulb if dark and motion detected)
	if (deviceState.autoMode) {
		// Example logic: if it's dark and we detect motion, turn on the bulb
		if(liveData.occupancy){
			if(!deviceState.bulbOn){
				if(liveData.ambient_light < (deviceSettings.ldr_threshold - deviceSettings.ldr_hysteresis) && (millis() - lastBulbStateChange > bulbSwitchDelay*1000)) { // only trigger if it's dark and we haven't toggled the bulb in the last 2 minutes
					switchLoad(true, true);
					lastBulbStateChange = millis();						
			 } 
			}
			// Example logic: if temperature is above threshold, turn on AC
			if(!deviceState.acOn){
				if(liveData.temperature > (deviceSettings.temp_threshold + deviceSettings.temp_hysteresis) && (millis() - lastACStateChange > acSwitchDelay*1000)) { // only trigger if it's hot and we haven't toggled the AC in the last 3 minutes
					switchLoad(false, true);
					lastACStateChange = millis();
				} 
			} else {
				if(liveData.temperature < (deviceSettings.temp_threshold - deviceSettings.temp_hysteresis) && (millis() - lastACStateChange > acSwitchDelay * 1000)) { // only turn off if it's cooled down and we haven't toggled the AC in the last 3 minutes
					switchLoad(false, false);
					lastACStateChange = millis();
				}
			}
		} else {
			if (deviceState.bulbOn && millis() - lastBulbStateChange > bulbSwitchDelay*1000) {
				lastBulbStateChange = millis();
				switchLoad(true, false);
			}

			if (deviceState.acOn && millis() - lastACStateChange > acSwitchDelay*1000) {
				lastACStateChange = millis();
				switchLoad(false, false);
			}
		}
	}
}
