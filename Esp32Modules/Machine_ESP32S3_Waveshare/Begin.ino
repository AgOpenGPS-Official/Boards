
void setup()
{
	Serial.begin(115200);
	delay(500);
	Serial.println("");
	Serial.println(InoDescription);

	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);
	neopixelWrite(RGB_LED_PIN, 20, 20, 0);

	// relays first so the outputs are in a known (off) state as early as possible
	RelaysSetup();

	EEPROM.begin(EEPROM_SIZE);
	LoadData();

	NetworkSetup();
	WebSetup();

	Serial.println("");
	Serial.println("Finished setup.");
	Serial.println("");
}

void LoadData()
{
	uint16_t StoredID;
	uint8_t StoredType;
	EEPROM.get(0, StoredID);
	EEPROM.get(2, StoredType);
	if (StoredID == InoID && StoredType == InoType)
	{
		// load stored data
		Serial.println("Loading stored settings.");
		EEPROM.get(4, MDL);
		EEPROM.get(8, aogConfig);
		EEPROM.get(20, pin);
	}
	else
	{
		Serial.println("Loading default settings.");
		SaveData();
	}
}

void SaveData()
{
	// update stored data
	Serial.println("Updating stored settings.");
	EEPROM.put(0, InoID);
	EEPROM.put(2, InoType);
	EEPROM.put(4, MDL);
	EEPROM.put(8, aogConfig);
	EEPROM.put(20, pin);
	EEPROM.commit();
}
