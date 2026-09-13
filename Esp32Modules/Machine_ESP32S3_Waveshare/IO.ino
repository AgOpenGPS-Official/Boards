
bool TCA9554Write(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(TCA9554_ADDR);
	Wire.write(reg);
	Wire.write(value);
	return Wire.endTransmission() == 0;
}

void RelaysSetup()
{
	Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);

	// write "all off" into the output register before switching the pins to outputs, so nothing glitches on
	relayOutputs = 0;
	RelayChipFound = TCA9554Write(TCA9554_REG_OUTPUT, relayOutputs) && TCA9554Write(TCA9554_REG_CONFIG, 0x00);
	LastRelayWrite = millis();
	Serial.println(RelayChipFound ? "TCA9554 relay expander found." : "TCA9554 relay expander NOT found!");

	for (uint8_t i = 0; i < 8; i++) pinMode(InputPins[i], INPUT_PULLUP);
}

void CheckRelays()
{
	//Load the current pgn relay state - Sections
	for (uint8_t i = 0; i < 8; i++)
	{
		relayState[i] = bitRead(relayLo, i);
		relayState[i + 8] = bitRead(relayHi, i);
	}

	// Hydraulics
	relayState[16] = isLower;
	relayState[17] = isRaise;

	//Tram
	relayState[18] = bitRead(tramline, 0); //right
	relayState[19] = bitRead(tramline, 1); //left

	//GeoStop
	relayState[20] = (geoStop == 0) ? 0 : 1;

	uint8_t outputs = 0;
	if (testMode)
	{
		outputs = testOutputs;
	}
	else
	{
		for (uint8_t i = 0; i < 8; i++)
		{
			if (pin[i] && pin[i] <= sizeof(relayState) && relayState[pin[i] - 1]) outputs |= bit(i);
		}
	}

	// rewrite periodically so the relays recover if the expander resets (brown-out, I2C glitch)
	if (outputs != relayOutputs || millis() - LastRelayWrite > 1000)
	{
		LastRelayWrite = millis();
		RelayChipFound = TCA9554Write(TCA9554_REG_OUTPUT, outputs) && TCA9554Write(TCA9554_REG_CONFIG, 0x00);
		if (RelayChipFound) relayOutputs = outputs;
	}
}

void AllRelaysOff()
{
	testMode = false;
	testOutputs = 0;
	relayOutputs = 0;
	TCA9554Write(TCA9554_REG_OUTPUT, 0);
}

void ReadInputs()
{
	uint8_t states = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (digitalRead(InputPins[i]) == LOW) states |= bit(i);
	}
	inputStates = states;
}

void UpdateLED()
{
	static uint32_t LastUpdate;
	static bool BlinkOn;
	if (millis() - LastUpdate < 250) return;
	LastUpdate = millis();
	BlinkOn = !BlinkOn;

	uint8_t r = 0, g = 0, b = 0;
	if (!RelayChipFound || !EthChipFound) r = BlinkOn ? 40 : 0;	// hardware fault: blinking red
	else if (!EthLinkUp) r = 40;									// no ethernet link: red
	else if (testMode) { r = 25; b = 25; }							// web relay test: purple
	else if (watchdogTimer > WatchdogTimeout) b = 40;				// link up, no data from AgOpenGPS: blue
	else g = 40;													// receiving machine data: green
	neopixelWrite(RGB_LED_PIN, r, g, b);
}
