
void NetworkSetup()
{
	WiFi.onEvent(EthEvent);

	Serial.println("Starting W5500 Ethernet ...");
	EthChipFound = ETH.beginSPI(ETH_MISO_PIN, ETH_MOSI_PIN, ETH_SCLK_PIN, ETH_CS_PIN, ETH_RST_PIN, ETH_INT_PIN,
		SPI3_HOST, ETH_PHY_ADDR, ETH_SPI_MHZ);
	if (!EthChipFound)
	{
		Serial.println("W5500 start failed!");
		return;
	}

	IPAddress LocalIP(MDL.IP0, MDL.IP1, MDL.IP2, MDL.IP3);
	IPAddress Gateway(MDL.IP0, MDL.IP1, MDL.IP2, 1);
	IPAddress Mask(255, 255, 255, 0);
	ETH.config(LocalIP, Gateway, Mask);

	Serial.print("Static IP: ");
	Serial.println(LocalIP);

	UDP_AGIO.begin(ListeningPortAGIO);
}

void EthEvent(WiFiEvent_t event)
{
	switch (event)
	{
	case ARDUINO_EVENT_ETH_START:
		Serial.println("ETH Started");
		ETH.setHostname("aog-machine");
		break;
	case ARDUINO_EVENT_ETH_CONNECTED:
		Serial.println("ETH Connected");
		EthLinkUp = true;
		break;
	case ARDUINO_EVENT_ETH_GOT_IP:
		Serial.print("ETH MAC: ");
		Serial.print(ETH.macAddress());
		Serial.print(", IPv4: ");
		Serial.print(ETH.localIP());
		Serial.print(", ");
		Serial.print(ETH.linkSpeed());
		Serial.println("Mbps");
		EthLinkUp = true;
		break;
	case ARDUINO_EVENT_ETH_DISCONNECTED:
		Serial.println("ETH Disconnected");
		EthLinkUp = false;
		break;
	case ARDUINO_EVENT_ETH_STOP:
		Serial.println("ETH Stopped");
		EthLinkUp = false;
		break;
	default:
		break;
	}
}
