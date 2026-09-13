
void ReceiveAGIO()
{
	if (!EthChipFound) return;

	int len = UDP_AGIO.parsePacket();
	if (len <= 0) return;

	uint8_t Data[MaxReadBuffer];
	len = UDP_AGIO.read(Data, MaxReadBuffer);

	// 128, 129, source 127 (AgIO), pgn, length, data..., crc
	if (len < 6 || Data[0] != 128 || Data[1] != 129 || Data[2] != 127) return;
	if (len < Data[4] + 6) return;

	if (Data[3] != 239)
	{
		Serial.print("PGN ");
		Serial.print(Data[3]);
		Serial.print(" from ");
		Serial.println(UDP_AGIO.remoteIP());
	}

	switch (Data[3])
	{
	case 200:
	{
		// hello from AgIO
		uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, relayLo, relayHi, 0, 0, 0, 0 };
		AddCRC(helloFromMachine, sizeof(helloFromMachine));
		SendUDP(IPAddress(MDL.IP0, MDL.IP1, MDL.IP2, 255), helloFromMachine, sizeof(helloFromMachine));
		break;
	}

	case 201:
		// subnet change
		if ((Data[4] == 5) && (Data[5] == 201) && (Data[6] == 201))
		{
			MDL.IP0 = Data[7];
			MDL.IP1 = Data[8];
			MDL.IP2 = Data[9];

			SaveData();
			AllRelaysOff();
			delay(100);
			ESP.restart();
		}
		break;

	case 202:
	{
		//make really sure this is the subnet scan pgn
		if (Data[4] == 3 && Data[5] == 202 && Data[6] == 202)
		{
			IPAddress rem = UDP_AGIO.remoteIP();
			uint8_t scanReply[] = { 128, 129, 123, 203, 7,
				MDL.IP0, MDL.IP1, MDL.IP2, MDL.IP3,
				rem[0], rem[1], rem[2], 0 };
			AddCRC(scanReply, sizeof(scanReply));
			SendUDP(IPAddress(255, 255, 255, 255), scanReply, sizeof(scanReply));
		}
		break;
	}

	case 236:
		// machine pin config
		for (uint8_t i = 0; i < 24; i++)
		{
			pin[i] = Data[i + 5];
		}
		SaveData();
		break;

	case 238:
	{
		// machine config
		aogConfig.raiseTime = Data[5];
		aogConfig.lowerTime = Data[6];
		aogConfig.enableToolLift = Data[7];

		//set1
		uint8_t sett = Data[8];  //setting0
		aogConfig.isRelayActiveHigh = bitRead(sett, 0);

		aogConfig.user1 = Data[9];
		aogConfig.user2 = Data[10];
		aogConfig.user3 = Data[11];
		aogConfig.user4 = Data[12];

		SaveData();
		break;
	}

	case 239:
		// machine data
		uTurn = Data[5];
		gpsSpeed = (float)Data[6];

		hydLift = Data[7];
		tramline = Data[8];  //bit 0 is right bit 1 is left
		geoStop = Data[9];

		relayLo = Data[11];          // read relay control from AgOpenGPS
		relayHi = Data[12];

		if (aogConfig.isRelayActiveHigh)
		{
			tramline = 255 - tramline;
			relayLo = 255 - relayLo;
			relayHi = 255 - relayHi;
		}

		//reset watchdog
		watchdogTimer = 0;
		testMode = false;
		break;
	}
}

void SendUDP(IPAddress Destination, uint8_t* Data, uint8_t Length)
{
	if (!EthLinkUp) return;
	bool ok = UDP_AGIO.beginPacket(Destination, DestinationPortAGIO);
	UDP_AGIO.write(Data, Length);
	ok = UDP_AGIO.endPacket() && ok;
	if (!ok)
	{
		Serial.print("UDP send to ");
		Serial.print(Destination);
		Serial.println(" failed");
	}
}

// last byte = sum of bytes 2 .. Length-2
void AddCRC(uint8_t* Data, uint8_t Length)
{
	uint8_t CK = 0;
	for (uint8_t i = 2; i < Length - 1; i++) CK += Data[i];
	Data[Length - 1] = CK;
}
