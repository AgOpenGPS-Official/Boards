
void WebSetup()
{
	server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PageRoot); });
	server.on("/status", HTTP_GET, HandleStatus);
	server.on("/test", HTTP_GET, HandleTest);
	server.on("/network", HTTP_POST, HandleNetwork);
	server.on("/update", HTTP_POST, HandleUpdateDone, HandleUpdateUpload);
	server.onNotFound([]() { server.send_P(200, "text/html", PageRoot); });
	server.begin();
	Serial.println("Web server started.");
}

void HandleStatus()
{
	char pins[40];
	snprintf(pins, sizeof(pins), "%u,%u,%u,%u,%u,%u,%u,%u", pin[0], pin[1], pin[2], pin[3], pin[4], pin[5], pin[6], pin[7]);

	char json[400];
	snprintf(json, sizeof(json),
		"{\"fw\":\"%s\",\"ip\":\"%s\",\"link\":%s,\"aog\":%s,\"relayChip\":%s,\"uptime\":%lu,"
		"\"out\":%u,\"in\":%u,\"test\":%s,\"pins\":[%s],\"net\":[%u,%u,%u,%u]}",
		InoDescription, ETH.localIP().toString().c_str(), EthLinkUp ? "true" : "false",
		watchdogTimer <= WatchdogTimeout ? "true" : "false", RelayChipFound ? "true" : "false", millis() / 1000,
		relayOutputs, inputStates, testMode ? "true" : "false", pins, MDL.IP0, MDL.IP1, MDL.IP2, MDL.IP3);
	server.send(200, "application/json", json);
}

void HandleTest()
{
	if (server.hasArg("on"))
	{
		testMode = server.arg("on") == "1" && watchdogTimer > WatchdogTimeout;
		testOutputs = 0;
	}
	if (server.hasArg("ch") && testMode)
	{
		int ch = server.arg("ch").toInt();
		if (ch >= 0 && ch < 8) testOutputs ^= bit(ch);
	}
	server.send(200, "text/plain", "ok");
}

void HandleNetwork()
{
	int ip[3];
	const char* names[] = { "ip0", "ip1", "ip2" };
	for (uint8_t i = 0; i < 3; i++)
	{
		ip[i] = server.arg(names[i]).toInt();
		if (!server.hasArg(names[i]) || ip[i] < 0 || ip[i] > 255)
		{
			server.send(400, "text/plain", "Invalid IP address");
			return;
		}
	}
	MDL.IP0 = ip[0];
	MDL.IP1 = ip[1];
	MDL.IP2 = ip[2];
	SaveData();

	String msg = "Saved. Rebooting, the module will be at http://" + String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(MDL.IP3);
	server.send(200, "text/plain", msg);
	AllRelaysOff();
	delay(500);
	ESP.restart();
}

void HandleUpdateUpload()
{
	HTTPUpload& upload = server.upload();
	if (upload.status == UPLOAD_FILE_START)
	{
		// the main loop is not running during the upload, so switch outputs off first
		AllRelaysOff();
		Serial.printf("Firmware update: %s\n", upload.filename.c_str());
		if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
	}
	else if (upload.status == UPLOAD_FILE_WRITE)
	{
		if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
	}
	else if (upload.status == UPLOAD_FILE_END)
	{
		if (Update.end(true)) Serial.printf("Firmware update done: %u bytes\n", upload.totalSize);
		else Update.printError(Serial);
	}
	else if (upload.status == UPLOAD_FILE_ABORTED)
	{
		Update.abort();
		Serial.println("Firmware update aborted.");
	}
}

void HandleUpdateDone()
{
	bool ok = !Update.hasError() && Update.isFinished();
	server.sendHeader("Connection", "close");
	server.send(200, "text/plain", ok ? "Update successful, rebooting." : "Update FAILED, see serial output.");
	if (ok)
	{
		delay(500);
		ESP.restart();
	}
}
