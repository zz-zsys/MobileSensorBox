#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "otaupdate.h"
#include "led.h"

TaskHandle_t xTaskOta;

WebServer server(80);

const char* host = "msbml";
const int led = LED_BUILTIN; // ESP32 Pin to which onboard LED is connected
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 1000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED
RTC_DATA_ATTR bool isOtaActivated = false;					// time set flag

const char* otaupdate = R"rawText(
<script src=https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js></script>
<div class=header>
</div>

<div class=container>
	<section id=content>
		<form action=# enctype=multipart/form-data id=upload_form method=POST>
			<h1>Mobile Sensor Box FOTA Upload</h1>
			<input type=file id=file name=update onchange=sub(this) style=display:none>
			<label for=file id=file-input>Choose file...</label>
			<input type=submit class=btn value=Update><br><br>
			<div id=prg></div><br>
			<div id=prgbar>
				<div id=bar></div>
			</div><br>
		</form>
   </section>
</div>
<script>
	function sub(t) {
		var e = t.value.split("\\");
		document.getElementById("file-input").innerHTML = "   " + e[e.length - 1]
	}
	$("form").submit(function(t) {
		t.preventDefault();
		var e = $("#upload_form")[0],
			n = new FormData(e);
		$.ajax({
			url: "/ota/update",
			type: "POST",
			data: n,
			contentType: !1,
			processData: !1,
			xhr: function() {
				var t = new window.XMLHttpRequest;
				return t.upload.addEventListener("progress", function(t) {
					if (t.lengthComputable) {
						var e = t.loaded / t.total;
						$("#prg").html("progress: " + Math.round(100 * e) + "%"), $("#bar").css("width", Math.round(100 * e) + "%")
					}
				}, !1), t
			},
			success: function(t, e) {
				console.log("success!")
			},
			error: function(t, e, n) {}
		})
	})
</script>
<style>

input {
	width: 100%;
	height: 44px;
	border-radius: 4px;
	margin: 10px auto;
	font-size: 15px
}

input {
	background: #f1f1f1;
	border: 0;
	padding: 0 15px
}

body {
	background: #e80202;
	font-family: sans-serif;
	font-size: 14px;
	color: #777
}

#file-input {
	padding: 0;
	border: 1px solid #ddd;
	line-height: 44px;
	text-align: left;
	display: block;
	cursor: pointer
}

#prgbar{
	background-color: #f1f1f1;
	border-radius: 10px
}

#bar {
	background-color: #e80202;
	width: 0%;
	height: 10px
}

form {
	background: #fff;
	/* max-width: 258px; */
	width:30%;
	margin: 75px auto;
	padding: 30px;
	border-radius: 5px;
	text-align: center
}

.btn {
	background: #e80202;
	color: #fff;
	cursor: pointer
}

.header {
    display: block;
	text-align: center;
    height: 15%;
}

.center {
	background-color: #fff;
	display: inline-block;
	height: 100%;
	border-radius: 10px;
	margin:0 10px;
    vertical-align: middle;
}
</style>
)rawText";

void ota_initWifi() {
	//Init WiFi as Station, start SmartConfig
	WiFi.mode(WIFI_AP_STA);
	WiFi.beginSmartConfig();

	//Wait for SmartConfig packet from mobile
	printf("Waiting for SmartConfig...\n");
	while (!WiFi.smartConfigDone()) {
		vTaskDelay(500 / portTICK_RATE_MS);
		printf(".");
	}

	printf("\n");
	printf("SmartConfig received.\n");

	//Wait for WiFi to connect to AP
	printf("Waiting for WiFi...");
	while (WiFi.status() != WL_CONNECTED) {
		vTaskDelay(500 / portTICK_RATE_MS);
		printf(".");
	}

	printf("WiFi Connected!\n");
	printf("IP Address: %s\n", WiFi.localIP().toString().c_str());

	/*use mdns for host name resolution*/
	while (!MDNS.begin(host)) { // http://hostname.local
		printf("Error setting up MDNS responder!\n");
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

	MDNS.setInstanceName("msbml");
	MDNS.addService("_http", "_tcp", 80);  // **Without _ the mDNS was not working**

	printf("mDNS responder started\n");
}

void ota_initServer() {
	server.on("/ota", HTTP_GET, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/html", otaupdate);
	});
	/*handling uploading firmware file */
	server.on("/ota/update", HTTP_POST, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
	}, []() {
		HTTPUpload& upload = server.upload();
		if (upload.status == UPLOAD_FILE_START) {
			printf("Update: %s\n", upload.filename.c_str());
			if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
				Update.printError(Serial);
			}
		}
		else if (upload.status == UPLOAD_FILE_WRITE) {
			/* flashing firmware to ESP*/
			if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
				Update.printError(Serial);
			}
		}
		else if (upload.status == UPLOAD_FILE_END) {
			if (Update.end(true)) { //true to set the size to the current progress
				printf("Update Success: %u\nRebooting...\n", upload.totalSize);
			}
			else {
				Update.printError(Serial);
			}
		}
	});
	server.begin();

}

void otaTask(void * parameter) {
	isOtaActivated = true;
	full_batt_led_init();
	low_batt_led_init();
	batt_full_led_state(LED_ON);
	batt_low_led_state(LED_ON);
	ota_initWifi();
	ota_initServer();

	for (;;) {
		server.handleClient();
		delay(1);

		batt_full_led_state(LED_ON);
		batt_low_led_state(LED_ON);

		////loop to blink without delay
		//unsigned long currentMillis = millis();
		//if (currentMillis - previousMillis >= interval) {

		//	batt_full_led_state(LED_ON);
		//	batt_low_led_state(LED_ON);

		//	// save the last time you blinked the LED
		//	previousMillis = currentMillis;
		//}
	}
}

void ota_taskInit(uint32_t ulStackDepth, UBaseType_t uxPriority, const BaseType_t xCoreID) {
	xTaskCreatePinnedToCore(otaTask, "otaTask", ulStackDepth, (void*)1, uxPriority, &xTaskOta, xCoreID);

}