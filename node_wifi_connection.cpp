#include <WiFi.h>




char* ssid = "NIWC-Guest";

char* password = "Leyte Gulf";

 

void setup() {

 

  Serial.begin(115200);

  WiFi.begin(ssid, password);




  while (WiFi.status() != WL_CONNECTED) {

    delay(1000);

    Serial.println("Establishing connection to WiFi..");

  }

 

  Serial.println("Connected to network");

}

 

void loop() {}