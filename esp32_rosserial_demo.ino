
#include <ros.h>
#include <std_msgs/String.h>
#include <WiFi.h>
#include <Husarnet.h>
#include <WiFiMulti.h>
#define PUB_FREQ 4 // frequency of publishing data

uint16_t port = 11411; //this must be set the same as tcp_port in launch file (esp_connect.launch)

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;

// Husarnet credentials
const char* hostNameESP = "****"; //this will be the name of the 1st ESP32 device at https://app.husarnet.com
const char* hostNameComputer = "*****"; //this will be the name of the host/rosmaster device at https://app.husarnet.com
/* to get your join code go to https://app.husarnet.com
  -> select network
  -> click "Add element"
  -> select "join code" tab
  Keep it secret!
*/

const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/******";
// WiFi credentials
#define NUM_NETWORKS 2 //number of Wi-Fi network credentials saved
const char* ssidTab[NUM_NETWORKS] = {
  "****",
  "*****",
};
const char* passwordTab[NUM_NETWORKS] = {
  "******",
  "******",
};

WiFiMulti wifiMulti;
HusarnetClient client;

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      // do your initialization here. this probably includes TCP server/client setup
      Serial.printf("WiFiHardware: init, hostname = %s, port = %d\r\n", hostNameComputer, port);
      while (! client.connect(hostNameComputer, port)) {
        Serial.printf("Waiting for connection\r\n");
        delay(500);
      }
    }
    // read a byte from the serial port. -1 = failure
    int read() {
      // implement this method so that it reads a byte from the TCP connection and returns it
      // you may return -1 is there is an error; for example if the TCP connection is not open
      return client.read(); //will return -1 when it will works
    }
    // write data to the connection to ROS
    void write(uint8_t* data, int length) {
      // implement this so that it takes the arguments and writes or prints them to the TCP connection
      for (int i = 0; i < length; i++) {
        client.write(data[i]);
      }
    }
    // returns milliseconds since start of program
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

ros::NodeHandle_<WiFiHardware> nh;
std_msgs::String string_msg;
ros::Publisher esp_publisher("esp_husarnet", &string_msg);
char buffer[50];
int counter = 0;

void taskWifi( void * parameter );
void setup() {
  for (int i = 0; i < NUM_NETWORKS; i++) {
    String ssid = ssidTab[i];
    String pass = passwordTab[i];
    wifiMulti.addAP(ssid.c_str(), pass.c_str());
    Serial.printf("WiFi %d: SSID: \"%s\" ; PASS: \"%s\"\r\n", i, ssid.c_str(), pass.c_str());
  }
  xTaskCreate(
    taskWifi, /* Task function. */
    "taskWifi", /* String with name of task. */
    10000, /* Stack size in bytes. */
    NULL, /* Parameter passed as input of the task */
    1, /* Priority of the task. */
    NULL); /* Task handle. */
  Serial.begin(115200);
  nh.initNode();
  nh.advertise(esp_publisher);
}

void loop() {
  now = millis();
  if ((now - lastTrigger) > (1000 / PUB_FREQ)) {
    if (client.connected()) {
      Serial.println("Publishing Data");
      sprintf(buffer, "Hello ROS %d", counter++);
      string_msg.data = buffer;
      esp_publisher.publish( &string_msg );
      lastTrigger = millis();
      nh.spinOnce();
    }
    else {
      while (! client.connect(hostNameComputer, port)) {
        Serial.printf("Waiting for connection\r\n");
        delay(500);
      }
    }
  }
}

void taskWifi( void * parameter ) {
  while (1) {
    uint8_t stat = wifiMulti.run();
    if (stat == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Husarnet.join(husarnetJoinCode, hostNameESP);
      Husarnet.start();
      while (WiFi.status() == WL_CONNECTED) {
        delay(500);
      }
    } else {
      Serial.printf("WiFi error: %d\r\n", (int)stat);
      delay(500);
    }
  }
}
