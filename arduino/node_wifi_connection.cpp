#include <WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

/*
  A ros-to-esp32 project created by
  Isaac Verbrugge isaacverbruge@gmail.com,
  Kayla Uyema kayla.uyema@gmail.com

  Install: WiFi, esp32 arduino espressif
  Install via arduino libraries: esp32 espressif boards
  Install via arduino libraries: rosserial

*/

const char* ssid = "RANGER";

const char* password = "militaryfps";

ros::NodeHandle nh;

IPAddress server(192,168,0,137);



void messageCb(const geometry_msgs::Twist& msg) {
  float linear_x = msg.linear.x;
  float linear_y = msg.linear.y;
  float linear_z = msg.linear.z;
  float angular_x = msg.angular.x;
  float angular_y = msg.angular.y;
  float angular_z = msg.angular.z;
  Serial.print("linear_x = ");
  Serial.println(linear_x);
  Serial.print("linear_y = ");
  Serial.println(linear_y);
  Serial.print("linear_z = ");
  Serial.println(linear_z);
  Serial.print("angular_x = ");
  Serial.println(angular_x);
  Serial.print("angular_y = ");
  Serial.println(angular_y);
  Serial.print("angular_z = ");
  Serial.println(angular_z);
}



ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);
geometry_msgs::Twist twist_msg;
ros::Publisher cmd_vel_pub("cmd_vel", &twist_msg);



void setup() {

  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {

    delay(1000);

    Serial.println("Establishing connection to WiFi..");

  }

  Serial.println("Connected to network");

  nh.getHardware()->setConnection(server, 11411);  // Set your ROS master IP and port
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(cmd_vel_pub);

    Serial.println("Setup complete");

}

 

void loop() {
  if (nh.connected()) {
    // Set the values for the twist message
    twist_msg.linear.x = 5.0;  // Example linear velocity
    twist_msg.linear.y = 0;  // Example linear velocity
    twist_msg.linear.z = 0;  // Example linear velocity
    twist_msg.angular.x = 0; // Example angular velocity
    twist_msg.angular.y = 0; // Example angular velocity
    twist_msg.angular.z = 0; // Example angular velocity

    // Publish the message
    cmd_vel_pub.publish(&twist_msg);

    // Debug print
    Serial.println("Published cmd_vel message");
  } else{
    Serial.println("Not connected");
  }

  nh.spinOnce();
  delay(1000);
}
