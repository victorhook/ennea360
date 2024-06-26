#include <Wire.h>
#include "lidar.h"
#include "mavlink/ardupilotmega/mavlink.h"


#define PIN_I2C_SDA       4
#define PIN_I2C_SCL       5
#define PIN_MAV_SERIAL_RX 1
#define PIN_MAV_SERIAL_TX 0

#define MavSerial         Serial1
#define MAV_SERIAL_BAUD   115200

#define MAV_SYSTEM_ID     10
#define MAV_COMPONENT_ID  MAV_COMP_ID_PERIPHERAL

#define MAV_DISTANCE_SENSOR_PERIOD_MS 50


static void send_mavlink_heartbeat();
static void send_mavlink_distance(const uint8_t orientation, const uint16_t distance_cm);


std::vector<Lidar> lidars{
    Lidar(6,  0x36, MAV_SENSOR_ROTATION_NONE,   &Wire),
    Lidar(7,  0x37, MAV_SENSOR_ROTATION_YAW_45, &Wire),
    Lidar(8,  0x38, MAV_SENSOR_ROTATION_YAW_90, &Wire),
    Lidar(9,  0x39, MAV_SENSOR_ROTATION_YAW_135, &Wire),
    Lidar(10, 0x40, MAV_SENSOR_ROTATION_YAW_180, &Wire),
    Lidar(11, 0x41, MAV_SENSOR_ROTATION_YAW_225, &Wire),
    Lidar(12, 0x42, MAV_SENSOR_ROTATION_YAW_270, &Wire),
    Lidar(13, 0x43, MAV_SENSOR_ROTATION_YAW_315, &Wire),
    Lidar(14, 0x44, MAV_SENSOR_ROTATION_PITCH_90, &Wire),
};


void setup()
{
    Serial.begin(115200);
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();

    while (!Serial);

    uint32_t t0 = millis();
    Serial.printf("Configuring %d lidars\n", lidars.size());

    // Begin all I/O
    Serial.println("Initialize I/O");
    for (Lidar& lidar : lidars)
    {
        Serial.printf("  %d: %d\n", lidar.orientation, lidar.init());
    }
    Serial.println("Init done");

    // Configure
    Serial.println("Configuring i2c addresses:");
    for (Lidar& lidar : lidars)
    {
        Serial.printf("%d: %d\n", lidar.orientation, lidar.configure());
    }
    Serial.println("Configuration done");

    Serial.println("Starting:");
    for (Lidar& lidar : lidars)
    {
        Serial.printf("%d: %d\n", lidar.orientation, lidar.start());
    }
    Serial.println("Starting done");

    Serial.printf("Initialization took %d ms\n", millis() - t0);
    Serial.println("Let's run!");

    // Mav serial
    MavSerial.setRX(PIN_MAV_SERIAL_RX);
    MavSerial.setTX(PIN_MAV_SERIAL_TX);
    MavSerial.begin(MAV_SERIAL_BAUD);
}


void loop()
{
    static uint32_t latest_mav_heartbeat = 0;
    static uint32_t latest_mav_distance = 0;

    if ((millis() - latest_mav_heartbeat) > 1000)
    {
        send_mavlink_heartbeat();
        latest_mav_heartbeat = millis();
    }


    if ((millis() - latest_mav_distance) < MAV_DISTANCE_SENSOR_PERIOD_MS)
    {
        return;
    }

    // Poll each sensor to get the latest data
    for (Lidar& lidar : lidars)
    {
       lidar.read_distance();
    }

    Serial.printf("%ld\n", millis());
    for (Lidar& lidar : lidars)
    {
        float distance_cm = lidar.latest_distance() / 10.0;
        Serial.printf("  %d: %.2f cm\n", lidar.orientation, distance_cm);
        send_mavlink_distance(lidar.orientation, distance_cm);
    }
    Serial.print("\n");

    latest_mav_distance = millis();
}


// -- Helper functions -- //
static void send_mavlink_heartbeat()
{
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MavSerial.write(buf, len);
}


static void send_mavlink_distance(const uint8_t orientation, const uint16_t distance_cm)
{
    static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_msg_distance_sensor_pack(
        MAV_SYSTEM_ID, MAV_COMPONENT_ID, &msg, millis(),
        5, // Min distance
        120, // Max distance
        distance_cm, // Current distance
        MAV_DISTANCE_SENSOR_LASER, // Type
        orientation, // ID
        orientation, // Orienation
        0xFF, // Covariance
        0.436332313f, // Horizontal FoV (radians), 25 deg
        0.436332313f, // Vertical FoV (radians), 25 deg
        0, // Quaternion, 0 = invalid
        0  // Signal quality
    );
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    MavSerial.write(buf, len);
}