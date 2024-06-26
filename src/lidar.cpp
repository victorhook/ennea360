#include "lidar.h"

Lidar::Lidar(const uint8_t enable_pin, const uint8_t i2c_addr, const uint8_t orientation, TwoWire* wire) :
enable_pin(enable_pin),
m_i2c_addr(i2c_addr),
orientation(orientation),
m_wire(wire)
{

}
Lidar::~Lidar()
{

}
bool Lidar::init()
{
    pinMode(enable_pin, INPUT_PULLUP);
    digitalWrite(enable_pin, LOW);
    return true;
}

bool Lidar::configure()
{
    digitalWrite(enable_pin, HIGH);
    delay(5);
    m_lidar.begin(m_i2c_addr);
    return true;
}

bool Lidar::start()
{
    m_lidar.setMode(m_lidar.eContinuous, m_lidar.eHigh);
    m_lidar.start();
    return true;
}

void Lidar::read_distance()
{
    m_latest_distance = m_lidar.getDistance();
}

float Lidar::latest_distance() const
{
    return m_latest_distance;
}

bool Lidar::dist(float& distance)
{
    distance = m_lidar.getDistance();
    return true;
}