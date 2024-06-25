#ifndef LIDAR_H
#define LIDAR_H

#include <Wire.h>
#include "VL53L0X.h"
#include "DFRobot_VL53L0X.h"


class Lidar
{
    public:
        Lidar(const uint8_t enable_pin, const uint8_t i2c_addr, const uint8_t orientation, TwoWire* wire);
        ~Lidar();
        bool init();
        bool configure();
        bool start();
        void read_distance();
        float latest_distance() const;
        bool dist(float& distance);
        const uint8_t orientation;

    private:
        const uint8_t m_enable_pin;
        const uint8_t m_i2c_addr;
        float m_latest_distance;
        DFRobot_VL53L0X m_lidar;
        TwoWire* m_wire;
};


#endif /* LIDAR_H */
