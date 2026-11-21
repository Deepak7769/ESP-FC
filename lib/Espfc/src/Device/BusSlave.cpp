#include "BusSlave.hpp"
#include "Hal/Time.hpp"
#include "Utils/MemoryHelper.h"

#define MPU6050_I2C_SLV0_ADDR 0x25
#define MPU6050_I2C_SLV0_REG 0x26
#define MPU6050_I2C_SLV0_DO 0x63
#define MPU6050_I2C_SLV0_CTRL 0x27
#define MPU6050_I2C_SLV0_EN 0x80
#define MPU6050_EXT_SENS_DATA_00 0x49

namespace Espfc::Device {

BusSlave::BusSlave() {}

int BusSlave::begin(BusDevice* dev, uint8_t addr)
{
  setBus(dev, addr);

  return 1;
}

BusType BusSlave::getType() const
{
  return BUS_SLV;
}

int8_t BusSlave::read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  if (!data || length == 0 || length > 127) return 0;
  if (length > 15)
  {
    // SLV0_CTRL has a four-bit transfer count.
    if (read(devAddr, regAddr, 15, data) != 15) return 0;
    const uint8_t rest = length - 15;
    return read(devAddr, regAddr + 15, rest, data + 15) == rest ? length : 0;
  }
  // Program slave 0 for a register read.
  if (!writeMaster(MPU6050_I2C_SLV0_ADDR, devAddr | 0x80))
  {
    return 0;
  }
  // set the register to the desired AK8963 sub address
  if (!writeMaster(MPU6050_I2C_SLV0_REG, regAddr))
  {
    return 0;
  }
  // enable I2C and request the bytes
  if (!writeMaster(MPU6050_I2C_SLV0_CTRL, MPU6050_I2C_SLV0_EN | length))
  {
    return 0;
  }

  // takes some time for these registers to fill
  delay(1);

  // read the bytes off the EXT_SENS_DATA registers
  return readFast(devAddr, regAddr, length, data);
}

// readFast() ignores devAddr and regAddr args and read ext sensor data reg from master
int8_t FAST_CODE_ATTR BusSlave::readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  // The MPU can acknowledge the host even when the auxiliary sensor has disconnected.
  uint8_t status;
  if (_bus->readByte(_addr, 0x36, &status) != 1 || (status & 0x21)) return 0;
  return _bus->readFast(_addr, MPU6050_EXT_SENS_DATA_00, length, data);
}

// The slave-0 data-out register holds one byte.
bool BusSlave::write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data)
{
  if (!data || length != 1) return false;
  // Configure slave 0 for a register write.
  if (!writeMaster(MPU6050_I2C_SLV0_ADDR, devAddr))
  {
    return false;
  }
  // set the register to the desired AK8963 sub address
  if (!writeMaster(MPU6050_I2C_SLV0_REG, regAddr))
  {
    return false;
  }
  // store the data for write
  if (!writeMaster(MPU6050_I2C_SLV0_DO, *data))
  {
    return false;
  }
  // enable I2C and send 1 byte
  if (!writeMaster(MPU6050_I2C_SLV0_CTRL, MPU6050_I2C_SLV0_EN | 0x01))
  {
    return false;
  }

  uint8_t status;
  return _bus->readByte(_addr, 0x36, &status) == 1 && !(status & 0x21);
}

int8_t BusSlave::writeMaster(uint8_t regAddr, uint8_t data)
{
  int8_t res = _bus->write(_addr, regAddr, 1, &data);
  delay(10);
  return res;
}

int8_t BusSlave::readMaster(uint8_t regAddr, uint8_t length, uint8_t* data)
{
  return _bus->read(_addr, regAddr, length, data);
}

} // namespace Espfc::Device
