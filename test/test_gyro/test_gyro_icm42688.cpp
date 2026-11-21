#include "Device/Baro/BaroBMP280.hpp"
#include "Device/Baro/BaroBMP085.hpp"
#include "Device/Baro/BaroMS5611.hpp"
#include "Device/Baro/BaroLPS.hpp"
#include "Device/Baro/BaroSPL06.hpp"
#include "Device/BusSlave.hpp"
#include "Device/Mag/MagQMC5883L.hpp"
#include "Device/Gyro/GyroICM42688.hpp"
#include "Device/GyroDevice.hpp"
#include "Device/Mag/MagHMC5883L.hpp"
#include <ArduinoFake.h>
#include <platform.h>
#include <unity.h>

using namespace Espfc;
using namespace Espfc::Device;
using namespace Espfc::Device::Gyro;
using namespace Espfc::Device::Mag;
using namespace Espfc::Device::Baro;
using namespace fakeit;

class MockBusDevice : public BusDevice
{
public:
  uint8_t readRegs[256] = {};  // registers ret
  uint8_t writeRegs[256] = {}; // registers captured by write
  int writeCalls = 0;
  bool failRead = false;
  int failReadReg = -1;
  int failWriteReg = -1;

  BusType getType() const override
  {
    return BUS_SPI;
  }

  int8_t read(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    if (failRead ||
        regAddr == failReadReg)
    {
      return 0;
    }

    for (uint8_t i = 0; i < length; i++)
      data[i] = readRegs[(regAddr + i) & 0xFF];
    return length;
  }

  int8_t readFast(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) override
  {
    return read(devAddr, regAddr, length, data);
  }

  bool write(uint8_t devAddr, uint8_t regAddr, uint8_t length, const uint8_t* data) override
  {
    writeCalls++;

    if (regAddr == failWriteReg)
    {
      return false;
    }

    for (uint8_t i = 0; i < length; i++)
      writeRegs[(regAddr + i) & 0xFF] = data[i];

    return true;
  }
};

void setUp()
{
  ArduinoFakeReset();

  When(
      Method(
          ArduinoFake(),
          delay))
      .AlwaysReturn();
}

void tearDown()
{
}


void test_whoami_match()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  TEST_ASSERT_TRUE(dev.testConnection());
}

void test_whoami_mismatch()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x12; // ICM-20602 WHO_AM_I
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  TEST_ASSERT_FALSE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x12, chipId.value());
}

void test_chip_id_is_empty_before_connection()
{
  GyroICM42688 dev;
  TEST_ASSERT_FALSE(dev.getChipId().has_value());
}

void test_chip_id_cached_on_success()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);

  TEST_ASSERT_TRUE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x47, chipId.value());
}

void test_chip_id_updated_on_mismatch()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);

  TEST_ASSERT_TRUE(dev.testConnection());
  bus.readRegs[0x75] = 0x12;
  TEST_ASSERT_FALSE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x12, chipId.value());
}

void test_chip_id_preserved_on_read_failure()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x47;
  GyroICM42688 dev;
  dev.setBus(&bus, 0);

  TEST_ASSERT_TRUE(dev.testConnection());
  bus.failRead = true;
  TEST_ASSERT_FALSE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x47, chipId.value());
}

void test_mag_hmc5883l_uses_first_id_byte()
{
  MockBusDevice bus;
  bus.readRegs[0x0A] = 'H';
  bus.readRegs[0x0B] = '4';
  bus.readRegs[0x0C] = '3';
  MagHMC5883L dev;
  dev.setBus(&bus, 0x1E);

  TEST_ASSERT_TRUE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8('H', chipId.value());
}

void test_mag_hmc5883l_begin_rejects_config_write_failure()
{
  MockBusDevice bus;
  bus.readRegs[0x0A] = 'H';
  bus.readRegs[0x0B] = '4';
  bus.readRegs[0x0C] = '3';
  bus.failWriteReg = 0x02;

  MagHMC5883L dev;

  TEST_ASSERT_EQUAL_INT(
      0,
      dev.begin(
          &bus,
          0x1E));
}

void test_mag_hmc5883l_rejects_overflow_sample()
{
  MockBusDevice bus;

  // X overflow sentinel = -4096 (0xF000).
  bus.readRegs[0x03] = 0xF0;
  bus.readRegs[0x04] = 0x00;
  bus.readRegs[0x05] = 0x00;
  bus.readRegs[0x06] = 0x10;
  bus.readRegs[0x07] = 0x00;
  bus.readRegs[0x08] = 0x20;

  MagHMC5883L dev;
  dev.setBus(
      &bus,
      0x1E);

  VectorInt16 sample{};

  TEST_ASSERT_EQUAL_INT(
      0,
      dev.readMag(
          sample));
}

void test_baro_bmp280_caches_whoami()
{
  MockBusDevice bus;
  bus.readRegs[0xD0] = 0x58;
  BaroBMP280 dev;
  dev.setBus(&bus, 0x76);

  TEST_ASSERT_TRUE(dev.testConnection());

  const auto chipId = dev.getChipId();
  TEST_ASSERT_TRUE(chipId.has_value());
  TEST_ASSERT_EQUAL_HEX8(0x58, chipId.value());
}

static void seedBmp280Calibration(
    MockBusDevice& bus)
{
  // Only the two unsigned non-zero coefficients are required by the probe
  // sanity gate; the remaining coefficients may be zero for this init test.
  bus.readRegs[0x88] = 0x01;
  bus.readRegs[0x89] = 0x00; // dig_T1 = 1
  bus.readRegs[0x8E] = 0x01;
  bus.readRegs[0x8F] = 0x00; // dig_P1 = 1
}

void test_baro_bmp280_begin_rejects_calibration_read_failure()
{
  MockBusDevice bus;
  bus.readRegs[0xD0] = 0x58;
  seedBmp280Calibration(bus);
  bus.failReadReg = 0x88;

  BaroBMP280 dev;

  TEST_ASSERT_EQUAL_INT(
      0,
      dev.begin(
          &bus,
          0x76));
}

void test_baro_bmp280_begin_rejects_blank_calibration()
{
  MockBusDevice bus;
  bus.readRegs[0xD0] = 0x58;

  BaroBMP280 dev;

  TEST_ASSERT_EQUAL_INT(
      0,
      dev.begin(
          &bus,
          0x76));
}

void test_baro_bmp280_begin_rejects_failed_config_write()
{
  MockBusDevice bus;
  bus.readRegs[0xD0] = 0x58;
  seedBmp280Calibration(bus);
  bus.failWriteReg = 0xF5;

  BaroBMP280 dev;

  TEST_ASSERT_EQUAL_INT(
      0,
      dev.begin(
          &bus,
          0x76));
}

void test_baro_bmp280_begin_accepts_valid_initialization()
{
  MockBusDevice bus;
  bus.readRegs[0xD0] = 0x58;
  seedBmp280Calibration(bus);

  BaroBMP280 dev;

  TEST_ASSERT_EQUAL_INT(
      1,
      dev.begin(
          &bus,
          0x76));

TEST_ASSERT_EQUAL_HEX8(
    0xB6,
    bus.writeRegs[0xE0]);

TEST_ASSERT_EQUAL_HEX8(
    0x08,
    bus.writeRegs[0xF5]);

TEST_ASSERT_EQUAL_HEX8(
    0x53,
    bus.writeRegs[0xF4]);
}

void test_begin_aborts_on_failed_connection()
{
  MockBusDevice bus;
  bus.readRegs[0x75] = 0x00;
  GyroICM42688 dev;
  int result = dev.begin(&bus, 0);
  TEST_ASSERT_EQUAL_INT(0, result);
  TEST_ASSERT_EQUAL_INT(0, bus.writeCalls);
}

void test_read_gyro_decoding()
{
  MockBusDevice bus;
  // gyro data registers start at 0x25
  bus.readRegs[0x25] = 0x01;
  bus.readRegs[0x26] = 0x00; // X = 256
  bus.readRegs[0x27] = 0x02;
  bus.readRegs[0x28] = 0x00; // Y = 512
  bus.readRegs[0x29] = 0x03;
  bus.readRegs[0x2A] = 0x00; // Z = 768
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  VectorInt16 v;
  dev.readGyro(v);
  TEST_ASSERT_EQUAL_INT16(256, v.x);
  TEST_ASSERT_EQUAL_INT16(512, v.y);
  TEST_ASSERT_EQUAL_INT16(768, v.z);
}

void test_read_accel_decoding()
{
  MockBusDevice bus;
  // accel data registers start at 0x1F
  bus.readRegs[0x1F] = 0xFF;
  bus.readRegs[0x20] = 0xFE; // X = -2
  bus.readRegs[0x21] = 0x00;
  bus.readRegs[0x22] = 0x01; // Y = 1
  bus.readRegs[0x23] = 0x7F;
  bus.readRegs[0x24] = 0xFF; // Z = 32767
  GyroICM42688 dev;
  dev.setBus(&bus, 0);
  VectorInt16 v;
  dev.readAccel(v);
  TEST_ASSERT_EQUAL_INT16(-2, v.x);
  TEST_ASSERT_EQUAL_INT16(1, v.y);
  TEST_ASSERT_EQUAL_INT16(32767, v.z);
}

void test_get_type()
{
  GyroICM42688 dev;
  TEST_ASSERT_EQUAL_INT(GYRO_ICM42688, dev.getType());
}

void test_get_rate()
{
  GyroICM42688 dev;
  TEST_ASSERT_EQUAL_INT(8000, dev.getRate());
}

void test_enum_values()
{
  static_assert(GYRO_ICM42688 == 9, "GYRO_ICM42688 must equal 9 (append-only rule)");
  static_assert(GYRO_MAX == 10, "GYRO_MAX must equal 10 after ICM42688 addition");
  TEST_PASS();
}

void test_name_table()
{
  const char* name = GyroDevice::getName(GYRO_ICM42688);
  TEST_ASSERT_EQUAL_STRING("ICM42688", name);
}

static void setWord(MockBusDevice& bus, size_t reg, uint16_t value)
{
  bus.readRegs[reg] = value >> 8;
  bus.readRegs[reg + 1] = value;
}

static void setAdc(MockBusDevice& bus, size_t reg, uint32_t value)
{
  bus.readRegs[reg] = value >> 16;
  setWord(bus, reg + 1, value);
}

static void test_bmp085_factory_calibration_and_failed_transactions()
{
  MockBusDevice bus;
  BaroBMP085 dev;
  bus.readRegs[0xd0] = 0x55;
  const int32_t calibration[] = {408,-72,-14383,32741,32757,23153,6190,4,-32768,-8711,2868};
  for (unsigned i = 0; i < 11; ++i) setWord(bus,0xaa + 2*i, calibration[i]);
  TEST_ASSERT_TRUE(dev.begin(&bus));
  dev.setMode(BARO_MODE_TEMP);
  setWord(bus,0xf6,27898);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,15.f,dev.readTemperature());
  dev.setMode(BARO_MODE_PRESS);
  setAdc(bus,0xf6,23843u << 8);
  // The datasheet vector uses OSS=0. At the runtime's OSS=3 the same
  // sample has three more bits and rounds within a few pascals.
  TEST_ASSERT_FLOAT_WITHIN(4.f,69964.f,dev.readPressure());
  bus.failReadReg = 0xf6;
  TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
  dev.setMode(BARO_MODE_TEMP);
  TEST_ASSERT_TRUE(std::isnan(dev.readTemperature()));
  bus.failReadReg = -1;
  dev.setMode(BARO_MODE_PRESS);
  TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
  bus.failReadReg = 0xaa;
  TEST_ASSERT_FALSE(dev.begin(&bus));
}

static void test_dps310_spl06_compensation_and_stale_data()
{
  for (auto type : {BARO_SPL06, BARO_DPS310})
  {
    MockBusDevice bus;
    BaroSPL06 dev;
    dev.setType(type);
    bus.readRegs[0x0d] = 0x10;
    bus.readRegs[0x08] = 0xf7;
    // c0 = -20, c1 = 2, c00 = 100000, c10 = 1000, c01 = -100.
    const uint8_t coeff[] = {0xfe,0xc0,2,0x18,0x6a,0,3,0xe8,0xff,0x9c};
    std::copy(coeff, coeff + sizeof(coeff), bus.readRegs + 0x10);
    bus.readRegs[0] = 0x3c; // pressure = scale / 2
    bus.readRegs[3] = 0xc4; // temperature = -scale / 2
    TEST_ASSERT_TRUE(dev.begin(&bus, 0x77));
    TEST_ASSERT_EQUAL_HEX8(type == BARO_DPS310 ? 0x53 : 0xd3, bus.writeRegs[7]);
    TEST_ASSERT_TRUE(std::isnan(dev.readTemperature()));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100550.f, dev.readPressure());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -11.f, dev.readTemperature());
    bus.readRegs[8] = 0xc7; // neither result is ready
    TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
    bus.readRegs[8] = 0xf7;
    bus.failReadReg = 0;
    TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
    TEST_ASSERT_TRUE(std::isnan(dev.readTemperature()));
    bus.failReadReg = 0x19; // second calibration block unreadable
    TEST_ASSERT_FALSE(dev.begin(&bus, 0x77));
    TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
  }
}

static void test_mpu_bridge_rejects_missing_sensor_and_chunks_long_reads()
{
  MockBusDevice master;
  BusSlave bridge;
  bridge.begin(&master, 0x68);
  uint8_t data[24] = {};
  TEST_ASSERT_EQUAL_INT(24, bridge.read(0x76, 0x88, sizeof(data), data));
  TEST_ASSERT_EQUAL_HEX8(0x97, master.writeRegs[0x26]);
  TEST_ASSERT_EQUAL_HEX8(0x89, master.writeRegs[0x27]);
  for (uint8_t status : {0x01, 0x20})
  {
    master.readRegs[0x36] = status;
    TEST_ASSERT_EQUAL_INT(0, bridge.readFast(0x76, 0xf7, 6, data));
    TEST_ASSERT_FALSE(bridge.writeByte(0x76, 0xf4, 0x33));
  }
  TEST_ASSERT_FALSE(bridge.write(0x76, 0, 0, nullptr));
}

static void test_qmc5883l_field_scale_and_config_write_failure()
{
  MockBusDevice bus;
  MagQMC5883L dev;
  bus.readRegs[0x0d] = 0xff;
  bus.readRegs[9] = 0xc9;
  TEST_ASSERT_TRUE(dev.begin(&bus));
  TEST_ASSERT_EQUAL_UINT8(1, bus.writeRegs[0x0b]);
  TEST_ASSERT_EQUAL_UINT8(0xc9, bus.writeRegs[9]);
  TEST_ASSERT_EQUAL_INT(100, dev.getRate());
  bus.readRegs[0] = 0xe0;
  bus.readRegs[1] = 0x2e;
  VectorInt16 raw;
  TEST_ASSERT_TRUE(dev.readMag(raw));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.f, dev.convert(raw).x);
  bus.failWriteReg = 9;
  TEST_ASSERT_FALSE(dev.begin(&bus));
}

static void test_lps_temperature_pressure_and_status()
{
  for (uint8_t id : {0xbd, 0xb1, 0xb4})
  {
    MockBusDevice bus;
    BaroLPS dev;
    bus.readRegs[0x0f] = id;
    const uint8_t ctrl1 = id == 0xbd ? 0x20 : 0x10;
    const uint8_t config = id == 0xbd ? 0xc4 : id == 0xb1 ? 0x32 : 0x22;
    bus.readRegs[ctrl1] = config;
    TEST_ASSERT_TRUE(dev.begin(&bus, 10));
    TEST_ASSERT_EQUAL_HEX8(config, bus.writeRegs[ctrl1]);
    TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
    bus.readRegs[0x27] = 3;
    bus.readRegs[0x28] = 0;
    bus.readRegs[0x29] = 0x80;
    bus.readRegs[0x2a] = 0x3e; // 1000 hPa
    const int16_t temperature = id == 0xbd ? -25200 : -1000;
    bus.readRegs[0x2b] = static_cast<uint8_t>(temperature);
    bus.readRegs[0x2c] = static_cast<uint16_t>(temperature) >> 8;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.f, dev.readTemperature());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 100000.f, dev.readPressure());
    bus.failReadReg = 0x29;
    TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
    bus.failReadReg = -1;
    bus.readRegs[0x27] = 1;
    TEST_ASSERT_TRUE(std::isnan(dev.readTemperature()));
    bus.readRegs[ctrl1 + 1] = 4; // reset never completed
    TEST_ASSERT_FALSE(dev.begin(&bus, 10));
    TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
  }
}

static void test_ms5611_factory_crc_and_second_order_compensation()
{
  MockBusDevice bus;
  BaroMS5611 dev;
  const uint16_t words[] = {0,40127,36924,23317,23282,33464,28312,0};
  for (unsigned i = 0; i < 8; ++i) setWord(bus,0xa0 + 2*i,words[i]);
  TEST_ASSERT_TRUE(dev.begin(&bus,7));
  struct Sample { uint32_t d2; float celsius; int pascals; };
  const Sample samples[] = {{8569150,20.07f,100009},{8069150,2.05f,96512},{7369150,-27.10f,90613}};
  for (const auto& sample : samples)
  {
    dev.setMode(BARO_MODE_TEMP);
    setAdc(bus,0,sample.d2);
    TEST_ASSERT_FLOAT_WITHIN(0.005f,sample.celsius,dev.readTemperature());
    dev.setMode(BARO_MODE_PRESS);
    setAdc(bus,0,9085466);
    TEST_ASSERT_EQUAL_FLOAT(sample.pascals,dev.readPressure());
  }
  bus.failWriteReg = 0x58;
  dev.setMode(BARO_MODE_TEMP);
  TEST_ASSERT_TRUE(std::isnan(dev.readTemperature()));
  dev.setMode(BARO_MODE_PRESS);
  TEST_ASSERT_TRUE(std::isnan(dev.readPressure()));
  bus.readRegs[0xaf] = 1; // wrong PROM checksum
  TEST_ASSERT_FALSE(dev.begin(&bus,7));
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_dps310_spl06_compensation_and_stale_data);
  RUN_TEST(test_mpu_bridge_rejects_missing_sensor_and_chunks_long_reads);
  RUN_TEST(test_qmc5883l_field_scale_and_config_write_failure);
  RUN_TEST(test_lps_temperature_pressure_and_status);
  RUN_TEST(test_bmp085_factory_calibration_and_failed_transactions);
  RUN_TEST(test_ms5611_factory_crc_and_second_order_compensation);
  RUN_TEST(test_whoami_match);
  RUN_TEST(test_whoami_mismatch);
  RUN_TEST(test_chip_id_is_empty_before_connection);
  RUN_TEST(test_chip_id_cached_on_success);
  RUN_TEST(test_chip_id_updated_on_mismatch);
  RUN_TEST(test_chip_id_preserved_on_read_failure);
  RUN_TEST(test_mag_hmc5883l_uses_first_id_byte);
  RUN_TEST(test_mag_hmc5883l_begin_rejects_config_write_failure);
  RUN_TEST(test_mag_hmc5883l_rejects_overflow_sample);
  RUN_TEST(test_baro_bmp280_caches_whoami);
  RUN_TEST(test_baro_bmp280_begin_rejects_calibration_read_failure);
  RUN_TEST(test_baro_bmp280_begin_rejects_blank_calibration);
  RUN_TEST(test_baro_bmp280_begin_rejects_failed_config_write);
  RUN_TEST(test_baro_bmp280_begin_accepts_valid_initialization);
  RUN_TEST(test_begin_aborts_on_failed_connection);
  RUN_TEST(test_read_gyro_decoding);
  RUN_TEST(test_read_accel_decoding);
  RUN_TEST(test_get_type);
  RUN_TEST(test_get_rate);
  RUN_TEST(test_enum_values);
  RUN_TEST(test_name_table);
  return UNITY_END();
}
