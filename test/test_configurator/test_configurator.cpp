#include "Connect/MspProcessor.hpp"
#include "Device/InputSumXbus.hpp"
#include "Sensor/VoltageSensor.hpp"
#include "Output/Mixer.h"
#include "Telemetry/TelemetryLtm.hpp"
#include "Sensor/GpsSensor.hpp"
#include "Sensor/SurfaceSensor.hpp"
#include "Connect/Vtx.hpp"
#include "Control/Controller.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_common.h"
#include "Utils/Storage.h"
#include "msp/msp_protocol.h"
#include <ArduinoFake.h>
#include <unity.h>
#include <algorithm>
#include <cstring>
#include <vector>

using namespace Espfc;
using namespace fakeit;

// Queued serial bytes serve as both the RX wire and the MSP port.
class TestPort : public Stream::ReadWritable
{
public:
  std::vector<uint8_t> bytes;
  size_t index = 0;
  std::vector<uint8_t> written;
  void begin(const Hal::SerialDeviceConfig&) override {}
  void updateBaudRate(int) override {}
  int available() override { return bytes.size() - index; }
  int read() override { return available() ? bytes[index++] : -1; }
  int peek() override { return available() ? bytes[index] : -1; }
  size_t readMany(uint8_t* out, size_t count) override
  {
    count = std::min<size_t>(available(), count);
    if (count) std::copy_n(bytes.data() + index, count, out);
    index += count;
    return count;
  }
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t* data, size_t size) override { written.insert(written.end(),data,data+size); return size; }
  int availableForWrite() override { return 256; }
  void flush() override {}
  bool isTxFifoEmpty() override { return true; }
};

static unsigned long nowUs;
static Connect::MspResponse command(Model& model, uint16_t id, std::vector<uint8_t> payload = {})
{
  TestPort port;
  Connect::MspProcessor processor(model);
  Connect::MspMessage msg;
  Connect::MspResponse reply;
  msg.cmd = id;
  for (uint8_t byte : payload) msg.buffer[msg.received++] = byte;
  processor.processCommand(msg, reply, port);
  return reply;
}

static void clocks()
{
  ArduinoFakeReset();
  nowUs = 1000000;
  When(Method(ArduinoFake(), micros)).AlwaysDo([] { return nowUs; });
  When(Method(ArduinoFake(), millis)).AlwaysDo([] { return nowUs / 1000; });
}

static void test_sumd_drops_bad_crc_failsafe_and_recovers_after_partial_frame()
{
  clocks();
  TestPort port;
  Device::InputSumXbus input;
  input.begin(&port, SERIALRX_SUMD);
  const std::vector<uint8_t> good = {0xa8, 0x01, 0x04, 0x1f, 0x40, 0x2e, 0xe0, 0x3e, 0x80, 0x22, 0x60, 0x7c, 0x02};
  const std::vector<uint8_t> failsafe = {0xa8, 0x81, 0x04, 0x1f, 0x40, 0x2e, 0xe0, 0x3e, 0x80, 0x22, 0x60, 0x98, 0x36};
  port.bytes = good;
  port.bytes.back() ^= 1;
  TEST_ASSERT_EQUAL(INPUT_IDLE, input.update());
  TEST_ASSERT_EQUAL(0, input.getChannelCount());
  port.bytes.insert(port.bytes.end(), good.begin(), good.begin() + 4);
  TEST_ASSERT_EQUAL(INPUT_IDLE, input.update());
  nowUs += 5000;
  port.bytes.insert(port.bytes.end(), failsafe.begin(), failsafe.end());
  port.bytes.insert(port.bytes.end(), good.begin(), good.end());
  TEST_ASSERT_EQUAL(INPUT_FAILSAFE, input.update());
  TEST_ASSERT_EQUAL(INPUT_RECEIVED, input.update());
  TEST_ASSERT_EQUAL(4, input.getChannelCount());
  TEST_ASSERT_EQUAL(1500, input.get(1));
  TEST_ASSERT_EQUAL(2000, input.get(2));
}

static void test_sumh_channel_conversion_and_trailer()
{
  clocks();
  TestPort port;
  Device::InputSumXbus input;
  input.begin(&port, SERIALRX_SUMH);
  port.bytes.assign(21, 0);
  port.bytes[0] = 0xa8;
  for (unsigned i = 3; i < 19; i += 2) { port.bytes[i] = 0x2e; port.bytes[i+1] = 0xe0; }
  TEST_ASSERT_EQUAL(INPUT_RECEIVED, input.update());
  TEST_ASSERT_EQUAL(8, input.getChannelCount());
  TEST_ASSERT_EQUAL(1500, input.get(7));
  port.index = 0;
  port.bytes[19] = 1;
  TEST_ASSERT_EQUAL(INPUT_IDLE, input.update());
}

static void test_xbus_mode_b_and_rj01_nested_checksums()
{
  clocks();
  const std::vector<uint8_t> modeB = {0xa1, 0x00, 0x00, 0x08, 0x00, 0x0f, 0xff, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x2b, 0xf4};
  const std::vector<uint8_t> rj01 = {0xa1, 0x1e, 0x00, 0xa1, 0x00, 0x00, 0x08, 0x00, 0x0f, 0xff, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x2b, 0xf4, 0x00, 0x00, 0x26};
  TestPort port;
  Device::InputSumXbus input;
  for (auto provider : {SERIALRX_XBUS_MODE_B, SERIALRX_XBUS_MODE_B_RJ01})
  {
    port.bytes = provider == SERIALRX_XBUS_MODE_B ? modeB : rj01;
    port.index = 0;
    input.begin(&port, provider);
    TEST_ASSERT_EQUAL(INPUT_RECEIVED, input.update());
    TEST_ASSERT_EQUAL(12, input.getChannelCount());
    TEST_ASSERT_EQUAL(800, input.get(0));
    TEST_ASSERT_EQUAL(1500, input.get(1));
    TEST_ASSERT_EQUAL(2199, input.get(2));
    port.index = 0;
    port.bytes.back() ^= 1;
    TEST_ASSERT_EQUAL(INPUT_IDLE, input.update());
  }
  port.bytes = rj01;
  port.bytes[12] ^= 1;
  port.bytes.back() = 237;
  port.index = 0;
  TEST_ASSERT_EQUAL(INPUT_IDLE, input.update());
}

static void test_msp_receiver_selection_survives_sanitize_and_rejects_unknown()
{
  Model model;
  for (auto provider : {SERIALRX_SUMD, SERIALRX_SUMH, SERIALRX_XBUS_MODE_B, SERIALRX_XBUS_MODE_B_RJ01, SERIALRX_NONE})
  {
    std::vector<uint8_t> fields = {static_cast<uint8_t>(provider), 108,7, 220,5, 82,4, 0, 117,3, 67,8};
    TEST_ASSERT_EQUAL(1, command(model, MSP_SET_RX_CONFIG, fields).result);
    model.sanitize();
    TEST_ASSERT_EQUAL(provider, command(model, MSP_RX_CONFIG).data[0]);
    fields[0] = 255;
    TEST_ASSERT_EQUAL(-1, command(model, MSP_SET_RX_CONFIG, fields).result);
    TEST_ASSERT_EQUAL(provider, model.config.input.serialRxProvider);
  }
}

static void test_battery_wire_round_trip_alarm_and_atomic_bad_write()
{
  Model model;
  std::vector<uint8_t> settings = {33,44,36, 0x28,0x0a, 2,4, 0x4d,1, 0xb3,1, 0x69,1};
  TEST_ASSERT_EQUAL(1, command(model, MSP_SET_BATTERY_CONFIG, settings).result);
  auto reply = command(model, MSP_BATTERY_CONFIG);
  TEST_ASSERT_EQUAL(13, reply.len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(settings.data(), reply.data, 13);
  auto& battery = model.state.battery;
  battery.cells = 4;
  battery.samples = 0;
  battery.voltage = 13.2f;
  battery.cellVoltage = 3.3f;
  battery.mspMahDrawn = 1300;
  battery.mspCurrentValid = true;
  reply = command(model, MSP_BATTERY_STATE);
  TEST_ASSERT_EQUAL(2,reply.data[8]);
  TEST_ASSERT_EQUAL(2600,reply.data[1] | reply.data[2] << 8);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,50.f,battery.remaining(CURRENT_METER_MSP, model.config.batteryProfile));
  settings[11] = 0;
  settings[12] = 0;
  TEST_ASSERT_EQUAL(-1,command(model, MSP_SET_BATTERY_CONFIG, settings).result);
  settings.resize(9);
  TEST_ASSERT_EQUAL(-1,command(model, MSP_SET_BATTERY_CONFIG, settings).result);
  TEST_ASSERT_EQUAL(361,model.config.vbat.cellWarning);
}

static void test_legacy_battery_packet_and_eeprom_migration()
{
  Model model;
  TEST_ASSERT_EQUAL(1, command(model, MSP_SET_BATTERY_CONFIG, {32,43,35,100,0,1,1}).result);
  TEST_ASSERT_EQUAL(320,model.config.batteryProfile.cellMin);
  TEST_ASSERT_EQUAL(430,model.config.batteryProfile.cellMax);
  model.config.input.serialRxProvider = SERIALRX_IBUS;
  model.config.antiGravityGain = 98;
  for (auto version : {2,3})
  {
    const size_t size = version == 2 ? offsetof(ModelConfig,batteryProfile) : sizeof(ModelConfig);
    std::vector<uint8_t> image = {0xa5, static_cast<uint8_t>(version), static_cast<uint8_t>(size), static_cast<uint8_t>(size>>8)};
    const auto* bytes = reinterpret_cast<const uint8_t*>(&model.config);
    image.insert(image.end(), bytes, bytes + size);
    ModelConfig restored;
    auto reader = [&](size_t address) { return image.at(address); };
    TEST_ASSERT_EQUAL(STORAGE_LOAD_SUCCESS, Utils::Storage::readConfig(restored, reader));
    TEST_ASSERT_EQUAL(98,restored.antiGravityGain);
    TEST_ASSERT_EQUAL(SERIALRX_IBUS,restored.input.serialRxProvider);
    TEST_ASSERT_EQUAL(version == 2 ? 340 : 320,restored.batteryProfile.cellMin);
    --image[2];
    TEST_ASSERT_EQUAL(STORAGE_ERR_BAD_SIZE,Utils::Storage::readConfig(restored, reader));
  }
}

static void test_virtual_meter_calibration_and_elapsed_charge()
{
  clocks();
  Model model;
  model.config.ibat.source = CURRENT_METER_VIRTUAL;
  TEST_ASSERT_EQUAL(1, command(model, MSP_SET_CURRENT_METER_CONFIG, {80,100,0,200,0}).result);
  Sensor::VoltageSensor sensor(model);
  sensor.begin();
  sensor.readIbat();
  TEST_ASSERT_FLOAT_WITHIN(0.01f,2.f,model.state.battery.current);
  model.updateModes(1u << MODE_ARMED);
  model.state.input.us[AXIS_THRUST] = 1500;
  for (int i = 0; i < 200; ++i) { nowUs += 20000; sensor.readIbat(); }
  TEST_ASSERT_FLOAT_WITHIN(0.01f,7.5f,model.state.battery.current);
  TEST_ASSERT_EQUAL(8, model.state.battery.mahDrawn(CURRENT_METER_VIRTUAL));
  auto meters = command(model, MSP_CURRENT_METERS);
  TEST_ASSERT_EQUAL(80,meters.data[5]);
  TEST_ASSERT_EQUAL(8,meters.data[6]);
  TEST_ASSERT_EQUAL(7500,meters.data[8] | meters.data[9] << 8);
}

static void test_esc_meter_uses_fresh_voltage_and_current_in_physical_units()
{
  clocks();
  Model model;
  model.config.output.dshotTelemetry = true;
  model.config.vbat.source = 2;
  model.config.ibat.source = CURRENT_METER_ESC;
  Sensor::VoltageSensor sensor(model);
  sensor.begin();
  auto& esc = model.state.output.telemetry;
  for (size_t i = 0; i < 2; ++i)
  {
    model.config.pin[PIN_OUTPUT_0+i] = 5 + i;
    esc.voltage[i] = 64;
    esc.current[i] = 3+i;
    esc.currentTimeMs[i] = esc.voltageTimeMs[i] = nowUs/1000;
  }
  for (int i = 0; i < 100; ++i) sensor.readVbat();
  sensor.readIbat();
  TEST_ASSERT_FLOAT_WITHIN(0.02f,16.f,model.state.battery.voltage);
  TEST_ASSERT_EQUAL(4,model.state.battery.cells);
  TEST_ASSERT_FLOAT_WITHIN(0.01f,7.f,model.state.battery.current);
  auto motors = command(model, MSP_MOTOR_TELEMETRY);
  TEST_ASSERT_EQUAL(1600,motors.data[8] | motors.data[9] << 8);
  TEST_ASSERT_EQUAL(300,motors.data[10] | motors.data[11] << 8);
  nowUs += 600000;
  sensor.readIbat();
  for (int i = 0; i < 100; ++i) sensor.readVbat();
  TEST_ASSERT_EQUAL(0,model.state.battery.cells);
  TEST_ASSERT_EQUAL_FLOAT(0.f,model.state.battery.current);
}

static void test_servo_tab_loads_and_writes_rate_forward_and_endpoints()
{
  clocks();
  Model model;
  model.config.output.channel[0].servo = true;
  TEST_ASSERT_EQUAL(1, command(model, MSP_SERVO_MIX_RULES).result);
  TEST_ASSERT_EQUAL(12 * OUTPUT_CHANNELS, command(model, MSP_SERVO_CONFIGURATIONS).len);
  // min 500, max 2500, middle 1500, -50% mixed deflection plus AUX1 as center
  std::vector<uint8_t> settings = {0,244,1,196,9,220,5,206,4,0,0,0,0};
  TEST_ASSERT_EQUAL(1, command(model, MSP_SET_SERVO_CONFIGURATION, settings).result);
  model.sanitize();
  auto reply = command(model, MSP_SERVO_CONFIGURATIONS);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(settings.data()+1,reply.data,12);
  model.config.featureMask = 0;
  model.updateModes(1u << MODE_ARMED);
  model.state.input.channelsValid = true;
  model.state.input.channelCount = 5;
  model.state.input.us[4] = 1700;
  MixerEntry rules[] = {MixerEntry()};
  MixerConfig mix(1,rules);
  float values[OUTPUT_CHANNELS] = {1.f};
  Output::Mixer mixer(model);
  mixer.writeOutput(mix,values);
  TEST_ASSERT_EQUAL(1450,model.state.output.us[0]);
  model.state.input.rxLoss = true;
  mixer.writeOutput(mix,values);
  TEST_ASSERT_EQUAL(1250,model.state.output.us[0]);
  settings[7] = 101;
  TEST_ASSERT_EQUAL(-1, command(model, MSP_SET_SERVO_CONFIGURATION, settings).result);
  TEST_ASSERT_EQUAL(-50,model.config.servoRouting[0].rate);
}

static void test_gps_nmea_sentence_and_signal_loss()
{
  clocks();
  Model model;
  const std::vector<uint8_t> config = {GPS_NMEA,0,0,0,1,0};
  TEST_ASSERT_EQUAL(1,command(model,MSP_SET_GPS_CONFIG,config).result);
  auto reply = command(model,MSP_GPS_CONFIG);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(config.data(),reply.data,6);
  TestPort port;
  Sensor::GpsSensor receiver(model);
  receiver.begin(&port,9600);
  const char sentence[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
  port.bytes.assign(sentence,sentence+sizeof(sentence)-1);
  nowUs += 100000;
  receiver.update();
  TEST_ASSERT_TRUE(model.state.gps.fix);
  TEST_ASSERT_EQUAL(8,model.state.gps.numSats);
  TEST_ASSERT_INT32_WITHIN(1,481173000,model.state.gps.location.raw.lat);
  TEST_ASSERT_INT32_WITHIN(1,115166667,model.state.gps.location.raw.lon);
  TEST_ASSERT_EQUAL(545400,model.state.gps.location.raw.height);
  const auto timestamp = model.state.gps.lastMsgTs;
  port.index = 0;
  port.bytes[port.bytes.size()-3] = '0';
  nowUs += 100000;
  receiver.update();
  TEST_ASSERT_EQUAL(timestamp,model.state.gps.lastMsgTs);
  TEST_ASSERT_TRUE(port.written.empty());
  Sensor::GpsSensor::expireSolution(model.state.gps,timestamp+2500001);
  TEST_ASSERT_FALSE(model.state.gps.present);
  TEST_ASSERT_FALSE(model.state.gps.fix);
}

static void test_gps_msp_feed_selection_and_units()
{
  clocks();
  Model model;
  model.config.featureMask |= FEATURE_GPS;
  std::vector<uint8_t> legacy = {1,9,0,0,0,0,0,0,0,0,100,0,250,0};
  TEST_ASSERT_EQUAL(-1,command(model,MSP_SET_RAW_GPS,legacy).result);
  TEST_ASSERT_EQUAL(1,command(model,MSP_SET_GPS_CONFIG,{GPS_MSP,0,0,0,1,0}).result);
  TEST_ASSERT_EQUAL(1,command(model,MSP_SET_RAW_GPS,legacy).result);
  TEST_ASSERT_EQUAL(100000,model.state.gps.location.raw.height);
  TEST_ASSERT_EQUAL(2500,model.state.gps.velocity.raw.groundSpeed);
  std::vector<uint8_t> extended(52);
  extended[7] = 3;
  extended[8] = 10;
  extended[29] = 30;
  extended[33] = 40;
  extended[41] = 0x28;
  extended[42] = 0x23; // 90 degrees
  TEST_ASSERT_EQUAL(1,command(model,MSP2_SENSOR_GPS,extended).result);
  TEST_ASSERT_EQUAL(500,model.state.gps.velocity.raw.groundSpeed);
  TEST_ASSERT_EQUAL(9000000,model.state.gps.velocity.raw.heading);
  TEST_ASSERT_TRUE(model.state.gps.fix);
}

static void test_ltm_frames_have_live_values_and_xor_checksums()
{
  clocks();
  Model model;
  model.state.battery.voltage = 12.34f;
  model.config.ibat.source = CURRENT_METER_MSP;
  model.state.battery.mspMahDrawn = 1000;
  model.updateModes((1u << MODE_ARMED) | (1u << MODE_ANGLE));
  Telemetry::TelemetryLtm ltm;
  TestPort port;
  ltm.update(model,port);
  const auto& bytes = port.written;
  TEST_ASSERT_EQUAL(39,bytes.size());
  TEST_ASSERT_EQUAL('A',bytes[2]);
  TEST_ASSERT_EQUAL('S',bytes[12]);
  TEST_ASSERT_EQUAL('O',bytes[23]);
  TEST_ASSERT_EQUAL(12340,bytes[13] | bytes[14]<<8);
  TEST_ASSERT_EQUAL(1000,bytes[15] | bytes[16]<<8);
  TEST_ASSERT_EQUAL(9,bytes[19]);
  uint8_t checksum = 0;
  for (size_t i = 13; i < 20; ++i) checksum ^= bytes[i];
  TEST_ASSERT_EQUAL(checksum,bytes[20]);
  ltm.update(model,port);
  TEST_ASSERT_EQUAL(39,bytes.size());
  model.state.gps.present = true;
  nowUs += 100000;
  ltm.update(model,port);
  TEST_ASSERT_EQUAL('G',bytes[51]);
}

static void test_rangefinder_tf_frames_quality_recovery_and_disconnect()
{
  clocks();
  for (auto source : {RANGE_TFMINI, RANGE_TF02, RANGE_TFNOVA})
  {
    Model model;
    const std::vector<uint8_t> options = {1,1,1,static_cast<uint8_t>(source),0};
    TEST_ASSERT_EQUAL(1, command(model, MSP_SET_SENSOR_CONFIG, options).result);
    auto config = command(model, MSP_SENSOR_CONFIG);
    TEST_ASSERT_EQUAL(source, config.data[3]);
    Sensor::SurfaceSensor sensor(model);
    TestPort port;
    sensor.beginSerial(&port);
    TEST_ASSERT_EQUAL(source == RANGE_TFNOVA ? 0 : 8, port.written.size());
    std::vector<uint8_t> frame = {0x59,0x59,0xc8,0,0xff,0,7,95,0};
    for (size_t i = 0; i < 8; ++i) frame[8] += frame[i];
    port.bytes = {0x59,0x59,0x12}; // incomplete packet immediately followed by a new header
    port.bytes.insert(port.bytes.end(),frame.begin(),frame.end());
    sensor.update(nowUs);
    TEST_ASSERT_EQUAL(source, command(model, MSP2_SENSOR_CONFIG_ACTIVE).data[4]);
    TEST_ASSERT_EQUAL(source == RANGE_TFMINI ? 187 : 200, command(model, MSP_SONAR_ALTITUDE).data[0]);
    frame[6] = 6;
    frame[7] = 70;
    frame[8] -= 26;
    port.bytes.insert(port.bytes.end(),frame.begin(),frame.end());
    sensor.update(nowUs);
    TEST_ASSERT_EQUAL(source == RANGE_TFMINI ? 200 : 255, command(model, MSP_SONAR_ALTITUDE).data[0]);
    nowUs += 210000;
    sensor.update(nowUs);
    TEST_ASSERT_EQUAL(255,command(model, MSP_SONAR_ALTITUDE).data[0]);
    TEST_ASSERT_FALSE(Sensor::SurfaceSensor::rangeActive(model));
  }
  TEST_ASSERT_EQUAL(1000,Sensor::SurfaceSensor::echoDistanceMm(5800));
  TEST_ASSERT_EQUAL(-1,Sensor::SurfaceSensor::echoDistanceMm(100));
  TEST_ASSERT_EQUAL(-1,Sensor::SurfaceSensor::echoDistanceMm(24000));
}

static void test_mt_sensor_selection_range_limits_flow_quality_and_timeout()
{
  clocks();
  Model model;
  const std::vector<uint8_t> range = {200,0xb8,0x0b,0,0}; // 3000 mm
  const std::vector<uint8_t> flow = {204,5,0,0,0,0xfd,0xff,0xff,0xff};
  TEST_ASSERT_EQUAL(-1,command(model,MSP2_SENSOR_RANGEFINDER_LIDARMT,range).result);
  for (auto type : {RANGE_MTF01,RANGE_MTF02,RANGE_MTF01P,RANGE_MTF02P})
  {
    TEST_ASSERT_EQUAL(1,command(model,MSP_SET_SENSOR_CONFIG,{1,1,1,static_cast<uint8_t>(type),1}).result);
    TEST_ASSERT_EQUAL(1,command(model,MSP2_SENSOR_RANGEFINDER_LIDARMT,range).result);
    TEST_ASSERT_EQUAL(type == RANGE_MTF02 ? -1 : 3000, model.state.surface.distanceMm);
    TEST_ASSERT_EQUAL(1,command(model,MSP2_SENSOR_OPTICALFLOW_MT,flow).result);
    TEST_ASSERT_EQUAL(type == RANGE_MTF02 ? 0 : 80,model.state.surface.flowQuality);
    TEST_ASSERT_EQUAL(-3,model.state.surface.flowY);
    TEST_ASSERT_EQUAL(1,command(model,MSP2_SENSOR_CONFIG_ACTIVE).data[5]);
    nowUs += 220000;
    TEST_ASSERT_FALSE(Sensor::SurfaceSensor::flowActive(model));
    TEST_ASSERT_EQUAL(1,command(model,MSP2_SENSOR_OPTICALFLOW_MT,flow).result);
    TEST_ASSERT_EQUAL(0,model.state.surface.flowQuality);
  }
  TEST_ASSERT_EQUAL(-1,command(model,MSP_SET_SENSOR_CONFIG,{1,1,1,9,1}).result);
  TEST_ASSERT_EQUAL(RANGE_MTF02P, model.config.surface.rangefinder);
}

static void test_smartaudio_zero_power_and_until_first_arm()
{
  clocks();
  Model model;
  TestPort port;
  Connect::Vtx vtx(model);
  vtx.begin(&port);
  vtx.update();
  TEST_ASSERT_EQUAL(7,port.written.size());
  nowUs += 300000;
  vtx.update();
  TEST_ASSERT_EQUAL(7,port.written.size());
  TEST_ASSERT_EQUAL(1,command(model,MSP_SET_VTX_CONFIG,{7,0,3,0,2}).result);
  nowUs += 300000;
  vtx.update();
  TEST_ASSERT_EQUAL(0,port.written.at(12));
  model.updateModes(1u << MODE_ARMED);
  nowUs += 300000;
  vtx.update();
  TEST_ASSERT_EQUAL(2,port.written.at(19));
  model.updateModes(0);
  nowUs += 300000;
  vtx.update();
  TEST_ASSERT_EQUAL(21,port.written.size());
}

static void test_ports_msp_versions_and_atomic_invalid_baud()
{
  clocks();
  Model model;
  model.config.featureMask |= FEATURE_SOFTSERIAL;
  const uint8_t portId = model.config.serial[0].id;
  for (bool v2 : {false,true})
  {
    const uint16_t get = v2 ? MSP2_COMMON_SERIAL_CONFIG : MSP_CF_SERIAL_CONFIG;
    const uint16_t set = v2 ? MSP2_COMMON_SET_SERIAL_CONFIG : MSP_SET_CF_SERIAL_CONFIG;
    std::vector<uint8_t> row = {portId, 2, 0};
    if (v2) row.insert(row.end(), {0,0});
    row.insert(row.end(), {5,4,2,10});
    std::vector<uint8_t> packet = row;
    if (v2) packet.insert(packet.begin(),1);
    TEST_ASSERT_EQUAL(1,command(model,set,packet).result);
    const auto readback = command(model,get);
    const size_t skip = v2 ? 1 : 0;
    TEST_ASSERT_GREATER_OR_EQUAL(skip + row.size(),readback.len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(row.data(),readback.data + skip,row.size());
    TEST_ASSERT_EQUAL(57600,model.config.serialSensorRates[0].gps);
    TEST_ASSERT_EQUAL(19200,model.config.serialSensorRates[0].telemetry);
    TEST_ASSERT_EQUAL(500000,model.config.serial[0].blackboxBaud);
    packet[skip+row.size()-4] = 1;
    packet.insert(packet.end(),row.begin(),row.end());
    if (v2) packet[0] = 2;
    packet.back() = 255;
    TEST_ASSERT_EQUAL(-1,command(model,set,packet).result);
    TEST_ASSERT_EQUAL(115200,model.config.serial[0].baud);
    packet.pop_back();
    TEST_ASSERT_EQUAL(-1,command(model,set,packet).result);
  }
}

static void test_manual_throttle_matches_saved_curve_and_preserves_failsafe()
{
  clocks();
  Model model;
  auto config = command(model,MSP_RC_TUNING);
  std::vector<uint8_t> settings(config.data,config.data+config.len);
  TEST_ASSERT_EQUAL(24,settings.size());
  settings[6] = 50;
  settings[7] = 100;
  settings[23] = 40;
  TEST_ASSERT_EQUAL(1,command(model,MSP_SET_RC_TUNING,settings).result);
  auto reply = command(model,MSP_RC_TUNING);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(settings.data(),reply.data,reply.len);
  Control::Controller controller(model);
  controller.begin();
  model.state.input.rxLoss = false;
  model.state.input.ch[AXIS_THRUST] = -0.5f;
  controller.outerLoop();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f,-0.4f,model.state.setpoint.rate[AXIS_THRUST]);
  model.state.input.rxLoss = true;
  controller.outerLoop();
  TEST_ASSERT_EQUAL_FLOAT(-0.5f,model.state.setpoint.rate[AXIS_THRUST]);
  settings[23] = 101;
  TEST_ASSERT_EQUAL(-1,command(model,MSP_SET_RC_TUNING,settings).result);
  TEST_ASSERT_EQUAL(40,model.config.throttleCurve.hover);
  settings.resize(23);
  settings[6] = 30;
  TEST_ASSERT_EQUAL(1,command(model,MSP_SET_RC_TUNING,settings).result);
  TEST_ASSERT_EQUAL(30,model.config.throttleCurve.hover);
  for (uint8_t mid : {0,100})
  {
    model.config.throttleCurve.mid = mid;
    TEST_ASSERT_EQUAL_FLOAT(-1.f,Control::Rates::manualThrottle(model.config.throttleCurve,-1.f));
    TEST_ASSERT_EQUAL_FLOAT(1.f,Control::Rates::manualThrottle(model.config.throttleCurve,1.f));
    TEST_ASSERT_TRUE(std::isfinite(Control::Rates::manualThrottle(model.config.throttleCurve,0.f)));
  }
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_manual_throttle_matches_saved_curve_and_preserves_failsafe);
  RUN_TEST(test_ports_msp_versions_and_atomic_invalid_baud);
  RUN_TEST(test_rangefinder_tf_frames_quality_recovery_and_disconnect);
  RUN_TEST(test_mt_sensor_selection_range_limits_flow_quality_and_timeout);
  RUN_TEST(test_smartaudio_zero_power_and_until_first_arm);
  RUN_TEST(test_gps_nmea_sentence_and_signal_loss);
  RUN_TEST(test_gps_msp_feed_selection_and_units);
  RUN_TEST(test_ltm_frames_have_live_values_and_xor_checksums);
  RUN_TEST(test_servo_tab_loads_and_writes_rate_forward_and_endpoints);
  RUN_TEST(test_sumd_drops_bad_crc_failsafe_and_recovers_after_partial_frame);
  RUN_TEST(test_sumh_channel_conversion_and_trailer);
  RUN_TEST(test_xbus_mode_b_and_rj01_nested_checksums);
  RUN_TEST(test_msp_receiver_selection_survives_sanitize_and_rejects_unknown);
  RUN_TEST(test_battery_wire_round_trip_alarm_and_atomic_bad_write);
  RUN_TEST(test_legacy_battery_packet_and_eeprom_migration);
  RUN_TEST(test_virtual_meter_calibration_and_elapsed_charge);
  RUN_TEST(test_esc_meter_uses_fresh_voltage_and_current_in_physical_units);
  return UNITY_END();
}
