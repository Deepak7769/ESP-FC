#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "Connect/MspProcessor.hpp"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include <EscDriver.h>
#include <Gps.hpp>
#include <Hal/Gpio.hpp>
#include <helper_3dmath.hpp>
#include <printf.h>
#include <unity.h>
#include <cstring>
#include <string>
#include <vector>

using namespace Espfc;
using namespace Espfc::Connect;

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }

#define MSP_V2_FLAG 0


class MspTestStream : public Espfc::Stream::ReadWritable
{
public:
  void begin(const Espfc::Hal::SerialDeviceConfig&) override {}
  void updateBaudRate(int) override {}
  int available() override { return 0; }
  int read() override { return -1; }
  size_t readMany(uint8_t*, size_t) override { return 0; }
  int peek() override { return -1; }
  size_t write(uint8_t c) override
  {
    tx.push_back(c);
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) override
  {
    tx.insert(tx.end(), data, data + len);
    return len;
  }
  int availableForWrite() override { return 1024; }
  void flush() override { tx.clear(); }
  bool isTxFifoEmpty() override { return true; }

  std::vector<uint8_t> tx;
};

static void appendU16(MspMessage& msg, uint16_t value)
{
  const uint8_t bytes[] = {
      static_cast<uint8_t>(value & 0xff),
      static_cast<uint8_t>((value >> 8) & 0xff)};
  msg.append(bytes, sizeof(bytes));
}

static void appendU32(MspMessage& msg, uint32_t value)
{
  const uint8_t bytes[] = {
      static_cast<uint8_t>(value & 0xff),
      static_cast<uint8_t>((value >> 8) & 0xff),
      static_cast<uint8_t>((value >> 16) & 0xff),
      static_cast<uint8_t>((value >> 24) & 0xff)};
  msg.append(bytes, sizeof(bytes));
}

static uint16_t readResponseU16(const MspResponse& response, size_t offset)
{
  return static_cast<uint16_t>(response.data[offset]) |
         (static_cast<uint16_t>(response.data[offset + 1]) << 8);
}


void test_msp_v1_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'M', '<'};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V1, msg.version);
}

void test_msp_v1_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'M', '<', 0, MSP_API_VERSION, 1};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(1, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v1_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'M', '<', 2, MSP_API_VERSION, 1, 2, 0};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(0, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'X', '<'};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V2, msg.version);
}

void test_msp_v2_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'X', '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 0, 0, 69};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(69, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = {'$', 'X', '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 2, 0, 1, 2, 102};
  for (size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(102, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}


void test_msp_compat_failsafe_roundtrip()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  MspMessage set;
  set.cmd = MSP_SET_FAILSAFE_CONFIG;
  const uint8_t prefix[] = {3, 9};
  set.append(prefix, sizeof(prefix));
  appendU16(set, 1234);
  const uint8_t killSwitch = 1;
  set.append(&killSwitch, 1);
  appendU16(set, 567);
  const uint8_t procedure = FAILSAFE_PROCEDURE_DROP;
  set.append(&procedure, 1);

  MspResponse setResponse;
  processor.processCommand(set, setResponse, stream);

  TEST_ASSERT_EQUAL_UINT8(3, model.config.failsafe.delay);
  TEST_ASSERT_EQUAL_UINT8(9, model.config.compat.failsafeOffDelay);
  TEST_ASSERT_EQUAL_UINT16(1234, model.config.compat.failsafeThrottle);
  TEST_ASSERT_EQUAL_UINT8(1, model.config.failsafe.killSwitch);
  TEST_ASSERT_EQUAL_UINT16(567, model.config.compat.failsafeThrottleLowDelay);
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PROCEDURE_DROP, model.config.failsafe.procedure);

  MspMessage get;
  get.cmd = MSP_FAILSAFE_CONFIG;
  MspResponse getResponse;
  processor.processCommand(get, getResponse, stream);

  TEST_ASSERT_EQUAL_UINT16(8, getResponse.len);
  TEST_ASSERT_EQUAL_UINT8(3, getResponse.data[0]);
  TEST_ASSERT_EQUAL_UINT8(9, getResponse.data[1]);
  TEST_ASSERT_EQUAL_UINT16(1234, readResponseU16(getResponse, 2));
  TEST_ASSERT_EQUAL_UINT8(1, getResponse.data[4]);
  TEST_ASSERT_EQUAL_UINT16(567, readResponseU16(getResponse, 5));
  TEST_ASSERT_EQUAL_UINT8(FAILSAFE_PROCEDURE_DROP, getResponse.data[7]);
}

void test_msp_3d_metadata_roundtrip_is_non_actuating()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  MspMessage set;
  set.cmd = MSP_SET_MOTOR_3D_CONFIG;
  appendU16(set, 1300);
  appendU16(set, 1700);
  appendU16(set, 1500);

  MspResponse setResponse;
  processor.processCommand(set, setResponse, stream);

  TEST_ASSERT_EQUAL_UINT16(1300, model.config.compat.deadband3dLow);
  TEST_ASSERT_EQUAL_UINT16(1700, model.config.compat.deadband3dHigh);
  TEST_ASSERT_EQUAL_UINT16(1500, model.config.compat.neutral3d);

  MspMessage get;
  get.cmd = MSP_MOTOR_3D_CONFIG;
  MspResponse getResponse;
  processor.processCommand(get, getResponse, stream);

  TEST_ASSERT_EQUAL_UINT16(6, getResponse.len);
  TEST_ASSERT_EQUAL_UINT16(1300, readResponseU16(getResponse, 0));
  TEST_ASSERT_EQUAL_UINT16(1700, readResponseU16(getResponse, 2));
  TEST_ASSERT_EQUAL_UINT16(1500, readResponseU16(getResponse, 4));
}

void test_msp_pid_advanced_reports_compat_metadata_without_mutating_pid()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  model.config.compat.vbatPidCompensation = 1;
  model.config.compat.feedForwardTransition = 42;
  model.config.compat.rateAccelLimit = 321;
  model.config.compat.yawRateAccelLimit = 654;
  model.config.compat.itermRotation = 1;
  model.config.compat.smartFeedForward = 1;
  model.config.compat.itermRelaxType = 2;
  model.config.compat.absControlGain = 7;
  model.config.compat.throttleBoost = 11;
  model.config.compat.acroTrainerAngleLimit = 33;
  model.config.compat.dMax[0] = 12;
  model.config.compat.dMax[1] = 13;
  model.config.compat.dMax[2] = 14;
  model.config.compat.dMaxGain = 15;
  model.config.compat.dMaxAdvance = 16;
  model.config.compat.integratedYaw = 1;
  model.config.compat.integratedYawRelax = 17;
  model.config.compat.autoProfileCellCount = 4;
  model.config.compat.idleMinRpm = 18;
  model.config.compat.ffAveraging = 19;
  model.config.compat.ffSmoothFactor = 20;
  model.config.compat.ffBoost = 21;
  model.config.compat.ffMaxRateLimit = 22;
  model.config.compat.ffJitterFactor = 23;
  model.config.compat.vbatSagCompensation = 24;
  model.config.compat.thrustLinearization = 25;

  const PidConfig rollBefore = model.config.pid[FC_PID_ROLL];
  const PidConfig pitchBefore = model.config.pid[FC_PID_PITCH];
  const PidConfig yawBefore = model.config.pid[FC_PID_YAW];

  MspMessage get;
  get.cmd = MSP_PID_ADVANCED;
  MspResponse response;
  processor.processCommand(get, response, stream);

  TEST_ASSERT_EQUAL_UINT16(61, response.len);
  TEST_ASSERT_EQUAL_UINT8(1, response.data[7]);
  TEST_ASSERT_EQUAL_UINT8(42, response.data[8]);
  TEST_ASSERT_EQUAL_UINT16(321, readResponseU16(response, 13));
  TEST_ASSERT_EQUAL_UINT16(654, readResponseU16(response, 15));
  TEST_ASSERT_EQUAL_UINT8(1, response.data[25]);
  TEST_ASSERT_EQUAL_UINT8(1, response.data[26]);
  TEST_ASSERT_EQUAL_UINT8(2, response.data[28]);
  TEST_ASSERT_EQUAL_UINT8(7, response.data[29]);
  TEST_ASSERT_EQUAL_UINT8(11, response.data[30]);
  TEST_ASSERT_EQUAL_UINT8(33, response.data[31]);
  TEST_ASSERT_EQUAL_UINT8(12, response.data[39]);
  TEST_ASSERT_EQUAL_UINT8(13, response.data[40]);
  TEST_ASSERT_EQUAL_UINT8(14, response.data[41]);
  TEST_ASSERT_EQUAL_UINT8(15, response.data[42]);
  TEST_ASSERT_EQUAL_UINT8(16, response.data[43]);
  TEST_ASSERT_EQUAL_UINT8(1, response.data[44]);
  TEST_ASSERT_EQUAL_UINT8(17, response.data[45]);
  TEST_ASSERT_EQUAL_UINT8(4, response.data[48]);
  TEST_ASSERT_EQUAL_UINT8(18, response.data[49]);
  TEST_ASSERT_EQUAL_UINT8(19, response.data[50]);
  TEST_ASSERT_EQUAL_UINT8(20, response.data[51]);
  TEST_ASSERT_EQUAL_UINT8(21, response.data[52]);
  TEST_ASSERT_EQUAL_UINT8(22, response.data[53]);
  TEST_ASSERT_EQUAL_UINT8(23, response.data[54]);
  TEST_ASSERT_EQUAL_UINT8(24, response.data[55]);
  TEST_ASSERT_EQUAL_UINT8(25, response.data[56]);

  TEST_ASSERT_EQUAL_UINT8(rollBefore.P, model.config.pid[FC_PID_ROLL].P);
  TEST_ASSERT_EQUAL_UINT8(rollBefore.I, model.config.pid[FC_PID_ROLL].I);
  TEST_ASSERT_EQUAL_UINT8(rollBefore.D, model.config.pid[FC_PID_ROLL].D);
  TEST_ASSERT_EQUAL_INT16(rollBefore.F, model.config.pid[FC_PID_ROLL].F);
  TEST_ASSERT_EQUAL_UINT8(pitchBefore.P, model.config.pid[FC_PID_PITCH].P);
  TEST_ASSERT_EQUAL_UINT8(pitchBefore.I, model.config.pid[FC_PID_PITCH].I);
  TEST_ASSERT_EQUAL_UINT8(pitchBefore.D, model.config.pid[FC_PID_PITCH].D);
  TEST_ASSERT_EQUAL_INT16(pitchBefore.F, model.config.pid[FC_PID_PITCH].F);
  TEST_ASSERT_EQUAL_UINT8(yawBefore.P, model.config.pid[FC_PID_YAW].P);
  TEST_ASSERT_EQUAL_UINT8(yawBefore.I, model.config.pid[FC_PID_YAW].I);
  TEST_ASSERT_EQUAL_UINT8(yawBefore.D, model.config.pid[FC_PID_YAW].D);
  TEST_ASSERT_EQUAL_INT16(yawBefore.F, model.config.pid[FC_PID_YAW].F);
}

void test_msp_rtc_is_software_metadata_only()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  MspMessage set;
  set.cmd = MSP_SET_RTC;
  appendU32(set, 0x12345678u);
  appendU16(set, 789);

  MspResponse response;
  processor.processCommand(set, response, stream);

  TEST_ASSERT_EQUAL_UINT32(0x12345678u, model.config.compat.rtcSeconds);
  TEST_ASSERT_EQUAL_UINT16(789, model.config.compat.rtcMillis);
  TEST_ASSERT_EQUAL_UINT8(1, model.config.compat.rtcValid);

  MspMessage get;
  get.cmd = MSP_RTC;

  MspResponse getResponse;
  processor.processCommand(
      get,
      getResponse,
      stream);

  TEST_ASSERT_EQUAL_UINT16(
      6,
      getResponse.len);

  const uint32_t seconds =
      static_cast<uint32_t>(getResponse.data[0]) |
      (static_cast<uint32_t>(getResponse.data[1]) << 8) |
      (static_cast<uint32_t>(getResponse.data[2]) << 16) |
      (static_cast<uint32_t>(getResponse.data[3]) << 24);

  TEST_ASSERT_EQUAL_UINT32(
      0x12345678u,
      seconds);

  TEST_ASSERT_EQUAL_UINT16(
      789,
      readResponseU16(
          getResponse,
          4));
}

void test_msp2_set_text_rejects_truncated_craft_name_without_erasing_it()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  std::strncpy(
      model.config.modelName,
      "KEEP",
      MODEL_NAME_LEN);

  MspMessage set;
  set.cmd = MSP2_SET_TEXT;

  const uint8_t data[] = {
      MSP2TEXT_CRAFT_NAME,
      5,
      'B',
      'A'};

  set.append(
      data,
      sizeof(data));

  MspResponse response;
  processor.processCommand(
      set,
      response,
      stream);

  TEST_ASSERT_EQUAL_INT8(
      -1,
      response.result);

  TEST_ASSERT_EQUAL_STRING(
      "KEEP",
      model.config.modelName);
}

void test_msp_shadow_modes_are_explicitly_named()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  MspMessage get;
  get.cmd = MSP_BOXNAMES;
  MspResponse response;
  processor.processCommand(get, response, stream);

  const std::string names(
      reinterpret_cast<const char*>(response.data),
      response.len);

  TEST_ASSERT_NOT_EQUAL(std::string::npos, names.find("HORIZON SHADOW"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, names.find("GPS RESCUE SHADOW"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, names.find("POSHOLD SHADOW"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, names.find("HEADFREE SHADOW"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, names.find("ACRO TRAINER SHADOW"));
  TEST_ASSERT_NOT_EQUAL(std::string::npos, names.find("WAYPOINT SHADOW"));
}


void test_msp_response_write_string_uses_full_response_capacity()
{
  MspResponse response;

  const std::string payload(
      200,
      'X');

  response.writeString(
      payload.c_str());

  TEST_ASSERT_EQUAL_UINT16(
      200,
      response.len);

  for (size_t i = 0; i < response.len; ++i)
  {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>('X'),
        response.data[i]);
  }
}


void test_msp_set_mode_range_rejects_invalid_shadow_or_link_ids()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  const ActuatorCondition before =
      model.config.conditions[0];

  MspMessage invalidMode;
  invalidMode.cmd =
      MSP_SET_MODE_RANGE;

  const uint8_t invalidModeData[] = {
      0,
      static_cast<uint8_t>(MODE_COUNT),
      0,
      12,
      20};

  invalidMode.append(
      invalidModeData,
      sizeof(invalidModeData));

  MspResponse invalidModeResponse;
  processor.processCommand(
      invalidMode,
      invalidModeResponse,
      stream);

  TEST_ASSERT_EQUAL_INT8(
      -1,
      invalidModeResponse.result);

  TEST_ASSERT_EQUAL_UINT8(
      before.id,
      model.config.conditions[0].id);

  MspMessage invalidLink;
  invalidLink.cmd =
      MSP_SET_MODE_RANGE;

  const uint8_t invalidLinkData[] = {
      0,
      static_cast<uint8_t>(MODE_HORIZON_SHADOW),
      0,
      12,
      20,
      0,
      static_cast<uint8_t>(MODE_COUNT)};

  invalidLink.append(
      invalidLinkData,
      sizeof(invalidLinkData));

  MspResponse invalidLinkResponse;
  processor.processCommand(
      invalidLink,
      invalidLinkResponse,
      stream);

  TEST_ASSERT_EQUAL_INT8(
      -1,
      invalidLinkResponse.result);

  TEST_ASSERT_EQUAL_UINT8(
      before.id,
      model.config.conditions[0].id);
}


void test_msp_box_names_and_ids_cover_every_declared_mode()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  MspMessage namesRequest;
  namesRequest.cmd =
      MSP_BOXNAMES;

  MspResponse namesResponse;
  processor.processCommand(
      namesRequest,
      namesResponse,
      stream);

  size_t nameCount =
      0;

  for (size_t i = 0;
       i < namesResponse.len;
       ++i)
  {
    if (namesResponse.data[i] ==
        static_cast<uint8_t>(';'))
    {
      ++nameCount;
    }
  }

  MspMessage idsRequest;
  idsRequest.cmd =
      MSP_BOXIDS;

  MspResponse idsResponse;
  processor.processCommand(
      idsRequest,
      idsResponse,
      stream);

  TEST_ASSERT_EQUAL_UINT32(
      MODE_COUNT,
      nameCount);

  TEST_ASSERT_EQUAL_UINT16(
      MODE_COUNT,
      idsResponse.len);

  for (size_t i = 0;
       i < idsResponse.len;
       ++i)
  {
    TEST_ASSERT_EQUAL_UINT8(
        i,
        idsResponse.data[i]);
  }
}


void test_msp2_text_build_metadata_is_read_only()
{
  Model model;
  MspProcessor processor(model);
  MspTestStream stream;

  MspMessage releaseGet;
  releaseGet.cmd =
      MSP2_GET_TEXT;

  const uint8_t releaseType =
      MSP2TEXT_RELEASENAME;

  releaseGet.append(
      &releaseType,
      1);

  MspResponse releaseResponse;
  processor.processCommand(
      releaseGet,
      releaseResponse,
      stream);

  TEST_ASSERT_EQUAL_UINT8(
      MSP2TEXT_RELEASENAME,
      releaseResponse.data[0]);

  TEST_ASSERT_GREATER_THAN_UINT8(
      0,
      releaseResponse.data[1]);

  MspMessage buildSet;
  buildSet.cmd =
      MSP2_SET_TEXT;

  const uint8_t buildPayload[] = {
      MSP2TEXT_BUILDKEY,
      3,
      'B',
      'A',
      'D'};

  buildSet.append(
      buildPayload,
      sizeof(buildPayload));

  MspResponse setResponse;
  processor.processCommand(
      buildSet,
      setResponse,
      stream);

  TEST_ASSERT_EQUAL_INT8(
      -1,
      setResponse.result);
}

int main(int argc, char** argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_msp_v1_parse_header);
  RUN_TEST(test_msp_v1_parse_no_payload);
  RUN_TEST(test_msp_v1_parse_payload);
  RUN_TEST(test_msp_v2_parse_header);
  RUN_TEST(test_msp_v2_parse_no_payload);
  RUN_TEST(test_msp_v2_parse_payload);

  RUN_TEST(test_msp_compat_failsafe_roundtrip);
  RUN_TEST(test_msp_3d_metadata_roundtrip_is_non_actuating);
  RUN_TEST(test_msp_pid_advanced_reports_compat_metadata_without_mutating_pid);
  RUN_TEST(test_msp_rtc_is_software_metadata_only);
  RUN_TEST(test_msp_shadow_modes_are_explicitly_named);
  RUN_TEST(test_msp_response_write_string_uses_full_response_capacity);
  RUN_TEST(test_msp_set_mode_range_rejects_invalid_shadow_or_link_ids);
  RUN_TEST(test_msp2_set_text_rejects_truncated_craft_name_without_erasing_it);
  RUN_TEST(test_msp_box_names_and_ids_cover_every_declared_mode);
  RUN_TEST(test_msp2_text_build_metadata_is_read_only);
  return UNITY_END();
}