#pragma once

#include "Model.h"

namespace Espfc::Sensor {

class SurfaceSensor
{
public:
  explicit SurfaceSensor(Model& model) : _model(model) {}
  void beginUltrasonic();
  void beginSerial(Stream::ReadWritable* port);
  void update(uint32_t now);
  static bool isMt(uint8_t type) { return type >= RANGE_MTF01 && type <= RANGE_MTF02P; }
  static bool rangeActive(const Model& model);
  static bool flowActive(const Model& model);
  static void recordRange(Model& model, uint8_t type, int32_t mm, uint32_t now);
  static void recordFlow(Model& model, uint8_t quality, int32_t x, int32_t y, uint32_t now);
  static int32_t echoDistanceMm(uint32_t widthUs);

private:
  void parse(uint8_t byte, uint32_t now);
  void configureSerial(uint32_t now);
  static void echoInterrupt(void* arg);
  Model& _model;
  Stream::ReadWritable* _port = nullptr;
  int8_t _trigger = -1;
  int8_t _echo = -1;
  uint32_t _lastTrigger = 0;
  volatile uint32_t _rise = 0;
  volatile bool _rising = false;
  volatile uint32_t _pulseWidth = 0;
  uint32_t _lastConfigure = 0;
  uint8_t _frame[9] = {};
  uint8_t _length = 0;
};

} // namespace Espfc::Sensor
