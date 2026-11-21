#pragma once

#include "Model.h"

namespace Espfc::Connect {

class Cli
{
public:
  enum ParamType : uint8_t
  {
    PARAM_NONE,           // unused
    PARAM_BOOL,           // boolean
    PARAM_BYTE,           // 8 bit int
    PARAM_BYTE_U,         // 8 bit uint
    PARAM_SHORT,          // 16 bit int
    PARAM_INT,            // 32 bit int
    PARAM_FLOAT,          // 32 bit float
    PARAM_INPUT_CHANNEL,  //  input channel config
    PARAM_OUTPUT_CHANNEL, // output channel config
    PARAM_SCALER,         // scaler config
    PARAM_MODE,           // scaler config
    PARAM_MIXER,          // mixer config
    PARAM_SERIAL,         // mixer config
    PARAM_STRING,         // string
    PARAM_BITMASK,        // set or clear bit
  };

  class Param
  {
  public:
    Param(): Param(nullptr, PARAM_NONE, nullptr, nullptr) {}
    Param(const Param&) = default;

    Param(const char* n, ParamType t, char* a, const char* const* c, uint8_t l = 16)
        : name(n), type(t), addr(a), choices(c), maxLen(l)
    {
    }

    Param(const char* n, bool* a): Param(n, PARAM_BOOL, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, int8_t* a): Param(n, PARAM_BYTE, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, uint8_t* a): Param(n, PARAM_BYTE_U, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, int16_t* a): Param(n, PARAM_SHORT, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, int32_t* a): Param(n, PARAM_INT, reinterpret_cast<char*>(a), nullptr) {}

    Param(const char* n, int8_t* a, const char* const* c): Param(n, PARAM_BYTE, reinterpret_cast<char*>(a), c) {}
    Param(
    const char* n,
    uint8_t* a,
    const char* const* c):
    Param(
        n,
        PARAM_BYTE_U,
        reinterpret_cast<char*>(a),
        c)
{
}
    Param(const char* n, int32_t* a, uint8_t b): Param(n, PARAM_BITMASK, reinterpret_cast<char*>(a), nullptr, b) {}

    Param(const char* n, InputChannelConfig* a): Param(n, PARAM_INPUT_CHANNEL, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, OutputChannelConfig* a): Param(n, PARAM_OUTPUT_CHANNEL, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, ScalerConfig* a): Param(n, PARAM_SCALER, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, ActuatorCondition* a): Param(n, PARAM_MODE, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, MixerEntry* a): Param(n, PARAM_MIXER, reinterpret_cast<char*>(a), nullptr) {}
    Param(const char* n, SerialPortConfig* a): Param(n, PARAM_SERIAL, reinterpret_cast<char*>(a), nullptr) {}

    void print(Stream::Printer& stream) const;
    void print(Stream::Printer& stream, const OutputChannelConfig& och) const;
    void print(Stream::Printer& stream, const InputChannelConfig& ich) const;
    void print(Stream::Printer& stream, const ScalerConfig& sc) const;
    void print(Stream::Printer& stream, const ActuatorCondition& ac) const;
    void print(Stream::Printer& stream, const MixerEntry& me) const;
    void print(Stream::Printer& stream, const SerialPortConfig& sc) const;
    void print(Stream::Printer& stream, int32_t v) const;

    void update(const char** args) const;

    void write(OutputChannelConfig& och, const char** args) const;
    void write(InputChannelConfig& ich, const char** args) const;
    void write(ScalerConfig& sc, const char** args) const;
    void write(ActuatorCondition& ac, const char** args) const;
    void write(MixerEntry& ac, const char** args) const;
    void write(SerialPortConfig& sc, const char** args) const;

    template<typename T>
    void write(const T v) const
    {
      *reinterpret_cast<T*>(addr) = v;
    }

    void write(const char* v) const;
    int32_t parse(const char* v) const;

    const char* name;
    ParamType type;
    char* addr;
    const char* const* choices;
    uint8_t maxLen;
  };

  // A shared table describes offsets in persistent configuration. Each CLI
  // binds an entry to its own model when the command is executed.
  struct Setting
  {
    const char* name;
    const char* const* choices;
    uint16_t offset;
    ParamType type;
    uint8_t maxLen;

    Param bind(ModelConfig& config) const
    {
      auto* address = reinterpret_cast<char*>(&config) + offset;
      constexpr size_t dMaxStart = offsetof(ModelConfig, dMaxProfiles);
      if (offset >= dMaxStart && offset < dMaxStart + sizeof(DMaxConfig))
        address = reinterpret_cast<char*>(&config.dMax()) + (offset - dMaxStart);
      return Param(name, type, address, choices, maxLen);
    }
  };

  Cli(Model& model);
  static const Setting* initialize(ModelConfig& c);
  bool process(const char c, CliCmd& cmd, Stream::Printer& stream);
  void parse(CliCmd& cmd);
  void execute(CliCmd& cmd, Stream::Printer& s);

#if !defined(UNIT_TEST)
private:
#endif
  void print(const Param& param, Stream::Printer& s) const;
  void printGpsStatus(Stream::Printer& s, bool full) const;
  void printVersion(Stream::Printer& s) const;
  void printStats(Stream::Printer& s) const;

  Model& _model;
  const Setting* _params;
  bool _ignore;
  bool _active;
  bool _interactive;
};

} // namespace Espfc::Connect
