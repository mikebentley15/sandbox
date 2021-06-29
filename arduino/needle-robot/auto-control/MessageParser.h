#ifndef MessageParser_h
#define MessageParser_h

#include "Array.h"
#include "serial_assert.h"

#include <string.h>
#include <stdlib.h>

/** Parses messages, typically from the serial port
 *
 * Parses messages by feeding it one character at a time from any source.  Once
 * a completed message is received, the appropriate callback will be called (if
 * set).
 *
 * For each received message type, there can be a single callback function
 * registered.  The callback function parameters will depend on the type of
 * message or command received.
 */
class MessageParser {
public:
  using Callback_noargs = void();
  using Callback_bool   = void(bool);
  using Callback_S      = void(int32_t);
  using Callback_U      = void(uint32_t);
  using Callback_SU     = void(int32_t, uint32_t);
  enum class OutputType { TEXT, BINARY };

public:
  /** Parse one more character.
   * 
   * Returns true if this was the end of an input message.
   */
  bool append(char input);

  // callbacks
  void setHelpCallback(Callback_noargs *cb)           { _help_cb             = cb; }
  void setSettingsCallback(Callback_noargs *cb)       { _settings_cb         = cb; }
  void setStateCallback(Callback_noargs *cb)          { _state_cb            = cb; }
  void setSendBinaryCallback(Callback_bool *cb)       { _send_binary_cb      = cb; }
  void setStreamForceCallback(Callback_bool *cb)      { _stream_force_cb     = cb; }
  void setStreamStateOnCallback(Callback_U *cb)       { _stream_state_on_cb  = cb; }
  void setStreamStateOffCallback(Callback_noargs *cb) { _stream_state_off_cb = cb; }
  void setTareCallback(Callback_noargs *cb)           { _tare_cb             = cb; }
  void setLinearVelocityCallback(Callback_S *cb)      { _linear_velocity_cb  = cb; }
  void setLinearAbsCallback(Callback_SU *cb)          { _linear_abs_cb       = cb; }
  void setLinearRelCallback(Callback_SU *cb)          { _linear_rel_cb       = cb; }
  void setRotaryVelocityCallback(Callback_S *cb)      { _rotary_velocity_cb  = cb; }
  void setRotaryAbsCallback(Callback_SU *cb)          { _rotary_abs_cb       = cb; }
  void setRotaryRelCallback(Callback_SU *cb)          { _rotary_rel_cb       = cb; }

private:
  enum class ParseState {
    WAITING,                // waiting for _msg_begin
    MSG_BEGIN,              // after seeing _msg_begin
    PARSING_TEXT_MESSAGE,   // after the second character, we will know if it's
    PARSING_BINARY_MESSAGE, // a text or binary message.
  };

  // size of payload for the given binary message type.
  int payload_size(char message_type) const;

  void parse_text(const char *data);
  void parse_binary(const char *data);

  bool parse_text_cmd(const char *data, const char *command,
                      Callback_noargs *cb);
  template <typename T>
  bool parse_text_cmd(const char *data, const char *command,
                      void(*cb) (T));
  template <typename A, typename B>
  bool parse_text_cmd(const char *data, const char *command,
                      void(*cb) (A, B));

  void parse_bin_cmd(const char *data, void(*cb)());
  template <typename A>
  void parse_bin_cmd(const char *data, void(*cb)(A));
  template <typename A, typename B>
  void parse_bin_cmd(const char *data, void(*cb)(A, B));

  template <typename T>
  T parse_text_arg(const char *data, char **end);
  template <typename T>
  T parse_bin_arg(const char *data, char **end);

private:
  // parsing internal variables
  Array<char, 100> _buffer;  // message buffer for current message, without
                             // start and end characters.
  const char _msg_begin   = '<';
  const char _msg_end     = '>';
  const char _msg_bin_id  = 'B';
  OutputType _output_type = OutputType::TEXT;
  ParseState _parse_state = ParseState::WAITING;

  // callbacks
  Callback_noargs *_help_cb             = nullptr;
  Callback_noargs *_settings_cb         = nullptr;
  Callback_noargs *_state_cb            = nullptr;
  Callback_bool   *_send_binary_cb      = nullptr;
  Callback_bool   *_stream_force_cb     = nullptr;
  Callback_U      *_stream_state_on_cb  = nullptr;
  Callback_noargs *_stream_state_off_cb = nullptr;
  Callback_noargs *_tare_cb             = nullptr;
  Callback_S      *_linear_velocity_cb  = nullptr;
  Callback_SU     *_linear_abs_cb       = nullptr;
  Callback_SU     *_linear_rel_cb       = nullptr;
  Callback_S      *_rotary_velocity_cb  = nullptr;
  Callback_SU     *_rotary_abs_cb       = nullptr;
  Callback_SU     *_rotary_rel_cb       = nullptr;
};

inline bool MessageParser::parse_text_cmd(
    const char *data, const char *command, Callback_noargs *cb)
{
  if (0 == strcmp(data, command)) {
    if (cb) { cb(); }
    return true;
  }
  return false;
}

template <typename T>
bool MessageParser::parse_text_cmd(
    const char *data, const char *command, void(*cb) (T))
{
  const size_t len = strlen(command);
  if (0 == strncmp(data, command, len)) {
    serial_assert(data[len] == '/',
        "MessageParser: parse error: command not followed by /");
    char *p_end;
    T val = this->parse_text_arg<T>(data + len + 1, &p_end);
    if (cb) { cb(val); }
    return true;
  }
  return false;
}

template <typename A, typename B>
bool MessageParser::parse_text_cmd(
    const char *data, const char *command, void(*cb) (A, B))
{
  const size_t len = strlen(command);
  if (0 == strncmp(data, command, len)) {
    serial_assert(data[len] == '/',
        "MessageParser: parse error: command not followed by /");
    const char *p = data + len + 1;
    char *p_end;
    A val_1 = this->parse_text_arg<A>(p, &p_end);
    p = p_end;
    serial_assert(*p == '/',
        "MessageParser: parse error: expected two arguments, only got one");
    B val_2 = this->parse_text_arg<B>(p + 1, &p_end);
    p = p_end;
    serial_assert(*p == '\0',
        "MessageParser: parse error: command longer than expected");
    if (cb) { cb(val_1, val_2); }
    return true;
  }
  return false;
}



template <>
bool MessageParser::parse_text_arg<bool>(const char *data, char **end) {
  bool val = false;
  if (0 == strncmp(data, "on", 2)) {
    val = true;
    *end = const_cast<char*>(data + 2);
  } else if (0 != strncmp(data, "off", 3)) {
    *end = const_cast<char*>(data + 3);
  } else {
    *end = const_cast<char*>(data);
#   ifndef NDEBUG
    Serial.print("MessageParser: Warning: must be on or off: ");
    Serial.print(data);
    Serial.println();
#   endif
  }
  return val;
}

template <>
int32_t MessageParser::parse_text_arg<int32_t>(const char *data, char **end) {
  return strtol(data, end, 10);
}


template <>
uint32_t MessageParser::parse_text_arg<uint32_t>(const char *data, char **end) {
  return strtoul(data, end, 10);
}

void MessageParser::parse_bin_cmd(const char *data, void(*cb)()) {
  (void)data; // unused
  if (cb) { cb(); }
}

template <typename A>
void MessageParser::parse_bin_cmd(const char *data, void(*cb)(A)) {
  if (cb) {
    char *end;
    A val = this->parse_bin_arg<A>(data, &end);
    cb(val);
  }
}

template <typename A, typename B>
void MessageParser::parse_bin_cmd(const char *data, void(*cb)(A, B)) {
  if (cb) {
    char *end;
    A val_1 = this->parse_bin_arg<A>(data, &end);
    B val_2 = this->parse_bin_arg<B>(end,  &end);
    cb(val_1, val_2);
  }
}

template <>
uint32_t MessageParser::parse_bin_arg<uint32_t>(const char *data, char **end) {
  *end = const_cast<char*>(data + 4);
  uint8_t copy[4];
  memcpy(copy, data, 4);
  uint32_t uval = (uint32_t(copy[0]) << 24) |
                  (uint32_t(copy[1]) << 16) |
                  (uint32_t(copy[2]) <<  8) |
                   uint32_t(copy[3]);
  return uval;
}

template <>
int32_t MessageParser::parse_bin_arg<int32_t>(const char *data, char **end) {
  uint32_t uval = this->parse_bin_arg<uint32_t>(data, end);
  int32_t val;
  // this assumes the binary representation of the sender is the same as the
  // binary representation on this platform for a signed integer.
  memcpy(&val, &uval, sizeof(uval));
  return val;
}

#endif // MessageParser_h
