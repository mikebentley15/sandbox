#include "MessageParser.h"
#include "serial_assert.h"

#include <string.h> // for strcmp() and memcpy()
#include <stdlib.h> // for strtoul()

bool MessageParser::append(char input) {
  bool finished_message = false;

  // state machine logic
  switch (this->_parse_state) {
    case ParseState::WAITING:
      serial_assert(this->_buffer.size() == 0,
          "MessageParser: internal error, in WAITING state when buffer is not"
          " empty");
      if (input == this->_msg_begin) {
        this->_parse_state = ParseState::MSG_BEGIN;
      }
      // else ignore the input and stay in the WAITING state
      break;

    // only in this state during _msg_begin character
    case ParseState::MSG_BEGIN: {
      serial_assert(this->_buffer.size() == 0,
          "MessageParser: internal error, in MSG_BEGIN state when buffer is"
          " not empty");
      this->_buffer.append(input);
      if (input == this->_msg_bin_id) {
        this->_parse_state = ParseState::PARSING_BINARY_MESSAGE;
      } else {
        this->_parse_state = ParseState::PARSING_TEXT_MESSAGE;
      }
      break;
    }

    case ParseState::PARSING_TEXT_MESSAGE: {
      if (input == this->_msg_end) {
        this->_parse_state = ParseState::WAITING;
        this->_buffer.append('\0');  // null termination
        this->parse_text(this->_buffer.data());
        this->_buffer.clear();
        finished_message = true;
      } else {
        // stay in the PARSING_TEXT_MESSAGE state
        this->_buffer.append(input);
      }
      break;
    }

    case ParseState::PARSING_BINARY_MESSAGE: {
      if (this->_buffer.size() <= 1) {
        serial_assert(this->_buffer.size() == 1,
            "MessageParser: parse error, in PARSING_BINARY_MESSAGE with empty"
            " buffer");
        this->_buffer.append(input);
        // stay in the PARSING_BINARY_MESSAGE state
        break;
      }

      auto payload = this->payload_size(this->_buffer[1]);
      if (payload < 0) { // unsupported binary command
#       ifndef NDEBUG
        Serial.print("MessageParser: Warning: unrecognized binary command: '");
        Serial.print(_buffer[1]);
        Serial.print("'");
        Serial.println();
#       endif
        this->_parse_state = ParseState::WAITING;
        this->_buffer.clear();
        break;
      }

      auto remaining = _buffer.size() - 2 - payload;
      if (remaining > 0) {
        this->_buffer.append(input);
        // stay in the PARSING_BINARY_MESSAGE state
        break;
      }

      serial_assert(remaining == 0,
          "MessageParser: parse error, binary message went too far");
      serial_assert(input == '>',
          "MessageParser: parse error, binary message did not end in '>'");
      this->_parse_state = ParseState::WAITING;
      // an extra guard in case we missed some of the message
      if (input == '>') {
        this->parse_binary(this->_buffer.data());
      }
      this->_buffer.clear();
      break;
    }

    default: {
      serial_assert(false,
          "MessageParser: parse error: unsupported parse state encountered!");
    }
  }

  return finished_message;
}

int MessageParser::payload_size(char message_type) const {
  switch (message_type) {
    case 'A': return  8;
    case 'B': return  8;
    case 'C': return  4;
    case 'D': return  8;
    case 'E': return  8;
    case 'F': return  4;
    default:  return -1;  // signal that it's an unsupported message type
  }
}

void MessageParser::parse_text(const char *data) {
  if (data == nullptr) { return; }

  // basically, we know how to parse by the callback type
       if (parse_text_cmd(data, "help"            , _help_cb            )) {}
  else if (parse_text_cmd(data, "settings"        , _settings_cb        )) {}
  else if (parse_text_cmd(data, "state"           , _state_cb           )) {}
  else if (parse_text_cmd(data, "send-binary"     , _send_binary_cb     )) {}
  else if (parse_text_cmd(data, "stream-force"    , _stream_force_cb    )) {}
  else if (parse_text_cmd(data, "stream-state-on" , _stream_state_on_cb )) {}
  else if (parse_text_cmd(data, "stream-state-off", _stream_state_off_cb)) {}
  else if (parse_text_cmd(data, "tare"            , _tare_cb            )) {}
  else if (parse_text_cmd(data, "linear-abs"      , _linear_abs_cb      )) {}
  else if (parse_text_cmd(data, "linear-rel"      , _linear_rel_cb      )) {}
  else if (parse_text_cmd(data, "linear-velocity" , _linear_velocity_cb )) {}
  else if (parse_text_cmd(data, "rotary-abs"      , _rotary_abs_cb      )) {}
  else if (parse_text_cmd(data, "rotary-rel"      , _rotary_rel_cb      )) {}
  else if (parse_text_cmd(data, "rotary-velocity" , _rotary_velocity_cb )) {}
  else {
#   ifndef NDEBUG
    Serial.print("MessageParser: Warning: unrecognized text command: <");
    Serial.print(data);
    Serial.print(">");
    Serial.println();
#   endif
  }
}

void MessageParser::parse_binary(const char *data) {
  // we're given the full message, so data should start with B then the binary
  // type
  // we know how to parse based on the type of the callback
  auto bin = data + 2;
  switch (data[1]) {
    case 'A': parse_bin_cmd(bin, _linear_abs_cb     ); break;
    case 'B': parse_bin_cmd(bin, _linear_rel_cb     ); break;
    case 'C': parse_bin_cmd(bin, _linear_velocity_cb); break;
    case 'D': parse_bin_cmd(bin, _rotary_abs_cb     ); break;
    case 'E': parse_bin_cmd(bin, _rotary_rel_cb     ); break;
    case 'F': parse_bin_cmd(bin, _rotary_velocity_cb); break;
    default: {
#     ifndef NDEBUG
      Serial.print("MessageParser: Warning: unrecognized binary command: '");
      Serial.print(data[1]);
      Serial.print("'");
      Serial.println();
#     endif
      break;
    }
  }
}
