#include "MessageParser.h"
#include "serial_assert.h"

#include <string.h> // for strcmp()
#include <stdlib.h> // for strtoul()

bool MessageParser::append(char input) {
  (void)input; // unused
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

      auto payload = this->payload_size(_buffer[1]);
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
  (void)message_type; // unused
  switch (message_type) {
    default:
      return -1;  // signal that it's an unsupported message type
  }
}

void MessageParser::parse_text(const char *data) {
  if (data == nullptr) { return; }

  if (0 == strcmp(data, "help")) {
    if (this->_help_callback != nullptr) {
      this->_help_callback();
    }

  } else if (0 == strcmp(data, "settings")) {
    if (this->_settings_callback != nullptr) {
      this->_settings_callback();
    }

  } else if (0 == strcmp(data, "state")) {
    if (this->_state_callback != nullptr) {
      this->_state_callback();
    }

  } else if (0 == strncmp(data, "send-binary", 11)) {
    serial_assert(data[11] == '/', "MessageParser: parse error");
    if (this->_send_binary_callback != nullptr) {
      this->_send_binary_callback(this->parse_on_off(data + 12));
    }

  } else if (0 == strncmp(data, "stream-force", 12)) {
    serial_assert(data[12] == '/', "MessageParser: parse error");
    if (this->_stream_force_callback != nullptr) {
      this->_stream_force_callback(this->parse_on_off(data + 13));
    }

  } else if (0 == strncmp(data, "stream-state-on", 15)) {
    serial_assert(data[15] == '/', "MessageParser: parse error");
    uint32_t interval = strtoul(data + 16, nullptr, 10);
    if (this->_stream_state_on_callback != nullptr) {
      this->_stream_state_on_callback(interval);
    }

  } else if (0 == strcmp(data, "stream-state-off")) {
    if (this->_stream_state_off_callback != nullptr) {
      this->_stream_state_off_callback();
    }

  } else {
#   ifndef NDEBUG
    Serial.print("MessageParser: Warning: unrecognized text command: <");
    Serial.print(data);
    Serial.print(">");
    Serial.println();
#   endif
  }
}

void MessageParser::parse_binary(const char *data) {
  (void)data; // unused
}

bool MessageParser::parse_on_off(const char *data) {
  if (0 == strncmp(data, "on", 2)) {
    return true;
  } else if (0 == strncmp(data, "off", 3)) {
    return false;
  } else {
#   ifndef NDEBUG
    Serial.print("MessageParser: Warning: must be on or off: ");
    Serial.print(data);
    Serial.println();
#   endif
  }
  return false;
}
