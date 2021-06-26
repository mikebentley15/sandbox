#ifndef MessageSender_h
#define MessageSender_h

/** Send formatted messages
 *
 * Each type of message has a corresponding function.  For example, to send the
 * <help-command/{name}/{description}> command, call the sendHelpCommand()
 * function giving the name and description fields.
 */
class MessageSender {
public:
  MessageSender(Stream &out) : _out(out) {}

  Stream& out() { return _out; }
  const Stream& out() const { return _out; }

  void set_binary(bool turn_on) { _is_binary = turn_on; }
  bool is_binary() const { return _is_binary; }

  template <typename A, typename B>
  void sendHelpCommand(const A &name, const B &description) {
    _out.write("<help-command/");
    _out.print(name);        _out.write('/');
    _out.print(description); _out.write('>');
    _out.println();
  }

  template <typename A, typename B>
  void sendSetting(const A &name, const B &value) {
    _out.write("<setting/");
    _out.print(name);  _out.write('/');
    _out.print(value); _out.write('>');
    _out.println();
  }

  void sendCurrentState(int32_t linear_abs,
                        int32_t rotary_abs,
                        int32_t linear_vel,
                        int32_t rotary_vel,
                        int32_t force_reading)
  {
    if (_is_binary) {
      _out.write("<Bs");
      this->send_binary_32bit(linear_abs);
      this->send_binary_32bit(rotary_abs);
      this->send_binary_32bit(linear_vel);
      this->send_binary_32bit(rotary_vel);
      this->send_binary_32bit(force_reading);
      _out.write('>');
    } else {
      _out.write("<current-state/");
      _out.print(linear_abs);    _out.write('/');
      _out.print(rotary_abs);    _out.write('/');
      _out.print(linear_vel);    _out.write('/');
      _out.print(rotary_vel);    _out.write('/');
      _out.print(force_reading); _out.write('>');
      _out.println();
    }
  }

  void sendForce(int32_t force_reading) {
    if (_is_binary) {
      _out.write("<Bf");
      this->send_binary_32bit(force_reading);
      _out.write('>');
    } else {
      _out.write("<force/");
      _out.print(force_reading);  _out.write('>');
      _out.println();
    }
  }

private:
  void send_binary_32bit(int32_t val) {
    _out.write(int8_t(val >> 24));
    _out.write(int8_t(val >> 16));
    _out.write(int8_t(val >> 8));
    _out.write(int8_t(val));
  }

  void send_binary_32bit(uint32_t val) {
    _out.write(uint8_t(val >> 24));
    _out.write(uint8_t(val >> 16));
    _out.write(uint8_t(val >> 8));
    _out.write(uint8_t(val));
  }

private:
  Stream &_out;
  bool _is_binary;
};

#endif // MessageSender_h
