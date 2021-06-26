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
    _out.write("<current-state/");
    _out.print(linear_abs);    _out.write('/');
    _out.print(rotary_abs);    _out.write('/');
    _out.print(linear_vel);    _out.write('/');
    _out.print(rotary_vel);    _out.write('/');
    _out.print(force_reading); _out.write('>');
    _out.println();
  }

private:
  Stream &_out;
};

#endif // MessageSender_h
