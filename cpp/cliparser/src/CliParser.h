#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/** A simple command-line argument parser
 *
 * CliParser is a simple C++ class the can parser arguments.
 *
 * \code
 *   int main(int argc, char** argv) {
 *     CliParser parser;
 *     parser.add_flag("-h", "-help", "--help");
 *     parser.add_flag("-v", "--verbose");
 *     parser.add_argflag("-N");
 *     parser.add_argflag("-o", "--output");
 *     parser.add_positional("input");
 *     parser.set_required("input");
 *     parser.set_required("--output");
 *
 *     try {
 *       parser.parse();
 *     } catch (ParseError &ex) {
 *       std::cerr << "ParseError: " << ex.what() << std::endl;
 *       return 1;
 *     }
 *
 *     if (parser.has("-h")) {
 *       std::cout << parser.usage() << std::endl;
 *       return 0;
 *     }
 *
 *     bool verbose = parser.has("-v");
 *     int N = parser.get<int>("-N", 10); // use 10 as default
 *     auto output = parser["--output"];
 *     auto input = parser["input"];
 *     std::vector<std::string> remaining = parser.remaining();
 *
 *     // the rest of main...
 *   }
 * \endcode
 */
class CliParser {
public:
  using string = std::string;
  template <typename T> using vector = std::vector<T>;
  template <typename T> using set = std::unordered_set<T>;

  class ParseError : public std::invalid_argument {
    using std::invalid_argument::invalid_argument; // use the same constructor
  };

protected:
  struct Option {
    vector<string> variants;
    bool expects_arg;
    bool required;
    Option(vector<string> &&_variants, bool _expects_arg)
      : variants(std::move(_variants))
      , expects_arg(_expects_arg)
      , required(false)
    {}
  };

  struct PositionArg {
    string name;
    bool required;
    PositionArg(string _name, bool _required)
      : name(_name), required(_required) {}
  };

  using OpPtr = std::shared_ptr<Option>;
  using PosPtr = std::shared_ptr<PositionArg>;
  using OptionMap = std::unordered_map<string, OpPtr>;
  using ParseMap = std::unordered_map<string, string*>;

public:
  CliParser() {}
  CliParser(const CliParser& other) = delete;
  CliParser(CliParser &&other) = default;
  virtual ~CliParser() = default;

  const vector<string> &args() const { return _args; }
  const string &program_name() const { return _args.at(0); }
  const vector<string> &remaining() const { return _remaining; }

  /// flag is just like "--help" needing no additional argument
  template <typename ... Args> void add_flag(string flag, Args ... flags) {
    vector<string> all;
    add_flag(all, false, flag, flags ...);
  }

  /// argflag is a flag with an argument, e.g., "--outfile out.txt"
  template <typename ... Args> void add_argflag(string flag, Args ... flags) {
    vector<string> all;
    add_flag(all, true, flag, flags ...);
  }

  /// next positional argument that is not part of a flag
  void add_positional(string name) {
    if (_recognized.count(name) > 0) {
      throw std::invalid_argument("Already registered argument '" + name + "'");
    }
    _positional.emplace_back(std::make_shared<PositionArg>(name, false));
    _recognized.emplace(std::move(name));
  }

  /// set a flag or positional argument as required (all are optional by default)
  void set_required(const string &name) {
    // check flags
    if (_optionmap.find(name) != _optionmap.end()) {
      _optionmap[name]->required = true;
      return;
    }

    // check positional
    auto it = std::find_if(_positional.begin(), _positional.end(),
                           [&name](PosPtr p) { return p->name == name; });
    if (it != _positional.end()) {
      it->get()->required = true;
      return;
    }

    // unrecognized type
    throw std::invalid_argument("set_required(): Unrecognized option '" + name + "'");
  }

  /** return the string representation of the value for the parsed arg
   *
   * Throws a std::out_of_bounds exception if the flag or positional argument
   * was not found in parsing.
   *
   * This function is only valid after calling parse().
   *
   * For flags without arguments, the value will be the same as the flag name.
   * For flags that have multiple variants, you can use any of the variants.
   *
   * \code
   *   CliParser parser;
   *   parser.add_flag("-h", "--help");
   *   parser.add_argflag("-i", "--input");
   *   parser.add_positional("outfile");
   *   parser.set_required("outfile");
   *   parser.parse({"program-name", "-h", "--input", "file.txt", "out.txt"});
   *   std::cout << parser["-i"] << std::endl;
   *   std::cout << parser["--input"] << std::endl;
   *   std::cout << parser["-h"] << std::endl;
   *   std::cout << parser["--help"] << std::endl;
   *   std::cout << parser["outfile"] << std::endl;
   * \endcode
   * This will print
   *
   * \code
   *   file.txt
   *   file.txt
   *   -h
   *   --help
   *   out.txt
   * \endcode
   *
   * It is better for non-argument flags to call has() instead.  Also, it is
   * best to call has() before calling operator[]() for non-required flags
   * because it throws if the flag wasn't there, or you can use get() with a
   * default value when it is missing.
   */
  const string &operator[](const string &name) const {
    auto it = _parsed.find(name);
    if (it != _parsed.end()) {
      return *(it->second);
    }

    // check to see if we recognize name
    // TODO: error check to see if it is a flag
    if (_optionmap.find(name) != _optionmap.end()) {
      throw std::out_of_range("Not found in parsing: '" + name + "'");
    }

    // check positional args
    for (auto &p : _positional) {
      if (p->name == name) {
        throw std::out_of_range("Not found in parsing: '" + name + "'");
      }
    }

    throw std::invalid_argument("Unrecognized argument name: '" + name + "'");
  }

  /// returns true if the flag or positional argument was found in parse()
  bool has(const string &name) const {
    if (_parsed.find(name) != _parsed.end()) {
      return true;
    }

    // TODO: error check to see if it is a flag
    if (_optionmap.find(name) != _optionmap.end()) {
      return false;
    }

    // check positional args
    for (auto &p : _positional) {
      if (p->name == name) {
        return false;
      }
    }

    // otherwise, it is unrecognized
    throw std::invalid_argument("has(): unrecognized argument '" + name + "'");
  }

  /** returns as a specific type the value obtained from operator[]().
   *  an optional default value can be given if the argument was not seen.
   */
  template <typename T> T get(const string &name) const;
  //template <typename T> T get(const string &name, T defaultval = T()) const;

  std::string usage() { return ""; }

  /// parse command-line options
  void parse(int argc, const char * const * argv) {
    parse(vector<string>(argv, argv+argc));
  }

  void parse(vector<string> args) {
    _args = std::move(args);
    _parsed.clear();
    _remaining.clear();

    int pos = 0;
    for (auto it = _args.begin() + 1; it != _args.end(); it++) {
      // first check known flags
      auto opit = _optionmap.find(*it);
      if (opit != _optionmap.end()) {
        OpPtr &op = opit->second;
        if (op->expects_arg) {
          // TODO: check for the end of the argument list
          it++;
        }
        for (auto &name : op->variants) {
          _parsed[name] = &(*it);
        }
      }
      // then check positional arguments
      else if (pos < _positional.size()) {
        _parsed[_positional[pos]->name] = &(*it);
        pos++;
      }
    }
  }

protected:

  void add_flag(vector<string> &variants, bool hasarg, string flag) {
    variants.emplace_back(std::move(flag));
    OpPtr op = std::make_shared<Option>(std::move(variants), hasarg);
    for (auto &name : op->variants) {
      if (_recognized.count(name) > 0) {
        throw std::invalid_argument("Already registered argument '" + name + "'");
      }
    }
    for (auto &name : op->variants) {
      _recognized.emplace(name);
      _optionmap[name] = op;
    }
  }

  template <typename ... Args>
  void add_flag(vector<string> &variants, bool hasarg, string flag, Args ... flags) {
    variants.emplace_back(std::move(flag));
    add_flag(variants, hasarg, flags ...);
  }

protected:
  vector<string> _args;
  vector<PosPtr> _positional;
  set<string> _recognized;
  OptionMap _optionmap;
  ParseMap _parsed;
  vector<string> _remaining;
};

template <> inline
int CliParser::get<int>(const std::string &name) const {
  return std::stoi(operator[](name));
}

template <> inline
long CliParser::get<long>(const std::string &name) const {
  return std::stol(operator[](name));
}

template <> inline
long long CliParser::get<long long>(const std::string &name) const {
  return std::stoll(operator[](name));
}

template <> inline
unsigned long CliParser::get<unsigned long>(const std::string &name) const {
  return std::stoul(operator[](name));
}

template <> inline
unsigned long long CliParser::get<unsigned long long>(const std::string &name) const {
  return std::stoull(operator[](name));
}

template <> inline
float CliParser::get<float>(const std::string &name) const {
  return std::stof(operator[](name));
}

template <> inline
double CliParser::get<double>(const std::string &name) const {
  return std::stod(operator[](name));
}

template <> inline
long double CliParser::get<long double>(const std::string &name) const {
  return std::stold(operator[](name));
}

template <> inline
std::string CliParser::get<std::string>(const std::string &name) const {
  return operator[](name);
}

#endif // CLIPARSER_H
