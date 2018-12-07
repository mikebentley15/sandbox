#include \
  <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

enum class TokType {
  ERROR = 0,         // value that should never happen
  END_OF_FILE,
  OPERATOR,          // +=/?<>~!@$^&*,|[].:-
  IDENTIFIER,        // (a-zA-Z_) {(a-zA-Z0-9_)}.
  LITERAL,           // string, character, or number
  MACRO,             // "#" {anything} "\n"
  LPAREN,            // "("
  RPAREN,            // ")"
  LCURLY,            // "{"
  RCURLY,            // "}"
  SEMICOLON,         // ";"
};

std::string toktype_tostring(TokType type) {
  switch(type) {
    case TokType::ERROR:       return "ERROR";
    case TokType::END_OF_FILE: return "END_OF_FILE";
    case TokType::OPERATOR:    return "OPERATOR";
    case TokType::IDENTIFIER:  return "IDENTIFIER";
    case TokType::LITERAL:     return "LITERAL";
    case TokType::MACRO:       return "MACRO";
    case TokType::LPAREN:      return "LPAREN";
    case TokType::RPAREN:      return "RPAREN";
    case TokType::LCURLY:      return "LCURLY";
    case TokType::RCURLY:      return "RCURLY";
    case TokType::SEMICOLON:   return "SEMICOLON";
    default:
      throw std::runtime_error("Unrecognized TokType");
  }
}

struct Token {
  TokType type;
  std::string content;
};

class Lexer {
public:
  using Error = std::domain_error;

  Lexer(std::istream &in)
    : _in(in)
    , _ch('\0')
    , _prev('\0')
    , _tok()
    , _line(1)
    , _column(0)
    , _prevcolumn(0)
  {
    getchar();
    cleartok();
  }

  const Token& tok() { return _tok; }

  void next_tok() {
    while (eaten()) {}

    cleartok();

    if (!_in) {
      _tok.type = TokType::END_OF_FILE;
    } else if (_operator()) {
      _tok.type = TokType::OPERATOR;
    } else if (identifier()) {
      _tok.type = TokType::IDENTIFIER;
    } else if (literal()) {
      _tok.type = TokType::LITERAL;
    } else if (macro()) {
      _tok.type = TokType::MACRO;
    } else if (_ch == '(') {
      getchar();
      _tok.type = TokType::LPAREN;
    } else if (_ch == ')') {
      getchar();
      _tok.type = TokType::RPAREN;
    } else if (_ch == '{') {
      getchar();
      _tok.type = TokType::LCURLY;
    } else if (_ch == '}') {
      getchar();
      _tok.type = TokType::RCURLY;
    } else if (_ch == ';') {
      getchar();
      _tok.type = TokType::SEMICOLON;
    } else {
      error(std::string("Unrecognized character: '") + _ch + "'");
    }
  }

private:
  void error(const std::string &msg) {
    std::ostringstream accumulator;
    accumulator << "Lexer error on line " << _line
                << " and column " << _column << ": " << msg;
    throw Error(accumulator.str());
  }

  // returns false if there are no more characters.  Sets _ch
  bool getchar() {
    _prev = _ch;
    _tok.content.push_back(_ch);
    _in.get(_ch);

    if (_in && _ch == '\n') {
      _line++;
      _prevcolumn = _column;
      _column = 0;
    } else {
      _column++;
    }

    return bool(_in);
  }

  void ungetchar() {

    if (_prev == '\n') {
      _line--;
      _column = _prevcolumn;
    } else {
      _column--;
    }

    _ch = _prev;
    _tok.content.pop_back();
    _in.unget();
  }

  void cleartok() {
    _tok.content = "";
    _tok.type = TokType::ERROR;
  }

  bool eaten() {
    return whitespace() || comment();
  }

  bool whitespace() {
    bool is_whitespace = false;
    while (_in && (_ch == ' ' || _ch == '\t' || _ch == '\n')) {
      is_whitespace = true;
      getchar();
    }
    return is_whitespace;
  }

  bool comment() {
    return line_comment() || multi_comment();
  }

  bool line_comment() {
    if (_ch == '/') {
      if (getchar() && _ch == '/') {
        while (getchar() && _ch != '\n') {}
        getchar();
        return true;
      } else {
        ungetchar();
      }
    }
    return false;
  }

  bool multi_comment() {
    if (_ch == '/') {
      if (getchar() && _ch == '*') {
        while (getchar()) {
          if (_ch == '*' && getchar() && _ch == '/') {
            break;
          }
        }
        getchar();
        return true;
      } else {
        ungetchar();
      }
    }
    return false;
  }

  bool _operator() {
    const std::string ops = "+=/?<>~!@$^&*,|[].:-";
    if (ops.find(_ch) != std::string::npos) {
      getchar();
      return true;
    }
    return false;
  }
  
  bool identifier() {
    if (_ch == '_' || std::isalpha(_ch)) {
      while (getchar() && (_ch == '_' || std::isalnum(_ch))) {}
      return true;
    }
    return false;
  }
  
  bool literal() {
    return number() || string() || character();
  }

  bool number() {
    if (std::isdigit(_ch)) {
      while (getchar() && (_ch == '.' || std::isalnum(_ch))) {}
      return true;
    }
    return false;
  }

  bool string() {
    bool in_escape = false;
    if (_ch == '"') {
      while (getchar()) {
        if (in_escape) {
          // just eat whatever character came after '\\'
          in_escape = false;
          continue;
        } else if (_ch == '\\') {
          in_escape = true;
        } else if (_ch == '"') {
          break;
        }
      }
      if (in_escape || _ch != '"') {
        error("Reached end of file while parsing string");
      }
      getchar();
      return true;
    }
    return false;
  }

  bool character() {
    if (_ch == '\'') {
      getchar();

      if (_ch == '\'') {
        error("Empty character literal");
      }

      if (_ch == '\\') {
        // skip one character since we are escaping
        getchar();
      }

      getchar();  // eat the character

      if (!_in) {
        error("End of file reached while parsing character literal");
      } else if (_ch != '\'') {
        error("Expected character literal to end with \"'\"");
      }

      getchar();  // eat the last "'"

      return true;
    }
    return false;
  }

  bool macro() {
    if (_ch == '#') {
      while (_in) {
        // TODO: collapse whitespace
        while (_in && _ch != '\n') { getchar(); }
        if (_prev != '\\') {
          break;
        } else {
          getchar();
          // Take off the "\\\n" off the end
          _tok.content.pop_back();
          _tok.content.pop_back();
        }
      }
      return true;
    }
    return false;
  }

private:
  std::istream &_in;
  char _ch;
  char _prev;
  Token _tok;
  int _line;
  int _column;
  int _prevcolumn;
};

class Parser {
public:
  Parser(std::istream &in, std::ostream &out) : _scanner(in), _out(out) {}

  void parse() {
    _out << "\n";
  }

private:
  Lexer _scanner;
  std::ostream &_out;
};

void usage() {
  std::cout <<
    "Usage:\n"
    "  cpp-heir-parser [--help]\n"
    "  cpp-heir-parser [options] [--] <input-cpp> <output-cpp>\n"
    "\n"
    "Description:\n"
    "  Parses a C++ file and basically pretty-prints it so that it can\n"
    "  easily be parsed into a hierarchical structure.  Each independent\n"
    "  statement and macro will be condensed onto a single line.  Each\n"
    "  block will have a starting and ending line on the same level, and\n"
    "  each statement within will have a space indentation of one higher.\n"
    "\n"
    "Options:\n"
    "  -h, --help    Show this help and exit.\n"
    "  --            Finish with option parsing and treat all options as\n"
    "                filenames.\n"
    "  -L, --lexer-only\n"
    "                 Instead of the full parsing, output each token from\n"
    "                 the lexer as a separate line into the given output\n"
    "                 file.\n"
    "\n"
    "Positional Argument:\n"
    "  input-cpp     Input c++ filepath to be parsed.\n"
    "  output-cpp    Output c++ filepath to be generated.  Will be\n"
    "                overwritten.  Cannot be the same as the input-cpp.\n"
    "\n";
}

int main(int argCount, char* argList[]) {
  std::vector<std::string> args(argCount - 1);
  std::vector<std::string> remaining;
  std::transform(argList + 1, argList + argCount, args.begin(),
                 [](const char* str) { return std::string(str); });
  bool done_with_options = false;
  bool lexer_only = false;
  for (std::string arg : args) {
    if (done_with_options || arg[0] != '-') {
      remaining.emplace_back(arg);
    } else if (arg == "-h" || arg == "--help") {
      usage();
      return 0;
    } else if (arg == "-L" || arg == "--lexer-only") {
      lexer_only = true;
    } else if (arg == "--") {
      done_with_options = true;
    } else { // starts with '-'
      std::cerr << "Error: unrecognized option '" << arg << "'\n"
                << "  Use the --help option for more information\n";
      return 1;
    }
  }

  if (remaining.size() < 2) {
    std::cerr << "Error: not enough arguments\n"
                 "  Use the --help option for more information\n";
    return 1;
  }
  if (remaining.size() > 2) {
    std::cerr << "Error: too many arguments\n"
                 "  Use the --help option for more information\n";
    return 1;
  }

  std::string infile  = remaining[0];
  std::string outfile = remaining[1];

  std::ifstream in(infile);
  std::ofstream out(outfile);

  if (in.fail()) {
    std::cerr << "Error: could not open '" << infile << "' for reading\n";
    return 1;
  }
  if (out.fail()) {
    std::cerr << "Error: could not open '" << outfile << "' for writing\n";
    return 1;
  }

  try {
    if (lexer_only) {
      Lexer lex(in);
      const Token &tok = lex.tok();
      lex.next_tok();
      while (tok.type != TokType::ERROR && tok.type != TokType::END_OF_FILE) {
        out << "(" << toktype_tostring(tok.type)
            << ", \"" << tok.content << "\")\n";
        lex.next_tok();
      }
    } else {
      // TODO: run the parser
      throw std::runtime_error("Parser is not yet implemented");
    }
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
