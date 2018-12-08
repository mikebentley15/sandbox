#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

std::string parse_string(const std::string);

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
  class Error : public std::domain_error {
    using std::domain_error::domain_error;
  };

  Lexer(std::istream &in)
    : _in(in), _ch('\0'), _prev('\0'), _tok(), _line(1), _column(0),
      _prevcolumn(0)
  {
    getchar();
    cleartok();
  }

  const Token& tok() { return _tok; }
  int line() { return _line; }
  int column() { return _column; }

  bool next_tok() {
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
    return _tok.type != TokType::ERROR && _tok.type != TokType::END_OF_FILE;
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
    const std::string ops = "+=/?<>~!@$%^&*,|[].:-";
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

      while (_in && _ch != '\'') {
        if (_ch == '\\') {
          // skip one character since we are escaping
          getchar();
        }
        getchar();
      }

      if (!_in) {
        error("End of file reached while scanning character literal");
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
      // collapse whitespace and remove comments
      std::string content(_tok.content.begin() + 1, _tok.content.end());
      _tok.content = "#" + parse_string(content);
      //std::string new_content = "#";
      //const Token &subtok = lex.tok();
      //while (lex.next_tok()) {
      //  new_content += subtok.content + " ";
      //}
      //new_content.pop_back(); // remove trailing space
      //_tok.content = new_content;
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
  class Error : public std::domain_error {
    using std::domain_error::domain_error;
  };

  class FinishedException : public std::runtime_error {
    using std::runtime_error::runtime_error;
  };

  Parser(std::istream &in, std::ostream &out)
    : _scanner(in), _out(out), _tok(_scanner.tok())
    , _prevtype(TokType::ERROR), _indent("")
  {}

  void parse() {
    // initialize the scanner
    next_tok();

    // parse
    // each subfunction, if accepting the token, handles it (by outputting to
    // _out), optionally handles more tokens, then calls _scanner.next_tok(),
    // followed by returning true to say that the token was handled.
    try {
      while (good() && element()) {}
    } catch (const FinishedException &ex) {
      // we're done.  End with a newline
      _out << std::endl;
    }

    // it's bad if we exited because something did not parse as an element
    // rather than reaching the end of the file.
    if (good()) {
      error("Expected element");
    }
  }

private:
  void error(std::string msg) {
    std::ostringstream accumulator;
    accumulator << "Parser error on line " << _scanner.line()
                << " and column " << _scanner.column() << ": " << msg;
    throw Error(accumulator.str());
  }

  bool eof() {
    return _tok.type == TokType::END_OF_FILE;
  }

  // means there is a valid token.  When false, done parsing
  bool good() {
    return _tok.type != TokType::ERROR && !eof();
  }

  void next_tok() {
    _prevtype = _tok.type;
    _scanner.next_tok();
    if (eof()) {
      throw FinishedException("We're done with tokens");
    }
    if (!good()) {
      error("Received an ERROR token from the scanner");
    }
  }

  bool element() {
    // indentation is handled individually in each of the following subgroups
    return label() || macro() || statementblock();
  }

  bool label() {
    std::vector<std::string> names {"public", "protected", "private"};
    if (_tok.type == TokType::IDENTIFIER &&
        std::find(names.begin(), names.end(), _tok.content) != names.end())
    {
      _out << _indent << _tok.content;
      next_tok();
      if (_tok.type == TokType::OPERATOR && _tok.content == ":") {
        _out << ":";
        next_tok();
        _out << std::endl;
      } else {
        error("Expected colon after label name, got " + _tok.content);
      }
      // pretend it was a statement rather than a label, for spacing reasons.
      _prevtype = TokType::SEMICOLON;
      return true;
    }
    return false;
  }

  bool macro() {
    if (_tok.type == TokType::MACRO) {
      // do not indent macros
      _out << _tok.content;
      next_tok();
      // The macro content already ends in a newline
      //_out << std::endl;
      return true;
    }
    return false;
  }

  bool statementblock() {
    if (_tok.type == TokType::IDENTIFIER && _tok.content == "template") {
      _out << _indent;
      while (_tok.content != "class" && _tok.content != "struct") {
        if (!pstatement() && !piece()) {
          break;
        }
      }
      _out << " ";
      if (_class()) {
        // nothing to do
      } else if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
        _out << std::endl;
      } else if (block()) {
        _out << std::endl;
      } else {
        error("Expected class, struct, block, or semicolon after template");
      }
      return true;
    } else if (_tok.content == "class" ||
               _tok.content == "struct" ||
               _tok.content == "union")
    {
      _out << _indent;
      if (_class()) {
        // nothing to do
      } else {
        error("Expected valid class, struct, or union");
      }
      return true;
    } else if (_tok.content == "enum") {
      _out << _indent;
      if (_enum()) {
        // nothing to do
      } else {
        error("Expected valid enum");
      }
      return true;
    } else if (_tok.type == TokType::LCURLY) {
      _out << _indent;
      if (block()) {
        _out << std::endl;
      } else {
        error("Expected block after left curly brace in statementblock");
      }
      return true;
      _out << std::endl;
      return true;
    } else if (_typedef()) {
      return true;
    } else if (_tok.type == TokType::LPAREN ||
               _tok.type == TokType::LITERAL ||
               _tok.type == TokType::IDENTIFIER ||
               _tok.type == TokType::OPERATOR)
    {
      _out << _indent;
      if (!statement_inner()) {
        error("Expected statement_inner in statement_block");
      }
      if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
        _out << std::endl;
      } else if (block()) {
        _out << std::endl;
      } else {
        //error("Expected semicolon or block after statement_inner, got " +
        //      _tok.content);
      }
      return true;
    } else if (_tok.type == TokType::LCURLY) {
      _out << _indent;
      if (block()) {
        _out << std::endl;
      } else {
        error("Expected block after left curly brace in statementblock");
      }
      return true;
    }
    return false;
  }

  bool _class() {
    if (_tok.type == TokType::IDENTIFIER &&
        (_tok.content == "class" ||
         _tok.content == "struct" ||
         _tok.content == "union"))
    {
      _out << _tok.content;
      next_tok();
      if (statement_inner()) {
        // nothing to do
      }
      if (semiblock()) {
        // nothing to do
      } else if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
        _out << std::endl;
      } else {
        error("Expected a semiblock after class, struct, or union");
      }
      return true;
    }
    return false;
  }

  bool _enum() {
    if (_tok.type == TokType::IDENTIFIER && _tok.content == "enum") {
      _out << _tok.content;
      next_tok();
      if (statement_inner()) {
        // nothing to do
      }
      if (enumblock()) {
        // nothing to do
      } else if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
        _out << std::endl;
      } else {
        error("Expected an enumblock or semicolon after enum");
      }
      return true;
    }
    return false;
  }

  bool _typedef() {
    if (_tok.type == TokType::IDENTIFIER && _tok.content == "typedef") {
      _out << _indent << _tok.content << " ";
      next_tok();
      if (_enum()) {
        // nothing to do
      } else if (_class()) {
        // nothing to do
      } else if (statement_inner()) {
        if (_tok.type == TokType::SEMICOLON) {
          _out << ";";
          next_tok();
          _out << std::endl;
        } else if (block()) {
          _out << std::endl;
        } else {
          error("Expected typedef statement to end in a semicolon or a block");
        }
      } else {
        error("Expected enum or statement after typedef");
      }
      return true;
    }
    return false;
  }

  bool statement_inner() {
    if (pstatement() || piece()) {
      while (pstatement() || piece()) {}
      return true;
    }
    return false;
  }

  bool pstatement() {
    if (_tok.type == TokType::LPAREN) {
      if (_prevtype != TokType::SEMICOLON && _prevtype != TokType::IDENTIFIER) {
        _out << " ";
      }
      _out << _tok.content;
      next_tok();
      while (statement_inner() || _tok.type == TokType::SEMICOLON) {
        if (_tok.type == TokType::SEMICOLON) {
          _out << _tok.content << " ";
          next_tok();
        }
      }
      if (_tok.type == TokType::RPAREN) {
        _out << _tok.content;
        next_tok();
      } else {
        error("Expected right parenthesis");
      }
      return true;
    }
    return false;
  }

  bool piece() {
    if (_tok.type == TokType::LITERAL ||
        _tok.type == TokType::IDENTIFIER ||
        _tok.type == TokType::OPERATOR)
    {
      if (_tok.type == TokType::LITERAL) {
        if (_prevtype == TokType::OPERATOR || _prevtype == TokType::RPAREN) {
          _out << " ";
        } else if (_tok.content[0] != '\'' && _tok.content[0] != '"') {
          _out << " ";
        }
      } else if (_tok.type == TokType::IDENTIFIER) {
        if (_prevtype == TokType::IDENTIFIER ||
            _prevtype == TokType::OPERATOR ||
            _prevtype == TokType::RPAREN)
        {
          _out << " ";
        }
      } else if (_tok.type == TokType::OPERATOR) {
        if (_prevtype != TokType::OPERATOR && _tok.content != ",") {
          _out << " ";
        }
      }

      _out << _tok.content;
      next_tok();
      return true;
    }
    return false;
  }

  bool semiblock() {
    if (basic_block()) {
      if (_tok.type == TokType::OPERATOR) {
        _out << " ";
        while (_tok.type == TokType::OPERATOR) {
          _out << _tok.content;
          next_tok();
        }
      }
      if (_tok.type == TokType::IDENTIFIER) {
        _out << " " << _tok.content;
        next_tok();
        while (_tok.type == TokType::OPERATOR && _tok.content == ",") {
          _out << ", ";
          next_tok();
          if (_tok.type == TokType::OPERATOR) {
            while (_tok.type == TokType::OPERATOR) {
              _out << _tok.content;
              next_tok();
            }
            _out << " ";
          }
          if (_tok.type == TokType::IDENTIFIER) {
            _out << _tok.content;
            next_tok();
          } else {
            error("Expected identifier after comma");
          }
        }
      }
      if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
        _out << std::endl;
      } else {
        error("Expected semicolon at the end of the semiblock");
      }
      return true;
    }
    return false;
  }

  bool block() {
    if (basic_block()) {
      if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
      }
      return true;
    }
    return false;
  }

  bool basic_block() {
    if (_tok.type == TokType::LCURLY) {
      if (_prevtype == TokType::RPAREN ||
          _prevtype == TokType::IDENTIFIER ||
          _prevtype == TokType::LITERAL)
      {
        _out << " ";
      }
      _out << "{";
      next_tok();
      _out << std::endl;
      _indent.push_back(' ');
      while (element()) {}
      if (_tok.type == TokType::RCURLY) {
        _indent.pop_back();
        _out << _indent << "}";
        next_tok();
      } else {
        error("Missing ending curly brace, got " + _tok.content);
      }
      return true;
    }
    return false;
  }

  bool enumblock() {
    if (braceinit()) {
      if (_tok.type == TokType::IDENTIFIER) {
        _out << " " << _tok.content;
        next_tok();
        while (_tok.content == ",") {
          _out << ", ";
          next_tok();
          if (_tok.type == TokType::IDENTIFIER) {
            _out << _tok.content;
            next_tok();
          } else {
            error("Expected identifier after comma");
          }
        }
      }
      if (_tok.type == TokType::SEMICOLON) {
        _out << ";";
        next_tok();
        _out << std::endl;
      } else {
        error("Expected enumblock to end in a semicolon");
      }
      return true;
    }
    return false;
  }

  bool braceinit() {
    if (_tok.type == TokType::LCURLY) {
      _out << " { ";
      next_tok();
      if (statement_inner()) {}
      if (_tok.type == TokType::RCURLY) {
        _out << " }";
        next_tok();
      } else {
        error("Expected right curly brace for brace initializer");
      }
      return true;
    }
    return false;
  }

private:
  Lexer _scanner;
  std::ostream &_out;
  const Token &_tok;
  TokType _prevtype;
  std::string _indent;
};

// Runs the parser on a given string
std::string parse_string(const std::string in) {
  std::istringstream instream(in);
  std::ostringstream outstream;
  Parser parser(instream, outstream);
  parser.parse();
  return outstream.str();
}

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
      while (lex.next_tok()) {
        out << "(" << toktype_tostring(tok.type)
            << ", \"" << tok.content << "\")\n";
      }
    } else {
      Parser p(in, out);
      p.parse();
    }
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
