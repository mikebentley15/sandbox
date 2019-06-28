#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

#include <cstdio>  // for popen(), tmpnam(), and remove()

struct ProcResult {
  int ret;
  std::string out;
  std::string err;
};

/** Calls a command in the shell and returns the output and result */
inline ProcResult call_with_output(const std::string &command) {
  char fname_buf[L_tmpnam];
  char *s = std::tmpnam(fname_buf);    // gives a warning, but I'm not worried
  if (s != fname_buf) {
    throw std::ios_base::failure("Could not create temporary filename");
  }

  // run the command
  std::string cmd = command + " 2>" + fname_buf;
  FILE* output = popen(cmd.c_str(), "r");

  // grab stdout
  std::ostringstream outbuilder;
  const int bufsize = 256;
  char buf[bufsize];
  if (output) {
    while (!feof(output)) {
      if (fgets(buf, bufsize, output) != nullptr) {
        outbuilder << buf;
      }
    }
  }

  // wait and grab the return code
  int ret = pclose(output);

  // read the temporary file and get the contents
  std::ifstream input(fname_buf);
  auto err = std::string(std::istreambuf_iterator<char>(input),
                         std::istreambuf_iterator<char>());

  if (WIFEXITED(ret)) {
    ret = WEXITSTATUS(ret);
  }

  return ProcResult { ret, outbuilder.str(), err };
}

int main(int argCount, char* argList[]) {
  std::ostringstream command_builder;
  if (argCount > 1) {
    for (int i = 1; i < argCount; i++) {
      command_builder << '\'' << argList[i] << '\'' << " ";
    }
  } else {
    command_builder << "ponysay What is my name again?";
  }
  std::string command(command_builder.str());
  auto result = call_with_output(command);
  std::cout << "Called command:\n"
            << "  " << command << std::endl
            << "Output was:\n"
            << result.out << std::endl
            << "Standard error was:\n"
            << result.err << std::endl
            << "Return code:\n"
            << "  " << result.ret << std::endl;
  return result.ret;
}
