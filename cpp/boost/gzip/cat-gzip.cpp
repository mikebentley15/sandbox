#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>

#include <fstream>
#include <iostream>
#include <string>

#include <cstdlib>

namespace bio = boost::iostreams;

namespace {

struct ParsedArgs {
  std::string outfile;
};

void print_usage(std::ostream &out, char *progname) {
  std::cerr << "Usage: " << progname << " <output-gzip-file>" << std::endl;
}

ParsedArgs parse_args(int arg_count, char *arg_list[]) {
  if (arg_count != 2) {
    print_usage(std::cerr, arg_list[0]);
    std::exit(1);
  }
  std::string arg (arg_list[1]);
  if (arg == "-h" || arg == "--help") {
    print_usage(std::cout, arg_list[0]);
    std::cout
      << "\n"
         "Description:\n"
         "\n"
         "  Reads from stdinput and outputs compressed to the given filename.\n"
      << std::endl;
    std::exit(0);
  }
  ParsedArgs parsed;
  parsed.outfile = arg;
  return parsed;
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  auto args = parse_args(arg_count, arg_list);

  std::ofstream file(args.outfile);
  bio::filtering_streambuf<bio::output> out;
  out.push(bio::gzip_compressor());
  out.push(file);
  bio::copy(std::cin, out);

  return 0;
}

