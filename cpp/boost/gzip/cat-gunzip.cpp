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
  std::string infile;
};

void print_usage(std::ostream &out, char *progname) {
  std::cerr << "Usage: " << progname << " <gzip-file>" << std::endl;
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
         "  unzips a gzip file and outputs to stdout.\n"
      << std::endl;
    std::exit(0);
  }
  ParsedArgs parsed;
  parsed.infile = arg;
  return parsed;
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  auto args = parse_args(arg_count, arg_list);

  std::ifstream file(args.infile);
  bio::filtering_streambuf<bio::input> in;
  in.push(bio::gzip_decompressor());
  in.push(file);
  bio::copy(in, std::cout);

  return 0;
}
