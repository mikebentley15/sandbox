#include <casadi/casadi.hpp>

int main() {
  using namespace casadi;

  std::cout << "nlpsol_n_in():           " << nlpsol_n_in() << "\n";
  std::cout << "nlpsol_in():             " << nlpsol_in() << "\n";
  std::cout << "nlpsol_default_in():     " << nlpsol_default_in() << "\n";
  std::cout << "nlpsol_n_out():          " << nlpsol_n_out() << "\n";
  std::cout << "nlpsol_out():            " << nlpsol_out() << "\n";
  //std::cout << "doc_nlpsol('ipopt'):     " << doc_nlpsol("ipopt") << "\n";
  std::cout << "has_nlpsol('ipopt'):     " << has_nlpsol("ipopt") << "\n";
  std::cout << "nlpsol_options('ipopt'): " << nlpsol_options("ipopt") << "\n";
  for (auto &op : nlpsol_options("ipopt")) {
    std::cout << "  " << op << " ("
              << nlpsol_option_type("ipopt", op) << "):  "
              << nlpsol_option_info("ipopt", op) << "\n";
  }

  return 0;
}
