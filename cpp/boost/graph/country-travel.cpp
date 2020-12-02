#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/array.hpp>

#include <array>
#include <iostream>
#include <string>
#include <utility>

namespace {

enum class Country {
  NETHERLANDS,
  BELGIUM,
  FRANCE,
  GERMANY,
  SWITZERLAND,
  AUSTRIA,
  ITALY,
};

std::string name(Country country) {
  switch (country) {
    case Country::NETHERLANDS: return "Netherlands";
    case Country::BELGIUM:     return "Belgium";
    case Country::FRANCE:      return "France";
    case Country::GERMANY:     return "Germany";
    case Country::SWITZERLAND: return "Switzerland";
    case Country::AUSTRIA:     return "Austria";
    case Country::ITALY:       return "Italy";
  }
  return "";
}

} // end of unnamed namespace

/** From https://theboostcpplibraries.com/boost.graph-algorithms
 *
 * Create a graph with vertices for the following countries: Netherlands,
 * Belgium, France, Germany, Switzerland, Austria and Italy. Connect the
 * vertices of those countries with common borders. Find the shortest path –
 * the path with fewest border crossings – to get from Italy to the
 * Netherlands. Write the names of all countries to standard output which you
 * cross on your way from Italy to the Netherlands.
 */
int main() {
  const int num_countries = 7;
  const int num_borders = 12;


  std::pair<int, int> borders[] {
    {int(Country::NETHERLANDS), int(Country::BELGIUM)},
    {int(Country::NETHERLANDS), int(Country::GERMANY)},
    {int(Country::BELGIUM),     int(Country::FRANCE)},
    {int(Country::BELGIUM),     int(Country::GERMANY)},
    {int(Country::FRANCE),      int(Country::GERMANY)},
    {int(Country::FRANCE),      int(Country::SWITZERLAND)},
    {int(Country::FRANCE),      int(Country::ITALY)},
    {int(Country::GERMANY),     int(Country::SWITZERLAND)},
    {int(Country::GERMANY),     int(Country::AUSTRIA)},
    {int(Country::SWITZERLAND), int(Country::AUSTRIA)},
    {int(Country::SWITZERLAND), int(Country::ITALY)},
    {int(Country::AUSTRIA),     int(Country::ITALY)},
  };

  using Graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                      boost::undirectedS>;
  Graph g {std::begin(borders), std::end(borders), num_countries};

  boost::array<int, num_countries> predecessors;
  boost::breadth_first_search(g, int(Country::NETHERLANDS),
    boost::visitor(
      boost::make_bfs_visitor(
        boost::record_predecessors(predecessors.begin(),
          boost::on_tree_edge{}))));

  Country c = Country::ITALY;
  while (c != Country::NETHERLANDS) {
    std::cout << name(c) << "\n";
    c = Country(predecessors[int(c)]);
  }
  std::cout << name(c) << std::endl;

  return 0;
} // end of main()
