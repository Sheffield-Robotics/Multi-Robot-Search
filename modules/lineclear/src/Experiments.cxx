#include "lineclear/CGALTypes.h"
#include "lineclear/Environment.h"
#include "lineclear/ChoiceTree.h"

#include "utilities/stringutils.h"
#include "utilities/Yaml_Config.h"

#include "gui/viewer.h"

#include <iostream>

using std::cout;
using std::endl;
using std::list;
using namespace lineclear;

Viewer *viewer = NULL;

int main(int argc, char **argv) {

  // ====================================
  // = Generate polygon and Environment =
  // ====================================

  string fname_yaml = "../maps/default.yaml";
  Yaml_Config::load_yaml_file_into_param(fname_yaml.c_str());

  string basefilename = random_string(20);

  // Generating a random polygon
  polygonization::Polygon_Environment *_pol;
  _pol = new polygonization::Polygon_Environment(
      Yaml_Config::yaml_param["poly_n"].as<int>());
  string poly_file = basefilename + "_random.poly";
  _pol->save_to_file(poly_file);

  lineclear::Environment *_env;
  _env = new lineclear::Environment(_pol->master_polygon);

  lineclear::ChoiceTree *_ct;
  _ct = new lineclear::ChoiceTree(_env, nullptr, _pol);
  _ct->init_choice_tree();
  _ct->save_to_file(basefilename + ".ct");

  int first_obstacle;
  std::list<int> obstacle_sequence;
  obstacle_sequence = _ct->get_optimal_obstacle_sequence(first_obstacle);
  obstacle_sequence.push_front(first_obstacle);
  M_INFO1("OPTIMAL COST %d \n", _ct->get_optimal_cost());

  string fname = basefilename + ".ose";
  std::ofstream outFile;
  outFile.open(fname.c_str(), std::ios::out);
  std::list<int>::iterator i = obstacle_sequence.begin();
  for (const int &i : obstacle_sequence) {
    std::cout << i << ",";
    outFile << i << " ";
  }
  outFile.close();
  std::cout << "final mu"
            << _ct->get_optimal_cut_sequence(first_obstacle)->get_final_cost()
            << std::endl;

  fname = basefilename + ".stats";
  std::ofstream statFile;
  statFile.open(fname.c_str(), std::ios::out);
  statFile << Yaml_Config::yaml_param["poly_n"].as<int>();
  statFile << ", " << _ct->n_fully_skipped;
  statFile << ", " << _ct->n_skipped;
  statFile << ", " << _ct->n_not_skipped;
  

  double proper_average;
  std::vector<int> n_choicesets_with_n_cutsequences;
  double avg = _ct->average_n_cutsequences_stats(
      proper_average, n_choicesets_with_n_cutsequences);
  
  statFile << ", " << proper_average;
  statFile << ", " << avg;
  statFile << std::endl;
  for ( const int &i : n_choicesets_with_n_cutsequences ) {
    statFile << i << ", ";
  }
  statFile.close();

  
  return 0;
}
