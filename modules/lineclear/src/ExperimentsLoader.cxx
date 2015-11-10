#include "lineclear/CGALTypes.h"
#include "lineclear/Environment.h"
#include "lineclear/ChoiceTree.h"

#include "utilities/stringutils.h"
#include "utilities/Yaml_Config.h"
#include "utilities/filesysTools.h"

#include "gui/viewer.h"

#include "dirent.h"

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
  
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir ("/Users/andreas/_Code/Multi-Robot-Search/build/")) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      string filename(ent->d_name);
      printf ("%s\n", filename.c_str());
      int pos = filename.find(".ct");
      if ( pos == string::npos ) {
        continue;
      }
      string choice_tree_filename = filename;
      
      lineclear::ChoiceTree *_ct;
      _ct = new lineclear::ChoiceTree();
      _ct->load_from_file(choice_tree_filename);

      int first_obstacle;
      std::list<int> obstacle_sequence;
      obstacle_sequence = _ct->get_optimal_obstacle_sequence(first_obstacle);
      obstacle_sequence.push_front(first_obstacle);
      int optimal_cost = _ct->get_optimal_cost();
      M_INFO1("OPTIMAL COST %d \n", optimal_cost);
      int final_cost = _ct->get_optimal_cut_sequence(first_obstacle)->get_final_cost();
      std::cout << "final mu"
                << final_cost
                << std::endl;
      std::string fname = getPureFilename(choice_tree_filename) + ".stats";
      std::ofstream statFile;
      statFile.open(fname.c_str(), std::ios::app);      
      statFile << std::endl;
      statFile << final_cost;
      statFile << ", " << optimal_cost;
      statFile.close();
      
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    return EXIT_FAILURE;
  }
  
  
  return 0;
}
