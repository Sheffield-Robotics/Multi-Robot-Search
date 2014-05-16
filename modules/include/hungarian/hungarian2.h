#ifndef HUNGARIAN_H2
#define HUNGARIAN_H2

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <fstream>
#include "hungarian/munkres/munkres.h"

double computeHungarianAssignment2(const std::vector<std::vector<double> >& costs, std::vector<std::vector<bool> >& assignment);

void printAssignmentMatrix(const std::vector< std::vector<bool> >& assignment);								
#endif