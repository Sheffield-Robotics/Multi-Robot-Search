#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <fstream>
#include "hungarian/munkres/munkres.h"

using namespace std;
using namespace munkres;

int debugLevel = 0;

double computeHungarianAssignment(const vector<vector<double> >& costs, 
									vector<vector<bool> >& assignment);

void printCostMatrix(const vector< vector<double> >& costs);
								
void printAssignmentMatrix(const vector< vector<bool> >& assignment);

int main(int argc, char** argv) 
{
   char c;
   int numAgents=5;
   int numTargets=5;
   //while((c = getopt(argc, argv, "t:a:")) != EOF)
   //{
   //   switch(c)
   //   {
   //      case 't':
   //         numTargets = atoi(optarg);
   //         break;
   //      case 'a':
   //         numAgents = atoi(optarg);
   //         break;
   //      default:
   //         printf("\nOptions:\n");
   //         printf("--------------\n");
   //         printf("-a <num> num agents.\n");
   //         printf("-t <num> targets.\n");
   //         printf("-h Help\n");
   //         exit(0);
   //         break;
   //   }
   //}

   srand48(time(0));

   // Allocate cost matrix
   vector<double> dummy(numTargets);
   vector< vector <double> > costs(numAgents, dummy);
   printf(" COST MATRIX\n");
   for (int d=0; d<numTargets; d++) {
      for (int p=0; p<numAgents; p++) {
         //costs[d][p] = drand48();
         costs[d][p] = (double) rand()/1000;
         costs[d][p] = 1;
         printf(" %1.2lf", costs[d][p]);
      }
      printf(" \n");
   }
   // Allocate assignment matrix
   vector<bool> dummy2(numTargets);
   vector< vector <bool> > assignment(numAgents, dummy2);
   for (int d=0; d<numTargets; d++) {
      for (int p=0; p<numAgents; p++) {
         assignment[d][p] = false;
      }
   }

   double ret = computeHungarianAssignment(costs, assignment);
   printf("Result: %1.2lf\n", ret);

   printAssignmentMatrix(assignment);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compute Assignment
////////////////////////////////////////////////////////////////////////////////////////////////////////////
double computeHungarianAssignment(const vector<vector<double> >& costs,
      vector<vector<bool> >& assignment)
{
   printf("Running HUNGARIAN assignment\n");

   int nrows = costs.size();
   int ncols = costs[0].size();

   // Initialize matrix
   Matrix<double> matrix(nrows, ncols);
   for ( int row = 0 ; row < nrows ; row++ ) {
      for ( int col = 0 ; col < ncols ; col++ ) {
         matrix(row,col) = (double) costs[row][col];
      }
   }
   // Apply Munkres algorithm to matrix.
   Munkres m;
   m.solve(matrix);

   // Copy data 
   double totalCost = 0.0;
   for ( int row = 0 ; row < nrows ; row++ ) {
      for ( int col = 0 ; col < ncols ; col++ ) {
         assignment[row][col] = matrix(row,col) == 0.0;
         if (assignment[row][col] == true)
            totalCost += costs[row][col];
      }
   }
   return totalCost;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Print the assignment matrix
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void printAssignmentMatrix(const vector< vector<bool> >& assignment)
{
   unsigned int ndrv = assignment.size();
   unsigned int npas = assignment[0].size();
   unsigned int w = 7;
   
   cout << endl;
   cout << setiosflags(ios::fixed) << setprecision(2);
   cout << "--------------- Assignment Matrix ";
   for (unsigned int i=0;i<(ndrv+npas);i++)
      cout << "------";
   cout << endl;
   cout << " npas= "<< npas << ", ndrv=" << ndrv << endl;
   cout << endl;
      
   // Header
   for (unsigned int d=0; d<ndrv; d++) {   
      cout.width(w);
      cout << "d" << d;
   }
   cout << endl;
      
   for (unsigned int col=0; col<(ndrv); col++) {
      if (col<npas)
         cout << "p" << col;
      else   
         cout << "d" << (col-npas);

      for (unsigned int row=0; row<(npas); row++) {
         cout.width(w+1);
         cout << assignment[col][row];
      }
      cout << endl;
   }
   cout << endl;      
}
