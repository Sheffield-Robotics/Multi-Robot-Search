#include "hungarian/hungarian2.h"

using namespace munkres;
    
double computeHungarianAssignment2(const std::vector<std::vector<double> >& costs, std::vector<std::vector<bool> >& assignment)
{
   //printf("Running HUNGARIAN assignment\n");

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

void printAssignmentMatrix(const std::vector< std::vector<bool> >& assignment)
{
    using namespace std;
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
