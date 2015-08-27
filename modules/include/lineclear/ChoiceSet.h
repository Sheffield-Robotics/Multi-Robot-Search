// 
//  ChoiceSet.h
//  icra12_pe_code
//  
//  Created by Andreas on 2012-08-29.
//  Copyright 2012 Andreas Kolling. All rights reserved.
// 
/* 
 * A choiceset keeps track of the choices to expand obstacles towards
 * contaminated areas (see paper)
 * 
 * Author: Andreas Kolling ( Wed Aug 29 10:02:18 CEST 2012 )
 */

#ifndef CHOICESET_H
#define CHOICESET_H

#define DEBUG_CHOICESET 0

#include <vector>
#include <utility>
#include <list>
#include "lineclear/CutSequence.h"

using std::vector;
using std::pair;

class ChoiceSet {
  public:
    ChoiceSet(); // creates the default T_0 Choice set wiht 0 cut
    ChoiceSet( int n, int i, int k );
    ChoiceSet( int n, int i, int k, int b );
    ~ChoiceSet();
        
    /* returns the o-th obstacle index in the choice set note that the
     * ordering is circular, i.e. modulo _n, and j=1,...,k */
    int get_obstacle_for_choice( int j ) {
        if ( j > _k ) 
            return -1;
        else
            return (_i + j - 2) % _n + 1;
    }
    
    bool is_dominated_weakly( CutSequence& cs );
    void set_b( int b ) { _b = b; }
    int get_b() { return _b; }
    
    
    
    /* set_c_at takes indices of choices c from 1 to k */
    void set_c_at( int j, int c) {
        j--;
        if ( 0 <= j && j < _k ) {
            _c[j] = c;
        }
    }

    void set_all_c( int c ) {
        for ( int i = 0; i < _k; i++ ) {
            _c[i] = c;
        }
    }

    int get_c_at( int j) {
        j--;
        if ( 0 <= j && j < _k ) {
            return _c[j];
        } 
        return -1;
    }

    int get_k() {
        return _k;
    }

    int get_blocking() {
        return _b;
    }

    int cut_sequences_size() {
        return _sequences.size();
    }
    
    CutSequence* get_cut_sequence(int k);    
    CutSequence* get_cut_sequence(int i, int& o);
    CutSequence* get_best_cut_sequence();
    std::list<CutSequence*> get_best_cut_sequences();
    std::list<CutSequence*> get_cut_sequences_at_cost(int at_cost);
    
    void print();
    
    void add_cut_sequence( CutSequence& cs );
    void add_cut_sequence( CutSequence& cs, int j );
    void remove_dominated();
  private:
    
    int _n; // number of obstacles in E (needed for index cycles)
    int _i; // first obstacle index of the choice set
    int _k; // number of indices in choice set
    int _b; // cost of blocking the choice set
    int* _c; // array (size k) for costs of each choice 
    
    // cut sequence list for each choice
    vector< pair<CutSequence,int> > _sequences; 
    
};

#endif