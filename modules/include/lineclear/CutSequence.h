#ifndef CUTSEQUENCE_H
#define CUTSEQUENCE_H

#include "lineclear/Cut.h"
#include <list>
#include <iostream>

#define DEBUG_CUTSEQUENCE 4

class CutSequence {    
  public:
    typedef std::list<Cut>::iterator CutSequenceIterator;
    typedef std::list<int>::iterator CutSequenceOIterator;
        
    CutSequence();
    ~CutSequence();
        
    void print();
    void push_back( Cut c );
    
    bool is_dominated_by( CutSequence &other );
    bool is_dominated_by_weakly( CutSequence &other );
    CutSequenceIterator end();
    CutSequenceIterator begin();
    Cut& back();
    Cut& front();
    int size();
    
    CutSequenceOIterator o_end();
    CutSequenceOIterator o_begin();
    int& o_back();
    int& o_front();
    int o_size();
    
    void set_right(CutSequence* cs, int b);
    void set_left(CutSequence* cs, int b);
    void set_base( int b, int mu);
    
    int get_base_mu();
    int get_base_b();

    void add_obstacle_index( int o );
    
    int get_final_cost();
    std::list<int> get_obstacle_sequence();
    
  private:
    
    std::list<Cut> _sequence;
    std::list<int> _obstacle_sequence;
    
    CutSequence* _left_cs;
    CutSequence* _right_cs;
    int _base_b;
    int _base_mu;
    int _left_b;
    int _right_b;
    
};

#endif