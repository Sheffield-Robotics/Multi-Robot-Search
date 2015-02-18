#include "lineclear/CutSequence.h"

CutSequence::CutSequence() {
    _sequence = std::list<Cut>();
    _base_b = 0;
    _base_mu = 0;
    _left_b = 0;
    _right_b = 0;
}

CutSequence::~CutSequence() {
    
}

void CutSequence::print() {
    CutSequenceIterator i = _sequence.begin();
    std::cout << " CUT SEQUENCE: ";
    std::cout << " base_b = " << _base_b;
    std::cout << ", base_mu = " << _base_mu;
    std::cout << ", left_b  = " << _left_b;
    std::cout << ", right_b = " << _right_b << std::endl;    
    std::cout << " CUT SEQUENCE FULL CUTS: ";
    while ( i != _sequence.end() ) {
        std::cout << " (b=" << i->get_b() 
            << ",mu=" << i->get_mu()
            << ",rho=" << i->get_rho()
            << ",o=" << i->get_o() << ")";
        i++;
    }
    std::cout << std::endl;
    std::cout << " CUT SEQUENCE OBSTACLE SEQUENCE: ";
    CutSequenceOIterator j = _obstacle_sequence.begin();
    std::cout << " ( ";
    while ( j != _obstacle_sequence.end() ) {
        std::cout << *j << ",";
        j++;
    }
    std::cout << ")" << std::endl;
}

void CutSequence::push_back( Cut c ) {
    if ( _sequence.size() > 0 ) {
        // compute a rho
        c.set_rho( c.get_mu() - _sequence.back().get_b() );
    } else {
        c.set_rho( c.get_mu() - _base_b );
    }
    _sequence.push_back(c);
}

bool CutSequence::is_dominated_by( CutSequence &other ) {
    CutSequenceIterator i = _sequence.begin();
    CutSequenceIterator i_end = _sequence.end();
    CutSequenceIterator j = other.begin();
    CutSequenceIterator j_end = other.end();
    // search for a state that is better than in other    
    if ( DEBUG_CUTSEQUENCE >= 3 ) {
        std::cout << " IS DOMINATED by? " << std::endl;
    }
    
    // Note: at end, blocking cost is 0, sufficient to go through one
    // mu is strictly increasing - b is strictly decreasing
    // b has to be smaller than oldBlock to be of benefit
    while ( i != i_end && j != j_end ) {
        if ( DEBUG_CUTSEQUENCE >= 3 ) {
            std::cout << " mu i " << i->get_mu() << std::endl;
            std::cout << " mu j " << j->get_mu() << std::endl;
            std::cout << " b i " << i->get_b() << std::endl;
            std::cout << " b j " << j->get_b() << std::endl;
        }
        if ( i->get_mu() <= j->get_mu()) {
            if ( i->get_b() <= j->get_b() ) {
                if ( i->get_mu() == j->get_mu() 
                    && i->get_b() == j->get_b() ) {
                     // identical state, pick the sequence with the smaller 
                    // base values
                    if ( this->get_base_mu() < other.get_base_mu() ) {
                        if ( DEBUG_CUTSEQUENCE >= 3 ) {
                             std::cout << " RETURNING MU FALSE " << std::endl;
                        }
                        return false;
                    } 
                    if ( this->get_base_b() < other.get_base_b() ) {
                        if ( DEBUG_CUTSEQUENCE >= 3 ) {
                             std::cout << " RETURNING B FALSE " << std::endl;
                        }
                        return false;
                    }
                }  else { 
                    if ( DEBUG_CUTSEQUENCE >= 3 ) {
                         std::cout << " RETURNING PURE FALSE " << std::endl;
                    }
                    return false;
                }
            }
            i++;
        } else {
            j++;
        }
    }
     if ( DEBUG_CUTSEQUENCE >= 3 ) {
         std::cout << " RETURNING TRUE " << std::endl;
     }
    return true;
}

bool CutSequence::is_dominated_by_weakly( CutSequence &other ) {
    CutSequenceIterator i = _sequence.begin();
    CutSequenceIterator i_end = _sequence.end();
    CutSequenceIterator j = other.begin();
    CutSequenceIterator j_end = other.end();
    // search for a state that is better than in other    
    
    // Note: at end, blocking cost is 0, sufficient to go through one
    // mu is strictly increasing - b is strictly decreasing
    // b has to be smaller than oldBlock to be of benefit
    while ( i != i_end && j != j_end ) {
        //std::cout << " mu i " << i->get_mu() << std::endl;
        //std::cout << " mu j " << j->get_mu() << std::endl;
        //std::cout << " b i " << i->get_b() << std::endl;
        //std::cout << " b j " << j->get_b() << std::endl;
        if ( i->get_mu() < j->get_mu()) {
            if ( i->get_b() <= j->get_b() ) {
                return false;
            }
            i++;
        } else if (i->get_mu() == j->get_mu()) {
            if ( i->get_b() < j->get_b()) {
                return false;
            } else if (  i->get_b() == j->get_b() ) {
                if ( this->get_base_mu() < other.get_base_mu() ) {
                    return false;
                } 
                if ( this->get_base_b() < other.get_base_b() ) {
                    return false;
                }
            }
            j++;
        } else {
            j++;
        }
    }
    return true;
}


CutSequence::CutSequenceIterator CutSequence::begin() {
    return _sequence.begin();
}

CutSequence::CutSequenceIterator CutSequence::end() {
    return _sequence.end();
}

Cut& CutSequence::front() {
    return _sequence.front();
}

Cut& CutSequence::back() {
    return _sequence.back();
}

int CutSequence::size() {
    return _sequence.size();
}

CutSequence::CutSequenceOIterator CutSequence::o_begin() {
    return _obstacle_sequence.begin();
}

CutSequence::CutSequenceOIterator CutSequence::o_end() {
    return _obstacle_sequence.end();
}

int& CutSequence::o_front() {
    return _obstacle_sequence.front();
}

int& CutSequence::o_back() {
    return _obstacle_sequence.back();
}

int CutSequence::o_size() {
    return _obstacle_sequence.size();
}

void CutSequence::set_left(CutSequence* cs, int b) {
    _left_cs = cs;
    _left_b = b;
}

void CutSequence::set_right(CutSequence* cs, int b) {
    _right_cs = cs;
    _right_b = b;
}

void CutSequence::set_base( int b, int mu) {
    _base_b = b;
    _base_mu = mu;
}

int CutSequence::get_base_mu() {
    return _base_mu;
}

int CutSequence::get_base_b() {
    return _base_b;
}


void CutSequence::add_obstacle_index( int o) {
    _obstacle_sequence.push_back(o);
}

int CutSequence::get_final_cost() {
    if ( _sequence.size() < 1 ) {
        return -1;
    }
    return _sequence.back().get_mu();
}

std::list<int> CutSequence::get_obstacle_sequence() {
    return _obstacle_sequence;
}