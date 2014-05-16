#include "lineclear/ChoiceSet.h"

ChoiceSet::ChoiceSet() {
    _n = 0; _i = 0; _k = 0; _c = NULL; _b = 0;
    CutSequence s;
    Cut c(0,0);
    s.push_back(c);
    _sequences.push_back( pair<CutSequence,int>(s,0) );
}

ChoiceSet::ChoiceSet( int n, int i, int k ) {
    _n = n;
    _i = i;
    _k = k;
    _c = new int[_k];
}

ChoiceSet::ChoiceSet( int n, int i, int k, int b ) {
    _n = n;
    _i = i;
    _k = k;
    _b = b;
    _c = new int[_k];
}

ChoiceSet::~ChoiceSet() {
    delete _c;
}


void ChoiceSet::print() {
    std::cout << "  b(T(" << _i << ":" << _k << "))=" << _b;
    for ( int j = 1; j <= _k; j++ ) {
        std::cout << " " << _c[j-1];
    }
    std::cout << std::endl;
}

CutSequence* ChoiceSet::get_cut_sequence(int i) {
    if ( 0 <= i && i < int(_sequences.size()) ) {
        return &( _sequences[i].first );
    }
    return NULL;
}

CutSequence* ChoiceSet::get_cut_sequence(int i, int& o) {
    if ( 0 <= i && i < int(_sequences.size()) ) {
        o = _sequences[i].second;
        return &( _sequences[i].first );
    }
    return NULL;
}

CutSequence* ChoiceSet::get_best_cut_sequence() {
    // go through cut sequence and pick the best
    if ( _sequences.size() == 0 ) 
        return NULL;
    int best_i = 0;
    int cost = -1, c;
    int up_to_i = int(_sequences.size());
    for ( int i = 0; i < up_to_i; i++ ) {
        c = _sequences[i].first.get_final_cost();
        if ( (c < cost || cost == -1) && c != -1 ) {
            best_i = i; cost = c;
        }
    }
    return &(_sequences[best_i].first);
}

std::list<CutSequence*> ChoiceSet::get_best_cut_sequences() {
    // go through cut sequence and pick the best
    std::list<CutSequence*> best_list;
    if ( _sequences.size() == 0 ) 
        return best_list;
    int best_i = 0;
    int cost = -1, c;
    int up_to_i = int(_sequences.size());
    for ( int i = 0; i < up_to_i; i++ ) {
        c = _sequences[i].first.get_final_cost();
        if ( (c < cost || cost == -1) && c != -1 ) {
            best_i = i; cost = c;
        }
    }
    up_to_i = int(_sequences.size());
    for ( int i = 0; i < up_to_i; i++ ) {
        c = _sequences[i].first.get_final_cost();
        if ( c == cost ) {
            best_list.push_back( &(_sequences[i].first) );
        }
    }
    return best_list;
}

std::list<CutSequence*> ChoiceSet::get_cut_sequences_at_cost(int at_cost) {
    // go through cut sequence and pick those at cost lower or equal
    // to at_cost
    std::list<CutSequence*> best_list;
    if ( _sequences.size() == 0 ) 
        return best_list;
    for ( int i = 0; i < int(_sequences.size()); i++ ) {
        int c = _sequences[i].first.get_final_cost();
        if ( c <= at_cost ) {
            best_list.push_back( &(_sequences[i].first) );
        }
    }
    return best_list;
}

void ChoiceSet::add_cut_sequence( CutSequence& cs ) {
    _sequences.push_back( pair<CutSequence,int>(cs,0));
}

void ChoiceSet::add_cut_sequence( CutSequence& cs, int j ) {
    _sequences.push_back( pair<CutSequence,int>(cs,j) );
}

bool ChoiceSet::is_dominated_weakly( CutSequence& cs ) {
    for ( int i = 0; i < this->cut_sequences_size(); i++ ) {
        if ( cs.is_dominated_by_weakly( _sequences[i].first ) ) {
            return true;
        }
    }
    return false;
}

void ChoiceSet::remove_dominated() {
    // 
    std::vector<bool> to_be_deleted(this->cut_sequences_size());
    for ( int i = 0; i < this->cut_sequences_size(); i++ ) {
        // check if this cut sequence is dominated
        to_be_deleted[i] = false;
        for ( int j = 0; j < this->cut_sequences_size(); j++ ) {
            if ( j == i ) 
                continue;
            if ( _sequences[i].first.is_dominated_by( _sequences[j].first ) ) {
                to_be_deleted[i] = true;
            } 
        }
    }
    int removed = 0;
    for ( int i = this->cut_sequences_size()-1; i > 0; i-- ){
        if ( to_be_deleted[i] ) {
            removed++;
            _sequences.erase(_sequences.begin() + i );
        }
    }
    //std::cout << "Removed " << removed << " cut sequences " << std::endl;
    //std::cout << "Down to  " << this->cut_sequences_size() << " cut sequences " << std::endl;
}


















