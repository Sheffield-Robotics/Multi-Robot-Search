#include "lineclear/Cut.h"

#include <iostream>

Cut::Cut() {
    
}

Cut::Cut ( int b, int mu ) {
    _mu = mu;
    _b = b;
    _rho = 0; 
    _o = 0;
};

Cut::Cut ( int b, int mu, int o) {
    _mu = mu;
    _b = b;
    _o = o;
    _rho = 0; 
};


Cut::~Cut() {
    
}


int Cut::get_b() { return _b;}
int Cut::get_mu() { return _mu;}
int Cut::get_rho() { return _rho;}
int Cut::get_o() { return _o;}

void Cut::set_rho( int rho ) {
    _rho = rho;
}

void Cut::set_o( int o ) {
    _o = o;
}

void Cut::print() {
    std::cout << "b=" << _b 
        << ",mu" << _mu 
        << ",rho=" << _rho 
        << ",o=" << _o
        << std::endl;
}