#ifndef CUT_H
#define CUT_H


class Cut {  
  public:
    Cut();
    Cut( int b, int mu ); 
    Cut ( int b, int mu, int o);
    ~Cut();
    
    int get_b();
    int get_mu();
    int get_rho();
    int get_o();
    
    void set_rho( int rho );
    void set_o( int o );
    
    void print();
        
  private:
    int _b;   // cost to block
    int _mu;  // cost to clear
    int _rho; // ordering
    int _o;   // obstacle index at which this cut is reached
};

#endif