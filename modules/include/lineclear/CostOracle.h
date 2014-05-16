#ifndef COSTORACLE_H
#define COSTORACLE_H

class CostOracle {
        
public:
    
    virtual int get_shortest_extension_cost(int i, int j, int k) = 0;
    virtual int get_shortest_line_inside_cost(int i, int j) = 0;
    
};

#endif