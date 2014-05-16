#include "lineclear/SurveillanceGraph.h"

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>


SurveillanceGraph::SurveillanceGraph(int n) {
    _n = n;
    nodes = new SurveillanceGraphNode[n+1];
    root = &(nodes[0]);
}

void SurveillanceGraph::print() {
    print_children( root,0 );
}

void SurveillanceGraph::compute() {
    compute_sequential_cost();
    compute_parallel_cost();
}

void SurveillanceGraph::print_children(SurveillanceGraphNode* node,int depth) {
    if ( node != NULL ) {
        node->_depth = depth;
        std::cout << std::string(3*depth, '-');
        std::cout << " Node: " << node->obstacle_index;
        std::cout << " Split: " << node->split_cost;
        std::cout << " Block: " << node->incoming_blocking_cost;
        std::cout << " Paral: " << node->parallel_cost;
        std::cout << " Seq.C: " << node->sequential_cost;
        std::cout << " Delay: " << node->delay;
        std::cout << " Time: " << node->time;
        std::cout << std::endl;
        for ( int i = 0; i < node->_n_children; i++ ) {
            print_children( node->children[i], depth+1 );
        }
    }
}

void SurveillanceGraph::compute_sequential_cost() {
    compute_sequential_cost_node(root);
}

int SurveillanceGraph::get_parallel_cost() {
    return root->parallel_cost;
}

void SurveillanceGraph::compute_sequential_cost_node(SurveillanceGraphNode* node ) {
    if ( node == NULL ) return;
    for ( int i = 0; i < node->_n_children; i++ ) {
        compute_sequential_cost_node( node->children[i] );
    }
    int total_cost = 0;
    int left_b = 0,  left_c = 0;
    if ( node->children[0] != NULL ) {
        left_b = node->children[0]->incoming_blocking_cost;
        left_c = node->children[0]->sequential_cost;
    }
    int right_b = 0, right_c = 0;
    if ( node->children[1] != NULL ) {
        right_b = node->children[1]->incoming_blocking_cost;
        right_c = node->children[1]->sequential_cost;
    }
    if ( left_c + right_b < right_c + left_b 
     || (right_b == 0 && right_c == 0 )) {
        total_cost = left_c + right_b;
    } else {
        total_cost = right_c + left_b;
    }
    node->sequential_cost = std::max(total_cost,node->split_cost);
    return;
}

void SurveillanceGraph::compute_parallel_cost() {
    compute_parallel_cost_node( root );
}

void SurveillanceGraph::compute_parallel_cost_node(SurveillanceGraphNode* node ) {
    if ( node == NULL ) return;
    for ( int i = 0; i < node->_n_children; i++ ) {
        compute_parallel_cost_node( node->children[i] );
    }
    int par_cost = 0;
    for ( int i = 0; i < node->_n_children; i++ ) {
        if ( node->children[i] != NULL ) 
            par_cost += node->children[i]->parallel_cost;
    }
    node->parallel_cost = std::max(par_cost,node->split_cost);
    return;
}

void SurveillanceGraph::compute_depth_first( int start_at, int end_at ) {
    for ( int k = start_at; k < end_at; k++ ) {
        std::cout << " Time at k=" << k << " ";
        std::cout << depth_first_par(root,k,0) << std::endl;
        if ( DEBUG_DEPTH_FIRST >= 2 ) {
            print();
            std::cout << std::endl;
        }
    }
}

float SurveillanceGraph::depth_first_par(SurveillanceGraphNode* node, int k, float delta) {
    if ( node == NULL ) return 0.0;
    
    if ( k < node->split_cost ) {
        std::cout << " node " << node->obstacle_index;
        std::cout << " split " << node->split_cost;
        std::cout << " CANNOT CLEAR DEPTH_FIRST too few robots " 
            << k << std::endl;
    }
    
    node->delay = delta;
    int depth = node->_depth;
    if ( DEBUG_DEPTH_FIRST >= 1 ) {
        std::cout << std::string(3*depth, '-');
        std::cout << " at node " << node->obstacle_index << " del" 
                  << node->delay << std::endl;
    }
    float t = 0;
    std::vector<float> t_vec(2);
    t_vec[0] = 0; t_vec[1] = 0;
    //float *t_ar = new float[node->_n_children];
    if ( node->parallel_cost <= k ) {
        for ( int i = 0; i < node->_n_children; i++ ) {
            t_vec[i] = 0;
            if ( node->children[i] != NULL ) {
                t_vec[i] = depth_first_par(node->children[i],
                                     node->children[i]->parallel_cost,
                                     0);
            }
        }
        t = std::max(t_vec[0],t_vec[1]);
        if ( DEBUG_DEPTH_FIRST >= 2 ) {
            std::cout << std::string(3*depth, '-');
            std::cout << " subtree t=" << t << std::endl;
        }
    } else {
        int left_b = 0,  left_c = 0;
        if ( node->children[0] != NULL ) {
            if ( DEBUG_DEPTH_FIRST >= 3 ) {
                std::cout << std::string(3*depth, '-');
                std::cout << "   have left" << std::endl;
            }
            left_b = node->children[0]->incoming_blocking_cost;
            left_c = node->children[0]->sequential_cost;
        }
        int right_b = 0, right_c = 0;
        if ( node->children[1] != NULL ) {
            if ( DEBUG_DEPTH_FIRST >= 3 ) {
                std::cout << std::string(3*depth, '-');
                std::cout << "   have right" << std::endl;
            }
            right_b = node->children[1]->incoming_blocking_cost;
            right_c = node->children[1]->sequential_cost;
        }
        if ( left_c + right_b < right_c + left_b 
         || (right_b == 0 && right_c == 0 )) {
            if ( DEBUG_DEPTH_FIRST >= 3 ) {
                 std::cout << std::string(3*depth, '-');
                 std::cout << "      go left with " << k-right_b << std::endl;
            }
            t += depth_first_par(node->children[0],k-right_b,0);
            t += depth_first_par(node->children[1],k,t);
        } else {
            if ( DEBUG_DEPTH_FIRST >= 3 ) {
                std::cout << std::string(3*depth, '-');
                std::cout << "      go right with " << k-left_b << std::endl;
            }
            t += depth_first_par(node->children[1],k-left_b,0);
            t += depth_first_par(node->children[0],k,t);
        }
        if ( DEBUG_DEPTH_FIRST >= 2 ) {
            std::cout << std::string(3*depth, '-');
            std::cout << " subtree t=" << t << std::endl;
        }
    }
    return t + node->time;
//  \IF{$c^p(o_s) \leq k$}
//  	% both children can be parallezied
//  	\STATE $t \leftarrow Depth\_First\_Par(o_l,c^p(o_l),0)$
//  	\STATE $t \leftarrow t+Depth\_First\_Par(o_r,c^p(o_r),0)$
//  \ELSE
//  	% cannot execute both sides entirely in parallel
//  	% need to delay the parallel execution of one of the branches
//  	\IF{$c^d(o_l) + b(o_r) \leq c^d(o_r) + b(o_l)$}
//  		% block the right and clear the left first
//  		\STATE $t \leftarrow Depth\_First\_Par(o_l,k-b(o_r),0)$
//  		\STATE $t \leftarrow t+Depth\_First\_Par(o_r,k,t)$
//  	\ELSE
//  		% block the left and clear the right first
//  		\STATE $t \leftarrow Depth\_First\_Par(o_r,k-b(o_l),0)$
//  		\STATE $t \leftarrow t+Depth\_First\_Par(o_l,k,t)$
//  	\ENDIF
//  \ENDIF
//  \STATE {\bf return $t + t_s$}
}

void SurveillanceGraph::compute_parallel_delays() {
    
}