#include <bits/stdc++.h>
#include "Path_Planning.hpp"

using namespace PathPlanner;

int main(){

	Planner<int> g;
	g.set_directed(true);
	// g.insert_edge(0, 1, 4);
	// g.insert_edge(0, 7, 8);
	// g.insert_edge(1, 2, 8);
	// g.insert_edge(1, 7, 11);
	// g.insert_edge(2, 3, 7);
	// g.insert_edge(2, 8, 2);

	// g.insert_edge(2, 5, 4); 
 //    g.insert_edge(3, 4, 9); 
 //    g.insert_edge(3, 5, 14); 
 //    g.insert_edge(4, 5, 10); 
 //    g.insert_edge(5, 6, 2); 
 //    g.insert_edge(6, 7, 1); 
 //    g.insert_edge(6, 8, 6); 
 //    g.insert_edge(7, 8, 7); 
 //    g.insert_edge(9, 8, 7); 

	// g.insert_edge(1,2,1);
	// g.insert_edge(1,3,3);
	// g.insert_edge(1,4,2);
	// g.insert_edge(4,5,5);
	// g.insert_edge(3,5,2);
	// g.insert_edge(2,5,7);

	// g.insert_edge(1,2,-3);
	// g.insert_edge(2,3,1);
	// g.insert_edge(3,4,1);
	// g.insert_edge(4,5,1);
	// g.insert_edge(5,6,1);

	// valid Bellman Ford
	g.insert_edge(1,2,4);
	g.insert_edge(1,4,5);
	g.insert_edge(4,3,3);
	g.insert_edge(3,2,-10);

	// invalid Bellman Ford
	// g.insert_edge(1,2,4);
	// g.insert_edge(1,4,5);
	// g.insert_edge(4,3,3);
	// g.insert_edge(3,2,-10);
	// g.insert_edge(2,4,5);

	// g.printGraph();
	// g.dijkstra(1);
	g.bellman_ford(1);
	
	// Location start{1,1},end{4,4};
	// g.a_star(4, 4, start, end);

	return 0;
}