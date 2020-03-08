#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H
#include <bits/stdc++.h>
#include "Graph.hpp"

using namespace mGraph;
using namespace std;

namespace PathPlanner
{

template<typename T>
class Planner:public Graph<T>
{	

	public:
		typedef T vertex;
		vertex vtx;
	private:
		
	public:
		Planner(): Graph<T>(){}

	void hello(){
		cout<<"Welcome to Path Planner's\n";
	}

	void dijkstra(vertex src, vertex dest=INT_MIN, bool dest_defined=false){
		this->init();
		this->distance_insert(src,0);
		while(1){
			this->lookfor_connected(src);
			src = this->select_min_distance_node();
			if(this->stop(dest, dest_defined)) break;
		}
		this->printEdgeWeight();
	}

	void bellman_ford(vertex src, vertex dest=INT_MIN, bool dest_defined=false){
		this->init();
		this->set_distance(src, 0);
		int n = this->num_of_vertices()-1;
		while(n){
			this->relax_edges();
			this->reset_visited();
			--n;
		}
		if(this->check_negative_cycle(src)){
			cout<<"Path cannot be found: Negative edge cycle is found\n";
			return;
		}
		this->printEdgeWeight();
	}

	void a_star(int row, int col, Location start, Location end){
		Grid g(row, col, start, end);
		g.run();
		cout<<std::left<<std::setw(8);
		cout<<"to "<<" "<<"from\n";
		g.printPath();
	}

};

}

#endif