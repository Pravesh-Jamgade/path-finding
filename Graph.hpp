#ifndef GRAPH_H
#define GRAPH_H
#include <bits/stdc++.h>
#include "logger.h"
using namespace std;

namespace mGraph
{

	template<typename T>
	class Graph
	{		
			int no_vertex;
			int no_edges;
		public:
			typedef T vertex;
			typedef T weight;
			typedef pair<vertex, vertex> edge;
			typedef set<vertex> vertex_set;
			typedef set<edge> edge_set;
			typedef map<vertex, weight> edgeWeight;

			typedef map<vertex, edgeWeight> neighbors;
			typedef map<vertex, vertex_set> degree;

			typedef neighbors adj_graph;
			
			typedef pair<vertex, weight> vertex_weight_pair;
			typedef vector<vertex_weight_pair> dis_vector;
			typedef priority_queue<vertex_weight_pair, 
			dis_vector, greater<vertex_weight_pair>> minDistance;

			typedef typename adj_graph::iterator iterator;
			typedef typename degree::iterator degree_iterator;
			typedef typename edgeWeight::iterator ew_iterator;
			typedef typename vertex_set::iterator vset_iter;
			typedef typename map<vertex, vertex>::iterator vviterator;
		
		private:
			adj_graph G;
			degree indegree;
			degree outdegree;
			int num_edges;
			bool undirected;
			vertex_set vset;
			map<vertex, bool> visited;
			minDistance minDis;
			edgeWeight distance;
			map<vertex, vertex> parent;


		public:

			Graph(): G(), indegree(), outdegree(), 
			vset(), num_edges(0), undirected(true), 
			visited(), minDis(), distance(), parent(){}

			void welcome(){
				cout<<"Graph\n";
			}

			bool is_undirected(){
				return undirected;
			}

			void set_directed(bool value){
				this->undirected = false;
			}

			void set_distance(vertex u, weight w){
				distance[u] = w;
			}	

			void insert_vertex(vertex vtx){
				G[vtx];
			}

			int num_of_vertices()
			{
				return G.size();
			}

			int num_of_edges(){
				return num_edges;
			}

			void printEdgeWeight(){
				vviterator parentIterator;
				for(parentIterator=parent.begin(); parentIterator!=parent.end(); parentIterator++){
					cout<<parentIterator->first<<" - "<<parentIterator->second<<endl;
				}

			}

			void insert_vertex_degree(vertex u, vertex v)
			{	// u -> v
				// outdegree(u)-->v
				degree_iterator ud = outdegree.find(u);
				if(ud != outdegree.end()) outdegree[u].insert(v);
				else outdegree[v];

				// indegree(v)-->u
				degree_iterator vd = indegree.find(v);
				if(vd != indegree.end()) indegree[v].insert(u);
				else indegree[u];
			}

			void insert_edge(iterator pu, iterator pv, weight w)
			{
				vertex u = pu->first;
				vertex v = pv->first;
				G[u][v] = w;
				insert_vertex_degree(u,v);
			}

			void insert_edge(vertex u, vertex v, weight w = 0)
			{
				vset.insert(u);
				vset.insert(v);

				num_edges++;

				iterator pu = G.find(u);
				if(pu == G.end())
				{
					insert_vertex(u);
					pu = G.find(u);
				}
				
				iterator pv = G.find(v);
				if(pv == G.end())
				{
					insert_vertex(v);
					pv = G.find(v);
				}

				if(is_undirected())
				{
					insert_edge(pu,pv,w);
					insert_edge(pv,pu,w);
				}
				else
				{
					// iterator t = u>v?pu:pv;
					// if(t == pu){
						insert_edge(pu, pv, w);
					// }else{
					// 	insert_edge(pu, t, w);
					// }
				}	
			}

			void printGraph(){
				iterator it = G.begin();
				for(; it!=G.end(); it++){
					cout<<it->first<<"-->";
					ew_iterator newit = (it->second).begin();
					for(; newit!= it->second.end(); newit++){
						cout<<newit->first<<" w("<<newit->second<<") -->";
					}
					cout<<endl;
				}
			}

			//== dijkstra

			void init(){
				vset_iter iter = vset.begin();
				for(; iter!= vset.end(); iter++){
					visited[*iter] = false;
					distance[*iter] = INT_MAX;
					parent[*iter];
					// cout<<visited[*iter]<<" "<<distance[*iter]<<endl;
				}
			}

			void distance_insert(vertex v, weight w){
				minDis.push(make_pair(v,w));
				distance[v] = w;
			}

			void lookfor_connected(vertex v){
				// logger l(true);
				iterator it = G.find(v);
				ew_iterator edgeWeightIt = (it->second).begin();

				weight wt = minDis.top().second;
				minDis.pop();

				for(; edgeWeightIt != it->second.end(); edgeWeightIt++)
				{
					weight w = edgeWeightIt->second + wt;

					// l.log(v, edgeWeightIt->first, edgeWeightIt->second, wt, w, '\n');

					if(distance[edgeWeightIt->first] > w 
						and !visited[edgeWeightIt->first])
					{
						distance_insert(edgeWeightIt->first, w);
						parent[edgeWeightIt->first]=v;
					}
				}

				// mark visited to v
				visited[v] = true;
			}

			vertex select_min_distance_node(){
				return minDis.top().first;
			}

			bool stop(vertex dest, bool dest_defined){
				if(dest_defined){
					if(visited[dest]) return true;
				}
				if(minDis.size() == 0) return true;
				return false;
			}

			//== Bellman Ford

			void reset_visited(){
				for(vset_iter viter= vset.begin(); viter!= vset.end(); viter++){
					visited[*viter] = false;
				}
			}

			void relax_edges(){
				iterator it = G.begin();
				for(; it!= G.end(); it++){
					ew_iterator ewiter = (it->second).begin();
					for(; ewiter!=(it->second).end(); ewiter++){
						weight w = ewiter->second;
						weight wt = w + distance[it->first];
						if(!visited[it->first] and wt < distance[ewiter->first]){
							distance[ewiter->first] = wt;
							parent[ewiter->first] = it->first;
						}
					}
					visited[it->first] = true;
				}
			}
			
			bool check_negative_cycle(vertex v){
				iterator it = G.begin();

				for(; it!=G.end(); it++){ // src
					ew_iterator ewiter = (it->second).begin();
					for(; ewiter!=(it->second).end(); ewiter++){ // dst
						weight w = ewiter->second; // edge src-dst weight
						weight wt = distance[it->first] + w;
						if( distance[it->first] != INT_MAX and wt < distance[ewiter->first]){
							return true;
						}
					}
				}
				return false;
			}
	};

	struct Location{
				int x,y;

				constexpr bool operator == (const Location &b){
					// cout<<this->x<<" "<<this->y<<endl;
					return this->x == b.x and this->y == b.y;
				}

				constexpr bool operator != (const Location &b){
					return !(Location{this->x, this->y} == b);
				}

				constexpr Location operator + (const Location &b) {
					return Location {this->x+b.x, this->y+b.y};
				}

				constexpr bool operator < (const Location &b) {
					return tie(this->x, b.y) < tie(this->x, b.y);
				}

	};			

				ostream& operator<<(ostream& os , const Location& a){
					os << '('<<a.x<<','<<a.y<<")"<<" ";//<<a<<'('<<b.x<<','<<b.y<<')'<<"\n";
					return os;
				}			

	class Hash{
				public:
					size_t operator()(const Location& location)const{
						return location.x * location.y + location.y * location.x;
					}

					size_t operator()(const Location& a, const Location& b)const{
						return (a.x ^ (b.y << 2));
					}
			};

	class Grid{
		public:

			Location start, end;
			int r,c;
			int w;
			
			// will be in use when grids will be drawn
			// struct Property{
			// 	Location location;
			// 	double weight;
			// 	double f;
			// 	double h;
			// 	double g;

			// 	inline void setf(double f){ this->f = f;}
			// 	inline void seth(double h){ this->h = h;}
			// 	inline void setg(double g){ this->g = g;}
			// };

			// Property graph[100][100];
			Location DIR[8] = {
				// Location{1,1}, Location{-1,-1}, 
				// Location{-1,1}, Location{1,-1}, 
				Location{1,0}, Location{0,1},
				Location{-1,0}, Location{0,-1}
			};

			struct PrioriyQueue{
				typedef pair<double, Location> PQElement;
				priority_queue<PQElement, vector<PQElement>, greater<PQElement>> elements;

				inline bool empty() {
					return elements.empty();
				}

				inline void put(Location location, double value){
					elements.emplace(value, location);
				}

				inline Location get() {
					Location location = elements.top().second;
					elements.pop();
					return location;
				}
			};

			
			unordered_map<Location, Location, Hash> came_from;

			unordered_map<Location, double, Hash> cost_so_far;

			Grid(int row, int col, Location start, Location end):
			 r(row), c(col), start(start), end(end) {};

			// will be used when grids will be drawn for visuals
			inline void init(){
				// for(int i=0; i< r; i++){
				// 	for(int j=0; j< c; j++){
				// 		graph[i][j] = Property{Location{i,j}};
				// 	}
				// }


			}

			

			inline bool inbounds(Location newLoc){
				if((1<= newLoc.x and newLoc.x <= r) and (1<= newLoc.y and newLoc.y <= c)) return true;
				else return false;
			}

			vector<Location> neighbors(Location current){
				vector<Location> all;
				for(int i=0; i< 8; i++){
					Location newLoc = current+DIR[i];
					if(inbounds(newLoc)) all.push_back(newLoc);
				}
				return all;
			}

			double heuristic(Location next, Location goal){
				return abs(next.x - goal.x)+ abs(next.y - goal.y);
			}

			void run(){
				PrioriyQueue pq;
				pq.put(start, 0);
				
				came_from[start] = start; // to Location to from Location
				cost_so_far[start] = 0; // initial start node cost 0

				Location current;
				Location prev = start;
				
				while(!pq.empty()){
					current = pq.get();

					if(current == end){
						break;
					}

					for(Location next: neighbors(current)){
						double new_cost = cost_so_far[current] + 5;// let 5 be the cost of moving to other cell //+ cost(current, next);
						if(cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]){
							cost_so_far[next] = new_cost;
							double priority = new_cost + heuristic(next, end);
							pq.put(next, priority);
							came_from[next] = current;
						}
					}	
				}
			}

			void printPath(){
				auto iter = came_from.begin();
				for(; iter!=came_from.end(); iter++){
					cout<<iter->first;
					cout<<iter->second;
					cout<<"\n";
				}
			}
			
	};

}
#endif