//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__
#define STARTIND 1

#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"
#include <functional>

//Implementation of binary_min_heap
template <class ElemType, class Find = equal_to<ElemType>, class Compare = less<ElemType>>
class min_heap
{
private:
	std::vector<ElemType> heap_vec;//heap vector
	Compare comp;// compare function
	Find find; //finding function
	void percolate_up(int hole, ElemType ele);
	void percolate_down(int hole, ElemType ele);
public:
	min_heap();
	~min_heap();
	void push_heap(ElemType ele);
	ElemType pop_heap();
	//get min_key but not pop it.
	ElemType top();
	//get one element
	ElemType get(ElemType ele);
	int find_heap(ElemType ele);
	void remove_heap(ElemType ele);
	void print_heap();
	bool is_empty();
};

template <class ElemType, class Find, class Compare >
min_heap<ElemType, Find, Compare>::min_heap() {
	//start from 1, easy to calculate
	heap_vec.push_back(ElemType{});
}

template <class ElemType, class Find, class Compare>
min_heap<ElemType, Find, Compare>::~min_heap() {}

template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::push_heap(ElemType ele) {
	heap_vec.push_back(ele);
	percolate_up(heap_vec.size() - 1, ele);
}

template <class ElemType, class Find, class Compare>
ElemType min_heap<ElemType, Find, Compare>::pop_heap() {
	if (heap_vec.size() <= 1)
		return ElemType{};
	ElemType min = heap_vec[STARTIND];
	heap_vec[STARTIND] = heap_vec.back();
	//need to minus one from size
	heap_vec.pop_back();
	if (heap_vec.size() >= STARTIND + 1)
		percolate_down(STARTIND, heap_vec[STARTIND]);
	return min;
}

template<class ElemType, class Find, class Compare>
inline ElemType min_heap<ElemType, Find, Compare>::top()
{
	if (heap_vec.size() <= 1)
		return ElemType{};
	return heap_vec[STARTIND];
}

template <class ElemType, class Find, class Compare>
int min_heap<ElemType, Find, Compare>::find_heap(ElemType ele) {
	for (int i = STARTIND; i < heap_vec.size(); i++) {
		if (find(ele, heap_vec[i]))
			return i;
	}
	return -1;
}

template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::remove_heap(ElemType ele) {
	int index = find_heap(ele);
	//No such element
	if (index < 1 || heap_vec.size() < 2) return;
	heap_vec[index] = heap_vec.back();
	//need to minus one from size
	heap_vec.pop_back();
	if (index < heap_vec.size() - 1)
		percolate_down(index, heap_vec[index]);
}

//when insert
template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::percolate_up(int hole, ElemType ele) {
	for (; hole > STARTIND && comp(ele, heap_vec[hole / 2]); hole /= 2)
		heap_vec[hole] = heap_vec[hole / 2];
	heap_vec[hole] = ele;
}

//When remove
template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::percolate_down(int hole, ElemType ele) {
	if (heap_vec.size() <= STARTIND + 1 || hole >= heap_vec.size())
		return;
	int child_index;
	for (; hole * 2 <= heap_vec.size() - 1; hole = child_index) {
		child_index = hole * 2;
		//compare two child
		if (child_index < heap_vec.size() - 2 && comp(heap_vec[child_index + 1], heap_vec[child_index])) {
			//to the one smaller
			child_index++;
		}
		//compare child and element
		if (comp(heap_vec[child_index], ele)) {
			//swap if child small
			heap_vec[hole] = heap_vec[child_index];
		}
		else
			break;
	}
	heap_vec[hole] = ele;
}
template <class ElemType, class Find, class Compare>
void min_heap<ElemType, Find, Compare>::print_heap() {
	for (int i = STARTIND; i < heap_vec.size(); i++) {
		cout << heap_vec[i] << ",";
	}
	cout << endl;
}

template <class ElemType, class Find, class Compare>
bool min_heap<ElemType, Find, Compare>::is_empty() {
	return heap_vec.size() < 2;
}

template <class ElemType, class Find, class Compare>
ElemType min_heap<ElemType, Find, Compare>::get(ElemType ele) {
	int index = find_heap(ele);
	if (index == -1) return ElemType{};
	return heap_vec[index];
}
class h_funcs {
public:
	virtual float calc_heuristic(Util::Point x, Util::Point goal) = 0;
};
class Manhattan_dis :public h_funcs {
public:
	Manhattan_dis() {}
	~Manhattan_dis() {}
	float h_funcs::calc_heuristic(Util::Point x, Util::Point goal) {
		return (abs(goal.x - x.x) + abs(goal.z - x.z));
	}
};
class Euclidean_dis :public h_funcs {
public:
	Euclidean_dis() {};
	~Euclidean_dis() {};
	float h_funcs::calc_heuristic(Util::Point x, Util::Point goal) {
		return (sqrt(pow((goal.x - x.x), 2) + pow((goal.z - x.z), 2)));
	}
};
class heuristic {
public:
	virtual float heuristic_func(Util::Point x, Util::Point goal) = 0;
};
class weight_astar_heuristic : public heuristic {
public:
	weight_astar_heuristic(float w, h_funcs *hf) :hf(hf), weight(w) {}
	~weight_astar_heuristic() {}
	float heuristic::heuristic_func(Util::Point x, Util::Point goal) {
		return hf->calc_heuristic(x, goal) * weight;
	}
private:
	h_funcs *hf;
	float weight;
};
namespace SteerLib
{

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode() {}
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			bool astar_go(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase);
			void generate_node(SteerLib::AStarPlannerNode current, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase);
		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
			float weight;
			typedef struct _open_cmp_ {
				bool operator()(const SteerLib::AStarPlannerNode a, const SteerLib::AStarPlannerNode b) {
					return a.f < b.f;
				}
			}open_cmp;
			typedef struct _open_find_ {
				bool operator()(const SteerLib::AStarPlannerNode a, const SteerLib::AStarPlannerNode b) {
					return (a.point == b.point);
				}
			}node_find;
			typedef struct _g_cmp_ {
				bool operator()(const SteerLib::AStarPlannerNode a, const SteerLib::AStarPlannerNode b) {
					return a.g < b.g;
				}
			}g_cmp;
			min_heap<SteerLib::AStarPlannerNode, node_find, open_cmp> open_list;
			min_heap<SteerLib::AStarPlannerNode, node_find, g_cmp> g_list;
			min_heap<SteerLib::AStarPlannerNode, node_find, open_cmp> close_list;
			heuristic *hn;
	};
}
#endif
