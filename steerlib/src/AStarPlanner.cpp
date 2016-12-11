//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}
	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		h_funcs *hf = new Euclidean_dis();
		hn = new weight_astar_heuristic(1.5, hf);
		append_to_path = astar_go(start, goal, _gSpatialDatabase);
		std::cout << append_to_path << std::endl;
		if (append_to_path) {
			SteerLib::AStarPlannerNode goal_point(goal, 0, 0, nullptr);
			SteerLib::AStarPlannerNode path = close_list.get(goal_point);
			agent_path.push_back(goal);
			while (&path != path.parent) {
				agent_path.push_back(path.point);
				path = *(path.parent);
			}
			std::reverse(agent_path.begin(), agent_path.end());
		}
		delete hf;
		delete hn;
		return append_to_path;
		return false;
	}

	bool AStarPlanner::astar_go(Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase) {
		SteerLib::AStarPlannerNode node(start, 0, 0, nullptr);
		node.f = hn->heuristic_func(start, goal);
		node.parent = &node;
		open_list.push_heap(node);
		while (!open_list.is_empty()) {
			SteerLib::AStarPlannerNode tmp = open_list.pop_heap();
			if (goal == tmp.point)
				return true;
			close_list.push_heap(tmp);

			generate_node(tmp, goal, _gSpatialDatabase);
		}
	}
	void AStarPlanner::generate_node(SteerLib::AStarPlannerNode current, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase) {
		std::cout << current.point.x << "," << current.point.z << std::endl;
		for (int i = MAX(current.point.x - 1, 0); i < MIN(current.point.x + 2, _gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
			for (int j = MAX(current.point.z - 1, 0); j < MIN(current.point.z + 2, _gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
				if (i == current.point.x && j == current.point.z) continue;
				int index = _gSpatialDatabase->getCellIndexFromGridCoords(i, j);
		std::cout << index << std::endl;
				if (canBeTraversed(index)) {
		std::cout << index << std::endl;
					SteerLib::AStarPlannerNode succ(Point(i, 0, j), 0, 0, nullptr);
					if (close_list.find_heap(succ) == -1) {
						if (open_list.find_heap(succ) < 0) {
							succ.g = INT_MAX;
							succ.parent = nullptr;
						}
						std::cout << succ.f << std::endl;
						float cost = current.g + _gSpatialDatabase->getTraversalCost(succ.point.x, succ.point.z);
						std::cout << cost << std::endl;
						if (cost < succ.g) {
							open_list.remove_heap(succ);
							SteerLib::AStarPlannerNode tmp(succ.point, cost + hn->heuristic_func(succ.point, goal), cost, &current);
							open_list.push_heap(tmp);
						}
					}
				}
			}
		}
	}
}
