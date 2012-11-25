#ifndef EXPLORINGGRID_H
#define EXPLORINGGRID_H

#include "Map.h"
#include "amee/NodeMsg.h"
#include "amee/ExploringGridVis.h"
#include "../Graph/Graph.h"
#include <vector>

namespace amee {

class ExploringGrid {

	public:
		/* Creates a new exploring grid. numCells describes the total number of cells in one row/column (the grid is a square). 
		cellSize the size of one cell (square). The grid is centralized around (0,0). At the beginning all cells are unexplored.
		It is assumed that once a wall is seen on the right side the whole cell in which the wall lies 
		will be explored at some time although it is immediately set as explored. 
		This will happen, if the exploring strategy always follows a wall until it moved a whole cycle. That way the robot is guaranteed 
		to reach the other side of the wall (and thus of the cell) if that side is reachable. 
		*/
		ExploringGrid(int numCells, float cellSize);
		void discover(const Map::MeasurementSet& mset);
		void addUnexploredNodes(Graph& graph);
		ExploringGridVis& getVisualization();

	private:
		std::vector< std::vector<bool> > mGrid;
		ExploringGridVis mVis;
		const float mCellSize;
		const int mNumCells;
		const float mOffset;
		std::pair<unsigned int,unsigned int> getCell(const Map::Point& pos);
	};
}
#endif