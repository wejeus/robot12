#ifndef EXPLORINGGRID_H
#define EXPLORINGGRID_H

#include "Map.h"
#include "amee/Pose.h"
#include "amee/NodeMsg.h"
#include "amee/ExploringGridVis.h"
#include "../Graph/Graph.h"
#include <algorithm>
#include <vector>

namespace amee {

class ExploringGrid {

	public:

		struct BlobPoint{
			uint r, c;
		};

		struct Blob {
			std::vector< std::vector<bool> > region;
			uint size;

			Blob(uint size) {
				region.resize(size);
				for(uint i=0; i < size; ++i) {
					region[i].resize(size, false);
					// fill(region[i].begin(), region[i].end(), BLOB_GLOBAL_POS_VISITED);
				}

				this->size = 0;
			}
			void add(uint r, uint c) {
				region[r][c] = true;
				size++;
			}
		};


		/* Creates a new exploring grid. numCells describes the total number of cells in one row/column (the grid is a square). 
		cellSize the size of one cell (square). The grid is centralized around (0,0). At the beginning all cells are unexplored.
		It is assumed that once a wall is seen on the right side the whole cell in which the wall lies 
		will be explored at some time although it is immediately set as explored. 
		This will happen, if the exploring strategy always follows a wall until it moved a whole cycle. That way the robot is guaranteed 
		to reach the other side of the wall (and thus of the cell) if that side is reachable. 
		*/
		ExploringGrid(int numCells, float cellSize);
		void discover(const Map::MeasurementSet& mset, float theta);
		void addUnexploredNodes(Graph& graph);
		bool getNextUnexploredPose(Map& map, Graph& graph, Pose& out_pose, unsigned int& id);
		ExploringGridVis& getVisualization();

		//functions for testing the grid
		void unexploredTester(int startRow, int endRow, int startCol, int endCol);
		void findBlobsTester();
		void printMGrid() ;
		void printGrid(std::vector< std::vector<bool> > &ref, const char * symbol) const;

		

	private:
		std::vector< std::vector<bool> > mGrid;
		ExploringGridVis mVis;
		const float mCellSize;
		const int mNumCells;
		const float mOffset;
		std::pair<unsigned int,unsigned int> getCell(const Map::Point& pos);
		Map::Point getPoint(uint x, uint y);
		std::pair<int, int> getGridDirToRoboAxis(float theta);
		float euclidDist(float x0, float y0, float x1, float y1);

		int findBlobs(std::vector<Blob>& blobs) const;
		void blobExtender(Blob& b, uint r, uint c, std::vector<std::vector<bool> >& visited) const;
	};
}
#endif