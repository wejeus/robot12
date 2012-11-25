#include "ExploringGrid.h"
#include "amee/ExploringGridColumn.h"
#include <cmath>

using namespace amee;

	ExploringGrid::ExploringGrid(int numCells, float cellSize) : mCellSize(cellSize), mNumCells(numCells), mOffset(mNumCells * mCellSize / 2.0f) {

		std::vector<bool> column;
		column.resize(numCells, false);
		mGrid.resize(numCells, column);
		ExploringGridColumn colVis;
		colVis.cells.resize(numCells, false);
		mVis.exploringGrid.resize(numCells, colVis);

		mVis.originX = -mOffset;
		mVis.originY = -mOffset;
		mVis.cellSize = mCellSize;
		mVis.numCells = numCells;
	}

	// only call if not rotating
	void ExploringGrid::discover(const Map::MeasurementSet& mset) {
		std::pair<unsigned int, unsigned int> leftBack = getCell(mset.leftBack.sensorPos);
		std::pair<unsigned int, unsigned int> leftFront = getCell(mset.leftFront.sensorPos);
		std::pair<unsigned int, unsigned int> rightFront;
		if (mset.rightFront.valid) {
			rightFront = getCell(mset.rightFront.pos);
		} else {
			rightFront = getCell(mset.rightFront.sensorPos); // TODO here we can discover more cells!
		}

		std::pair<unsigned int, unsigned int> rightBack;
		if (mset.rightBack.valid) {
			rightBack = getCell(mset.rightBack.pos);
		} else {
			rightBack = getCell(mset.rightBack.sensorPos); // TODO here again
		}

		// TODO left cells we can discover some cells to the left too!


		std::pair<unsigned int, unsigned int> min;
		min.first = std::min(leftBack.first, std::min(leftFront.first, std::min(rightBack.first, rightFront.first)));
		min.second = std::min(leftBack.second, std::min(leftFront.second, std::min(rightBack.second, rightFront.second)));

		std::pair<unsigned int, unsigned int> max;
		max.first = std::max(leftBack.first, std::max(leftFront.first, std::max(rightBack.first, rightFront.first)));
		max.second = std::max(leftBack.second, std::max(leftFront.second, std::max(rightBack.second, rightFront.second)));

		for (unsigned int i = min.first; i <= max.first; ++i) {
			for (unsigned int j = min.second; j <= max.second; ++j) {
				mGrid[i][j] = true;
				mVis.exploringGrid[i].cells[j] = true;
			}
		}
	}

	void ExploringGrid::addUnexploredNodes(Graph& graph) {

	}

	ExploringGridVis& ExploringGrid::getVisualization() {
		return mVis;
	}

	std::pair<unsigned int, unsigned int> ExploringGrid::getCell(const Map::Point& pos) {
		
		float tx = pos.x + mOffset;
		float ty = pos.y + mOffset;

		std::pair<unsigned int,unsigned int> cell;
		cell.first = floor(tx / mCellSize);
		cell.second = floor(ty / mCellSize);
		return cell;
	}