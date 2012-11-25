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
	void ExploringGrid::discover(const Map::MeasurementSet& mset, float theta) {
		std::pair<int, int> dirToAxis = getGridDirToRoboAxis(theta); // from right side to axis

		// sstd::cout << "theta " << theta << " dir " << dirToAxis.first << " " << dirToAxis.second << std::endl;

		std::pair<int, int> leftBack = getCell(mset.leftBack.sensorPos);
		std::pair<int, int> leftFront = getCell(mset.leftFront.sensorPos);
		if (mset.leftFront.valid) { // if we see the wall on the left side, we don't want to set that cell as explored (camera is right)
		 	leftFront = getCell(mset.leftFront.pos);
			leftFront.first = leftFront.first - dirToAxis.first;
			leftFront.second = leftFront.second - dirToAxis.second;
		} else if (!mset.leftBack.valid) { // else we can at least add one cell on the left side to the explored ones
			leftFront.first = leftFront.first + dirToAxis.first;
			leftFront.second = leftFront.second + dirToAxis.second;
		}
		// std::cout << "leftFront " << mset.leftFront.sensorPos.x << " " << mset.leftFront.sensorPos.y << " leftBack " << mset.leftBack.sensorPos.x << " " << mset.leftBack.sensorPos.y << std::endl;
		// std::cout << "rightFront " << mset.rightFront.sensorPos.x << " " << mset.rightFront.sensorPos.y << " rightBack " << mset.rightBack.sensorPos.x << " " << mset.rightBack.sensorPos.y << std::endl;

		if (mset.leftBack.valid) { // if we see the wall on left side, we don't want to set that cell as explored (camera is right)
			leftBack = getCell(mset.leftBack.pos);
			leftBack.first = leftBack.first - dirToAxis.first;
			leftBack.second = leftBack.second - dirToAxis.second;
		} else if (!mset.leftFront.valid) { // else we can at least add one cell on the left side to the explored ones
			leftBack.first = leftBack.first + dirToAxis.first;
			leftBack.second = leftBack.second + dirToAxis.second;
		}


		std::pair<int, int> rightFront;
		if (mset.rightFront.valid) { // we see a wall to the right
			rightFront = getCell(mset.rightFront.pos);
		} else { // no wall thus
			rightFront = getCell(mset.rightFront.sensorPos); 
			// rightFront.first = rightFront.first - dirToAxis.first;
			// rightFront.second = rightFront.second - dirToAxis.second;
		}

		std::pair<int, int> rightBack;
		if (mset.rightBack.valid) {
			rightBack = getCell(mset.rightBack.pos);
		} else {
			rightBack = getCell(mset.rightBack.sensorPos); 
		}


		std::pair<int, int> min;
		min.first = std::min(leftBack.first, std::min(leftFront.first, std::min(rightBack.first, rightFront.first)));
		min.second = std::min(leftBack.second, std::min(leftFront.second, std::min(rightBack.second, rightFront.second)));

		std::pair<int, int> max;
		max.first = std::max(leftBack.first, std::max(leftFront.first, std::max(rightBack.first, rightFront.first)));
		max.second = std::max(leftBack.second, std::max(leftFront.second, std::max(rightBack.second, rightFront.second)));

		std::pair<unsigned int, unsigned int> sanitizedMin;
		sanitizedMin.first = min.first < 0 ? 0 : min.first;
		sanitizedMin.second = min.second < 0 ? 0: min.second;

		std::pair<unsigned int, unsigned int> sanitizedMax;
		sanitizedMax.first = max.first >= mNumCells ? mNumCells - 1 : max.first;
		sanitizedMax.second = max.second >= mNumCells ? mNumCells - 1 : max.second;

		for (unsigned int i = sanitizedMin.first; i <= sanitizedMax.first; ++i) {
			for (unsigned int j = sanitizedMin.second; j <= sanitizedMax.second; ++j) {
				mGrid[i][j] = true;
				mVis.exploringGrid[i].cells[j] = true;
			}
		}
	}

	void ExploringGrid::addUnexploredNodes(Graph& graph) {

	}

	// returns how to change the grid indices to move from the right side in direction of the robot axis
	std::pair<int, int> ExploringGrid::getGridDirToRoboAxis(float theta) {
		std::pair<int, int> result(0,0);
		if ((fabs(theta) <= 0.3f) || (fabs(theta - 2.0f * M_PI) <= 0.3f)) { // theta is small, almost 0
			// result.first = 0;
			result.second = 1;
		} else if (fabs(theta - M_PI/2.0f) <= 0.3f) { // 90 degrees
			result.first = -1;
		} else if (fabs(theta - 3.0f * M_PI / 2.0f) <= 0.3f) { // 270 degrees
			result.first = 1;
		} else if (fabs(theta - M_PI) <= 0.3f) { // 180 degrees
			result.second = -1;
		}
		return result;
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