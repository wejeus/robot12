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
			// leftFront.first = leftFront.first + dirToAxis.first;
			// leftFront.second = leftFront.second + dirToAxis.second;
		}
		// std::cout << "leftFront " << mset.leftFront.sensorPos.x << " " << mset.leftFront.sensorPos.y << " leftBack " << mset.leftBack.sensorPos.x << " " << mset.leftBack.sensorPos.y << std::endl;
		// std::cout << "rightFront " << mset.rightFront.sensorPos.x << " " << mset.rightFront.sensorPos.y << " rightBack " << mset.rightBack.sensorPos.x << " " << mset.rightBack.sensorPos.y << std::endl;

		if (mset.leftBack.valid) { // if we see the wall on left side, we don't want to set that cell as explored (camera is right)
			leftBack = getCell(mset.leftBack.pos);
			leftBack.first = leftBack.first - dirToAxis.first;
			leftBack.second = leftBack.second - dirToAxis.second;
		} else if (!mset.leftFront.valid) { // else we can at least add one cell on the left side to the explored ones
			// leftBack.first = leftBack.first + dirToAxis.first;
			// leftBack.second = leftBack.second + dirToAxis.second;
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

	Map::Point ExploringGrid::getPoint(uint x, uint y){
		Map::Point p;
		p.x = (float)x * mCellSize - mOffset;
		p.y = (float)y * mCellSize - mOffset;

		return p;
	}

	bool ExploringGrid::getNextUnexploredPose(Map &map, Graph& graph, Pose& out_pose, unsigned int& id) {
		std::cout << "ExploringGrid looking for a new unexplored position..." << std::endl;
		std::vector<Blob> blobs;
		int blobIdx = findBlobs(blobs);

		if(blobIdx == -1){
			std::cout << "Could not find any blobs..." << std::endl;
			return false;
		}

		//TODO: look for the closest pose in the blob and go there
		//out_pose = the newly founded pose

		Map::Point start(start_pose);
		std::vector< std::vector<bool> > &b = blobs[blobIdx].region;
		uint size = b.size();

		for(uint x=0; x<size; ++x){
			for(uint y=0; y<size; ++y){
				if (!(b[x][y] & BLOB_GLOBAL_POS_VISITED)) {
					Map::Point end = getPoint(x,y);
					std::vector<NodeMsg*> nodes = graph.getNodes();
					for (unsigned int i = 0; i < nodes.size(); ++i) {
						Map::Point start(nodes[i]->pose);
						if(euclidDist(start.x, start.y, end.x, end.y) < 0.5f && map.isPathCollisionFree(start, end, 0.02f, 0.12f)) {
						out_pose.x = end.x;
						out_pose.y = end.y;
						id = nodes[i]->nodeID;
						return true;
					}	
					}
				}
			}
		}

		std::cout << "Couldn't find close enough pose in the blob" << std::endl;
		return false;
	}

	int ExploringGrid::findBlobs(std::vector<Blob>& blobs) const {
		std::cout << "Looking for blobs..." << std::endl;

		uint rowSize = mGrid.size();
		uint colSize = mGrid[0].size();

		std::vector<std::vector<bool> > visited(rowSize);
		for(uint i=0; i < rowSize; ++i) {
			visited[i].resize(colSize, false);
		}

		for (uint r = 0; r < rowSize; ++r) {
			for (uint c = 0; c < colSize; ++c) {
				if(!mGrid[r][c] && !visited[r][c]) {
					Blob newBlob( (rowSize > colSize) ? rowSize : colSize);

					visited[r][c] = true;
					blobExtender(newBlob, r, c, visited);
					blobs.push_back(newBlob);
				}
			}
		}

		uint largest = 0;
		uint sndLargest = 0;
		int largestBlobIdx = -1;
		int sndLargestIdx = -1;
		for(uint i=0; i<blobs.size(); ++i){
			if(blobs[i].size > sndLargest){
				if (blobs[i].size > largest) {
					sndLargest = largest;
					sndLargestIdx = largestBlobIdx;
					largest = blobs[i].size;
					largestBlobIdx = i;
				} else {
					sndLargest = blobs[i].size;
					sndLargestIdx = i;
				}
			}
		}

		if(sndLargestIdx != -1)
			std::cout << "Found " << blobs.size() << " blobs, with the snd largest being: " << sndLargest << std::endl;

		return sndLargestIdx;
	}

	void ExploringGrid::blobExtender(Blob& b, uint r, uint c, std::vector<std::vector<bool> >& visited) const {
		b.add(r,c);

		//calculate indices - prevet out of boundary access
		uint ur = r+1 < mGrid.size() ? r+1 : r; //upper row
		uint lr = r == 0 ? r : r-1; //lower row
		uint lc = c == 0 ? c : c-1; //left column
		uint rc = c+1 < mGrid[0].size() ? c+1: c; //right col

		//all the surounding pixles
		BlobPoint p[8] = {{ur,lc},{ur,c},{ur,rc},{r,lc},{r,rc},{lr,lc},{lr,c},{lr,rc}};

		for(uint i=0; i<8; ++i){

			if(!visited[p[i].r][p[i].c]){
				visited[p[i].r][p[i].c] = true;

				if(!mGrid[p[i].r][p[i].c])
					blobExtender(b, p[i].r, p[i].c, visited);
			}
		}
	}


	float ExploringGrid::euclidDist(float x0, float y0, float x1, float y1){
		return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
	}




// ###########################################################
//These functions are for testing purpose only
	void ExploringGrid::unexploredTester(int startRow, int endRow, int startCol, int endCol){
		for(int i=startRow; i <= endRow; ++i){
			for(int j=startCol; j <= endCol; ++j){
				mGrid[i][j] = true;
			}
		}
	}

	void ExploringGrid::findBlobsTester(){
		std::vector<Blob> blobs;
		int blobIdx = findBlobs(blobs);

		std::cout << "Blob found: " << (char)blobIdx << std::endl;
		// const char * symbol = (char*)&blobIdx;
		printGrid(blobs[blobIdx].region, "B ");
	}

	void ExploringGrid::printMGrid() {
		printGrid(mGrid, "# ");
	}

	void ExploringGrid::printGrid(std::vector< std::vector<bool> > & g, const char * symbol) const {
		uint rSize = g.size();
		uint cSize = g[0].size();

		std::cout << "################ start of print ############### " << std::endl;

		for(uint c =0; c< cSize; ++c) std::cout << "__";
		std::cout << std::endl;

		for(uint r = 0; r < rSize; ++r){
			cSize = g[r].size();
			std::cout << "|";
			for(uint c = 0; c < cSize; ++c){
				std::cout << (g[r][c] ? symbol : "  ");
			}

			std::cout << "|" << std::endl;
		}

		for(uint c =0; c< cSize; ++c) std::cout << "__";
		std::cout << std::endl;

		std::cout << "################ end of print #################" << std::endl;
	}
	
// ###########################################################