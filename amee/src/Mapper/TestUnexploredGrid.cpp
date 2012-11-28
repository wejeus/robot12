#include <iostream>
#include "ExploringGrid.h"

using namespace amee;
using namespace std;

int main(int argc, char ** argv){

	cout << "Testing UnexploredGrid..." << endl;
	
	ExploringGrid eg(50, 0.01f);

	cout << "Print grid before: " << endl;
	eg.printMGrid();

	eg.unexploredTester(0 , 1, 0,19);//row above
	eg.unexploredTester(18,19, 0,19);//row below
	eg.unexploredTester(0 ,19, 0,1 );//col left
	eg.unexploredTester(0 ,19, 18,19);//col right

	cout << "Print grid after: " << endl;
	eg.printMGrid();


	eg.findBlobsTester();

	
	return 0;
}
