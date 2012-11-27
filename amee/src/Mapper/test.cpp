#include <iostream>
#include <cmath>

bool sameTheta(float t1, float t2) {
	return (fabs(t1 - t2) <= 0.3f || fabs(t1 - t2) >= (2 * M_PI - 0.3f));
}

int main(int agrc, char** argv) {
	float theta1 = 0.0f;
	float theta2 = 0.2f;
	
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;
	theta1 = 0.02f; 
	theta2 = 6.3f;
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;
	theta1 = 1.52f; 
	theta2 = 6.2f;
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;
	theta1 = 1.52f; 
	theta2 = 1.3f;
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;

	theta1 = 2.52f; 
	theta2 = 3.3f;
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;

	theta1 = 3.14f; 
	theta2 = 3.6f;
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;

	theta1 = 0.05f; 
	theta2 = 6.1f;
	std::cout << "same theta: " << theta1 << " " << theta2 << " " << sameTheta(theta1, theta2) << std::endl;
}