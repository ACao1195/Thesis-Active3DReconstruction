#include <iostream>
#include <string>
#include <sstream>


int main(void){
	using namespace std;

	// for (int i = 0; i < 5; i++){
	// 	cout << i;
	// }

	float defaultPoseArray[5][7] = {
		{0.055569156182, -0.262537902542, 0.712367531044, -0.00158058467431, -0.889907752176, 0.456136657784, 0.00102166242184},
		{0.232617945236, -0.53390833558, 0.545897337126, -0.567089743823, -0.749642003272, 0.276734110523, 0.199660515511},
		{0.149916960276, -0.740201232612, 0.374250976797, -0.83246004276, -0.334308001714, 0.163904798409, 0.410345774021},
		{-0.203529774284, -0.774755913974, 0.361349418812, -0.751897362776, 0.291794586708, -0.199444732106, 0.55653218582},
		{-0.351847196931, -0.172958221408, 0.360015748826, -0.284916478218, 0.832128092577, -0.421435371409, 0.220856663266}
	};

	cout << defaultPoseArray[0][0];




	// float wOrient, xPos, yPos, zPos;
	// std::string stdinString;

	// while(1){

	// std::cout << "Enter w orientation: ";
	// std::getline(std::cin,stdinString);
	// std::stringstream(stdinString) >> wOrient;

	// std::cout << "Enter x position: ";
	// std::getline(std::cin,stdinString);
	// std::stringstream(stdinString) >> xPos;

	// std::cout << "Enter y position: ";
	// std::getline(std::cin,stdinString);
	// std::stringstream(stdinString) >> yPos;

	// std::cout << "Enter z position: ";
	// std::getline(std::cin,stdinString);
	// std::stringstream(stdinString) >> zPos;

	// std::cout << wOrient/xPos;

	// }

	return 0;
}
