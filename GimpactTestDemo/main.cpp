
#include "GimpactTestDemo.h"
#include "GlutStuff.h"
#include "stdafx.h"
#include <cstdlib>
#include <math.h>
#include <cmath>
#include "optimization.h"

#include <iostream>
#include <fstream>
//################################## main #####################################
int main(int argc,char** argv)
{
	std::cout.setstate(std::ios::failbit);
		
		GimpactConcaveDemo* concaveDemo = new GimpactConcaveDemo();
		concaveDemo->initPhysics();
		
		/*
		btVector3 origin(3,23,-19);
		btVector3 angular;
		std::ofstream myfile;
		myfile.open("example_debug.txt",std::ios::out);
		int data_num = 0;
		for (int lin_x = 23; lin_x < 40; lin_x++) {
				for (int lin_y = 4; lin_y < 20; lin_y++) {
					for (double wx = -1; wx < 2.14; ) {
						wx += 1;
						for (double wy = -1; wy < 2.14; ) {
							wy += 1;
							for (double wz = -1; wz < 2.14;) {
								wz += 1;
								std::cout << "new\n";
								GimpactConcaveDemo* concaveDemo = new GimpactConcaveDemo();  /// This will not be Deleted!!!																							 //concaveDemo->initPhysics();
								concaveDemo->convergence_step = 0;
								origin.setX(lin_x + 3);
								origin.setY(lin_y + 23);
								origin.setZ( origin.getZ());
								angular.setX(wx);
								angular.setY(wy);
								angular.setZ(wz);
								bool flag;
								flag = concaveDemo->DebugDataTest(origin,angular);
								int step = concaveDemo->convergence_step;
								std::string stt = std::to_string(step);
								if (flag) { 
									std::string datas = std::to_string(data_num);
									std::string str1 = std::to_string(lin_x +3);
									std::string str2 = std::to_string(lin_y +23);
									std::string str3 = std::to_string(origin.getZ());
									std::string str4 = std::to_string(wx);
									std::string str5 = std::to_string(wy);
									std::string str6 = std::to_string(wz);
									myfile << datas << ":" << str1 << "," << str2 << "," << str3 << "," << str4 << "," << str5 << "," << str6 << "," << "1" << "," << step << std::endl;
									
								}
								else { 
									std::string datas = std::to_string(data_num);
									std::string str1 = std::to_string(lin_x + 3);
									std::string str2 = std::to_string(lin_y + 23);
									std::string str3 = std::to_string(origin.getZ());
									std::string str4 = std::to_string(wx);
									std::string str5 = std::to_string(wy);
									std::string str6 = std::to_string(wz);
									myfile << datas << ":" << str1 << "," << str2 << "," << str3 << "," << str4 << "," << str5 << "," << str6 << "," << "0" << "," << step << std::endl;

								}
								
								myfile.flush();
								data_num++;
								std::cout << "delete"<<concaveDemo<<"\n";
								delete concaveDemo;
								std::cout << "delete\n";
	
							}
						}
					}
			}
		}
		myfile.close();
		return 0;
		*/
       return glutmain(argc, argv,640,480,"DevO,s GIMPACT Test Demo",concaveDemo);
}
