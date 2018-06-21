#include<iostream>
#include"fuzzy_controller.h"
int main()
{
	float target=60;
	float actual=0;
	float u=0;
	
    Fuzzy_controller fuzzy(3,3,3);
cout<<fuzzy.test(-3,-3)<<endl;

	return 0;
}