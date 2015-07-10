#include"BezierTrajectory.h"
#include<fstream>

 void main() {                  //自己测试用的
		int n=5;
		double point[13]={0,0,0,1,1.49,1.49,2.49,3,1,2.1,3.1,3.11,3};
		BezierPlan Prac;
		Prac.AddTask(n,point);
		int m=0;
		double wheel[2];
		ofstream out("F:/BezierTrajectroy/w_r.txt");
			
		//for(int i=0;i<16644;i++)
		{
			Prac.GetPeriodRef(m,wheel,NULL);
			out<<wheel[0]<<endl;
		} 
		//Prac.GetPeriodRef(m,wheel,NULL);
		system("pause");
		
}	