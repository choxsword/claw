#include"fuzzy_controller.h"


Fuzzy_controller::Fuzzy_controller(float e_max,float de_max,float u_max):
target(0),actual(0),emax(e_max),demax(de_max),umax(u_max)
{
   e=target-actual;
   e_pre=0;
   de=e-e_pre;
   Ke=((float)N/2)/emax;
   Kde=((float)N/2)/demax;
   Ku=umax/((float)N/2);
}

Fuzzy_controller::~Fuzzy_controller()
{
  delete [] e_mf_paras;
  delete [] de_mf_paras;
  delete [] u_mf_paras;
}
//三角隶属度函数
float Fuzzy_controller::trimf(float x,float a,float b,float c)
{
   float u;

	if(a==b){
		if(x<=a)
			return 1;
	}
	else if(b==c){
		if(x>=c)
			return 1;
	}

   if(x>=a&&x<=b)
	   u=(x-a)/(b-a);
   else if(x>b&&x<=c)
	   u=(c-x)/(c-b);
   else
	   u=0.0;
   return u;

}

//设置模糊规则
void Fuzzy_controller::setRule(int rulelist[N][N])
{
	for(int i=0;i<N;i++)
	   for(int j=0;j<N;j++)
	     rule[i][j]=rulelist[i][j];
}

//设置模糊隶属度函数的类型和参数
void Fuzzy_controller::setMf(float *e_mf,float *de_mf,float *u_mf)
{
	e_mf_paras=new float [N*3];
	de_mf_paras=new float [N*3];
	u_mf_paras=new float [N*3];
    for(int i=0;i<N*3;i++)
	   e_mf_paras[i]=e_mf[i];
	for(int i=0;i<N*3;i++)
	   de_mf_paras[i]=de_mf[i];
	for(int i=0;i<N*3;i++)
	   u_mf_paras[i]=u_mf[i];
}
//实现模糊控制
float Fuzzy_controller::realize(float t,float a)   
{
	float u_e[N],u_de[N],u_u[N];
	int u_e_index[3],u_de_index[3];//假设一个输入最多激活3个模糊子集
	float u;
	int M;
	target=t;
	actual=a;
    e=target-actual;
	de=e-e_pre;
	e=Ke*e;
	de=Kde*de;
	M=3;               //三角函数有三个参数
	int j=0;
	for(int i=0;i<N;i++)
	{
		u_e[i]=trimf(e,e_mf_paras[i*M],e_mf_paras[i*M+1],e_mf_paras[i*M+2]);//e模糊化，计算它的隶属度
		if(u_e[i]!=0)
            u_e_index[j++]=i;                                              //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_e_index[j]=0;
	j=0;
	for(int i=0;i<N;i++)
	{
		u_de[i]=trimf(de,de_mf_paras[i*M],de_mf_paras[i*M+1],de_mf_paras[i*M+2]);//de模糊化，计算它的隶属度
		if(u_de[i]!=0)
			u_de_index[j++]=i;                                                    //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_de_index[j]=0;

	for(int i=0;i<3;++i){
		cout<<u_e_index[i]<<' ';
	}
	for(int i=0;i<3;++i){
		cout<<u_de_index[i]<<' ';
	}
	cout<<e<<' '<<e_pre<<' '<<de<<' ';
	cout<<endl;

	float den=0,num=0;
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*rule[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}
	u=num/den;
	u=Ku*u;
	if(u>=umax)   u=umax;
	else if(u<=-umax)  u=-umax;
	e_pre=e;
	return u;
}

float Fuzzy_controller::test(float e1,float ec)   
{
	float u_e[N],u_de[N],u_u[N];
	int u_e_index[3]={0,0,0},u_de_index[3]={0,0,0};//假设一个输入最多激活3个模糊子集
	float u;
	int M;
    e=e1;
	de=ec;
	M=3;               //三角函数有三个参数
	int j=0;

	for(int i=0;i<N;i++)
	{
		u_e[i]=trimf(e,e_mf_paras[i*M],e_mf_paras[i*M+1],e_mf_paras[i*M+2]);//e模糊化，计算它的隶属度
		if(u_e[i]!=0)
            u_e_index[j++]=i;                                              //存储被激活的模糊子集的下标，可以减小计算量
	}



	j = 0;
	for (int i = 0; i < N; i++)
	{
		u_de[i] = trimf(de, de_mf_paras[i * M], de_mf_paras[i * M + 1], de_mf_paras[i * M + 2]); //de模糊化，计算它的隶属度
		if (u_de[i] != 0)
			u_de_index[j++] = i; //存储被激活的模糊子集的下标，可以减小计算量
	}


	float den=0,num=0;
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*rule[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}
	u=num/den;

	return u;
}


float Fuzzy_controller::test1(float e1,float ec)   
{
	float u_e[N],u_de[N],u_u[N];
	int u_e_index[3],u_de_index[3];//假设一个输入最多激活3个模糊子集
	float u;
	int M;

	e=e1;
	de=ec;
	M=3;               //三角函数有三个参数
	int j=0;
	for(int i=0;i<N;i++)
	{
		u_e[i]=trimf(e,e_mf_paras[i*M],e_mf_paras[i*M+1],e_mf_paras[i*M+2]);//e模糊化，计算它的隶属度
		if(u_e[i]!=0)
            u_e_index[j++]=i;                                              //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_e_index[j]=0;
	j=0;
	for(int i=0;i<N;i++)
	{
		u_de[i]=trimf(de,de_mf_paras[i*M],de_mf_paras[i*M+1],de_mf_paras[i*M+2]);//de模糊化，计算它的隶属度
		if(u_de[i]!=0)
			u_de_index[j++]=i;                                                    //存储被激活的模糊子集的下标，可以减小计算量
	}
	for(;j<3;j++)u_de_index[j]=0;

	float den=0,num=0;
	for(int m=0;m<3;m++)
		for(int n=0;n<3;n++)
		{
		   num+=u_e[u_e_index[m]]*u_de[u_de_index[n]]*rule[u_e_index[m]][u_de_index[n]];
		   den+=u_e[u_e_index[m]]*u_de[u_de_index[n]];
		}

	
	u=num/den;

	return u;
}