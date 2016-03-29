#include "stm32f30x.h"
#include "stm32f30x_it.h"
#include<stdio.h>
#include<math.h>
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include <string.h>

#ifndef _ACC_C_
#define _ACC_C_
 

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define PI                         (float)     3.14159265f

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
L3GD20_InitTypeDef L3GD20_InitStructure;
L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

void Acc_Config(void);
void Acc_ReadData(float* pfData);
void TIM_Config(void);
void gyroConfig(void);
void gyroReadAngRate (float* pfData);
void GPIO_Config();
void compassConfig(void);
void compassReadMag (float* pfData);

uint16_t PrescalerValue = 0,read=0;
float avg=0.0f,ratio=0.0f;
float pitch=0.0f,pitch_acc=0.0f,set_pitch=0.0f,error_pitch[2]={0.0f},sum_pitch=0.0f;
float roll=0.0f,roll_acc=0.0f,set_roll=0.0f,error_roll[2]={0.0f},sum_roll=0.0f;
float yaw=0.0f,yaw_acc=0.0f,set_yaw=0.0f,error_yaw[2]={0.0f},sum_yaw=0.0f;
float u=0.0f,u_roll=0.0f,u_pitch=0.0f,u_yaw=0.0f,kp=0.0f,ki=0.0f,kd=0.0f,ccr;
//float k_p[3]={0.0,0.0,0.87},k_i[3]={0.0,0.0044,0.09},k_d[3]={0.0,0.0,0.28};
float k_p[3]={0.72,0.56,1.07},k_i[3]={0.0629,0.09,0.6},k_d[3]={0.24,0.29,0.28};        //stable values
//float k_p[3]={0.56,0.37,2.7},k_i[3]={0.0629,0.0044,0},k_d[3]={0.122,0.109,0.0};
//float k_p[3]={0.0,0.0,0.0},k_i[3]={0.0,0.0,0.0},k_d[3]={0.0,0.0,0.0};
float ccr_roll=0.0f,ccr_pitch=0.0f,ccr_yaw=0.0f,meas_roll=0.0f,meas_pitch=0.0f,meas_yaw=0.0f;
float ccr_front_motor=0.0f,ccr_back_motor=0.0f,ccr_left_motor=0.0f,ccr_right_motor=0.0f,CCR_prev=500.0f;
float meas[3]={0.0f};
float mpitch=0,mroll=0,myaw=0;

float ccr_throttle=0.0f;
float AccBuffer[3] = {0.0f};
float W=0.90;
float x=0.0f,y=0.0f,z=0.0f;
float sum_counts=0.0f;
int z1=0;

float a=0.0f,b=0.0f;
int counts=0;

float Axz_0=0.0f,Ayz_0=0.0f,Axz,Ayz,Rxest,Rxest_0=0.0f,Ryest,Ryest_0=0.0f,Rzest,Rzest_0=0.0f,Rate_Axz,Rate_Axz_0=0.0f;
float Rate_Ayz,Rate_Ayz_0=0.0f,Rate_Axz_avg,Rate_Ayz_avg;
float Rx_gyro,Ry_gyro,Rz_gyro,Rx_acc,Ry_acc,Rz_acc,Roll_gyro,Pitch_gyro,kangle=0.0f;

float GyroBuffer[3] = {0.0f},Gyrooffset[3]={0.0f},Gyrooffset_1[3]={0.0f},Gyrooffset_prev[3]={0.0f},Gyroavg[3],Gyroavg_1[3]={0.0f},Gyroavg_2[3]={0.0f},Gyroangle[3] = {0.0f},Accavg[3]={0.0f},Accavg_1[3]={0.0f},t=1.0f/50.0f,Final_gyro[3],observe[100];
float Mag_abs=0.0f,Magavg[3]={0.0f},Magavg_1[3]={0.0f};
int n=0,k=0,count=0,read5=0,check=0,counter=0,readsensor=0,counter2=0,retard=0,delayy=2,pass=0;
char m=1;
float MagBuffer[3] = {0.0f},mag=0.0f,dir=0.0f,initdir=0.0f;
float Mag_avg=0.0f,mag_X=0.0f,mag_Y=0.0f,mag_Z=0.0f,Yh=0.0f,Xh=0.0f;
float gap=0.0;
float temp,temp1;   //for temporary use

int fputc(int ch, FILE *f)
{
    return(ITM_SendChar(ch));
}

int flag=0;


uint16_t CCR1,i,go,start;
uint32_t ii;

uint8_t update_flag;
int takeaction=0;
int dutta=0;

uint16_t CCR1_Val = 0;
uint16_t CCR2_Val = 0;
uint16_t CCR3_Val = 0;
uint16_t CCR4_Val = 0;

unsigned int edge1=0,edge2=0,edge3=0,edge4=0,edge5=0,edge6=0,frequency=0,CCR=0,init=54,final=35949,CCRshow=0,flag1=0;
float Capture[2]={0.0f},CCR_3=0.0f,CCR_2=0.0f,gap1=0.0f;
int counts1=0,start1=0,check1=0,counter1=0;
int counts2=0,check2=0;

//variables for Extended Kalman filter
float q=0.0f,q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f,q0_prev=0.0f,q1_prev=0.0f,q2_prev=0.0f,q3_prev=0.0f;
float F[4][4],g_est[3][1],temp_mat0[4][4],temp_mat1[4][4],temp_mat2[4][4],H[3][4],S[3][3],K[4][3],KH[4][4],P_temp1[4][4],P_temp2[4][4],P_temp3[4][4];
float wx,wy,wz,acc_x,acc_y,acc_z;
float Q[4]={0.0001,0.0001,0.0001,0.0001},R[3]={2.0,2.0,2.0};
float acc[3],y_ekf[3],q_temp[4];
float I[4][4]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
float L[3][3],U[3][3],inv_S[3][3],x_LU[3],z_LU[3];
float P[4][4]={0.5,0.003,0.003,0.003,0.003,0.5,0.003,0.003,0.003,0.003,0.5,0.003,0.003,0.003,0.003,0.5};


int main(void)

{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM6, ENABLE);
	
  gyroConfig();
	Acc_Config();
  TIM_Config();
  GPIO_Config();  
	compassConfig();
	
	/* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
	
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
		TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	
	kangle=180.0f/3.14;
	
	
  /* Infinite loop */
   while (1)
  {
		
		
	
		
		if(takeaction==1)
		{
			takeaction=0; 
			ccr_throttle=CCR;
      
			

		  TIM3->CCR1=CCR1_Val;     //front
			TIM3->CCR4=CCR2_Val;     //back
 			TIM3->CCR2=CCR3_Val;     //left
			TIM3->CCR3=CCR4_Val;     //right
			
    }			
 			   			
	
	if(readsensor==1)		
	{
		readsensor=0;	
		
		gyroReadAngRate(GyroBuffer);
		Acc_ReadData(AccBuffer);
		compassReadMag(MagBuffer);	
		
		if(n<500)			
				{
					for(k=0; k<3; k++)
					{
 					 	Gyroavg_2[k]+=GyroBuffer[k];	
 				  }
				}
			n++;
			
		if(n==500)
			{
				
				for(k=0; k<3; k++)
					{
 					 	Gyrooffset[k]=Gyroavg_2[k]/500.0f;
								Gyrooffset_1[k]=Gyrooffset[k];
						if(-2.0f>Gyrooffset[k]||Gyrooffset[k]>2.0f)
							Gyrooffset[k]=Gyrooffset_prev[k];
						Gyrooffset_prev[k]=Gyrooffset[k];
						Gyroavg_2[k]=0.0f;	
 				  }
					
			}	
				
		if(n>=500)
			{
				n=521;
				read5++;
				
				if(read5==6)
					read5=1;
				
 				for(k=0; k<3; k++)
 				{
 						Gyroavg[k]+=read5*GyroBuffer[k];
						Accavg[k]+=read5*AccBuffer[k];
						Magavg[k]+=read5*MagBuffer[k];
 				}
				
				
				if(read5==5)
				{
					
					 
					
						for(k=0; k<3; k++)
						{
							Gyroavg_1[k]=Gyroavg[k]/15.0f;
							//temp=(float)((Gyroavg[k]-Gyrooffset[k]*5.0)/15.0f*t);
							temp=(float)((Gyroavg_1[k]-Gyrooffset[k])*t);
							//Gyroangle[k]=-1.0*meas[k]+temp;	
							Gyroangle[k]=-1.0f*meas[k]+temp;	
							//Gyrorate[k]=Gyroavg[k]/5.0f-Gyrooffset[k];
							Gyroavg[k]=0.0f;
						Accavg_1[k]=Accavg[k]/15.0f;
						Accavg[k]=0;
						
						Magavg_1[k]=Magavg[k]/15.0f;
						Magavg[k]=0;	
							
						}
						//0-pitch 1-roll 2-yaw
						
							
 				
				
							x=(0.001*Accavg_1[0])+(0.0197);
							y=(0.001*Accavg_1[1])-(0.0119);
							z=(0.001*Accavg_1[2])+(0.0508);
					
// 							x=(0.01*AccBuffer[0])+(0.0197);
// 							y=(0.01*AccBuffer[1])-(0.0119);
// 							z=(0.01*AccBuffer[2])+(0.0508);
// 				
							avg=(float)sqrt((((float)x)*((float)x))+(((float)y)*((float)y))+(((float)z)*((float)z)));
							roll_acc=(asin((y)/avg))*kangle;
							pitch_acc=(asin((x)/avg))*kangle;
						
						
						
						


						//meas[0]=-1.0*Gyroangle[0]*0.98 + pitch_acc*0.02;
						//meas[1]=-1.0*Gyroangle[1]*0.98 + roll_acc*0.02;


						
/*************************************************EXTENDED KALMAN FILTER**************************************************/

wx=(3.14/180)*(Gyroavg_1[1]-Gyrooffset_1[1]);
wy=(3.14/180)*(Gyroavg_1[0]-Gyrooffset_1[0]);
wz=(3.14/180)*(Gyroavg_1[2]-Gyrooffset_1[2]);
acc_x=9.8*x;
acc_y=9.8*y;
acc_z=9.8*z;

//computing Jacobian for dynamics matrix

//F=[0,-(0.5*wx),-(0.5*wy),-(0.5*wz);
  //  (0.5*wx),0,(0.5*wz),-(0.5*wy);
   // (0.5*wy),(-0.5*wz),0,(0.5*wx);
    //(0.5*wz),(0.5*wy),-(0.5*wx),0];

F[0][0]=1;
F[0][1]=-(0.5*wx);
F[0][2]=-(0.5*wy);
F[0][3]=-(0.5*wz);
F[1][0]=0.5*wx;
F[1][1]=1;
F[1][2]=0.5*wz;
F[1][3]=-(0.5*wy);
F[2][0]=0.5*wy;
F[2][1]=-(0.5*wz);
F[2][2]=1;
F[2][3]=(0.5*wx);
F[3][0]=(0.5*wz);
F[3][1]=(0.5*wy);
F[3][2]=-(0.5*wx);
F[3][3]=1;

// //Quaternion Prediction 
q=sqrt((q0*q0)+(q1*q1)+(q2*q2)+(q3*q3));
q0=q0/q;
q1=q1/q;
q2=q2/q;
q3=q3/q;
q0_prev=q0;
q1_prev=q1;
q2_prev=q2;
q3_prev=q3;
q0=q0_prev+0.5*((-wx*q1_prev)-(wy*q2_prev)-(wz*q3_prev))*(t);
q1=q1_prev+0.5*((wx*q0_prev)+(wz*q2_prev)-(wy*q3_prev))*(t);
q2=q2_prev+0.5*((wy*q0_prev)-(wz*q1_prev)+(wx*q3_prev))*(t);
q3=q3_prev+0.5*((wz*q0_prev)+(wy*q1_prev)-(wx*q2_prev))*(t);


//q=[q0;q1;q2;q3];

//Covariance Prediction
//P=(F*P*F.')+Q;

for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=3;j++)
			{
				temp_mat0[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						temp_mat0[i][j]+=F[i][k]*P[k][j];
					}
			}
	}
	
for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=3;j++)
			{
				P_temp3[i][j]=P[i][j];
				P[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						P[i][j]+=temp_mat0[i][k]*P_temp3[j][k];
						//temp_mat0[i][k]=0;
					}
					if(i==j)
							P[i][j]+=Q[i];
			}
	}
			

//Jacobian for measurement matrix

H[0][0]=(-q2);
H[0][1]=(q3);
H[0][2]=(-q0);
H[0][3]=(q1);
H[1][0]=(q1);
H[1][1]=(q0);
H[1][2]=(q3);
H[1][3]=(q2);
H[2][0]=(-q0);
H[2][1]=(q1);
H[2][2]=(q2);
H[2][3]=(-q3);


q_temp[0]=q0;
q_temp[1]=q1;
q_temp[2]=q2;
q_temp[3]=q3;


//measurement
//g_est=H*q;
for(int i=0;i<=2;i++)
	{
		for(int j=0;j<=0;j++)
			{
				g_est[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						g_est[i][j]+=H[i][k]*q_temp[k];
					}
					g_est[i][j]=9.8*g_est[i][j];
			}
	}



acc[0]=acc_x;           //measurement from accelerometer
acc[1]=-acc_y;
acc[2]=-acc_z;
	
	
y_ekf[0]=acc[0]-g_est[0][0];     //innovation
y_ekf[1]=acc[1]-g_est[1][0];
y_ekf[2]=acc[2]-g_est[2][0];
	
//H=2*H;
	
for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=2;j++)
			{
				H[i][j]=2*9.8*H[i][j];
			}
	}
			

//S=H*P*H.' + R;

for(int i=0;i<=2;i++)
	{
		for(int j=0;j<=3;j++)
			{
				temp_mat1[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						temp_mat1[i][j]+=H[i][k]*P[k][j];
					}
			}
	}
	
for(int i=0;i<=2;i++)
	{
		for(int j=0;j<=2;j++)
			{
				S[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						S[i][j]+=temp_mat1[i][k]*H[j][k];
					}
					if(i==j)
							S[i][j]+=R[i];
			}
	}
	
	
//K=P*H.'*inv(S);       //kalman gain

/***************inverse of a matrix***********************/

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (j < i)
                L[j][i] = 0;
            else
            {
                L[j][i] = S[j][i];
                for (int k = 0; k < i; k++)
                {
                    L[j][i] = L[j][i] - L[j][k] * U[k][i];
                }
            }
        }
        for (int j = 0; j < 3; j++)
        {
            if (j < i)
                U[i][j] = 0;
            else if (j == i)
                U[i][j] = 1;
            else
            {
                U[i][j] = S[i][j] / L[i][i];
                for (int k = 0; k < i; k++)
                {
                    U[i][j] = U[i][j] - ((L[i][k] * U[k][j]) / L[i][i]);
                }
            }
        }
     }


//forward substitution for col 1
z_LU[0]=1/L[0][0];
z_LU[1]=-(L[1][0]*z_LU[0])/L[1][1];
z_LU[2]=(-(L[2][0]*z_LU[0])-(L[2][1]*z_LU[1]))/L[2][2];
//backward substitution for col 1
inv_S[2][0]=z_LU[2]/U[2][2];
inv_S[1][0]=(z_LU[1]-(U[1][2]*inv_S[2][0]))/U[1][1];
inv_S[0][0]=(z_LU[0]-(U[0][1]*inv_S[1][0])-(U[0][2]*inv_S[2][0]))/U[0][0];

//forward substitution for col 2
z_LU[0]=0;
z_LU[1]=1/L[1][1];
z_LU[2]=(-(L[2][1]*z_LU[1]))/L[2][2];
//backward substitution for col 2
inv_S[2][1]=z_LU[2]/U[2][2];
inv_S[1][1]=(z_LU[1]-(U[1][2]*inv_S[2][1]))/U[1][1];
inv_S[0][1]=(z_LU[0]-(U[0][1]*inv_S[1][1])-(U[0][2]*inv_S[2][1]))/U[0][0];

//forward substitution for col 3
z_LU[0]=0;
z_LU[1]=0;
z_LU[2]=1/L[2][2];
//backward substitution for col 3
inv_S[2][2]=z_LU[2]/U[2][2];
inv_S[1][2]=(z_LU[1]-(U[1][2]*inv_S[2][2]))/U[1][1];
inv_S[0][2]=(z_LU[0]-(U[0][1]*inv_S[1][2])-(U[0][2]*inv_S[2][2]))/U[0][0];

for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=2;j++)
			{
				temp_mat2[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						temp_mat2[i][j]+=P[i][k]*H[j][k];
					}
			}
	}

for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=2;j++)
			{
				K[i][j]=0;
				for(int k=0;k<=2;k++)
					{
						K[i][j]+=temp_mat2[i][k]*inv_S[k][j];
					}
					//K[i][j]=temp_mat2[i][j];
			}
	}
	
	
	
//Measurement Update

	
//q=q+(K*y);
q0=q0+((K[0][0]*y_ekf[0])+(K[0][1]*y_ekf[1])+(K[0][2]*y_ekf[2]));
q1=q1+((K[1][0]*y_ekf[0])+(K[1][1]*y_ekf[1])+(K[1][2]*y_ekf[2]));
q2=q2+((K[2][0]*y_ekf[0])+(K[2][1]*y_ekf[1])+(K[2][2]*y_ekf[2]));
q3=q3+((K[3][0]*y_ekf[0])+(K[3][1]*y_ekf[1])+(K[3][2]*y_ekf[2]));
	
	
	
//P=(I-K*H)*P;
for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=3;j++)
			{
				KH[i][j]=0;
				for(int k=0;k<=2;k++)
					{
						KH[i][j]+=K[i][k]*H[k][j];
					}
			}
	}
	
for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=3;j++)
			{
				P_temp1[i][j]=I[i][j]-KH[i][j];
				P_temp2[i][j]=P[i][j];
			}
	}
	
	

for(int i=0;i<=3;i++)
	{
		for(int j=0;j<=3;j++)
			{
				//P_temp2[i][j]=P[i][j];
				P[i][j]=0;
				for(int k=0;k<=3;k++)
					{
						P[i][j]+=P_temp1[i][k]*P_temp2[k][j];
					}
			}
	}
	
	

//quaternion to Euler angles
//[yaw pitch roll]=quat2angle(q);
meas[1]=-(180/3.14)*atan2((2*(q0*q1+q2*q3)),(1-2*(q1*q1+q2*q2)));
meas[0]=-(180/3.14)*asin((2*(q0*q2-q3*q1)));
//pitch=0;
//yaw=(180/3.14)*atan2((2*(q0*q3+q1*q2)),(1-2*(q2*q2+q3*q3)));


/************************************************************************************************************************/				
				


				
						
						Mag_abs=sqrt((Magavg_1[0]*Magavg_1[0])+(Magavg_1[1]*Magavg_1[1])+(Magavg_1[2]*Magavg_1[2]));
						mag_X=MagBuffer[0]/Mag_abs;
						mag_Y=MagBuffer[1]/Mag_abs;
						mag_Z=MagBuffer[2]/Mag_abs;
						
						a=(mag_Y*cos(meas[1]/kangle));
						b=(mag_Z*sin(meas[1]/kangle));
						Yh=a-b;
						Xh=mag_X*cos(meas[0]/kangle)+mag_Y*sin(meas[1]/kangle)*sin(meas[0]/kangle)+mag_Z*cos(meas[1]/kangle)*sin(meas[0]/kangle);
						dir=atan(Yh/Xh)*(kangle);
						if(Xh>0&&Yh>0)
							dir=-180+dir;
						if(Xh>0&&Yh<0)
							dir=180+dir;
						
 						if(m==1)
 							initdir=dir;
 						
 						m=2;
						temp1=dir-initdir;
						meas[2]=-1.0*Gyroangle[2]*0.98+(dir-initdir)*0.02;
				
				if(ccr_throttle>20)
				{
				
				error_roll[0]=error_roll[1];
				error_roll[1]=meas[1]-set_roll;
				sum_roll=sum_roll+(error_roll[1]*t);
				
				//for CCR=550, Kp=0.56,Kd=0.3	
					
				//if(error_roll[1]>1||error_roll[1]<-1)
				//kp=0.01795,ki=0,kd=0.0055
				ccr_roll=k_p[0]*error_roll[1] + k_i[0]*sum_roll + k_d[0]*(error_roll[1]-error_roll[0])/t;
				//ccr_roll=0*error_roll[1] + 0*sum_roll + 0.3*GyroBuffer[1];
				//else
				//ccr_roll=0;
				
				//ccr_roll=(250*u_roll)/7.7;
				
				error_pitch[0]=error_pitch[1];
				error_pitch[1]=meas[0]-set_pitch;
				sum_pitch=sum_pitch+(error_pitch[1]*t);

				//if(error_pitch[1]>1||error_pitch[1]<-1)
				//kp=0.039,ki=0,kd=0.009, tested for CCR 580
				ccr_pitch=k_p[1]*error_pitch[1] + k_i[1]*sum_pitch + k_d[1]*(error_pitch[1]-error_pitch[0])/t;
				//else
				//ccr_pitch=0;
				
				//ccr_pitch=(250*u_pitch)/7.7;
				
				error_yaw[0]=error_yaw[1];
				error_yaw[1]=meas[2]-set_yaw;
				sum_yaw=sum_yaw+(error_yaw[1]*t);

				//if(error_yaw[1]>0.5||error_yaw[1]<-0.5)
				//kp=0.02000,ki=0.00199,kd=0.0100	
				ccr_yaw=k_p[2]*error_yaw[1] + k_i[2]*sum_yaw + k_d[2]*(error_yaw[1]-error_yaw[0])/t;
				//u_yaw=0;
				//else
				//u_yaw=0;
				
				//ccr_yaw=(250*u_yaw)/7.7;
				
			
				mpitch=meas[0];
				mroll=meas[1];
				myaw=meas[2];
				
				
				ccr_front_motor=ccr_throttle+ccr_pitch-ccr_yaw;
					if(ccr_front_motor>850)
						ccr_front_motor=850;
					if(ccr_front_motor<0)
						ccr_front_motor=20;
				
				ccr_back_motor=ccr_throttle-ccr_pitch-ccr_yaw;
					if(ccr_back_motor>850)
						ccr_back_motor=850;
					if(ccr_back_motor<0)
						ccr_back_motor=20;
					
				ccr_left_motor=ccr_throttle+ccr_roll+ccr_yaw;
					if(ccr_left_motor>850)
						ccr_left_motor=850;
					if(ccr_left_motor<0)
						ccr_left_motor=20;
					
				ccr_right_motor=ccr_throttle-ccr_roll+ccr_yaw;
					if(ccr_right_motor>850)
						ccr_right_motor=850;
					if(ccr_right_motor<0)
						ccr_right_motor=20;
				}
				else
				{
					ccr_front_motor=ccr_throttle;
					ccr_back_motor=ccr_throttle;
					ccr_left_motor=ccr_throttle;
					ccr_right_motor=ccr_throttle;
				}
			CCR1_Val = ccr_front_motor;
			CCR2_Val = ccr_back_motor;
			CCR3_Val = ccr_left_motor;
			CCR4_Val = ccr_right_motor;
			
		//	printf("\n%f",meas[0]);
				}//bracket for read5
			}
    	}
  
}
}




void TIM_Config(void)
{  
 //timer 2 input capture
  
	//PrescalerValue=(uint16_t)(SystemCoreClock/500000)-1;
  TIM_TimeBaseStructure.TIM_Period = 35999;
  TIM_TimeBaseStructure.TIM_Prescaler = 79;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
 
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
	
//timer 3 pwm outputs
	PrescalerValue=(uint16_t)(SystemCoreClock/500000)-1;
  TIM_TimeBaseStructure.TIM_Period = 9999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Init TIM_OCInitStructure */
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Output Compare Toggle Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel2 */
 
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel3 */
 
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel4 */
 
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);


  //***************************timer 6 loop control*************************************888
	
	PrescalerValue=(uint16_t)(SystemCoreClock/500000)-1;
	TIM_TimeBaseStructure.TIM_Period = 35999;
  TIM_TimeBaseStructure.TIM_Prescaler = 7;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);



}

void GPIO_Config()
{
/* TIM2 channel 2 pin (Pd.04) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_2);
//   
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  

	/* GPIOc Configuration: TIM3 CH1 (Pc6) and TIM3 CH2 (Pc7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* GPIOB Configuration: TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  /* Connect TIM Channels to AF */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_2); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);


}

void Acc_Config(void)
{
//   LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
//   LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
     
   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_200_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Single;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);
   
  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;
 
  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

void Acc_ReadData(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
   
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);   // this function is used to configure 2 control registers
	
	
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);
  
   
  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
   /* check in the control register4 the data alignment*/
	if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
		{
			for(i=0; i<3; i++)
				{
					pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
				}
		}
	else /* Big Endian Mode */
		{
			for(i=0; i<3; i++)
			pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
		}
/* Read the register content */
LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  

	
  for(i=0; i<3; i++)
  {
		if(i==0)
    pfData[i]=(float)((pnRawData[i]/(LSM_Acc_Sensitivity))+20);
		if(i==1)
		pfData[i]=(float)((pnRawData[i]/(LSM_Acc_Sensitivity))-5);
		else
		pfData[i]=(float)((pnRawData[i]/(LSM_Acc_Sensitivity))+0);	
  }
}

void gyroConfig(void)
{
  
  
  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;//1-95 ,,, 4-760
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_1;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; // 250 or 500 or 2000
  L3GD20_Init(&L3GD20_InitStructure);
   
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_8; // 0 to 9
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);// DISABLED
}


void gyroReadAngRate (float* pfData)          //probably this function actually accepts address of variables
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);         // probably used to configure CTRLREG4
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);         // used to get gyro readings
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
	
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)RawData[i]/sensitivity;
  }
}

void compassConfig(void)
{
  
  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_75_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);
  
//    /* Fill the accelerometer structure */
//   LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
//   LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
//   LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
//   LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
//   LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
//   LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
//   LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
//   /* Configure the accelerometer main parameters */
//   LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);
//   
//   /* Fill the accelerometer LPF structure */
//   LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
//   LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
//   LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
//   LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

//   /* Configure the accelerometer LPF main parameters */
//   LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}


/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void compassReadMag (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);
  
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }
  
  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}


void TIM2_IRQHandler(void)
{ 
	
   if(!(TIM_GetITStatus(TIM2, TIM_IT_CC4)) == RESET) 
   {
 		    
     TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
 		    
 		if(check==0)
			{
			  edge1 = TIM2->CCR4;
				check = 1;
			}
    else if(check==1) 
			{
				edge2 = TIM2->CCR4;
				check=0;
				counts=36000*counter + edge2-edge1;	
				CCR=counts*500/900;
				counter=0;
	     }
    }
	
	
	if(!(TIM_GetITStatus(TIM2, TIM_IT_CC3)) == RESET) 
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		
		if(check1==0)
			{
				edge3 = TIM2->CCR3;
				check1 = 1;
			}
    else if(check1==1) 
			{
				edge4 = TIM2->CCR3;
				check1=0;
				counts1=36000*counter1 + edge4-edge3;	
				if (counts1>36000)
						counts1=18000;
				CCR_3=(float)(counts1-1350)*100.0/2700.0;//5 degrees till 1600
				//k_i[0]=0.5+CCR_3;
				set_roll=-1*CCR_3;
				//set_pitch=CCR_3;
				counter1=0;
			}
	}
	
	if(!(TIM_GetITStatus(TIM2, TIM_IT_CC2)) == RESET) 
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		
		if(check2==0)
			{
				edge5 = TIM2->CCR2;
				check2 = 1;
			}
    else if(check2==1) 
			{
				edge6 = TIM2->CCR2;
				check2=0;
				counts2=36000*counter2 + edge6-edge5;	
				if (counts2>36000)
						counts2=18000;
				CCR_2=(float)(counts2-1350)*100.0/2700.0;//5 degrees till 1600
				set_pitch=CCR_2;
				counter2=0;
			}
	}
	
	
	if(!(TIM_GetITStatus(TIM2, TIM_IT_Update)) == RESET)
	{
		flag=1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (check==1)
				counter++;
		
		if (check1==1)
				counter1++;
		
		if (check2==1)
				counter2++;
	}
}


void TIM6_DAC1_IRQHandler(void)
{
	if(!(TIM_GetITStatus(TIM6, TIM_IT_Update)) == RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		takeaction=1;
		readsensor=1;
	}
}



#endif


