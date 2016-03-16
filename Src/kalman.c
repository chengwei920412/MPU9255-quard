#include "kalman.h"
#include "math.h"
#include "system.h"
#include "mpu9255.h"

extern float AngleGyro;

extern float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
extern float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;
//绕X轴旋转的叫横滚（roll），绕Y轴旋转的叫俯仰（pitch），绕Z轴旋转的叫航向（row）
extern float bs004_filter_high;
extern float bs004_filter_low;
extern float bs004_filter_time;
extern float bs004_quad_halfT;
extern float bs004_quad_Kp;
extern float bs004_quad_Ki;

extern float bs004_quad_halfT;
extern float exInt,eyInt,ezInt;	
extern float bs004_imu_pitch,bs004_imu_roll,bs004_imu_yaw;

extern float  bs004_mpu6050_acc_pitch_com,bs004_mpu6050_acc_roll_com;

extern float q0,q1,q2,q3;	

extern float bs004_mpu6050_gyro_scale;


/*float CalculateAngleAccel(float AccelX,float AccelZ)
{
	float AngleAccel;
	AngleAccel = atan2(AccelX ,AccelZ);
	return AngleAccel;
}*/
float CalculateAngleAccel(float AccelX,float AccelY,float AccelZ)
{
	float temp;
	float res=0;
	temp = AccelX/sqrt((AccelY*AccelY+AccelZ*AccelZ)) ;
	res=atan(temp);
	return res*180/PI;
}
float CalculateAngleRateGyro(int16_t GyroX)
{
	float AngleRateGyro;
	AngleRateGyro = - GyroX ;//* PI/ 180.00;
	AngleGyro = AngleGyro + AngleRateGyro * dt;
	return AngleGyro;
}
void BS004_Load_Filter_Parameter(void)
{
	int bs004_filter_par[12];
	//	
	bs004_filter_par[0]=950;
	bs004_filter_par[1]=50;
	bs004_filter_par[2]=1;
	bs004_filter_par[3]=1000;
	bs004_filter_par[4]=1640;
	bs004_filter_par[5]=5730;
	bs004_filter_par[6]=1000;
	bs004_filter_par[7]=1000;
	bs004_filter_par[8]=36;
	bs004_filter_par[9]=1;
	bs004_filter_par[10]=1600;
	bs004_filter_par[11]=1;		
	//
	bs004_filter_high=(float)bs004_filter_par[0]/1000.0f;						//圆点博士:滤波参数
	bs004_filter_low=(float)bs004_filter_par[1]/1000.0f;						//圆点博士:滤波参数	
	bs004_filter_time=(float)bs004_filter_par[2]/1000.0f;						//圆点博士:滤波参数		
	bs004_quad_halfT=(float)bs004_filter_par[9]/1000.0f;						//圆点博士:四元数时间系数
	bs004_quad_Kp=(float)bs004_filter_par[10]/1000.0f;							//圆点博士:四元数比例系数
	bs004_quad_Ki=(float)bs004_filter_par[11]/1000.0f;							//圆点博士:四元数积分系数	

}


unsigned char BS004_IMU_Update(float ax,float ay,float az,float gx,float gy,float gz) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;  
	float gz_input;
  //	
	//圆点博士:四元数乘法运算
	float q0q0 = q0 * q0;							
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q1q1 = q1 * q1;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	//	
	gz_input=gz*bs004_quad_halfT;
	//
	//圆点博士:归一化处理
	norm = sqrt(ax*ax + ay*ay + az*az);     
	if(norm==0) return 0;	
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;   
  //	
	//圆点博士:建立小四轴坐标系	
	vx = 2*(q1q3 - q0q2);								
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//
	//圆点博士:坐标系和重力叉积运算
	ex = (ay*vz - az*vy);								
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	//
	//圆点博士:比例运算
	exInt = exInt + ex*bs004_quad_Ki;
	eyInt = eyInt + ey*bs004_quad_Ki;
	ezInt = ezInt + ez*bs004_quad_Ki;
	//
	//圆点博士:陀螺仪融合
	gx = gx + bs004_quad_Kp*ex + exInt;
	gy = gy + bs004_quad_Kp*ey + eyInt;
	gz = gz + bs004_quad_Kp*ez + ezInt;
	//
	//圆点博士:整合四元数率
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*bs004_quad_halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*bs004_quad_halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*bs004_quad_halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*bs004_quad_halfT;  
	//
	//圆点博士:归一化处理
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(norm==0) return 0;	
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//
	//圆点博士:欧拉角转换
	bs004_imu_roll=asin(-2*q1q3 + 2*q0q2)*57.30f;
  bs004_imu_pitch=atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
  bs004_imu_yaw=bs004_imu_yaw+2*gz_input;
	//
	return 1;	
}


void Filter(float ax,float ay,float az,float gx,float gy,float gz)
{
	float bs004_mpu6050_acc_pitch_raw=ay;
	float bs004_mpu6050_acc_roll_raw=ax;
	float bs004_mpu6050_acc_yaw_raw=az;
	float bs004_mpu6050_gyro_pitch_raw=gy;
	float bs004_mpu6050_gyro_roll_raw=gx;
	float bs004_mpu6050_gyro_yaw_raw=gz;

	
	bs004_mpu6050_acc_pitch_com=bs004_filter_high*bs004_mpu6050_acc_pitch_com+bs004_filter_low*bs004_mpu6050_acc_pitch_raw;	
	bs004_mpu6050_acc_roll_com =bs004_filter_high*bs004_mpu6050_acc_roll_com +bs004_filter_low*bs004_mpu6050_acc_roll_raw;
	//
  bs004_mpu6050_acc_angel_pitch_ave=(bs004_mpu6050_acc_angel_pitch_ave+bs004_mpu6050_acc_pitch_com)/2.0f;
	bs004_mpu6050_acc_angel_roll_ave =(bs004_mpu6050_acc_angel_roll_ave +bs004_mpu6050_acc_roll_com)/2.0f;
	bs004_mpu6050_acc_angel_yaw_ave  =(bs004_mpu6050_acc_angel_yaw_ave  +bs004_mpu6050_acc_yaw_raw)/2.0f;
	//
	bs004_mpu6050_gyro_angel_pitch_ave=(bs004_mpu6050_gyro_angel_pitch_ave+bs004_mpu6050_gyro_pitch_raw)/2.0f;
	bs004_mpu6050_gyro_angel_roll_ave =(bs004_mpu6050_gyro_angel_roll_ave +bs004_mpu6050_gyro_roll_raw)/2.0f;
	bs004_mpu6050_gyro_angel_yaw_ave  =(bs004_mpu6050_gyro_angel_yaw_ave  +bs004_mpu6050_gyro_yaw_raw)/2.0f;

}

