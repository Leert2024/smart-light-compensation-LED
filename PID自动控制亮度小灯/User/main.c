#include "stm32f10x.h"// Device header
#include "Delay.h"
#include "OLED.h"
#include "leertHardware.h"
#define KP 0.1
#define KI 0.001
#define KD -0.1

uint16_t adc;	// 光敏电阻模块原始输出值
uint16_t level = 500;	// 初始高电平时间
uint16_t calcu;	// 期望的光敏电阻模块输出值
int16_t err = 0, prevErr = 0, errSum = 0, errDif = 0;

/*
void EXTI15_10_IRQHandler(void){
	if(EXTI_GetFlagStatus(EXTI_Line11) == SET && digitalRead('A', 11) == 1){
		EXTI_ClearITPendingBit(EXTI_Line11);
		if(level < 1000)level += 200;
		else level = 1;
		analogModify('A', 1, level);
	}
}
*/

// 卡尔曼滤波结构体
typedef struct {
    uint16_t x; // 估计值
    float P;    // 预测误差
    float Q;    // 系统噪声
    float R;    // 测量噪声
} KalmanFilter;

/**
 * @brief	初始化滤波器
 * @param   kf  卡尔曼滤波器结构体
 * @param   Q   系统噪声
 * @retval  无
 */
void KalmanInit(KalmanFilter* kf, float Q) {
    kf->Q = Q;

	// 在光照不变的条件下测量100次阻值，计算平均值作为初始阻值
    uint16_t adc[100];
    uint32_t ave = 0;
    uint32_t R = 0;
    for(int i = 0; i < 100; i++){
        adc[i] = analogRead('A', 0);
        ave += adc[i];
    }
    ave /= 100;
    kf->x = ave;

    // 计算方差作为R的初始值
    for(int i = 0; i < 100; i++){
        R += (adc[i] - ave) * (adc[i] - ave);
    }
    R /= 100;
    kf->R = R;

    kf->P = Q + R;  // 预测误差 = 系统噪声 + 测量噪声
}

/**
 * @brief	更新滤波器
 * @param   kf 卡尔曼滤波器结构体
 * @param   measurement 测量值
 * @retval  无
 */
void KalmanUpdate(KalmanFilter* kf, float measurement) {
    // 预测阶段
    uint16_t x_pred = kf->x;   // 假设变量当前值与上次值相同
    float P_pred = kf->P + kf->Q;   // 预测误差 = 上次误差 + 系统噪声

    // 计算卡尔曼增益
    float K = P_pred / (P_pred + kf->R);    // 这个系数决定我们更相信预测还是测量

    // 更新阶段
    float current_error = measurement - x_pred;   // 本次的误差(测量值 - 估计值)
    kf->x = x_pred + K * current_error; // 最终确定的值
    kf->P = (1 - K) * P_pred;   // 最终确定的误差
}

/**
 * @brief   限制积分算法的输出最大值
 * @param   lim 限制值的绝对值
 */
void limit(float lim){
    if(errSum > lim)errSum = lim;
    else if(errSum < -lim)errSum = -lim;
}

/**
 * @brief   PID算法
 * @param   measure 测量值(真实值)
 * @retval  系统输出值
 */
int16_t PID_Value(uint16_t measure){
    err = measure - calcu;//误差
    errDif = err - prevErr;
    prevErr = err;
    errSum += err;
	OLED_ShowSignedNum(3, 5, err, 4);

    return err*KP + errDif*KD + errSum*KI;
}

int16_t min(int16_t a, int16_t b){
	return a < b ? a : b;
}

int16_t max(int16_t a, int16_t b){
	return a > b ? a : b;
}

int main(void){
	OLED_Init();
	OLED_ShowString(1, 1, "ADC:");
	OLED_ShowString(2, 1, "KAL:");
	OLED_ShowString(3, 1, "ERR:");
	OLED_ShowString(4, 1, "OUT:");
	
	pinMode('A', GPIO_Pin_1, GPIO_Mode_AF_PP);
	analogWrite('A', 1, 1000, level);
	Delay_ms(100);
	
	pinMode('A', GPIO_Pin_0, GPIO_Mode_AIN);	// 初始化光敏传感器引脚
	analogReadInit('A', GPIO_Pin_0, 1);	// 初始化ADC
	
	/*
	pinMode('A', GPIO_Pin_11, GPIO_Mode_IPU);
	setInterruptGroup(1);
	setEXTI('A', 11, EXTI_Trigger_Rising, 1, 0);
	*/
	
    KalmanFilter kf;
    KalmanInit(&kf, 2); // 初始化滤波器
	calcu = kf.x;

	while(1){
		Delay_ms(10);
		adc = analogRead('A', 0);
		
        KalmanUpdate(&kf, adc);
		
		level = min(max(level + PID_Value(kf.x), 1), 1000);
		analogModify('A', 1, level);
		
		OLED_ShowNum(1, 5, adc, 4);
		OLED_ShowNum(2, 5, kf.x, 4);
		OLED_ShowNum(4, 5, level, 4);
	}
}
