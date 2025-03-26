/**
 * Author: leert
 * Version: 2025.2.20
 */

#include "stm32f10x.h"
#include "Delay.h"

const uint16_t numToPin[16] = {
	0x0001, 0x0002, 0x0004, 0x0008,
	0x0010, 0x0020, 0x0040, 0x0080,
	0x0100, 0x0200, 0x0400, 0x0800,
	0x1000, 0x2000, 0x4000, 0x8000
};

TIM_TypeDef *numToTimer[3] = {TIM2, TIM3, TIM4};

uint16_t DMASize;

const uint8_t matchTimer[2][16] = {
	{2, 2, 2, 2, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 2},
	{3, 3, 0, 2, 3, 3, 0, 0, 0, 0, 2, 2, 0, 0, 0, 0}
};

const uint8_t matchChannel[2][16] = {
	{1, 2, 3, 4, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 1},
	{3, 4, 0, 2, 1, 2, 0, 0, 0, 0, 3, 4, 0, 0, 0, 0}
};

/**
  * @brief	配置引脚GPIO模式
  * @param	c 代表GPIO组别的字符
  * @param	pin 引脚(GPIO_Pin_x)，多个则使用"|"相连，若全选则为GPIO_Pin_All
  * @param	mode GPIO模式(必须为GPIOMode_TypeDef类型)
  * @retval	无
  */
void pinMode(char c, uint16_t pin, GPIOMode_TypeDef mode){
	// 使用RCC开启时钟
	if(c == 'A')RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if(c == 'B')RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else return;

	// GPIO_InitTypeDef对象
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_Pin = pin; // 设置输入引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// 输出速度为50MHz

	//初始化GPIO
	if(c == 'A')GPIO_Init(GPIOA, &GPIO_InitStructure);
	else GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
*	@brief	读取GPIO输入
*	@param	c 代表GPIO组别的字符
*	@param	pinNum 引脚对应的标号(0至15)
*	@return	返回电平值(高为1，低为0)，若非法读取则默认返回0
*/
uint8_t digitalRead(char c, uint8_t pinNum){
	if(c == 'A')return GPIO_ReadInputDataBit(GPIOA, 0x0001 << pinNum);
	else if(c == 'B')return GPIO_ReadInputDataBit(GPIOB,0x0001 << pinNum);
	else return 0;
}

/**
  * @brief  GPIO输出
  * @param  c 代表GPIO组别的字符
  * @param	pinNum 引脚对应的标号(0至15)
  * @param	value 输出电平(高为1，低为0)
  * @retval 无
  */
void digitalWrite(char c, uint8_t pinNum, uint8_t value){
	if(c == 'A'){
		if(value)GPIO_SetBits(GPIOA, 0x0001 << pinNum);
		else GPIO_ResetBits(GPIOA, 0x0001 << pinNum);
	}else if(c =='B'){
		if(value)GPIO_SetBits(GPIOB, 0x0001 << pinNum);
		else GPIO_ResetBits(GPIOB, 0x0001 << pinNum);
	}else return;
}

/**
* @brief	检测按键按下事件
* @param	c 代表GPIO组别的字符
* @param	pinNum 引脚对应的标号(0至15)
* @param	status 按键按下时的电平
* @return 若检测到按键按下，返回1，否则返回0
*/
int getKey(char c, uint8_t pinNum, uint8_t status){
	if(digitalRead(c, pinNum) == status){
		Delay_ms(20); // 消除按键"抖动"带来的可能影响

		//若按键未松开，则阻塞
		while(digitalRead(c, pinNum) == status);

		Delay_ms(20);
		return 1;
	}
	return 0;
}

/**
*	@brief	配置NVIC分组
*	@param	group 取值为0到4，数值越大，抢占优先级种类越少，响应优先级种类越多
*/
void setInterruptGroup(uint8_t group){
	NVIC_PriorityGroupConfig(0x700 - group*0x100);
}

/**
*	@brief	配置引脚触发EXTI
*	@param	c 代表GPIO组别的字符
*	@param	pinNum 引脚(0至15)
*	@param	mode 触发方式(EXTITrigger_TypeDef类型)
*   @param	priority 抢占优先级
*   @param	subPriority 响应优先级
*	@note	请确保调用过setInterruptGroup函数
*	@note	为使中断得到正确响应，还须重写对应的中断响应函数(EXTIx_IRQHandler)
*/
void setEXTI(char c,uint8_t pinNum,EXTITrigger_TypeDef mode,uint8_t priority,uint8_t subPriority){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// AFIO外部中断引脚选择配置
	GPIO_EXTILineConfig((uint8_t)(c - 'A'), pinNum);

	// 初始化EXTI外设
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = (uint32_t)numToPin[pinNum]; // 指定要配置的中断线
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; // 指定选择的中断线的新状态
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // 指定外部中断线的模式为中断模式
	EXTI_InitStructure.EXTI_Trigger = mode; // 指定触发方式
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	if(pinNum < 5)NVIC_InitStructure.NVIC_IRQChannel = 6 + pinNum;
	else if(pinNum < 10)NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	else NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
	NVIC_Init(&NVIC_InitStructure);
}

/**
*	@brief	初始化时基单元(未开始计时)
*	@param	timer 通用定时器的编号(2至4)
*	@param	period 周期(微秒)
*	@note	本函数仅对时钟进行初始化，不会开始计时，如需开始计时，请调用TIM_Cmd函数
*/
void timerInit(uint8_t timer, uint32_t period){
	if(timer < 2 || timer > 4)return;
	RCC_APB1PeriphClockCmd((uint32_t)numToPin[timer - 2], ENABLE);
	TIM_InternalClockConfig(numToTimer[timer - 2]);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数

	//预分频器默认为72(微秒级)，若周期大于65536微秒，则预分频器为7200(百微秒级)
	if(period <= 65536){
		TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
		TIM_TimeBaseInitStructure.TIM_Period = period - 1; // 计数器
	}else{
		TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1; // 预分频器默认为72000(毫秒级)
		TIM_TimeBaseInitStructure.TIM_Period = period/100 - 1; // 计数器
	}
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(numToTimer[timer - 2], &TIM_TimeBaseInitStructure);
}

/**
*	@brief	配置通用定时器触发中断
*	@param	timer 通用定时器的编号(2至4)
*	@param	period 触发间隔时长(微秒)
*   @param	priority 抢占优先级
*   @param	subPriority 响应优先级
*/
void setTimerInterrupt(uint8_t timer, uint32_t period, uint8_t priority, uint8_t subPriority){
	timerInit(timer, period);

	TIM_ClearFlag(numToTimer[timer - 2], TIM_FLAG_Update);
	TIM_ITConfig(numToTimer[timer - 2], TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn + timer - 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(numToTimer[timer - 2], ENABLE);
}

/**
*	@brief	监测特定引脚输入方波的频率
*	@param	c 代表GPIO组别的字符
*	@param	pinNum 引脚编号
*	@param	channel 选择使用的定时器通道(1或2)
*/
void monitorFreq(char c, uint8_t pinNum, uint8_t channel){
	if(channel < 1 || channel > 2)return;
	uint8_t timer = matchTimer[c - 'A'][pinNum];
	if(!timer)return;

	//初始化引脚对应的时基单元
	timerInit(timer, 65536);

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = (channel - 1)*0x0004;
	TIM_ICInitStructure.TIM_ICFilter = 0xF; // 滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿触发
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 需要每次触发都有效，所以不分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直连，即CNT值锁存到自己的CCR
	TIM_ICInit(numToTimer[timer - 2], &TIM_ICInitStructure);

	//设置主从触发模式，CNT值锁存到CCR后自动清零，以备下次计数
	if(channel == 1)TIM_SelectInputTrigger(numToTimer[timer - 2], TIM_TS_TI1FP1);
	else if(channel == 2)TIM_SelectInputTrigger(numToTimer[timer - 2], TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(numToTimer[timer - 2], TIM_SlaveMode_Reset);

	TIM_Cmd(numToTimer[timer - 2], ENABLE);
}

/**
*	@brief	获取特定引脚输入方波的频率
*	@param	c 代表GPIO组别的字符
*	@param	pinNum 引脚编号
*	@param	channel 选择使用的定时器通道
*	@return	返回频率的值(整数)
*/
uint32_t getFreq(char c, uint8_t pinNum, uint8_t channel){
	uint8_t timer = matchTimer[c - 'A'][pinNum];
	if(channel == 1)return 1000000/(TIM_GetCapture1(numToTimer[timer - 2])+1);
	else if(channel == 2)return 1000000/(TIM_GetCapture2(numToTimer[timer - 2])+1);
	else return 0;
}

/**
*	@brief	在特定引脚输出特定周期与特定高电平时长的PWM
*	@param	c 代表GPIO组别的字符
*	@param	pinNum 引脚编号(0到15)
*   @param	period 周期(微秒)
*   @param	high 高电平持续时长(微秒)
*	@note	使用此函数前，请确保引脚已经配置为输出模式，且确保引脚可被用于PWM输出
*/
void analogWrite(char c, uint8_t pinNum, uint16_t period, uint16_t high){
	uint8_t timer = matchTimer[c - 'A'][pinNum], channel = matchChannel[c - 'A'][pinNum];

	RCC_APB1PeriphClockCmd((uint32_t)numToPin[timer - 2], ENABLE);
	TIM_InternalClockConfig(numToTimer[timer - 2]);

	timerInit(timer, period);

	// 初始化输出比较单元
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);

	// 输出比较模式设为PWM1
	// 即向上计数时，若CNT<CCR则输出1，若CNT>=CCR则输出0
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性设为高极性(即不翻转)
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = high - 1; //捕获比较寄存器(CCR)
	switch(channel){
		case 1: TIM_OC1Init(numToTimer[timer - 2], &TIM_OCInitStructure);break;
		case 2: TIM_OC2Init(numToTimer[timer - 2], &TIM_OCInitStructure);break;
		case 3: TIM_OC3Init(numToTimer[timer - 2], &TIM_OCInitStructure);break;
		case 4: TIM_OC4Init(numToTimer[timer - 2], &TIM_OCInitStructure);break;
	}
	TIM_Cmd(numToTimer[timer - 2], ENABLE);
}

/**
*	@brief	修改特定引脚PWM的占空比
*	@param	c 代表GPIO组别的字符
*	@param	pinNum 引脚编号(0到15)
*   @param	high 高电平持续时长(微秒)
*/
void analogModify(char c, uint8_t pinNum, uint16_t high){
	uint8_t timer = matchTimer[c - 'A'][pinNum], channel = matchChannel[c - 'A'][pinNum];
	switch(channel){
		case 1: TIM_SetCompare1(numToTimer[timer - 2], high); break;
		case 2: TIM_SetCompare2(numToTimer[timer - 2], high); break;
		case 3: TIM_SetCompare3(numToTimer[timer - 2], high); break;
		case 4: TIM_SetCompare4(numToTimer[timer - 2], high); break;
	}
}

/**
*	@brief	初始化模拟输入
*	@param	c 代表GPIO组别的字符(A或B)
*	@param	pin 引脚(GPIO_Pin_x，若选择多个则用"|"相连)
*	@param	isContinous 是否连续转换(1为是，0为否)
*	@note	若使用GPIOA则引脚编号只能为0到7，若使用GPIOB则只能为0或1
*/
void analogReadInit(char c, uint16_t pin, uint8_t isContinous){
	pinMode(c, pin, GPIO_Mode_AIN);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ADC时钟为72/6=12MHz
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//向右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = isContinous?ENABLE:DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 1;//通道数为1
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1)==SET);
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1)==SET);

	if(isContinous)ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
*	@brief	单次转换模式下模拟输入的读取
*	@param	c 代表GPIO组别的字符(A或B)
*	@param	pinNum 引脚编号(若选择GPIOA，则为0到7；若选择GPIOB，则为0或1)
*	@note	若使用此函数，须先调用过analogReadInit函数，且为单次转换模式
*/
uint16_t analogReadOnce(char c, uint8_t pinNum){
	ADC_RegularChannelConfig(ADC1, c=='A'?pinNum:pinNum+8, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
	//读取ADC后会自动清除标志位，故不需要手动清除
}

/**
*	@brief	连续转换模式下模拟输入的读取
*	@param	c 代表GPIO组别的字符(A或B)
*	@param	pinNum 引脚编号(若选择GPIOA，则为0到7；若选择GPIOB，则为0或1)
*	@note	若使用此函数，须先调用过analogReadInit函数，且为连续转换模式
*/
uint16_t analogRead(char c, uint8_t pinNum){
	ADC_RegularChannelConfig(ADC1, c=='A'?pinNum:pinNum+8, 1, ADC_SampleTime_55Cycles5);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
	//读取ADC后会自动清除标志位，故不需要手动清除
}

/**
 * @brief	读取脉冲，脉冲结束时脉冲持续时间会存储于定时器的捕获寄存器2中，并将引起中断
 * @param	c 代表GPIO组别的字符
 * @param	pinNum 引脚编号
 * @param	priority 抢占优先级
 * @param	subpriority 响应优先级
 */
void measurePulse(char c, uint8_t pinNum, uint8_t priority, uint8_t subPriority){
	uint8_t timer = matchTimer[c - 'A'][pinNum];
	if(!timer)return;

	//初始化引脚对应的时基单元
	timerInit(timer, 65536);

	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xF; // 滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿触发
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 需要每次触发都有效，所以不分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直连
	TIM_ICInit(numToTimer[timer - 2], &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xF; // 滤波器，数值越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling; // 下降沿触发
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 需要每次触发都有效，所以不分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI; // 交叉
	TIM_ICInit(numToTimer[timer - 2], &TIM_ICInitStructure);

	//设置主从触发模式，通道1检测到上升沿后CNT值清零
	TIM_SelectInputTrigger(numToTimer[timer - 2], TIM_TS_TI1FP1);
	TIM_SelectSlaveMode(numToTimer[timer - 2], TIM_SlaveMode_Reset);

	TIM_ClearFlag(numToTimer[timer - 2], TIM_FLAG_CC2);
	TIM_ITConfig(numToTimer[timer - 2], TIM_IT_CC2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn + timer - 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subPriority;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(numToTimer[timer - 2], ENABLE);
}

/**
 * @brief	初始化DMA传输
 * @param	from 源地址
 * @param	to 目的地址
 * @param	byteNum 数据为几位(1为8位，2为16位，4为32位)
 * @param	size 数据大小
 * @param	priority 优先级(0为最高，3为最低)
 * @note	本函数仅对DMA进行初始化，不会开始传输，如需开始传输，请调用transmit函数
 */
void setDMA(uint32_t from, uint32_t to, uint8_t byteNum, uint16_t size, uint8_t priority){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStructure;

	DMA_InitStructure.DMA_PeripheralBaseAddr = from;//转运的源地址

	if(byteNum == 1)DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	else if(byteNum == 2)DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	else if(byteNum == 4)DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	else return;
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;//地址须要自增

	DMA_InitStructure.DMA_MemoryBaseAddr = to;//转运的目的地址
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//地址须要自增

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//数据传输方向
	DMA_InitStructure.DMA_BufferSize = size;//数据大小
	DMASize = size;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;//软件触发
	DMA_InitStructure.DMA_Priority = (uint32_t)(0x00003000 - 0x00001000*priority);
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, DISABLE);//暂时不开始传输
}

/**
 * @brief	进行一次DMA传输
 * @note	本函数调用前，请确保已经调用过setDMA函数
 */
void transmit(void){
	DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel1, DMASize);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);//等待传输完成
	DMA_ClearFlag(DMA1_FLAG_TC1);//清除标志位
}
