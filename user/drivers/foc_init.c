#include "foc_init.h"
#include "foc/foc_core.h"
#include "as5600.h"

#include "stm32f1xx_hal.h"          // HAL库核心头文件
#include "stm32f1xx_hal_tim.h"      // 定时器HAL库
#include "stm32f1xx_hal_gpio.h"     // GPIO HAL库
#include "stm32f1xx_hal_rcc.h"      // 时钟HAL库
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

static void TIM2_Init(uint32_t frequency_Hz);
static void TIM1_Init(uint32_t frequency_Hz, uint32_t period);
static void GPIO_Init(void);

void foc_output(uint16_t a, uint16_t b, uint16_t c)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, c);
}

foc_t foc;
foc_cfg_t cfg = {

	.pole_pairs = 11,
	.master_voltage = 12,
	.pwm_period = 1600,
	
	.pwm_hz = 10000,
	.control_hz = 5000,
	.sensor_hz = 5000,
    
    .output = foc_output,
    .delay = HAL_Delay,
    .get_angle_rad = as5600GetAngleRadians,
};

void foc_root_init(void)
{
    foc_init(&foc,&cfg);
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);
    
    GPIO_Init();
    
    foc_output_enable(1);
    
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1600);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//    HAL_Delay(1000);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    
    foc_zero_reset(&foc);
    foc_output_enable(0);
    
    HAL_TIM_Base_Start_IT(&htim2);
}

float foc_get_angle(void)
{
    return foc_sensor_updata(&foc);
}



void foc_output_enable(uint8_t enable)
{
    if(enable)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);    
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);    
    }
}


// 中断处理函数（需要在适当的位置实现）
void TIM1_UP_IRQHandler(void)
{

    HAL_TIM_IRQHandler(&htim1);
}

void TIM2_IRQHandler(void)
{
//    foc_control(&foc);
        
    HAL_TIM_IRQHandler(&htim2);
}

//GPIO_PIN_14 Logic high enables OUT. Internalpulldown
//GPIO_PIN_3 Active-low reset input initializesinternal logicanddisablesthe
static void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  
  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


/**
  * @brief TIM1初始化 - PWM输出，可配置频率和周期
  * @param frequency_Hz: PWM频率，单位Hz
  * @param period: PWM周期值（分辨率）
  * @note 时钟频率为64MHz，中央对齐模式
  */
static void TIM1_Init(uint32_t frequency_Hz, uint32_t period)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t prescaler;
    
    // 1. 使能时钟
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 2. 配置GPIO - TIM1通道对应的GPIO引脚
    // TIM1_CH1: PA8, TIM1_CH2: PA9, TIM1_CH3: PA10
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 3. 计算预分频器
    // 中央对齐模式下，PWM频率 = 64MHz / (prescaler + 1) / period / 2
    if (frequency_Hz == 0) frequency_Hz = 1;
    if (period == 0) period = 100;
    
    prescaler = (64000000 / (frequency_Hz * period * 2)) - 1;
    if (prescaler > 65535) prescaler = 65535;
    
    // 4. 配置定时器时基
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = prescaler;                           // 预分频器
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;   // 中央对齐模式1
    htim1.Init.Period = period - 1;                            // 自动重载值
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;         // 时钟分频
    htim1.Init.RepetitionCounter = 0;                          // 重复计数器
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        printf("TIM1 Init error");  
    }
    
    // 5. 配置时钟源
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
    
    // 6. 配置主模式
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
    
    // 7. 配置PWM通道
    sConfigOC.OCMode = TIM_OCMODE_PWM1;        // PWM模式1
    sConfigOC.Pulse = 0;                       // 初始占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH; // 输出极性高
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE; // 快速模式禁用
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);  // PA8
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);  // PA9
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);  // PA10
    
    // 8. 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    // 9. 使能高级定时器的主输出（重要！）
    __HAL_TIM_MOE_ENABLE(&htim1);
}

/**
  * @brief TIM2初始化 - 基本定时器，可配置频率
  * @param frequency_Hz: 定时器频率，单位Hz
  * @note 时钟频率为64MHz
  */
static void TIM2_Init(uint32_t frequency_Hz)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    uint32_t prescaler, period;
    
    // 1. 使能TIM2时钟
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // 2. 计算预分频器和周期值
    // 定时器频率 = 64MHz / (prescaler + 1) / (period + 1)
    if (frequency_Hz == 0) frequency_Hz = 1; // 防止除零
    
    // 计算最优的预分频器和周期值
    prescaler = (64000000 / frequency_Hz) / 65536;
    if (prescaler > 65535) prescaler = 65535;
    
    period = (64000000 / frequency_Hz) / (prescaler + 1) - 1;
    if (period > 65535) period = 65535;
    
    // 3. 配置时基单元
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler;                    // 预分频器
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;         // 向上计数模式
    htim2.Init.Period = period;                          // 自动重载值
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   // 时钟分频
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        printf("TIM2 Init error");  
    }
    
    // 4. 配置时钟源
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
    
    // 5. 配置主模式
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
    
    // 6. 使能更新中断
    HAL_TIM_Base_Start_IT(&htim2);
    
    // 7. 配置NVIC
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    // 8. 使能定时器
    HAL_TIM_Base_Start(&htim2);
}








