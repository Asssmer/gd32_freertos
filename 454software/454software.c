#include "./454software.h"
#include "./sdcard.h"

//-----------------------------------------------------------------------
//                       ȫ�ֱ���
//-----------------------------------------------------------------------
char strOutput_454[MAX_STR_SIZE]; // �洢ת������ַ���
char usart0_res[12];
int usart0_res_length = 0;
volatile uint8_t MOTOR_received_frame[MOTOR_FRAME_SIZE];
volatile uint16_t adc_values_454[ADC_CHANNEL_COUNT];
int16_t temperature_P7;
int16_t temperature_P9;
// ѹ��
int16_t pressure_P10;
int16_t pressure_P11;
int16_t pressure_P12;
// ����
uint16_t flow_P13;
uint16_t flow_P14;
uint16_t flow_P15;

float oxygen_voltage_air = 1.0; //! 1.0���칤��ʵ���ҵ�����,��Ŀ�����Ҫʹ��FMC��У׼���ݴ�ŵ�flash��

//-----------------------------------------------------------------------
//                       ȫ������
//-----------------------------------------------------------------------
volatile uint16_t adc_values_454[ADC_CHANNEL_COUNT];
volatile MotorStatus motor_status;
volatile SensorData sensor_data;
volatile pwm_capture_data_t pwm_values = {0};
SemaphoreHandle_t RtSample_Mutex;
SemaphoreHandle_t iicCollect_Ready_Semaphore;
SemaphoreHandle_t iicCollect_Start_Semaphore;
//! ��Խ����ĵ�����
volatile struct struRtSample RtSample;
//-----------------------------------------------------------------------
//                       ��ʼ��
//-----------------------------------------------------------------------
void init_454(void)
{
    RCU_init_454();
    NVIC_init_454();
    I2C_init_454();
    LED_init_454();
    YDP_init_454();
    Pulverizer_init_454();
    TIMER_init_454();
    USART0_init_454();
    USART2_init_454();
    SPI1_init_454();
    MAX31865_HWInit(GPIO_PIN_15);
    MAX31865_HWInit(GPIO_PIN_12);

    PWM_init_454();
    PWM_IN_init_454();
    ADC2_DMA_init_454();
    ADC2_init_454();
    SDIO_init_454();

    delay_ms_454(2); //?��ʼ�������ʱ����

    /* ����PC9ΪCKOUT1 */
    // gpio_af_set(GPIOC, GPIO_AF_0, GPIO_PIN_9);
    // gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    // gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    // rcu_ckout1_config(RCU_CKOUT1SRC_SYSTEMCLOCK, RCU_CKOUT1_DIV1);
}
void RCU_init_454(void)
{
    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
    rcu_periph_clock_enable(RCU_GPIOG);

    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER3);
    // rcu_periph_clock_enable(RCU_TIMER7);
    rcu_periph_clock_enable(RCU_TIMER4);
    rcu_periph_clock_enable(RCU_TIMER12);

    rcu_periph_clock_enable(RCU_TIMER6);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_USART2);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_I2C1);
    rcu_periph_clock_enable(RCU_I2C2);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_ADC2);
    rcu_periph_clock_enable(RCU_SDIO);
}
void NVIC_init_454(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    // ����ԽС�����ȼ�Խ��
    // nvic_irq_enable(TIMER4_IRQn, 0, 4);         // PWM_IN
    nvic_irq_enable(DMA0_Channel1_IRQn, 0, 10); // USART2_RX
    nvic_irq_enable(DMA1_Channel0_IRQn, 0, 9);  // ADC2
    // nvic_irq_enable(DMA1_Channel0_IRQn, 0, 9);  // ADC2

    nvic_irq_enable(USART0_IRQn, 0, 8); // USART0_RX
    // nvic_irq_enable(I2C0_EV_IRQn, 0, 3);
    // nvic_irq_enable(I2C1_EV_IRQn, 0, 4);
    // nvic_irq_enable(I2C0_ER_IRQn, 0, 2);
    // nvic_irq_enable(I2C1_ER_IRQn, 0, 1);
}
void LED_init_454(void)
{
    // ����PG6,PG7,PG8
    gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    gpio_bit_reset(GPIOG, GPIO_PIN_6);
    gpio_bit_reset(GPIOG, GPIO_PIN_7);
    gpio_bit_reset(GPIOG, GPIO_PIN_8);
}
void YDP_init_454(void)
{
    // ����PB14Ϊ���ģʽ
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_14);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    // ��ʼ��ʱ����ѡ��ر�ѹ�緧Ƭ
    gpio_bit_reset(GPIOB, GPIO_PIN_14);
}
void Pulverizer_init_454(void)
{
    // ����PB14Ϊ���ģʽ
    gpio_mode_set(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    // ��ʼ��ʱ����ѡ��ر�ѹ�緧Ƭ
    gpio_bit_reset(GPIOG, GPIO_PIN_4);
}
void TIMER_init_454(void)
{
    /* Configure TIMER6 */
    timer_deinit(RCU_TIMER6);
    timer_parameter_struct timer_initpara;
    timer_initpara.prescaler = 199;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 999; // ʵ��1ms�Ķ�ʱ��
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER6, &timer_initpara);
    timer_enable(TIMER6);
}
void USART1_init_454(void)
{

    /* Configure USART1 */
    usart_deinit(USART1);
    usart_disable(USART1);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_baudrate_set(USART1, 115200U);
    usart_parity_config(USART1, USART_PM_NONE);

    usart_hardware_flow_rts_config(USART1, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART1, USART_CTS_DISABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);

    // Configure USART1 TX (PD5) as alternate function push-pull
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_5);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);

    // Configure USART1 RX (PD6) as floating input
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_6);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    usart_prescaler_config(USART1, 1U);
    usart_enable(USART1);
}
void USART0_init_454(void)
{
    /* Configure USART0 */
    usart_deinit(USART0);
    usart_disable(USART0);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_baudrate_set(USART0, 115200U);
    usart_parity_config(USART0, USART_PM_NONE);

    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);

    usart_dma_transmit_config(USART0, USART_DENT_ENABLE);

    /* Configure USART0_TX DMA */
    dma_deinit(DMA1, DMA_CH7);
    dma_multi_data_parameter_struct dma_init_struct;
    dma_multi_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART0);
    dma_init_struct.periph_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_multi_data_mode_init(DMA1, DMA_CH7, &dma_init_struct);
    dma_channel_subperipheral_select(DMA1, DMA_CH7, DMA_SUBPERI4);
    dma_interrupt_enable(DMA1, DMA_CH7, DMA_CHXCTL_FTFIE);

    // Configure USART0_TX (PA9) as alternate function push-pull
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_9);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    // Configure USART0_RX (PA10) as floating input
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_10);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    usart_prescaler_config(USART0, 1U);
    usart_enable(USART0);
}
void USART2_init_454(void)
{
    usart_deinit(USART2);
    usart_disable(USART2);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_baudrate_set(USART2, 250000U);
    usart_parity_config(USART2, USART_PM_NONE);

    usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);

    usart_dma_receive_config(USART2, USART_DENR_ENABLE);

    //! Configure USART2_RX DMA
    dma_channel_disable(DMA0, DMA_CH1);
    dma_deinit(DMA0, DMA_CH1);
    dma_single_data_parameter_struct dma_init_struct_RX;
    dma_single_data_para_struct_init(&dma_init_struct_RX);

    dma_init_struct_RX.periph_addr = (uint32_t)&USART_DATA(USART2);
    dma_init_struct_RX.memory0_addr = (uint32_t)MOTOR_received_frame;
    dma_init_struct_RX.number = MOTOR_FRAME_SIZE;
    dma_init_struct_RX.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct_RX.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct_RX.periph_memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct_RX.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_init_struct_RX.direction = DMA_PERIPH_TO_MEMORY;
    dma_init_struct_RX.priority = DMA_PRIORITY_HIGH;

    dma_single_data_mode_init(DMA0, DMA_CH1, &dma_init_struct_RX);
    dma_channel_subperipheral_select(DMA0, DMA_CH1, DMA_SUBPERI4);
    dma_interrupt_enable(DMA0, DMA_CH1, DMA_CHXCTL_FTFIE);
    dma_channel_enable(DMA0, DMA_CH1);

    // Configure USART2 TX (PD8) as alternate function push-pull
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_8);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    // Configure USART2 RX (PD9) as floating input
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_9);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    usart_prescaler_config(USART2, 1U);
    usart_enable(USART2);
}
void I2C_init_454(void)
{
#ifdef BOARD_VER_1
    /* Configure I2C0 P10 PB6 PB7*/
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_9); // I2C0_SDA
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_8); // I2C0_SCL

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    i2c_deinit(I2C0);
    i2c_clock_config(I2C0, 400000, I2C_DTCY_2);
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
    i2c_enable(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);

    /* Configure I2C1 P11 PB10 PB11 */
    gpio_af_set(GPIOF, GPIO_AF_4, GPIO_PIN_0); // I2C1_SDA
    gpio_af_set(GPIOF, GPIO_AF_4, GPIO_PIN_1); // I2C1_SCL

    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_0);

    gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_1);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    i2c_deinit(I2C1);
    i2c_clock_config(I2C1, 400000, I2C_DTCY_2);
    // i2c_interrupt_enable(I2C1,I2C_INT_EV);
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C1_OWN_ADDRESS7);
    i2c_enable(I2C1);
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);

    /* Configure I2C2 PA8 PC9 */
    gpio_af_set(GPIOC, GPIO_AF_4, GPIO_PIN_9); // I2C1_SDA
    gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_8); // I2C1_SCL

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    i2c_deinit(I2C2);
    i2c_clock_config(I2C2, 400000, I2C_DTCY_2);
    // i2c_interrupt_enable(I2C1,I2C_INT_EV);
    i2c_mode_addr_config(I2C2, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C1_OWN_ADDRESS7);
    i2c_enable(I2C2);
    i2c_ack_config(I2C2, I2C_ACK_ENABLE);
#endif

#ifdef BOARD_VER_2
    /* Configure I2C0 P10 PB6 PB7*/
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_7); // I2C0_SDA
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_6); // I2C0_SCL

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    i2c_deinit(I2C0);
    i2c_clock_config(I2C0, 400000, I2C_DTCY_2);
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
    i2c_enable(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);

    /* Configure I2C1 P11 PB10 PB11 */
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_11); // I2C1_SDA
    gpio_af_set(GPIOB, GPIO_AF_4, GPIO_PIN_10); // I2C1_SCL

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    i2c_deinit(I2C1);
    i2c_clock_config(I2C1, 400000, I2C_DTCY_2);
    // i2c_interrupt_enable(I2C1,I2C_INT_EV);
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C1_OWN_ADDRESS7);
    i2c_enable(I2C1);
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);

    /* Configure I2C2 PA8 PC9 */
    gpio_af_set(GPIOC, GPIO_AF_4, GPIO_PIN_9); // I2C1_SDA
    gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_8); // I2C1_SCL

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);

    i2c_deinit(I2C2);
    i2c_clock_config(I2C2, 400000, I2C_DTCY_2);
    // i2c_interrupt_enable(I2C1,I2C_INT_EV);
    i2c_mode_addr_config(I2C2, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C1_OWN_ADDRESS7);
    i2c_enable(I2C2);
    i2c_ack_config(I2C2, I2C_ACK_ENABLE);
#endif
}
void SPI1_init_454(void)
{
    // PG0 : DRDY2
    // PG1 : DRDY1
    // ����PG0��PG1Ϊ����ģʽ-->DRDY
    gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_mode_set(GPIOG, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_1);

    // PB13: SPI1_SCLK
    // PC3 : SPI1_MOSI
    // PC2 : SPI1_MISO

    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_13); // SCK
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

    gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_3); // MOSI
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    gpio_af_set(GPIOC, GPIO_AF_5, GPIO_PIN_2); // MISO
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    // PB12 : nCS2
    // PB15 : nCS1
    // ����PB12��PB15Ϊ���ģʽ
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);

    // ��PB12��PB15����Ϊ�ߵ�ƽ������Ƭѡ
    gpio_bit_set(GPIOB, GPIO_PIN_12);
    gpio_bit_set(GPIOB, GPIO_PIN_15);

    // ����SPI1����
    spi_i2s_deinit(SPI1);
    spi_parameter_struct spi_init_struct;

    // ��ģʽ�������� f_PCLK/256 (�ӽ���200MHz/256 = 781.25 kHz)����׼ģʽ��8-bit ����֡��ʽ
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    // spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.prescale = SPI_PSC_256;
    spi_init_struct.endian = SPI_ENDIAN_MSB;

    spi_init(SPI1, &spi_init_struct);

    // ʹ��SPI1
    spi_enable(SPI1);
}
void ADC2_DMA_init_454(void)
{

    // ��λDMAͨ��
    dma_channel_disable(DMA1, DMA_CH0);
    // ��ʼ��DMA
    dma_deinit(DMA1, DMA_CH0);
    // ����DMA�����ṹ��
    dma_single_data_parameter_struct dma_init_struct;
    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr = (uint32_t)&ADC_RDATA(ADC2);
    // ���д�������DMA����������ַ��������ADC2�����ݼĴ����ĵ�ַ����DMA�����У������ַ����Ϊ������Դ��
    dma_init_struct.memory0_addr = (uint32_t)&adc_values_454[0];
    // ���д�������DMA�����Ŀ�ĵ��ڴ��ַ��adc_values������������飬���ڴ洢��ADC���յ������ݡ�
    dma_init_struct.number = ADC_CHANNEL_COUNT;
    // ���д���ָ��DMAҪ���������������ADC_CHANNEL_COUNTӦ�����������һ����������ʾADCͨ����������
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    // ���д������������ַ��������ÿ��DMA������ɺ������ַ���ֲ��䣬���ڶ�ȡͬһ�����ַ������·ǳ����ã�����������ȡͬһ��ADC���ݼĴ�����
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    // ���д��������ڴ��ַ����������ζ��ÿ��DMA������ڴ��ַ���������Ա���һ�����ݴ洢���µ�λ�á�
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    // ���д�������DMA��������ݿ�ȡ���������Ϊ16λ����ζ��ÿ�δ���16λ��2�ֽڣ������ݡ�
    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    // ���д�������ѭ��ģʽ����ѭ��ģʽ�£���DMA���ﴫ��������ĩβʱ�������Զ����õ���ʼλ�ã��γ�һ������ѭ���Ĵ�����̣�����ڳ������������ǳ����á�
    dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
    // ���д�������DMA�Ĵ��䷽����������Ϊ�����赽�ڴ棬��ζ�����ݽ���ADC�Ĵ�����ȡ�����洢���ڴ��С�
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    // ���д�������DMAͨ�������ȼ�������������£���������Ϊ�����ȼ�������ζ�����������������ϵ����ȼ���DMA���䡣

    dma_single_data_mode_init(DMA1, DMA_CH0, &dma_init_struct);
    dma_channel_subperipheral_select(DMA1, DMA_CH0, DMA_SUBPERI2);
    dma_interrupt_enable(DMA1, DMA_CH0, DMA_CHXCTL_FTFIE);
    // ʹ��DMAͨ��
    dma_channel_enable(DMA1, DMA_CH0);
}
void ADC2_init_454(void)
{
    /* ����PF6, PF7, PF8, PF9, PF10, �� PF3 Ϊģ������ */
    gpio_mode_set(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_3);

    adc_deinit();
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    adc_clock_config(ADC_ADCCK_HCLK_DIV5);
    adc_resolution_config(ADC2, ADC_RESOLUTION_12B);

    adc_special_function_config(ADC2, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC2, ADC_CONTINUOUS_MODE, ENABLE);
    adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);
    /* ADC DMA function enable */
    adc_dma_request_after_last_enable(ADC2);
    adc_dma_mode_enable(ADC2);
    adc_enable(ADC2);
    adc_calibration_enable(ADC2); // У׼ADC
    adc_external_trigger_config(ADC2, ADC_REGULAR_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_channel_length_config(ADC2, ADC_REGULAR_CHANNEL, 6);
    adc_regular_channel_config(ADC2, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_15); // p4������
    adc_regular_channel_config(ADC2, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_15); // P5��ŷ�
    adc_regular_channel_config(ADC2, 2, ADC_CHANNEL_6, ADC_SAMPLETIME_15); // P6��ŷ�
    adc_regular_channel_config(ADC2, 3, ADC_CHANNEL_7, ADC_SAMPLETIME_15); // PSE540
    adc_regular_channel_config(ADC2, 4, ADC_CHANNEL_8, ADC_SAMPLETIME_15); // �¶�(����)
    adc_regular_channel_config(ADC2, 5, ADC_CHANNEL_9, ADC_SAMPLETIME_15); // P16������Ũ�ȴ�����
    adc_software_trigger_enable(ADC2, ADC_REGULAR_CHANNEL);
}
void PWM_init_454(void)
{

    // ����PE0Ϊ���ģʽ
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    // ����PE1Ϊ���ģʽ
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    // ����PE2Ϊ���ģʽ
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    // Ĭ�Ͻ���EG27519
    gpio_bit_set(GPIOE, GPIO_PIN_0);
    gpio_bit_set(GPIOE, GPIO_PIN_1);
    gpio_bit_set(GPIOE, GPIO_PIN_2);

    //////////////////////////////////////////////////////////////////////////////
    // PWM_OUT1  PC6
    // TIMER2_CH0����
    //////////////////////////////////////////////////////////////////////////////
    gpio_af_set(GPIOC, GPIO_AF_2, GPIO_PIN_6);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    timer_oc_parameter_struct timer2_ocinitpara;
    timer_parameter_struct timer2_initpara;

    timer_deinit(TIMER2);

    // ����TIMER2�Ļ�������
    timer2_initpara.prescaler = 9; // ������Ҫ����
    timer2_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer2_initpara.counterdirection = TIMER_COUNTER_UP;
    timer2_initpara.period = 999; // ������Ҫ������ȷ��PWMƵ��
    timer2_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer2_initpara);

    // ����TIMER2_CH0��PWMģʽ
    timer2_ocinitpara.outputstate = TIMER_CCX_ENABLE;          // ����TIMER_CHANNEL
    timer2_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;       // ����TIMER_CHANNEL�Ļ������
    timer2_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;     // ����PWM����Ϊ��
    timer2_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;   // ����PWM�����������Ϊ��
    timer2_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;   // �ڿ���״̬�£�PWM���Ϊ��
    timer2_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW; // �ڿ���״̬�£�PWM�������Ϊ��

    timer_channel_output_config(TIMER2, TIMER_CH_0, &timer2_ocinitpara);
    // ����PWMģʽ
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 0); // ������Ҫ����ռ�ձ�
    timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    // ����TIMER2
    timer_auto_reload_shadow_enable(TIMER2);
    timer_enable(TIMER2);

    //////////////////////////////////////////////////////////////////////////////
    // PWM_OUT2  PA6
    // TIMER12_CH0����
    //////////////////////////////////////////////////////////////////////////////

    // P5��ŷ�,PWM_OUT_2,PA6:TIMER2�ĸ��ù���
    gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_6);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    timer_oc_parameter_struct timer12_ocinitpara;
    timer_parameter_struct timer12_initpara;

    timer_deinit(TIMER12);

    // ����TIMER2�Ļ�������
    timer12_initpara.prescaler = 9; // ������Ҫ����
    timer12_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer12_initpara.counterdirection = TIMER_COUNTER_UP;
    timer12_initpara.period = 999; // ������Ҫ������ȷ��PWMƵ��
    timer12_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER12, &timer12_initpara);

    // ����TIMER2_CH0��PWMģʽ
    timer12_ocinitpara.outputstate = TIMER_CCX_ENABLE;          // ����TIMER_CHANNEL
    timer12_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;       // ����TIMER_CHANNEL�Ļ������
    timer12_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;     // ����PWM����Ϊ��
    timer12_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;   // ����PWM�����������Ϊ��
    timer12_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;   // �ڿ���״̬�£�PWM���Ϊ��
    timer12_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW; // �ڿ���״̬�£�PWM�������Ϊ��

    timer_channel_output_config(TIMER12, TIMER_CH_0, &timer12_ocinitpara);
    // ����PWMģʽ
    timer_channel_output_pulse_value_config(TIMER12, TIMER_CH_0, 0); // ������Ҫ����ռ�ձ�
    timer_channel_output_mode_config(TIMER12, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER12, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    // ����TIMER2
    timer_auto_reload_shadow_enable(TIMER12);
    timer_enable(TIMER12);

    //////////////////////////////////////////////////////////////////////////////
    // PWM_OUT3  PD12
    // TIMER3_CH0����
    //////////////////////////////////////////////////////////////////////////////

    // P6��ŷ�,PWM_OUT_3,PD12:TIMER3�ĸ��ù���
    gpio_af_set(GPIOD, GPIO_AF_2, GPIO_PIN_12);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    timer_oc_parameter_struct timer3_ocinitpara;
    timer_parameter_struct timer3_initpara;

    timer_deinit(TIMER3);

    // ����TIMER3�Ļ�������
    timer3_initpara.prescaler = 9; // ������Ҫ����
    timer3_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer3_initpara.counterdirection = TIMER_COUNTER_UP;
    timer3_initpara.period = 999; // ������Ҫ������ȷ��PWMƵ��
    timer3_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER3, &timer3_initpara);

    // ����TIMER3_CH0��PWMģʽ
    timer3_ocinitpara.outputstate = TIMER_CCX_ENABLE;          // ����TIMER_CHANNEL
    timer3_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;       // ����TIMER_CHANNEL�Ļ������
    timer3_ocinitpara.ocpolarity = TIMER_OC_POLARITY_HIGH;     // ����PWM����Ϊ��
    timer3_ocinitpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;   // ����PWM�����������Ϊ��
    timer3_ocinitpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;   // �ڿ���״̬�£�PWM���Ϊ��
    timer3_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW; // �ڿ���״̬�£�PWM�������Ϊ��

    timer_channel_output_config(TIMER3, TIMER_CH_0, &timer3_ocinitpara);
    // ����PWMģʽ
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, 0); // ������Ҫ����ռ�ձ�
    timer_channel_output_mode_config(TIMER3, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    // ����TIMER3
    timer_auto_reload_shadow_enable(TIMER3);
    timer_enable(TIMER3);
}
void PWM_IN_init_454(void)
{
    // PWM_IN-->TIMER4
    //  ����PA0Ϊ����ģʽ,���ù���ΪTIMER4_CH0
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_0);
    // ����PA1Ϊ����ģʽ,���ù���ΪTIMER4_CH1
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_1);
    // ����PA2Ϊ����ģʽ,���ù���ΪTIMER4_CH2
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_2, GPIO_PIN_2);

    timer_ic_parameter_struct timer4_icinitpara;
    timer_parameter_struct timer4_initpara;

    // ����TIMER4ʱ��
    rcu_periph_clock_enable(RCU_TIMER4);

    // ����TIMER4�Ļ�������
    timer_deinit(TIMER4);
    timer4_initpara.prescaler = 9;
    timer4_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer4_initpara.counterdirection = TIMER_COUNTER_UP;
    timer4_initpara.period = 10000;
    timer4_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER4, &timer4_initpara);

    // ����TIMER4_CH0Ϊ���벶��ģʽ
    // timer4_icinitpara.icpolarity = TIMER_IC_POLARITY_RISING;
    timer4_icinitpara.icpolarity = TIMER_IC_POLARITY_FALLING;
    timer4_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI; // ӳ�䵽TI0��
    timer4_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;           // ����Ƶ
    timer4_icinitpara.icfilter = 5;
    timer_input_capture_config(TIMER4, TIMER_CH_0, &timer4_icinitpara);

    timer_interrupt_enable(TIMER4, TIMER_INT_CH0);
    // !δ����TIMER4,��Ϊ���󱻿���
    // timer_enable(TIMER4);
}
void SDIO_init_454(void)
{
    /* configure SDIO_DAT0(PC8), SDIO_DAT3(PC11), SDIO_CLK(PC12) */
    gpio_af_set(GPIOC, GPIO_AF_12, GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12);

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8 | GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_11);

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* configure SDIO_DAT1(PB0), SDIO_DAT2(PB1) */
    gpio_af_set(GPIOB, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);

    /* configure SDIO_CMD(PD2) */
    gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_2);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
}

//-----------------------------------------------------------------------
//                       ���ߺ���
//-----------------------------------------------------------------------
void delay_ms_454(uint32_t ms)
{
    while (ms > 0)
    {
        uint32_t ticks = (SystemCoreClock / 1000 / (199 + 1)) * ms;
        if (ticks > 65535)
        {
            ticks = 65535; // Set to max possible value to avoid overflow
            ms -= 65535 / (SystemCoreClock / 1000 / (199 + 1));
        }
        else
        {
            ms = 0;
        }
        timer_autoreload_value_config(TIMER6, ticks - 1);
        timer_counter_value_config(TIMER6, 0);   // Reset counter
        timer_flag_clear(TIMER6, TIMER_FLAG_UP); // Clear any existing overflow flag
        while (RESET == timer_flag_get(TIMER6, TIMER_FLAG_UP))
            ;
        timer_flag_clear(TIMER6, TIMER_FLAG_UP); // Clear overflow flag for next iteration
    }
}
void delay_s_454(uint32_t seconds)
{
    for (uint32_t i = 0; i < seconds; i++)
    {
        delay_ms_454(1000);
    }
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
        ;
    return ch;
}
int fgetc(FILE *f)
{
    while (usart_flag_get(USART0, USART_FLAG_RBNE) == RESET)
        ;
    int data = usart_data_receive(USART0);
    return data;
}
int __backspace(FILE *f)
{
    return '\b';
}

char *intToStr(int num)
{
    char *start = strOutput_454;
    char *end = strOutput_454;
    uint16_t remaining = MAX_STR_SIZE;

    if (num < 0)
    {
        if (remaining <= 1)
        {
            *strOutput_454 = '\0';
            return NULL;
        }
        *end++ = '-';
        remaining--;
        num = -num;
    }

    // Convert number to string (in reverse)
    do
    {
        if (remaining == 0)
        {
            *start = '\0';
            return NULL;
        }
        *end++ = '0' + (num % 10);
        num /= 10;
        remaining--;
    } while (num > 0);

    *end = '\0'; // Null-terminate

    // Reverse the string
    char *revStart = start + (*start == '-' ? 1 : 0); // Skip the negative sign if it exists
    char *revEnd = end - 1;                           // Set the end pointer just before the null terminator
    char temp;
    while (revStart < revEnd)
    {
        temp = *revStart;
        *revStart++ = *revEnd;
        *revEnd-- = temp;
    }

    return start;
}
char *floatToStr(float num, int afterpoint)
{
    // Check buffer size
    if (afterpoint >= MAX_STR_SIZE - 1)
    {
        strOutput_454[0] = '\0'; // Not enough space for the number
        return NULL;
    }

    char *start = strOutput_454;
    char *end = strOutput_454;
    int isNegative = 0;

    if (num < 0)
    {
        isNegative = 1;
        num = -num;
        *end++ = '-';
    }

    // Handle integer part
    int intPart = (int)num;
    float floatPart = num - (float)intPart;

    // Convert integer part to string
    do
    {
        *end++ = '0' + (intPart % 10);
        intPart /= 10;
    } while (intPart > 0);

    // Reverse integer part string (excluding negative sign, if any)
    char *revStart = isNegative ? start + 1 : start;
    char *revEnd = end - 1;
    char temp;
    while (revStart < revEnd)
    {
        temp = *revStart;
        *revStart = *revEnd;
        *revEnd = temp;
        revStart++;
        revEnd--;
    }

    // Handle decimal part
    if (afterpoint > 0)
    {
        *end++ = '.'; // add decimal point

        // Convert decimal part to string
        while (afterpoint-- > 0)
        {
            floatPart *= 10;
            int digit = (int)floatPart;
            *end++ = '0' + digit;
            floatPart -= (float)digit;
        }
    }

    *end = '\0'; // Null-terminate

    return start;
}
float adc_to_voltage(uint16_t adc_value)
{
    return (adc_value * 3.3f) / 4095;
}
//-----------------------------------------------------------------------
//                       LED
//-----------------------------------------------------------------------
void LED1(FlagStatus state)
{
    gpio_bit_write(GPIOG, GPIO_PIN_6, state);
}
void LED2(FlagStatus state)
{
    gpio_bit_write(GPIOG, GPIO_PIN_7, state);
}
void LED3(FlagStatus state)
{
    gpio_bit_write(GPIOG, GPIO_PIN_8, state);
}
//-----------------------------------------------------------------------
//                       USART
//-----------------------------------------------------------------------
void usart_echo(uint32_t usart_periph)
{
    uint8_t data;
    while (1)
    {
        // ���ָ���� USART �Ƿ���յ�����
        if (usart_flag_get(usart_periph, USART_FLAG_RBNE))
        {
            data = usart_data_receive(usart_periph); // ��ȡ���յ�������

            // �ȴ����ͻ�����Ϊ��
            while (RESET == usart_flag_get(usart_periph, USART_FLAG_TBE))
                ;

            usart_data_transmit(usart_periph, data); // �����յ������ݻش�
        }
    }
}
uint8_t usart0_send_454(uint8_t *string, uint16_t count_size)
{
    while (DMA_CHCTL(DMA1, DMA_CH7) & DMA_CHXCTL_CHEN)
    {
    }
    while (RESET == usart_flag_get(USART0, USART_FLAG_TC))
    {
    }
    dma_memory_address_config(DMA1, DMA_CH7, DMA_MEMORY_0, string);
    dma_transfer_number_config(DMA1, DMA_CH7, count_size);
    dma_channel_enable(DMA1, DMA_CH7);
    return 0;
}
//-----------------------------------------------------------------------
//                       ѹ�� (����ֵ��int16_t,����ʵֵ��0.1��,���緵��10 000,��ô����100 000pa)
//-----------------------------------------------------------------------
int16_t P10_get(void)
{
    float i2c0_fTemp = 0;
    float i2c0_fPress = 0;
    ZXP8_get_data_454(I2C0, &i2c0_fTemp, &i2c0_fPress);
    return (int16_t)(i2c0_fPress / 10);
}
int16_t P11_get(void)
{
    float i2c2_fTemp = 0;
    float i2c2_fPress = 0;
    ZXP2_get_data_454(I2C2, &i2c2_fTemp, &i2c2_fPress);
    return (int16_t)(i2c2_fPress / 10);
}
int16_t P12_get(void)
{
    float i2c1_fTemp = 0;
    float i2c1_fPress = 0;
    ZXP8_get_data_454(I2C1, &i2c1_fTemp, &i2c1_fPress);
    return (int16_t)(i2c1_fPress / 10);
}

uint32_t i2c_flag_check_timeout(uint32_t i2c_periph, i2c_flag_enum flag, FlagStatus expected_Status)
{
    uint32_t timeout = 0xFFFF;
    while (i2c_flag_get(i2c_periph, flag) != expected_Status)
    {
        if (timeout == 0)
        {
            return 1; // Timeout
        }
        timeout--;
    }
    return 0; // Success
}
int i2c_master_receive(uint32_t i2c_periph, uint8_t *data, uint16_t length, uint16_t address)
{
    if (length == 2)
    {
        // ���Ӧ����START��1֮ǰ��POAP��1
        i2c_ackpos_config(i2c_periph, I2C_ACKPOS_NEXT);
    }
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
        ;
    i2c_start_on_bus(i2c_periph);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND))
        ;
    i2c_master_addressing(i2c_periph, address, I2C_RECEIVER);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND))
        ;

    if (length == 1)
    {
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
        i2c_stop_on_bus(i2c_periph);
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE))
            ;
        *data = i2c_data_receive(i2c_periph);
        while (I2C_CTL0(i2c_periph) & I2C_CTL0_STOP)
            ;
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
        return 0;
    }
    else if (length == 2)
    {
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);

        while (!i2c_flag_get(i2c_periph, I2C_FLAG_BTC))
            ;
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE))
            ;
        *data++ = i2c_data_receive(i2c_periph);
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE))
            ;
        *data = i2c_data_receive(i2c_periph);
        i2c_stop_on_bus(i2c_periph);
        while (I2C_CTL0(i2c_periph) & I2C_CTL0_STOP)
            ;
        i2c_ackpos_config(i2c_periph, I2C_ACKPOS_CURRENT);
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);
        return 0;
    }
    else if (length > 2)
    {
        i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
        // ����N-3�ֽ�
        while (length > 3)
        {
            while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE))
                ;
            *data++ = i2c_data_receive(i2c_periph);
            length--;
        }
        // ����ʣ��3�ֽ�ʱ��׼�����յ��������ֽ�
        // �ȴ����������ֽڽ��յ���λ�Ĵ���
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_BTC))
            ;
        // ���ACKENλ��׼������NACK
        i2c_ack_config(i2c_periph, I2C_ACK_DISABLE);
        // ��ȡ�����������ֽ�
        *data++ = i2c_data_receive(i2c_periph);
        length--;
        // ����ֹͣ�ź�
        i2c_stop_on_bus(i2c_periph);
        // ��ȡ�����ڶ����ֽ�
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE))
            ;
        *data++ = i2c_data_receive(i2c_periph);

        length--;
        // ��ȡ���һ���ֽ�
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_RBNE))
            ;
        *data = i2c_data_receive(i2c_periph);

        while (I2C_CTL0(i2c_periph) & I2C_CTL0_STOP)
            ;
        i2c_ack_config(i2c_periph, I2C_ACK_ENABLE);

        return 0;
    }
    else
    {
        return 0;
    }
}
int i2c_master_send(uint32_t i2c_periph, uint8_t *data, uint16_t length, uint16_t address)
{
    while (i2c_flag_get(i2c_periph, I2C_FLAG_I2CBSY))
        ;
    i2c_start_on_bus(i2c_periph);
    while (!i2c_flag_get(i2c_periph, I2C_FLAG_SBSEND))
        ;
    i2c_master_addressing(i2c_periph, address, I2C_TRANSMITTER);

    if (i2c_flag_check_timeout(i2c_periph, I2C_FLAG_ADDSEND, SET))
    {
        i2c_stop_on_bus(i2c_periph);
        while (I2C_CTL0(i2c_periph) & I2C_CTL0_STOP)
            ;
        return 1;
    }

    // while (!i2c_flag_get(i2c_periph, I2C_FLAG_ADDSEND))
    //     ;
    i2c_flag_clear(i2c_periph, I2C_FLAG_ADDSEND);
    while (length--)
    {
        while (!i2c_flag_get(i2c_periph, I2C_FLAG_TBE))
            ;
        i2c_data_transmit(i2c_periph, *data++);
    }
    i2c_stop_on_bus(i2c_periph);
    while (I2C_CTL0(i2c_periph) & I2C_CTL0_STOP)
        ;
    return 0;
}
void I2C_Scan(uint32_t i2c_periph)
{
    uint8_t address;

    for (address = 1; address < 127; address++)
    {
        if (i2c_master_send(i2c_periph, NULL, 0, address << 1))
        {
            continue;
        }
        else
        {
            printf("address:%d\n", address);
        }
    }
}
uint8_t ZXP_Initial(uint32_t i2c_periph)
{
    i2c_master_send(i2c_periph, ZXP3010D_Address, 1, ZXP3010D_Address);

    // ��ȡȫ���Ĵ�����ֵ
    // uint8_t reg_cmd[4] = {0};
    // uint8_t reg[4] = {0};
    // reg_cmd[0] = 0x30; // 0x01
    // reg_cmd[1] = 0x6c; // 0x02
    // reg_cmd[2] = 0xa5; // 0x11

    // i2c_master_send(i2c_periph, &reg_cmd[0], 1, ZXP3010D_Address);
    // i2c_master_receive(i2c_periph, &reg[0], 1, ZXP3010D_Address);
    // i2c_master_send(i2c_periph, &reg_cmd[1], 1, ZXP3010D_Address);
    // i2c_master_receive(i2c_periph, &reg[1], 1, ZXP3010D_Address);
    // i2c_master_send(i2c_periph, &reg_cmd[2], 1, ZXP3010D_Address);
    // i2c_master_receive(i2c_periph, &reg[2], 1, ZXP3010D_Address);
}
void ZXP_StartP(uint32_t i2c_periph)
{

    uint8_t buf[4] = {0};
    buf[0] = 0xA5;
    // buf[1] = 0x13;���ԭʼADCֵ
    buf[1] = 0x11; // ���У׼����

    i2c_master_send(i2c_periph, buf, 2, ZXP3010D_Address);

    buf[0] = 0x30;
    buf[1] = 0x09;
    i2c_master_send(i2c_periph, buf, 2, ZXP3010D_Address);
}
void ZXP_StartT(uint32_t i2c_periph)
{
    uint8_t buf[4];

    buf[0] = 0xA5;
    buf[1] = 0x01;
    i2c_master_send(i2c_periph, buf, 2, ZXP3010D_Address);

    buf[0] = 0x30;
    buf[1] = 0x08;
    i2c_master_send(i2c_periph, buf, 2, ZXP3010D_Address);
}
uint8_t ZXP_ConStatus(uint32_t i2c_periph)
{
    uint8_t status;
    uint8_t buf[4] = {0};

    buf[0] = ZXP3010D_CMD;
    i2c_master_send(i2c_periph, buf, 1, ZXP3010D_Address);
    i2c_master_receive(i2c_periph, buf, 1, ZXP3010D_Address);
    status = (buf[0] >> 3) & 0x01;
    return status;
}
int32_t ZXP_ResultP(uint32_t i2c_periph)
{

    int32_t ltemp;
    uint8_t buf[4];
    buf[0] = 0x06;
    i2c_master_send(i2c_periph, buf, 1, ZXP3010D_Address);
    i2c_master_receive(i2c_periph, buf, 3, ZXP3010D_Address);

    ltemp = buf[0] << 8;
    ltemp |= buf[1];
    ltemp <<= 8;
    ltemp |= buf[2];
    return (ltemp);
}
int32_t ZXP_ResultT(uint32_t i2c_periph)
{
    int32_t ltemp;
    uint8_t cmd[2] = {0};
    uint8_t buf[2] = {0};

    cmd[0] = 0x09;
    cmd[1] = 0x0a;
    i2c_master_send(i2c_periph, &cmd[0], 1, ZXP3010D_Address);
    i2c_master_receive(i2c_periph, &buf[0], 2, ZXP3010D_Address);

    ltemp = buf[0] << 8;
    ltemp |= buf[1];

    return (ltemp);
}
void ZXP8_Caculate(int32_t up, int32_t ut, float *rp, float *rt)
{
    float fp, ft, ftemp;

    // �¶�ֵ�����ж�
    ft = ut;
    if (ft >= pow(2.0, 15.0))
        ft = ft - pow(2.0, 16.0);
    // ѹ��ֵ�����ж�
    ftemp = up;
    if (ftemp >= pow(2.0, 23.0))
        ftemp = ftemp - pow(2.0, 24.0);
    ftemp = ftemp / pow(2.0, 9.0); // n=13  1Kpa// n=14  500pa//12 2kpa  //11 4kpa; // 9 10kpa //  6  100kpa

    ft = ft / 256.0;
    fp = ftemp;
    *rp = fp;
    *rt = ft;
}
void ZXP2_Caculate(int32_t up, int32_t ut, float *rp, float *rt)
{
    float fp, ft, ftemp;

    // �¶�ֵ�����ж�
    ft = ut;
    if (ft >= pow(2.0, 15.0))
        ft = ft - pow(2.0, 16.0);
    // ѹ��ֵ�����ж�
    ftemp = up;
    if (ftemp >= pow(2.0, 23.0))
        ftemp = ftemp - pow(2.0, 24.0);
    ftemp = ftemp / pow(2.0, 6.0); // n6  100kpa

    ft = ft / 256.0;
    fp = ftemp;
    *rp = fp;
    *rt = ft;
}
void ZXP8_get_data_454(uint32_t i2c_periph, float *fTemp, float *fPress)
{
    int32_t press;
    int32_t temp = 0;
    ZXP_Initial(i2c_periph);

    ZXP_StartT(i2c_periph);
    delay_ms_454(4);
    do
    {
        delay_ms_454(1);
    } while (ZXP_ConStatus(i2c_periph));
    temp = ZXP_ResultT(i2c_periph);

    ZXP_StartP(i2c_periph);
    delay_ms_454(12);
    do
    {
        delay_ms_454(1);
    } while (ZXP_ConStatus(i2c_periph));
    press = ZXP_ResultP(i2c_periph);

    ZXP8_Caculate(press, temp, fPress, fTemp);
}
void ZXP2_get_data_454(uint32_t i2c_periph, float *fTemp, float *fPress)
{
    int32_t press;
    int32_t temp = 0;
    ZXP_Initial(i2c_periph);
    ZXP_StartT(i2c_periph);
    delay_ms_454(4);
    do
    {
        delay_ms_454(1);
    } while (ZXP_ConStatus(i2c_periph));
    temp = ZXP_ResultT(i2c_periph);
    ZXP_StartP(i2c_periph);
    delay_ms_454(12);
    do
    {
        delay_ms_454(1);
    } while (ZXP_ConStatus(i2c_periph));
    press = ZXP_ResultP(i2c_periph);

    ZXP2_Caculate(press, temp, fPress, fTemp);
}
//-----------------------------------------------------------------------
//                       ���� (����ֵ��uint16_t,����ʵֵ��100��,���緵��20000,��ô����200L/M)
//-----------------------------------------------------------------------
uint16_t P13_get(void)
{
    uint16_t i2c1_flow = 0;
    FS4301_get_data_454(I2C1, &i2c1_flow);
    return i2c1_flow;
}
uint16_t P14_get(void)
{
    uint16_t i2c0_flow = 0;
    FS4301_get_data_454(I2C0, &i2c0_flow);
    return i2c0_flow;
}
uint16_t P15_get(void)
{
    uint16_t i2c2_flow = 0;
    FS4301_get_data_454(I2C2, &i2c2_flow);
    return i2c2_flow;
}

void FS4301_get_data_454(uint32_t i2c_periph, uint16_t *flow_data)
{
    uint8_t buf[2];
    i2c_master_send(i2c_periph, FS4301_CMD, 1, FS4301_Address);
    i2c_master_receive(i2c_periph, buf, 2, FS4301_Address);
    *flow_data = (uint16_t)buf[0] << 8 | buf[1];
}
//-----------------------------------------------------------------------
//                       �¶� (����ֵ��int16_t,����ʵֵ��10��,���緵��223,��ô����22.3)
//-----------------------------------------------------------------------
int16_t P7_get(void)
{
    return MAX31865_TempGet_454(GPIO_PIN_15);
}
int16_t P9_get(void)
{
    return MAX31865_TempGet_454(GPIO_PIN_12);
}

FlagStatus drdy1_status(void)
{
    return gpio_input_bit_get(GPIOG, GPIO_PIN_1);
}
FlagStatus drdy2_status(void)
{
    return gpio_input_bit_get(GPIOG, GPIO_PIN_0);
}
// if (cs_pin == GPIO_PIN_15)
// {
//     // ��ӳ�ʱ�߼�
//     while (drdy1_status())
//         ; // �ȴ�DRDY�ź�1
// }
// else if (cs_pin == GPIO_PIN_12)
// {
//     while (drdy2_status())
//         ; // �ȴ�DRDY�ź�2
// }
void MAX31865_CsOn(uint32_t cs_pin)
{
    gpio_bit_reset(GPIOB, cs_pin);
}
void MAX31865_CsOff(uint32_t cs_pin)
{
    gpio_bit_set(GPIOB, cs_pin);
}
uint8_t SPI1_Transfer(uint32_t cs_pin, uint8_t data)
{
    uint8_t data_receive = 0;
    while (spi_i2s_flag_get(SPI1, SPI_FLAG_TBE) == RESET)
        ;
    spi_i2s_data_transmit(SPI1, data); // ��������
    while (spi_i2s_flag_get(SPI1, SPI_FLAG_TRANS) == SET)
        ;
    while (spi_i2s_flag_get(SPI1, SPI_FLAG_RBNE) == RESET)
        ;
    data_receive = spi_i2s_data_receive(SPI1);
    return data_receive;
}
void MAX31865_Spi_WriteByte(uint32_t cs_pin, uint8_t data)
{
    SPI1_Transfer(cs_pin, data);
}
uint8_t MAX31865_Spi_ReadByte(uint32_t cs_pin)
{
    return SPI1_Transfer(cs_pin, 0x00);
}
void MAX31865_bufWrite(uint32_t cs_pin, uint8_t addr, uint8_t value)
{
    MAX31865_CsOn(cs_pin);
    MAX31865_Spi_WriteByte(cs_pin, addr | 0x80);
    MAX31865_Spi_WriteByte(cs_pin, value);
    MAX31865_CsOff(cs_pin);
}
uint8_t MAX31865_bufRead(uint32_t cs_pin, uint8_t addr)
{
    uint8_t data;
    MAX31865_CsOn(cs_pin);
    MAX31865_Spi_WriteByte(cs_pin, addr);
    data = MAX31865_Spi_ReadByte(cs_pin);
    MAX31865_CsOff(cs_pin);
    return data;
}
void MAX31865_HWInit(uint32_t cs_pin)
{
    MAX31865_bufWrite(cs_pin, 0x00, 0xC1);
}
uint16_t MAX31865_TempGet_454(uint32_t cs_pin)
{
    uint8_t fault;
    // ��ȡ���ϼĴ���������Ƿ��й���
    // fault = MAX31865_bufRead(cs_pin, 0x07);
    fault = MAX31865_bufRead(cs_pin, 0x00);
    // if (fault)
    // {
    //     printf("Fault detected!");
    //     if (fault & 0x01)
    //         printf("RTD low threshold exceeded.");
    //     if (fault & 0x02)
    //         printf("RTD high threshold exceeded.");
    //     if (fault & 0x04)
    //         printf("Low power mode.");
    //     if (fault & 0x08)
    //         printf("Refin- > 0.85 x Vbias.");
    //     if (fault & 0x10)
    //         printf("Refin- < 0.85 x Vbias. Force open.");
    //     if (fault & 0x20)
    //         printf("RTDIN- < 0.85 x Vbias. Force open.");
    //     if (fault & 0x40)
    //         printf("Overvoltage or undervoltage error.");
    // }

    float Z1, Z2, Z3, Z4, Rt, RTD, temp;
    int16_t temp18b20;
    uint16_t buf = 0;

    buf = MAX31865_bufRead(cs_pin, 0x01);
    buf = buf << 8;
    buf = buf + MAX31865_bufRead(cs_pin, 0x02);
    buf = buf >> 1;

    Rt = buf;
    RTD = Rt * 400 / 32768.00;
    temp = ((RTD - 100) / 0.385055) * 10;
    return (uint16_t)temp;
}
//-----------------------------------------------------------------------
//                       ��ŷ�
//-----------------------------------------------------------------------
void P4_PWM_set(uint32_t pulse)
{
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, pulse * 10);
}
void P5_PWM_set(uint32_t pulse)
{
    timer_channel_output_pulse_value_config(TIMER12, TIMER_CH_0, pulse * 10);
}
void P6_PWM_set(uint32_t pulse)
{
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, pulse * 10);
}
//-----------------------------------------------------------------------
//                       ѹ�緧Ƭ
//-----------------------------------------------------------------------
void YDP_control(FlagStatus on)
{
    if (on)
    {
        gpio_bit_set(GPIOB, GPIO_PIN_14); // ��ѹ�緧Ƭ
    }
    else
    {
        gpio_bit_reset(GPIOB, GPIO_PIN_14); // �ر�ѹ�緧Ƭ
    }
    delay_ms_454(2);
}
FlagStatus YDP_status_get(void)
{
    return gpio_output_bit_get(GPIOB, GPIO_PIN_14);
}

//-----------------------------------------------------------------------
//                       ����
//-----------------------------------------------------------------------
void Pulverizer_control(FlagStatus on)
{
    if (on)
    {
        gpio_bit_set(GPIOG, GPIO_PIN_4); // ��ѹ�緧Ƭ
    }
    else
    {
        gpio_bit_reset(GPIOG, GPIO_PIN_4); // �ر�ѹ�緧Ƭ
    }
    delay_ms_454(2);
}
FlagStatus Pulverizer_status_get(void)
{
    return gpio_output_bit_get(GPIOG, GPIO_PIN_4);
}
//-----------------------------------------------------------------------
//                       ���
//-----------------------------------------------------------------------
void motor_control(uint16_t speed)
{
    uint8_t frame[4];
    uint8_t check_sum;

    frame[0] = 0x80;                // ֡ͷ
    frame[1] = speed & 0xFF;        // ת��ָ���8λ
    frame[2] = (speed >> 8) & 0xFF; // ת��ָ���8λ

    // ����У���
    check_sum = frame[1] + frame[2]; // �ۼӺ͵�8λ
    frame[3] = check_sum;            // У��

    // ͨ�� USART ��������
    for (int i = 0; i < sizeof(frame); ++i)
    {
        while (usart_flag_get(USART2, USART_FLAG_TBE) == RESET)
            ;
        usart_data_transmit(USART2, frame[i]);
    }
}
void motor_speed_percent(uint8_t percent)
{
    uint16_t speed = percent * 8000 / 100;
    uint8_t frame[4];
    uint8_t check_sum;

    frame[0] = 0x80;                // ֡ͷ
    frame[1] = speed & 0xFF;        // ת��ָ���8λ
    frame[2] = (speed >> 8) & 0xFF; // ת��ָ���8λ

    // ����У���
    check_sum = frame[1] + frame[2]; // �ۼӺ͵�8λ
    frame[3] = check_sum;            // У��

    // ͨ�� USART ��������
    for (int i = 0; i < sizeof(frame); ++i)
    {
        while (usart_flag_get(USART2, USART_FLAG_TBE) == RESET)
            ;
        usart_data_transmit(USART2, frame[i]);
    }
}
//-----------------------------------------------------------------------
//                       SD
//-----------------------------------------------------------------------
sd_card_info_struct sd_cardinfo; /* information of SD card */
uint32_t buf_write[512];         /* store the data written to the card */
uint32_t buf_read[512];          /* store the data read from the card */
sd_error_enum sd_error;

void SD_init_454(void)
{
    uint16_t i = 5;

    /* initialize the card */
    do
    {
        sd_error = sd_io_init();
    } while ((SD_OK != sd_error) && (--i));
    if (i)
    {
        printf("\r\n Card init success!\r\n");
    }
    else
    {
        printf("\r\n Card init failed!\r\n");
        LED3(ON);
        while (1)
        {
        }
    }
    card_info_get();
}

sd_error_enum sd_io_init(void)
{
    sd_error_enum status = SD_OK;
    uint32_t cardstate = 0;
    status = sd_init();
    if (SD_OK == status)
    {
        status = sd_card_information_get(&sd_cardinfo);
    }
    if (SD_OK == status)
    {
        status = sd_card_select_deselect(sd_cardinfo.card_rca);
    }
    status = sd_cardstatus_get(&cardstate);
    if (cardstate & 0x02000000)
    {
        printf("\r\n the card is locked!");
        while (1)
        {
        }
    }
    /* set bus mode */
    if ((SD_OK == status) && (!(cardstate & 0x02000000)))
    {
        status = sd_bus_mode_config(SDIO_BUSMODE_4BIT);
    }
    /* set data transfer mode */
    if (SD_OK == status)
    {
        //        status = sd_transfer_mode_config(SD_DMA_MODE);
        status = sd_transfer_mode_config(SD_POLLING_MODE);
    }
    return status;
}

void card_info_get(void)
{
    uint8_t sd_spec, sd_spec3, sd_spec4, sd_security;
    uint32_t block_count, block_size;
    uint16_t temp_ccc;
    printf("\r\n Card information:");
    sd_spec = (sd_scr[1] & 0x0F000000) >> 24;
    sd_spec3 = (sd_scr[1] & 0x00008000) >> 15;
    sd_spec4 = (sd_scr[1] & 0x00000400) >> 10;
    if (2 == sd_spec)
    {
        if (1 == sd_spec3)
        {
            if (1 == sd_spec4)
            {
                printf("\r\n## Card version 4.xx ##");
            }
            else
            {
                printf("\r\n## Card version 3.0x ##");
            }
        }
        else
        {
            printf("\r\n## Card version 2.00 ##");
        }
    }
    else if (1 == sd_spec)
    {
        printf("\r\n## Card version 1.10 ##");
    }
    else if (0 == sd_spec)
    {
        printf("\r\n## Card version 1.0x ##");
    }

    sd_security = (sd_scr[1] & 0x00700000) >> 20;
    if (2 == sd_security)
    {
        printf("\r\n## SDSC card ##");
    }
    else if (3 == sd_security)
    {
        printf("\r\n## SDHC card ##");
    }
    else if (4 == sd_security)
    {
        printf("\r\n## SDXC card ##");
    }

    block_count = (sd_cardinfo.card_csd.c_size + 1) * 1024;
    block_size = 512;
    printf("\r\n## Device size is %dKB ##", sd_card_capacity_get());
    printf("\r\n## Block size is %dB ##", block_size);
    printf("\r\n## Block count is %d ##", block_count);

    if (sd_cardinfo.card_csd.read_bl_partial)
    {
        printf("\r\n## Partial blocks for read allowed ##");
    }
    if (sd_cardinfo.card_csd.write_bl_partial)
    {
        printf("\r\n## Partial blocks for write allowed ##");
    }
    temp_ccc = sd_cardinfo.card_csd.ccc;
    printf("\r\n## CardCommandClasses is: %x ##", temp_ccc);
    if ((SD_CCC_BLOCK_READ & temp_ccc) && (SD_CCC_BLOCK_WRITE & temp_ccc))
    {
        printf("\r\n## Block operation supported ##");
    }
    if (SD_CCC_ERASE & temp_ccc)
    {
        printf("\r\n## Erase supported ##");
    }
    if (SD_CCC_WRITE_PROTECTION & temp_ccc)
    {
        printf("\r\n## Write protection supported ##");
    }
    if (SD_CCC_LOCK_CARD & temp_ccc)
    {
        printf("\r\n## Lock unlock supported ##");
    }
    if (SD_CCC_APPLICATION_SPECIFIC & temp_ccc)
    {
        printf("\r\n## Application specific supported ##");
    }
    if (SD_CCC_IO_MODE & temp_ccc)
    {
        printf("\r\n## I/O mode supported ##");
    }
    if (SD_CCC_SWITCH & temp_ccc)
    {
        printf("\r\n## Switch function supported ##");
    }
}

//-----------------------------------------------------------------------
//                       ������־
//-----------------------------------------------------------------------
void motor_pressure_flow(int min_speed, int max_speed)
{
    if (min_speed < 0 || min_speed > 100 || max_speed < 0 || max_speed > 100)
    {
        printf("Error: Speed values must be between 0 and 100 and min_speed should be less than or equal to max_speed.\n");
        return;
    }
    int i, j;
    // �¶�
    float temperature_P7_temp;
    float temperature_P9_temp;
    // ѹ��
    float pressure_P10_temp;
    float pressure_P11_temp;
    float pressure_P12_temp;
    // ����
    float flow_P13_temp;
    float flow_P14_temp;
    float flow_P15_temp;
    TickType_t startTicks, endTicks, durationTicks;
    TickType_t currentTicks = xTaskGetTickCount();

    printf("motor_speed_percent,motor_speed,motor_temp,temperature_P7,temperature_P9,pressure_P10,pressure_P11,pressure_P12,flow_P13,flow_P14,flow_P15,diff_time\n");

    for (i = min_speed; i <= max_speed; i++)
    {
        startTicks = xTaskGetTickCount();
        motor_speed_percent(i);

        for (j = 0; j < 20; j++)
        {

            temperature_P7_temp = P7_get();
            temperature_P9_temp = P9_get();
            pressure_P10_temp = P10_get();
            pressure_P11_temp = P11_get();
            pressure_P12_temp = P12_get();
            flow_P13_temp = P13_get();
            flow_P14_temp = P14_get();
            flow_P15_temp = P15_get();
            endTicks = xTaskGetTickCount();
            durationTicks = endTicks - startTicks;
            printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
                   i,
                   motor_status.current_speed,
                   motor_status.motor_temperature,
                   temperature_P7,
                   temperature_P9,
                   pressure_P10,
                   pressure_P11,
                   pressure_P12,
                   flow_P13,
                   flow_P14,
                   flow_P15,
                   durationTicks);
        }
        delay_s_454(10);
    }
    motor_speed_percent(0);
}
void RTOS_motor_pressure_flow(int min_speed, int max_speed)
{
    if (min_speed < 0 || min_speed > 100 || max_speed < 0 || max_speed > 100)
    {
        printf("Error: Speed values must be between 0 and 100 and min_speed should be less than or equal to max_speed.\n");
        return;
    }
    int i, j;
    // �¶�
    float temperature_P7_temp;
    float temperature_P9_temp;
    // ѹ��
    float pressure_P10_temp;
    float pressure_P11_temp;
    float pressure_P12_temp;
    // ����
    float flow_P13_temp;
    float flow_P14_temp;
    float flow_P15_temp;
    TickType_t startTicks, endTicks, durationTicks;
    TickType_t currentTicks = xTaskGetTickCount();

    printf("motor_speed_percent,motor_speed,motor_temp,temperature_P7,temperature_P9,pressure_P10,pressure_P11,pressure_P12,flow_P13,flow_P14,flow_P15,diff_time\n");

    for (i = min_speed; i <= max_speed; i++)
    {
        startTicks = xTaskGetTickCount();
        motor_speed_percent(i);

        for (j = 0; j < 20; j++)
        {
            xSemaphoreGive(iicCollect_Start_Semaphore);
            xSemaphoreTake(iicCollect_Ready_Semaphore, portMAX_DELAY);

            temperature_P7_temp = P7_get();
            temperature_P9_temp = P9_get();
            pressure_P10_temp = pressure_P10;
            pressure_P11_temp = pressure_P11;
            pressure_P12_temp = pressure_P12;
            flow_P13_temp = flow_P13;
            flow_P14_temp = flow_P14;
            flow_P15_temp = flow_P15;
            endTicks = xTaskGetTickCount();
            durationTicks = endTicks - startTicks;
            printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u\n",
                   i,
                   motor_status.current_speed,
                   motor_status.motor_temperature,
                   temperature_P7_temp,
                   temperature_P9_temp,
                   pressure_P10_temp,
                   pressure_P11_temp,
                   pressure_P12_temp,
                   flow_P13_temp,
                   flow_P14_temp,
                   flow_P15_temp,
                   durationTicks);
        }
        delay_s_454(3);
    }
    motor_speed_percent(0);
}

//-----------------------------------------------------------------------
//                       FMC
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
//                       У׼
//-----------------------------------------------------------------------

void align_data(void)
{
    oxygen_voltage_air = sensor_data.oxygen;
    printf("oxygen_voltage_air : %f\n", oxygen_voltage_air);
    printf("voltage_p4 : %f\n", sensor_data.p4_valve);
    printf("voltage_p5 : %f\n", sensor_data.p5_valve);
    printf("voltage_p6 : %f\n", sensor_data.p6_valve);
}

//-----------------------------------------------------------------------
//
//
//                       �жϺ���
//
//
//-----------------------------------------------------------------------
void TIMER4_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER4, TIMER_INT_FLAG_CH0) != RESET)
    {
        if (TIMER_CHCTL2(TIMER4) & TIMER_CHCTL2_CH0P)
        {
            // CH0Pλ���ã���ʾ��ǰ����Ϊ�����½���
            pwm_values.falling_edge_value0 = timer_channel_capture_value_register_read(TIMER4, TIMER_CH_0);
            TIMER_CHCTL2(TIMER4) &= ~TIMER_CHCTL2_CH0P;
            timer_counter_value_config(TIMER4, 0);
        }
        else
        {
            pwm_values.rising_edge_value0 = timer_channel_capture_value_register_read(TIMER4, TIMER_CH_0);
            TIMER_CHCTL2(TIMER4) |= TIMER_CHCTL2_CH0P;
        }

        pwm_values.duty_cycle0 = (float)(pwm_values.falling_edge_value0 - pwm_values.rising_edge_value0) / 1000.0;

        timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_CH0);
    }
}
void I2C1_EV_IRQHandler(void)
{
    // if (i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_ADDSEND))
    // {
    //     /* clear the ADDSEND bit */
    //     // i2c_interrupt_flag_clear(I2C1, I2C_INT_FLAG_ADDSEND);
    // }
}
// ADC���ݴ���
void DMA1_Channel0_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA1, DMA_CH0, DMA_INT_FLAG_FTF) != RESET)
    {
        // ʹ��adc_to_voltage����ת��ADCֵ����ѹ
        float voltage_p4 = adc_to_voltage(adc_values_454[0]);
        float voltage_p5 = adc_to_voltage(adc_values_454[1]);
        float voltage_p6 = adc_to_voltage(adc_values_454[2]);
        float voltage_pse = adc_to_voltage(adc_values_454[3]);
        float oxygen_voltage = adc_to_voltage(adc_values_454[5]);

        // sensor_data.p4_valve = voltage_p4;
        // sensor_data.p5_valve = voltage_p5;
        // sensor_data.p6_valve = voltage_p6;
        // sensor_data.pse540 = voltage_pse;
        // sensor_data.oxygen = oxygen_voltage;

        sensor_data.p4_valve = (voltage_p4 > 1.5); // P4_PWM_set(0):1.02,P4_PWM_set(100):1.78
        sensor_data.p5_valve = (voltage_p5 > 1);   // P5_PWM_set(0):0,P5_PWM_set(100):1.95
        sensor_data.p6_valve = (voltage_p6 > 1);   // P6_PWM_set(0):0,P6_PWM_set(100):2.12
        // sensor_data.pse540 = voltage_pse;
        // sensor_data.oxygen = oxygen_voltage;

        // ���ݴ�����������ϵת����ѹ��������
        // sensor_data.p4_valve = voltage_p4 / 5.0f - 0.25f;
        // sensor_data.p5_valve = voltage_p5 / 10.0f;
        // sensor_data.p6_valve = voltage_p6 / 10.0f;
        // ����PSE540�������ı�����ϵת����ѹ��������
        if (voltage_pse < 0.25)
        { // ��Ӧ 62.5 kPa <= p <= 887.5 kPa
            sensor_data.pse540 = (voltage_pse + 0.25f) / 0.004f;
        }

        else if (voltage_pse > 3.05)
        { // ��Ӧ p > 887.5 kPa
            sensor_data.pse540 = 887.5f;
        }
        else
        { // ��Ӧ p < 62.5 kPa
            sensor_data.pse540 = 0.0f;
        }
        //! ���������������ı�����ϵת����ѹ������Ũ�� ��Ҫ����!
        //! oxygen_voltage_air��Ҫ���Ϊ���������е�ֵ,ĿǰĬ������Ϊ1.0
        sensor_data.oxygen = ((oxygen_voltage - 3.3) / (oxygen_voltage_air - 3.3)) * 20.9f * 100.0f;
    }
    dma_interrupt_flag_clear(DMA1, DMA_CH0, DMA_INT_FLAG_FTF);
}
//  USART2_RX �����������֡����
void DMA0_Channel1_IRQHandler(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH1, DMA_INT_FLAG_FTF) != RESET)
    {
        // ��֤֡ͷ
        if (MOTOR_received_frame[0] != 0x90)
        {
            usart_dma_receive_config(USART2, USART_DENR_DISABLE);
            motor_status.frame_header = MOTOR_received_frame[0];
            usart_data_receive(USART2);
            usart_dma_receive_config(USART2, USART_DENR_ENABLE);
            return;
        }
        // ����У���
        uint8_t calculated_checksum = 0;
        for (int i = 1; i < MOTOR_FRAME_SIZE - 1; i++)
        {
            calculated_checksum += MOTOR_received_frame[i];
        }
        // ��֤У���
        if (calculated_checksum != MOTOR_received_frame[MOTOR_FRAME_SIZE - 1])
        {
            motor_status.checksum_valid = 1;
            return;
        }
        // ��������
        motor_status.frame_header = MOTOR_received_frame[0];
        motor_status.current_speed = (uint16_t)MOTOR_received_frame[2] << 8 | (uint16_t)MOTOR_received_frame[1];
        motor_status.motor_temperature = (int8_t)MOTOR_received_frame[3];
        motor_status.fault_alarm = MOTOR_received_frame[4];
        motor_status.checksum = MOTOR_received_frame[5];
        // ��֤У���
        motor_status.checksum_valid = 0;
        // printf("\n SPEED: %f\n",motor_status.current_speed);
        // printf("\n TEMP_: %f\n",motor_status.motor_temperature);
    }
    dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
}
void USART0_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE) != RESET)
    {
        int received_data = usart_data_receive(USART0);
        if (received_data != '\n')
        {
            if (usart0_res_length < 11)
            {
                usart0_res[usart0_res_length++] = received_data;
                usart0_res[usart0_res_length] = '\0'; // ȷ���ַ�����null��β
            }
            else
            {
                usart0_res_length = 0;
                memset(usart0_res, 0, 12);
            }
        }
        else
        {
            // �ɹ���������

            // int motor_speed = atoi(usart0_res); // ��������������ڱ�����
            // motor_control(motor_speed);

            uint8_t motor_speed = atoi(usart0_res); // ��������������ڱ�����
            motor_speed_percent(motor_speed);
            printf("Set motor speed: %hu\n", motor_speed);
            usart0_res_length = 0; // ���û�����
            memset(usart0_res, 0, 12);
        }
    }
}
