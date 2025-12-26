/*
 * gpio_driver.h
 *
 *  Created on: Dec 26, 2025
 *      Author: wiki
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_
/*
 * The GPIO peripheral is to be consisted of following confgurable item for user application
 * GPIO part name
 * GPIo pin Number
 * GPIo mode
 * GPIO speed
 * GPIO outputType
 * GPIO Pullup-pulldown
 * GPIO Alternate function
 */
typedef struct
{
	uint8_t GPIO_PinNumber;//uint8_t kept based on the values its representing the pins are 15 so 8 bits are enough for the bits likewise based on registers
	uint8_t Mode;
	uint8_t Speed;
	uint8_t PuPdControl;
	uint8_t OPType;
	uint8_t AltFuncType;
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_TypeDef *pGPIOx; //BaseAddress;//Holds the base address of the the port which the gpio pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;//Holds Pin Configuration Settings

}GPIO_Handle_t;


//macro for resetting the port using the rcc resgister
#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (2<<0)); (RCC->AHB1RSTR &= ~(2<<0));}while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (3<<0)); (RCC->AHB1RSTR &= ~(3<<0));}while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (4<<0)); (RCC->AHB1RSTR &= ~(4<<0));}while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (5<<0)); (RCC->AHB1RSTR &= ~(5<<0));}while(0)
#define GPIOF_REG_RESET()   do{ (RCC->AHB1RSTR |= (6<<0)); (RCC->AHB1RSTR &= ~(6<<0));}while(0)
#define GPIOG_REG_RESET()   do{ (RCC->AHB1RSTR |= (7<<0)); (RCC->AHB1RSTR &= ~(7<<0));}while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (9<<0)); (RCC->AHB1RSTR &= ~(8<<0));}while(0)

//GPIO PIN Numbers
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3

#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15



//macro to setup the port address for
#define GPIO_BASEADDRESS_TO_CODE(x)  (  (x==GPIOA)? 0 :\
										(x==GPIOB)? 1 :\
										(x==GPIOC)? 2 :\
										(x==GPIOD)? 3 :\
										(x==GPIOE)? 4 :\
										(x==GPIOF)? 5 :\
										(x==GPIOG)? 6 :\
										(x==GPIOA)? 7:0 )

/*Macros for gpio modes
 * taken from the dataseet
 */
#define GPIO_MODE_IN 0
#define GPIO_MODE_OUT 1
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_ALTFN 2
#define GPIO_MODE_IT_FT 4//our made based on the trigger of interrupt
#define GPIO_MODE_IT_RT 5
#define GPIO_MODE_IT_RTFT 6
//GPIO Pin possible output types
//from the datasheet
#define GPIO_OP_TYPE_PP 0;
#define GPIO_OP_TYPE_OD 1;


//GPIO pin possible output speeds
#define GPIO_SPEED_LOW 0;
#define GPIO_SPEED_MEDIUM 1;
#define GPIO_SPEED_HIGH 2;
#define GPIO_SPEED_HIGH 3;

//GPIO PULL UP PULL DOWN configurations

#define GPIO_NO_PUPD 0;
#define GPIO_PIN_PU 1;
#define GPIO_PIN_PD 2;

//nvic -may be needed to change for the M0 processor
#define NVIC_ISER0  (volatile uint32_t)0xE000E100
#define NVIC_ISER1  (volatile uint32_t)0xE000E104
#define NVIC_ISER2  (volatile uint32_t)0xE000E108
#define NVIC_ISER3  (volatile uint32_t)0xE000E10C

//now to reset those
#define NVIC_ICER0  (volatile uint32_t)0xE000E180
#define NVIC_ICER1  (volatile uint32_t)0xE000E184
#define NVIC_ICER2  (volatile uint32_t)0xE000E188
#define NVIC_ICER3  (volatile uint32_t)0xE000E18C
/*Generic Macros
 *
 *
 */
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 1
/*Driver APIs requirements
 * Initialisation
 * Enable Disable of GPIO port
 * read from gpio pin
 * write to gpio pin
 * Configure alternate functionality
 * Interrupt handling
 */
//Prototypes of the APIs

//init and deinit
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);//pointer to handle structure
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);//to do important register in RCC register -Peripheral reset register what that need the base address of peripheral to know which port is to be

//clock setup enable or disable
void GPIO_PeriClockCont(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);//enable or diable clock on given gpio base address

//read and write
int8_t GPIO_Read_InPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);//return will be 0 or 1 so bool or int8_t is enough
int16_t GPIO_Read_InPort(GPIO_RegDef_t *pGPIOx);//port is of 16 bits so return is set accordingly
void GPIO_Write_OutPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t value);
void GPIO_Write_OutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_Toggle_OutPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);

//interrupt handling
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);//to setup irq number of gpio pin and setting the priority
void GPIO_IRQHandler(uint8_t Pin);//When irq happen to handle the trigger of which pin




#endif /* INC_GPIO_DRIVER_H_ */
