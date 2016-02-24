#ifndef _USART_H
#define _USART_H

//Tx  101  PA9  default
//Rx  102  PA10 default


#define USARTy                   USART1
#define USARTy_GPIO              GPIOA
#define USARTy_CLK               RCC_APB2Periph_USART1
#define USARTy_GPIO_CLK          RCC_APB2Periph_GPIOA
#define USARTy_TxPin             GPIO_Pin_9
#define USARTy_RxPin             GPIO_Pin_10
#define USARTy_IRQn              USART1_IRQChannel  
#define USARTy_IRQHandler        USART1_IRQHandler

typedef enum
{
	I_alpha_beta_watch,
	Speed_Iq_Id_watch,
	Electric_Angle_hallstate_section
}Watcher_type;

void usart_init(int baudrate);

void usart_watcher(Watcher_type type);
 
#endif
