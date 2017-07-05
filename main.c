#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define C GPIO_Pin_6
#define D GPIO_Pin_5
#define A GPIO_Pin_9
#define B GPIO_Pin_8
#define AD (GPIO_Pin_9 | GPIO_Pin_5)
#define AB (GPIO_Pin_9 | GPIO_Pin_8)
#define BC (GPIO_Pin_8 | GPIO_Pin_6)
#define CD (GPIO_Pin_6 | GPIO_Pin_5)


uint8_t turnRight = 1; //direction of rotation
uint8_t stepMode = 0;  //mode of step (0 - waveStep, 1 - halfStep, 2 - fullStep)

void ledInit(void);
// RTOS task
void vTaskLedRed(void *p);
void vTaskLedGreen(void *p);
void vTaskStepperMotor(void *p);

void turnWaveStep(void);
void turnHalfStep(void);
void turnFullStep(void);
void (*fun_ptr[3])(void) = {turnWaveStep, turnHalfStep, turnFullStep};

void interpretSequence(uint16_t * lineTab, uint8_t numOfElems, uint8_t numOfMs);
void changeDirectionOfRotation(void);
void changeStepMode(void);

void vTimerCallback(TimerHandle_t xTimer);

TimerHandle_t xTimer1;


int main(void){
    // Configure GPIO for LED
    ledInit();
    
	xTimer1 = xTimerCreate("Timer1", pdMS_TO_TICKS(3000), pdTRUE, ( void * ) 0, vTimerCallback);

	xTaskCreate(vTaskLedGreen, (const char*) "Green LED Blink", 128, NULL, 1, NULL);
    xTaskCreate(vTaskLedRed, (const char*) "Red LED Blink", 128, NULL, 1, NULL);
	xTaskCreate(vTaskStepperMotor, (const char*) "Stepper Motor move", 128, NULL, 1, NULL);
	
	if( xTimer1 != NULL )
		xTimerStart( xTimer1, 0);
			 
	// Start RTOS scheduler
    vTaskStartScheduler();

    return 0;
}


void ledInit(){
    GPIO_InitTypeDef  gpio;
    // Configure PA5, push-pull output
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8| GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);
	GPIO_Init(GPIOC, &gpio);
}

void vTaskLedGreen(void *p){
    for (;;){
        GPIOA->ODR ^= GPIO_Pin_5;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void vTaskLedRed(void *p){
    for (;;){
        GPIOC->ODR ^= GPIO_Pin_0;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskStepperMotor(void *p){
    for (;;){
		(*fun_ptr[stepMode])();
		
		//turnWaveStep();
		//turnHalfStep();
		//turnFullStep();
    }
}

void turnWaveStep(void){
	uint16_t rightSeq[] = {A,B,C,D};
	interpretSequence(rightSeq, 4, 10);
}


void turnHalfStep(void){
	uint16_t halfRightSeq[] = {AD, A, AB, B, BC, C, CD, D};
	interpretSequence(halfRightSeq, 8, 4);
}


void turnFullStep(void){
	uint16_t fullRightSeq[] = {AD, AB, BC, CD};
	interpretSequence(fullRightSeq, 4, 5);
}


void interpretSequence(uint16_t * lineTab, uint8_t numOfElems, uint8_t numOfMs){
	int8_t i;
	
	if(turnRight == 1){
		for(i=0; i<numOfElems; i++){
			GPIOC->ODR ^= (uint32_t)lineTab[i];
			vTaskDelay(pdMS_TO_TICKS(numOfMs));
			GPIOC->ODR ^= (uint32_t)lineTab[i];
		}
	}
	else{
		for(i=numOfElems-1; i>=0; i--){
			GPIOC->ODR ^= (uint32_t)lineTab[i];
			vTaskDelay(pdMS_TO_TICKS(numOfMs));
			GPIOC->ODR ^= (uint32_t)lineTab[i];
		}
	}		
}

void vTimerCallback(TimerHandle_t xTimer){
	const uint32_t ulMaxExpiryCountBeforeDirChange = 7;
	uint32_t ulCount;
	
	changeStepMode();
	ulCount = ( uint32_t ) pvTimerGetTimerID(xTimer);
	ulCount++;
	
	if( ulCount % ulMaxExpiryCountBeforeDirChange == 0){
        changeDirectionOfRotation();
    }
    vTimerSetTimerID( xTimer, ( void * ) ulCount );
    
}

void changeDirectionOfRotation(void){
	if(turnRight == 1)
		turnRight = 0;
	else
		turnRight = 1;
}

void changeStepMode(void){
	if(stepMode < 2)
		stepMode++;
	else
		stepMode = 0;
}
