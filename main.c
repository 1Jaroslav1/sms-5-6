/* USER CODE BEGIN Header */
/**
 * This whole project is in sync with its *.ioc file, and this includes the
 * ability to regenerate code. On the topic o BSP -- implementation of BSP in
 * this project is a custom implementation (it is not far from the original, but
 * some minor tweaks were made).
 */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
//
#include "stm32f746g_discovery_lcd.h"
//#include "stm32f746g_discovery_ts_remote.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
uint32_t ADC3_buffer[2] = {0};

#define ADC_BUFFER_LENGTH 100
uint32_t uhADCxConvertedValue[ADC_BUFFER_LENGTH] = {0};

__IO uint32_t input = 0;
__IO uint32_t output = 0;
char text[100] = "TAA  AEST";
char text2[100] = "TEST2";
int c = 0 ;
char bufor[16] = {0};

//float K = 16.5f;
//float Ti = 10000000.2f;
//float Td = 0.0f;
//float T = 0.1f;
//float Tv = 100000000.0f;

//float K = 9.9f;
//float Ti = 0.3f;
//float Td = 0.075f;
//float T = 0.1f;
//float Tv = 100000000.0f;

float K = 6.0f;
float Ti = 0.75f;
float Td = 0.1f;
float T = 0.1f;
float Tv = 1.0f;

float e = 0.0f;
float e_prev = 0.0f;
float u_i_prev = 0.0f;
float u_w_prev = 0.0f;
float u_prev = 0.0f;
float u = 1000.0f;

float y_zad = 1000.0f;

const int Nu = 30;
const int N = 30;
const int lambda = 1;
int D = 35;

float dU = 0;
float dUP[34]; // D-1
float s[30];

//35 4 4 1 float Ku[34] = {0.172792206449312, 0.218973067180662, 0.180562803539252, 0.148890111317859, 0.12277315600843, 0.101237400542277, 0.0834792523200574, 0.0688360776806552, 0.0567614761604508, 0.0468048919181643, 0.0385948015389605, 0.0318248508817446, 0.02624242367519, 0.021639215307155, 0.0178434600746161, 0.0147135218590456, 0.012132609067486, 0.0100044166308114, 0.00824953244319739, 0.00680247415144357, 0.00560924572388525, 0.00462532262386445, 0.00381399040582836, 0.00314497473985876, 0.00259331174489452, 0.00213841647787285, 0.0017633148200715, 0.00145401009899464, 0.00119896081171298, 0.000988649961246814, 0.000815229936061809, 0.000672229681588466, 0.00055431323706216, 0.000457080627645483}; // D-1 // matlab params
// float Ke = 0.4325; // matlab params
// 35 35 35 1 float Ku[34] = {0.446607219765467, 0.508453902203438, 0.419265539096817, 0.34572177416539, 0.285078390629342, 0.235072520381481, 0.193838227150474, 0.159836880311931, 0.131799741895174, 0.10868062445747, 0.0896168532853404, 0.0738970762530823, 0.060934720184458, 0.0502461032591032, 0.0414323867424341, 0.0341646925796901, 0.0281718315268883, 0.023230183901938, 0.019155355362778, 0.0157952963533663, 0.0130246284741587, 0.010739966069311, 0.00885605845869633, 0.00730260886465525, 0.00602165133381376, 0.00496538777552793, 0.00409440440746126, 0.00337620105613113, 0.00278397843423794, 0.00229563814282547, 0.00189295808400824, 0.0015609125153331, 0.00128711137404801, 0.00106133795003259};
//float Ke = 0.7927;
// 35 15 15 1 float Ku[34] = {0.446338874565018, 0.508173569300676, 0.419034379646628, 0.345531162613087, 0.284921214429789, 0.234942914607816, 0.193731355648203, 0.159748755240998, 0.131727074926319, 0.108620704070381, 0.0895674435900487, 0.0738563335591942, 0.0609011242050565, 0.0502184003822378, 0.0414095432534127, 0.034145856088693, 0.0281562991626005, 0.023217376084367, 0.0191447941766062, 0.0157865877148541, 0.0130174474261682, 0.0107340446557434, 0.0088511757258856, 0.00729858261662701, 0.00601833133376187, 0.0049626501398266, 0.00409214698302872, 0.00337433960865432, 0.00278244350502438, 0.00229437245699764, 0.0018919144133291, 0.00156005191591533, 0.00128640173318843, 0.00106075278794767};
//float Ke = 0.7924;

//35 5 5 1 float Ku[34] = {0.301856789818667, 0.364499203701961, 0.300562067491942, 0.247840190834612, 0.204366308448365, 0.168518220907453, 0.138958280322356, 0.11458347688557, 0.0944842807836161, 0.0779107036882163, 0.0642443134334109, 0.052975157624646, 0.0436827350994245, 0.0360203052020506, 0.0297019493833383, 0.0244919023373562, 0.0201957545735732, 0.0166531981541468, 0.0137320449082983, 0.0113232939173649, 0.00933706421696526, 0.00769924094772783, 0.00634870980789242, 0.00523507661319843, 0.0043167868709304, 0.00355957520125992, 0.00293518674705695, 0.00242032286241608, 0.00199577173895583, 0.00164569152978166, 0.00135701922135253, 0.00111898319569315, 0.000922701294529575, 0.00076084938737548};
//float Ke = 0.6536;

//35 10 10 1 float Ku[34] = {0.444749908691352, 0.506124756621716, 0.41734495104881, 0.344138078284186, 0.283772492341189, 0.233995690947724, 0.192950285386612, 0.159104693253059, 0.13119598846111, 0.10818277598456, 0.0892063328841392, 0.0735585656238994, 0.0606555880250465, 0.0500159339358951, 0.0412425916380013, 0.0340081896141143, 0.0280427808945907, 0.0231237701631623, 0.019067607758612, 0.0157229406394759, 0.0129649647445064, 0.0106907680109321, 0.00881549027829002, 0.00726915678716053, 0.00599406711688549, 0.00494264213219185, 0.0040756485989452, 0.00336073522173404, 0.00277122547648656, 0.00228512219346039, 0.00188428674727172, 0.00155376222597854, 0.0012812153237149, 0.00105647613146739};
//float Ke = 0.7882;

//35 10 4 1 float Ku[34] = {0.450919019215077, 0.510463039031406, 0.4209222511453, 0.34708788078583, 0.28620486719485, 0.236001400626736, 0.194604171632982, 0.160468469748008, 0.132320543630645, 0.109110071867727, 0.0899709709190118, 0.0741890777775573, 0.0611755014452265, 0.0504446488510877, 0.0415961052642603, 0.0342996930806886, 0.0282831514622661, 0.0233219771021178, 0.0192310470308575, 0.0158577108743268, 0.0130760948049395, 0.0107824046423109, 0.00889105284145002, 0.00733146484961765, 0.00604544566314972, 0.00498500831904035, 0.00411058329287077, 0.00338954198794213, 0.00279497922057648, 0.00230470927082317, 0.00190043803686056, 0.00156708040257789, 0.00129219734635501, 0.00106553178712475};
//float Ke = 0.7840;

//35 10 2 1 float Ku[34] = {0.546182835750304, 0.602035713169771, 0.496432077442746, 0.40935245700335, 0.337547555181916, 0.278338019132388, 0.229514483826699, 0.189255131046905, 0.156057709432518, 0.128683478955655, 0.106110988148849, 0.0874979593130627, 0.0721498594774267, 0.0594939843566759, 0.049058088266129, 0.0404527625835039, 0.0333569052214203, 0.0275057389134791, 0.0226809312241186, 0.0187024476168903, 0.0154218335837361, 0.0127166751623282, 0.0104860311393009, 0.00864666649503825, 0.00712994654347381, 0.00587927587376775, 0.00484798652964742, 0.00399759662520831, 0.00329637441856486, 0.00271815426270102, 0.00224136025150218, 0.00184820112896099, 0.00152400642012051, 0.00125667901191811};
//float Ke = 0.8576;

//35 10 1 1float Ku[34] = {0.809024347923088, 0.875715141698191, 0.722105145352398, 0.595439998534052, 0.490993304972512, 0.404867704758405, 0.33384947757183, 0.275288624815092, 0.226999986652005, 0.187181704200898, 0.154347984351491, 0.127273658368872, 0.104948465525198, 0.0865393558828306, 0.0713594055819438, 0.058842184726945, 0.0485206214261967, 0.0400095733105273, 0.0329914561981721, 0.0272043936491951, 0.0224324452177823, 0.0184975487760484, 0.0152528762424461, 0.012577354788145, 0.0103711490837813, 0.00855193600958135, 0.00705183282210699, 0.00581486415417875, 0.00479487333073928, 0.00395380006277764, 0.00326026025259161, 0.00268837491675334, 0.0022168045288049, 0.00182795274881706};
//float Ke = 1.1796;

//35 10 1 0.1
float Ku[34] = {1.07108434625265, 1.15937769049507, 0.956010186245933, 0.788315534878257, 0.650036360983421, 0.536012867824334, 0.44199034349187, 0.364460399118784, 0.300530055648754, 0.247813794219127, 0.204344542088179, 0.168500272605103, 0.138943480348704, 0.1145712729893, 0.094474217584339, 0.0779024056842465, 0.0642374709901694, 0.052969515420848, 0.0436780825999335, 0.036016468801898, 0.0296987859297665, 0.0244892937881688, 0.020193603592467, 0.0166514244786717, 0.0137305823549165, 0.0113220879118557, 0.0093360697580235, 0.00769842092776991, 0.00634803362841111, 0.00523451904299975, 0.00431632710464797, 0.00355919608301642, 0.00293487413030322, 0.002420065082063};
float Ke = 1.5617;

//35 10 1 0.01float Ku[34] = {1.10694047592495, 1.19818957020838, 0.988014038534511, 0.814705561154132, 0.671797287779411, 0.553956689860362, 0.456786622725711, 0.376661249011646, 0.310590742917191, 0.256109726814159, 0.21118527729692, 0.174141067978021, 0.143594818467812, 0.118406715487735, 0.0976368814849414, 0.0805103037174607, 0.0663879151617259, 0.0547427481454732, 0.0451402708944597, 0.037222173264126, 0.0306929966314105, 0.025309109049893, 0.0208696142834045, 0.0172088554946554, 0.0141902338689328, 0.0117011115188658, 0.00964860847548731, 0.00795613693307224, 0.00656054342536669, 0.00540975229538982, 0.00446082252642667, 0.00367834542613612, 0.00303312337440451, 0.00250108033329084};
//float Ke =  1.6140;

//35 10 1 10float Ku[34] = {0.234725734229751, 0.254075022759092, 0.209507489940261, 0.172757582889154, 0.142454011807468, 0.117466018803139, 0.0968611932958985, 0.0798706797276313, 0.0658604882211789, 0.0543078376636307, 0.0447816484717591, 0.0369264571399277, 0.030449152352371, 0.0251080377265706, 0.0207038130711645, 0.0170721376299392, 0.0140774978142321, 0.0116081506021934, 0.00957195392117024, 0.00789292842666033, 0.00650842238287394, 0.00536677385427879, 0.00442538297433794, 0.00364912235941268, 0.00300902635346668, 0.00248121019359687, 0.0020459787657613, 0.00168709169450809, 0.0013911573440104, 0.00114713311795326, 0.000945913412290022, 0.000779989845595801, 0.000643171088735927, 0.000530351839477847};
//float Ke =  0.3422;

float our_PID(){
	u = K*e + u_i_prev + K*T*(e_prev + e)/(2*Ti) + T*(u_w_prev - u_prev)/Tv + K*Td*(e - e_prev)/T;
	u_i_prev = u_i_prev + K*T*(e_prev + e)/(2*Ti) + T*(u_w_prev - u_prev)/Tv;
	u_prev = u;

	return u;
}

float our_DMC() {
	float Ku_dUP = 0;

	for(int i = 0; i < D - 1; i++) {
		Ku_dUP += Ku[i] * dUP[i];
	}

	dU = Ke*e - Ku_dUP;

	for(int i = D - 2; i > 0; i--){
		dUP[i] = dUP[i-1];
	}

	dUP[0] = dU;

	u = u_prev + dU;
	u_prev = u;

	return u;
}

TS_StateTypeDef TS_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == TS_INT_PIN){
		BSP_TS_GetState(&TS_State); /*!*/
	}	else if(GPIO_Pin == TAMPER_BUTTON_PIN){
		//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);/
		HAL_Delay(1000);
		//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	}
}

void DrawPointOfTouch(TS_StateTypeDef *TSS){
	static uint16_t lastx = 0;
	static uint16_t lasty = 0;
	BSP_LCD_SelectLayer(1);
	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
	BSP_LCD_DrawCircle(lastx,lasty, 3);
	BSP_LCD_DrawCircle(lastx,lasty, 2);
	if(TSS->touchDetected > 0){
		lastx = TSS->touchX[0];
		lasty = TSS->touchY[0];
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawCircle(lastx,lasty, 3);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawCircle(lastx,lasty, 2);
	}
	BSP_LCD_SelectLayer(0);
}

void updateControlSignalValue(uint16_t out){
	static uint8_t buffertx[] = {0x0F, 0xFF};
	buffertx[0] = (out>>8)&0x0F;
	buffertx[1] = (out>>0)&0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, 0xC2, (uint8_t*)buffertx, 2,1000);
}

void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc){
	DrawPointOfTouch(&TS_State);
	BSP_LCD_DisplayStringAt( 0, 10, (uint8_t*)text, CENTER_MODE); // TODO For testing purposes -- can be removed
	BSP_LCD_DisplayStringAt( 0, 40, (uint8_t*)text2, CENTER_MODE); // TODO For testing purposes -- can be removed
	HAL_LTDC_ProgramLineEvent(hltdc, 272);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	static int i=0;
	static uint32_t tmpval= 0;
	for(i=0,tmpval=0;i<ADC_BUFFER_LENGTH; ++i){
		tmpval += uhADCxConvertedValue[i];
	}
	input = tmpval/ADC_BUFFER_LENGTH;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		static float y = 0.0f;
//		static float u = 0.0f;
		y = (input-2048.0f); // przejscie z 0 - 4095 do -2048 - 2047

		e = y_zad - y;


		u = our_PID();
//		u = our_DMC();

//		u = 1000.0f;

		e_prev = e;


		if(u >  2047.0f) u =  2047.0f;
		if(u < -2048.0f) u = -2048.0f;

		u_w_prev = u;

		output = u+2048.0f; // przejscie z -2048 - 2047 do 0 - 4095
		updateControlSignalValue(output);

		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		sprintf(text,"U=%.2f;Y=%.2f;\n",u,y);
		if(HAL_UART_Transmit_IT(&huart1, (uint8_t*)text, strlen(text))!= HAL_OK){
			Error_Handler();
		}
	} else if (htim->Instance == TIM3){
	} else if (htim->Instance == TIM4){
	} else if (htim->Instance == TIM5){
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_Delay(100); /*! Delay so that LCD will not restart during initialisation !*/
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_LTDC_Init();
	MX_FMC_Init();
	MX_DMA2D_Init();
	MX_ADC3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	BSP_LCD_SetFont(&Font20); // choose size of the font: Font8, Font12, Font16, Font20, Font24
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE); // each character has background!!!
	BSP_TS_Init(0,0); // initialization of TouchScreen -- arguments are irrelevant
	BSP_TS_ITConfig(); // to cancel exti interrupts from the touch screen comment this line


	//BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);

	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
	HAL_LTDC_ProgramLineEvent(&hltdc, 272);

	if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*)uhADCxConvertedValue, ADC_BUFFER_LENGTH) != HAL_OK)
		Error_Handler();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_Delay(100);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); // preemption priority and subpriority of SysTick
	HAL_NVIC_SetPriority(TIM2_IRQn   , 0, 0); // preemption priority and subpriority of TIM2
	HAL_NVIC_SetPriority(TIM3_IRQn   , 0, 0); // preemption priority and subpriority of TIM3
	HAL_NVIC_SetPriority(TIM4_IRQn   , 0, 0); // preemption priority and subpriority of TIM4
	HAL_NVIC_SetPriority(TIM5_IRQn   , 0, 0); // preemption priority and subpriority of TIM5

	HAL_NVIC_EnableIRQ(TIM3_IRQn); // enabling handling functions of TIM3
	HAL_NVIC_EnableIRQ(TIM4_IRQn); // enabling handling functions of TIM4
	HAL_NVIC_EnableIRQ(TIM5_IRQn); // enabling handling functions of TIM5
	HAL_NVIC_EnableIRQ(TS_INT_EXTI_IRQn);


	HAL_Delay(100); // wait for everything to set up before the controller loop starts
	HAL_NVIC_EnableIRQ(TIM2_IRQn); // enabling handling functions of TIM2

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//		BSP_TS_GetState(&TS_State); /*!*/
		if(TS_State.touchDetected > 0){
			sprintf(text2,"X=%d;Y=%d;\n",TS_State.touchX[0],TS_State.touchY[0]); // 22 znaki
			if(TS_State.touchY[0] < 30){
//				u = 0.0f;
				y_zad = 0.0f;
			}
			if(TS_State.touchY[0] > 250) {
//				u = 1000.0f;
				y_zad = 1000.0f;
			}
		} else {
			sprintf(text2,"NO TOUCH\n"); // 22 znaki
		}
		HAL_Delay(100);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
			|RCC_PERIPHCLK_I2C1;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc3.Init.ContinuousConvMode = ENABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void)
{

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x10606AA0;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void)
{

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = {0};
	LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 40;
	hltdc.Init.VerticalSync = 9;
	hltdc.Init.AccumulatedHBP = 53;
	hltdc.Init.AccumulatedVBP = 11;
	hltdc.Init.AccumulatedActiveW = 533;
	hltdc.Init.AccumulatedActiveH = 283;
	hltdc.Init.TotalWidth = 565;
	hltdc.Init.TotalHeigh = 285;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK)
	{
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 480;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 272;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
	pLayerCfg.FBStartAdress = 0xC0000000;
	pLayerCfg.ImageWidth = 480;
	pLayerCfg.ImageHeight = 272;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
	{
		Error_Handler();
	}
	pLayerCfg1.WindowX0 = 0;
	pLayerCfg1.WindowX1 = 480;
	pLayerCfg1.WindowY0 = 0;
	pLayerCfg1.WindowY1 = 272;
	pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
	pLayerCfg1.Alpha = 255;
	pLayerCfg1.Alpha0 = 0;
	pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	pLayerCfg1.FBStartAdress = 0xC0000000+480*272*4;
	pLayerCfg1.ImageWidth = 480;
	pLayerCfg1.ImageHeight = 272;
	pLayerCfg1.Backcolor.Blue = 0;
	pLayerCfg1.Backcolor.Green = 0;
	pLayerCfg1.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */
	BSP_LCD_LayerDefaultInit(0, pLayerCfg.FBStartAdress);
	BSP_LCD_LayerDefaultInit(1, pLayerCfg1.FBStartAdress);
	/* Assert display enable LCD_DISP pin */
	HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

	/* Assert backlight LCD_BL_CTRL pin */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);
	/* USER CODE END LTDC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 10800-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 10800-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 200;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 10800-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 10800-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 10000;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

	/* USER CODE BEGIN FMC_Init 0 */
	FMC_SDRAM_CommandTypeDef Command;

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = {0};

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 2;
	SdramTiming.ExitSelfRefreshDelay = 6;
	SdramTiming.SelfRefreshTime = 4;
	SdramTiming.RowCycleDelay = 6;
	SdramTiming.WriteRecoveryTime = 2;
	SdramTiming.RPDelay = 2;
	SdramTiming.RCDDelay = 2;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
	{
		Error_Handler( );
	}

	/* USER CODE BEGIN FMC_Init 2 */
	__IO uint32_t tmpmrd =0;
	/* Step 3:  Configure a clock configuration enable command */
	Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 4: Insert 100 us minimum delay */
	/* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
	HAL_Delay(1);

	/* Step 5: Configure a PALL (precharge all) command */
	Command.CommandMode = FMC_SDRAM_CMD_PALL;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 1;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 6 : Configure a Auto-Refresh command */
	Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 8;
	Command.ModeRegisterDefinition = 0;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 7: Program the external memory mode register */
	tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
			SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
			SDRAM_MODEREG_CAS_LATENCY_2           |
			SDRAM_MODEREG_OPERATING_MODE_STANDARD |
			SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

	Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	Command.AutoRefreshNumber = 1;
	Command.ModeRegisterDefinition = tmpmrd;

	/* Send the command */
	HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

	/* Step 8: Set the refresh rate counter */
	/* (15.62 us x Freq) - 20 */
	/* Set the device refresh counter */
	hsdram1.Instance->SDRTR |= ((uint32_t)((1292)<< 1));

	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pin : LCD_BL_CTRL_Pin */
	GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_DISP_Pin */
	GPIO_InitStruct.Pin = LCD_DISP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_DISP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PI11 LCD_INT_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_11|LCD_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pins : PC7 PC6 */
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : TS_TOUCHED_Pin */
	GPIO_InitStruct.Pin = TS_TOUCHED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(TS_TOUCHED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
