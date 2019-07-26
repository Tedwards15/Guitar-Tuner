//
// [My partner and I] - Digital Signal Processing - Project
//
#include "audio.h"
#include "arm_math.h"
#include "mfs_hl.h"
#include "uart_init.h"
#include "arm_const_structs.h"

volatile uint32_t u32AdcResult = 0;   	 // global ADC result variable

// DMA_BUFFER_SIZE is the size of the ping and pong buffers

float32_t xData[DMA_BUFFER_SIZE]; // input buffer in float format
float32_t yData[DMA_BUFFER_SIZE]; // output buffer in float format

// YIN data
#define N_LAGS 200 // number of lag values to calculate
#define W 100 // window size
#define DATA_LEN 512 // this should be a multiple of DMA_BUFFER_SIZE and > N_LAGS+W
#define NUM_BUFFERS DATA_LEN/DMA_BUFFER_SIZE // number of dma buffers needed
float32_t input_data[DATA_LEN]; // input data to YIN function
float32_t rt0[N_LAGS];
float32_t rtau[N_LAGS];
float32_t dtau[N_LAGS];
float32_t dtauprime[N_LAGS];

uint16_t storePeak = 0; //Stores peak found globally
uint16_t confirmPeak = 0; //"storePeak" goes here when it is not 0.

//What note is user trying to tune? 6 = Low E, 5 = A, 4 = D, 3 = G, 2 = B, 1 = High e
int onNote = 6;

//YIN value for Low E note is 97
int inTuneYin = 97;

//Counter to keep track of time and only detect a button press every so often to prevent bouncing.
int theCounter = 0;

volatile int16_t audio_chR=0;
volatile int16_t audio_chL=0;

/**
 ******************************************************************************
 ** \brief  ADC0 initialization and single conversion start
 **
 ** \return  uint32_t   ADC result
 ******************************************************************************/
uint32_t Main_adc_polling(void)
{
  stc_adcn_t*  	pstcAdc = NULL;
  stc_adc_config_t stcConfig;
  uint32_t     	u32Result;
 
  PDL_ZERO_STRUCT(stcConfig);   	 // Clear local configuration to zero.
 
  //stcConfig.u32CannelSelect.AD_CH_0 = 1u;
    // This works for getting the ADC connected to the light sensor
    // stcConfig.u32CannelSelect.AD_CH_17 = 1u;
    
    // 102  PB0/AN16/SCK6_1/TIOA9_1ANA-1/SCK1   CN10 - pin 1
    stcConfig.u32CannelSelect.AD_CH_16 = 1u;
  stcConfig.bLsbAlignment = TRUE;
  stcConfig.enScanMode = ScanSingleConversion;
  stcConfig.u32SamplingTimeSelect.AD_CH_0 = 1u;
  stcConfig.enSamplingTimeN0 = Value32;
  stcConfig.u8SamplingTime0 = 30u;
  stcConfig.enSamplingTimeN1 = Value32;
  stcConfig.u8SamplingTime1 = 30u;
  stcConfig.u8SamplingMultiplier = 4u;
  stcConfig.u8EnableTime = 10u;
  stcConfig.bScanTimerStartEnable = FALSE;
  stcConfig.u8ScanFifoDepth = 0u;
  stcConfig.bPrioExtTrigEnable = FALSE;
  stcConfig.bPrioExtTrigStartEnable = FALSE;    
  stcConfig.bPrioTimerStartEnable = FALSE;  	 
  stcConfig.enPrioConversionTimerTrigger = AdcNoTimer;
  stcConfig.u8PrioLevel1AnalogChSel = 0;    
  stcConfig.u8PrioLevel2AnalogChSel = 0;
  stcConfig.bComparisonEnable = FALSE;     	 
  stcConfig.u16ComparisonValue = 0; 	 
  stcConfig.bCompareAllChannels = FALSE;    	 
  stcConfig.u8ComapreChannel = 0;     	 
  stcConfig.bRangeComparisonEnable = FALSE; 	 
  stcConfig.u16UpperLimitRangeValue = 0;  
  stcConfig.u16LowerLimitRangeValue = 0;    
  stcConfig.u8RangeCountValue = 0;  	 
  stcConfig.bOutOfRange = FALSE;            	 
  stcConfig.bRangeCompareAllChannels = FALSE;    
  stcConfig.u8RangeComapreChannel = 0;
 
  if (Ok == Adc_Init(&ADC0, &stcConfig))    	// Init ADC0
  {
	pstcAdc = (stc_adcn_t*) &ADC0;
    
	Adc_EnableWaitReady(pstcAdc);           	// Enable ADC0 and wait for ready
	Adc_ScanSwTrigger(pstcAdc);             	// Trigger ADC0
    
	while (OperationInProgress == Adc_ScanStatus(pstcAdc))  	// Poll for conversion done
	{}
    
	u32Result = Adc_ReadScanFifo(pstcAdc);
  }
 
  return u32Result;
}

void DMA_HANDLER (void)
{
    if (dstc_state(0)){ //check interrupt status on channel 0
   	 if(tx_proc_buffer == (PONG)){
   		 dstc_src_memory (0,(uint32_t)&(dma_tx_buffer_pong));   	 //Soucrce address
   		 tx_proc_buffer = PING;
   	 }
   	 else {
   		 dstc_src_memory (0,(uint32_t)&(dma_tx_buffer_ping));   	 //Soucrce address
   		 tx_proc_buffer = PONG;
   	 }
   	 tx_buffer_empty = 1;   																			 //Signal to main() that tx buffer empty
   	 dstc_reset(0);   																						 //Clean the interrup flag
    }
    if (dstc_state(1)){ //check interrupt status on channel 1
   	 if(rx_proc_buffer == PONG){
   		 dstc_dest_memory (1,(uint32_t)&(dma_rx_buffer_pong));     //Destination address
   		 rx_proc_buffer = PING;
   	 }
   	 else {
   		 dstc_dest_memory (1,(uint32_t)&(dma_rx_buffer_ping));     //Destination address
   		 rx_proc_buffer = PONG;
   	 }
   	 rx_buffer_full = 1;
   	 dstc_reset(1);
    }
}

void proces_buffer(void)
{
    int i;
  uint32_t *txbuf, *rxbuf;
    static int32_t buff_count = 0;
    float32_t tmp;
    float32_t threshold = 0.1;
    int peakflag_start = 0;
    int peakflag_end = 0;
    uint16_t peakindex = 0;
    float32_t min_value = 0.1;
    
    // Determine which buffer to use
  if(tx_proc_buffer == PING) txbuf = dma_tx_buffer_ping;
  else txbuf = dma_tx_buffer_pong;
  if(rx_proc_buffer == PING) rxbuf = dma_rx_buffer_ping;
  else rxbuf = dma_rx_buffer_pong;


    // Copy the data to a float array and then filter
    for (i = 0; i < (DMA_BUFFER_SIZE) ; i++)
  {

   	 audio_IN = rxbuf[i];
   	 audio_chL = (audio_IN & 0x0000FFFF);
   	 audio_chR = ((audio_IN >>16)& 0x0000FFFF);
   	 
   	 // Copy the R channel
   	 xData[i] = (float32_t)audio_chR;
   	 input_data[i + buff_count*DMA_BUFFER_SIZE] = (float32_t)audio_chR;
   	 
  }
    //buff_count = (buff_count+1)%NUM_BUFFERS;
    buff_count = (buff_count+1);
    if(buff_count == NUM_BUFFERS)buff_count=0;
    
    
    // calculate the YIN
    if(buff_count == 0){
   	 
   	 // rt0
   	 for (i = 0; i < N_LAGS ; i++)
   	 {
   		 arm_power_f32(&input_data[i],W, &rt0[i]);
   	 }
    
   	 // rtau
   	 for (i = 0; i < N_LAGS ; i++)
   	 {
   		 arm_dot_prod_f32(input_data,&input_data[i],W, &rtau[i]);
   	 }

   	 // dtau
   	 for (i = 0; i < N_LAGS ; i++)
   	 {
   		 dtau[i] = rtau[0] + rt0[i] - 2.0 * rtau[i];
   	 }
   	 
   	 // dtauprime
   	 tmp = dtau[0];
   	 dtauprime[0] = 1;
   	 for (i = 1; i < N_LAGS ; i++)
   	 {
   		 tmp = tmp + dtau[i];
   		 dtauprime[i] = ((float32_t)i)*dtau[i]/tmp;
   	 }
   	 
   	 // Now find the lowest point below 0.1 for the first peak
   	 for (i = 0; i < N_LAGS ; i++)
   	 {    
   		 if(dtauprime[i]<threshold){
   			 peakflag_start = 1;
   			 if(!peakflag_end){
   				 // in first peak
   				 if(dtauprime[i]<min_value){
   					 min_value = dtauprime[i];
   					 peakindex = i;
   					 
   				 }
   			 }
   		 }
   		 else {
   			 if(peakflag_start){
   				 // already found a peak
   				 peakflag_end = 1;
   			 }
   		 }
   	 }
   	 
   	 // send peak value 2 bytes
   	 Mfs_Hl_Write(&MFS0, (uint8_t *)&peakindex, 2, TRUE, FALSE);
   	 
   	 storePeak = peakindex; //Store's peak in global variable
    }
    
    // Process the data here
    
    // Now copy the data to the output buffer
    for (i = 0; i < (DMA_BUFFER_SIZE) ; i++)
  {

   	 audio_chL = 0; // Zero the left channel
   	 audio_chR = (int16_t)xData[i]; // just copy the input to the output
   	 // audio_chR = (int16_t)yData[i]; // copy the processed data to the output
   	 audio_OUT = ((audio_chR<<16 & 0xFFFF0000)) + (audio_chL & 0x0000FFFF);
   	 txbuf[i] = audio_OUT;
  }
    
    // reset the flags
  tx_buffer_empty = 0;
  rx_buffer_full = 0;
}

int main (void) {
    volatile en_result_t rt;
    uint8_t rxdata[10];
    uint16_t numread;
    volatile int gpio_rt;
    int16_t count = 0; // number of bytes received
    volatile int16_t* rxint16; // use to get 2 bytes into an int16_t types
    volatile arm_status status;

    // Sets all GPIO string selection idicators to output
    gpio_set_mode(A0, Output); //Low E
    gpio_set_mode(A1, Output); //A
    gpio_set_mode(A2, Output); //D
    gpio_set_mode(A3, Output); //G
    gpio_set_mode(A4, Output); //B
    gpio_set_mode(A5, Output); //High e

    //Set all of the tune indicator lights to output.
    gpio_set_mode(P7C, Output); //On when note's too low
    gpio_set_mode(P15, Output); //On when note's in tune
    gpio_set_mode(PB3, Output); //On when note's too high
    
    //Sets string-changing button to output.
    gpio_set_mode(BUTTON, Input);
    
    rxint16 = (int16_t *)rxdata; // point to the same location
    
    // Initialize the UART channels
    uart_init_virtual(); // &MFS0
    uart_init(); // &MFS12
    
    // Initialize the LEDs and turn them off=HIGH
    init_LED();
    gpio_set(LED_R, HIGH);
    gpio_set(LED_G, HIGH);
    gpio_set(LED_B, HIGH);
    
    // Initialize the audio I/O
  audio_init ( hz8000, mic_in, dma, DMA_HANDLER);

    while(1){
   	 
   	 // Process data when buffers are ready to be processed
   	 if((rx_buffer_full && tx_buffer_empty)){
   		 proces_buffer();
   	 };
   	 
   	 //Keeps track of time to prevent switch bouncing and only allow "presses" at certain times.
   	 theCounter++;
   		 
   	 //Lots of 0s show up in peak reader - waits for a non-zero.
   	 if(!(storePeak == 0))
   	 {
   			 confirmPeak = storePeak;
   	 }

   	 if(theCounter % 75000 == 0) //Prevents switch bouncing for changing the string
   	 {
   		 if(gpio_get(BUTTON) == 0) //Is button being pressed
   		 {
   			  if(onNote == 1) //If highest string's selected, set selection to Low E
   			  {
   					  onNote = 6;
   			  }
   			  else //If highest string's not selectd, select next highest string
   			  {
   					  onNote--;
   			  }
   		 }
   	 }
   	 
   	 //What note's selected?
   	 if(onNote == 1)
   	 {
   			  //High e
   			  gpio_set(A0, LOW);
   			  gpio_set(A1, LOW);
   			  gpio_set(A2, LOW);
   			  gpio_set(A3, LOW);
   			  gpio_set(A4, LOW);
   			  gpio_set(A5, HIGH);
   		 
   			//Yin value for High e
   			inTuneYin = 25;
   	 }
   	 else if(onNote == 2)
   	 {
   			//B
   			gpio_set(A0, LOW);
   			  gpio_set(A1, LOW);
   			  gpio_set(A2, LOW);
   			  gpio_set(A3, LOW);
   			  gpio_set(A4, HIGH);
   			  gpio_set(A5, LOW);
   		 
   			//Yin value for B
   			inTuneYin = 33;
   	 }
   	 else if(onNote == 3)
   	 {
   			//G
   			gpio_set(A0, LOW);
   			  gpio_set(A1, LOW);
   			  gpio_set(A2, LOW);
   			  gpio_set(A3, HIGH);
   			  gpio_set(A4, LOW);
   			  gpio_set(A5, LOW);
   		 
   			//Yin value for High G
   			inTuneYin = 41;
   	 }
   	 else if(onNote == 4)
   	 {
   			//D
   			gpio_set(A0, LOW);
   			  gpio_set(A1, LOW);
   			  gpio_set(A2, HIGH);
   			  gpio_set(A3, LOW);
   			  gpio_set(A4, LOW);
   			  gpio_set(A5, LOW);
   		 
   			//Yin value for High D
   			inTuneYin = 54;
   	 }
   	 else if(onNote == 5)
   	 {
   			//A
   			gpio_set(A0, LOW);
   			  gpio_set(A1, HIGH);
   			  gpio_set(A2, LOW);
   			  gpio_set(A3, LOW);
   			  gpio_set(A4, LOW);
   			  gpio_set(A5, LOW);
   		 
   			//Yin for A
   			inTuneYin = 73;
   	 }
   	 else if(onNote == 6)
   	 {
   			//Low E
   			gpio_set(A0, HIGH);
   			  gpio_set(A1, LOW);
   			  gpio_set(A2, LOW);
   			  gpio_set(A3, LOW);
   			  gpio_set(A4, LOW);
   			  gpio_set(A5, LOW);
   		 
   			//Yin for High e
   			inTuneYin = 97;
   	 }
   	 
   	 //Is string in tune, too sharp, or too flat?
      if( confirmPeak  == inTuneYin )
   	 {
   		   //Indicates string is in tune:
   		   gpio_set(P7C, LOW); //Light for too flat of pitch
   		   gpio_set(P15, HIGH); //On when note's in tune
   		   gpio_set(PB3, LOW); //Light for too sharp of pitch
   	 }
   	 else if( confirmPeak < inTuneYin )
   	 {
   		 	//Indicates string is too Sharp:
   				 gpio_set(P7C, LOW); //Light for too flat of pitch
   				 gpio_set(P15, LOW); //On when note's in tune
   				 gpio_set(PB3, HIGH); //Light for too sharp of pitch
   	 }
   	 else if( confirmPeak > inTuneYin )
   	 {
   			   //Indicates string is too Flat:
   				 gpio_set(P7C, HIGH); //Light for too flat of pitch
   				 gpio_set(P15, LOW); //On when note's in tune
   				 gpio_set(PB3, LOW); //Light for too sharp of pitch
   	 }
   	 
   	 
   	 // Check for any input on the two UART channels, read only one character
   	 // Don't block when reading
   	 // This is the virtual COM port MFS0
   	 // Receive 2 bytes so we can receive a 16 bit int
   	 // use count to put the bytes in the right place.
   	 rt = Mfs_Hl_Read(&MFS0, &rxdata[count], &numread, 1, FALSE);
   	 if (rt == Ok){
   		 if (numread>0){
   			 count++;
   			 
   			 }
   		 }
   	 }
   	 
    } // end while loop

// end main()







