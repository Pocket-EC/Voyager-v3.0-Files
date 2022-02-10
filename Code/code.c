#include "LMP91000.h"

unsigned uart_rd;
unsigned cfgRes;
unsigned cnt;

// ----------------------------- Variables -------------------------------------
unsigned int adcRes, value;
unsigned char buffer[2];
unsigned cfg_rd[3];
char adc_txt[7];
char cfg_txt[4];


// ------------------------- 16 x 12-bit ADCs ----------------------------------
void Send_Adc_Data(unsigned input) {
   adcRes = ADC1_Get_Sample(input);

   buffer[0] = adcRes & 0xFF;                                                   //LSB
   buffer[1] = (adcRes >> 8) & 0xFF;                                            //MSB
   
   UART1_Write(buffer[0]);
   UART1_Write(buffer[1]);
}

// --------------------------- I2C Procedures ----------------------------------
void I2C_write_byte(unsigned short address, unsigned short value)
{
     unsigned short write = LMP91000_I2C_ADDRESS << 1;

     I2C1_Start();
     I2C1_Write(write);
     I2C1_Write(address);
     I2C1_Write(value);
     I2C1_Stop();

     delay_us(50);
}

unsigned char I2C_read_byte(unsigned short address)
{
     unsigned short write = LMP91000_I2C_ADDRESS << 1;
     unsigned short read = (LMP91000_I2C_ADDRESS << 1) | 0x01;
     unsigned short value = 0x00;
     
     I2C1_Start();
     I2C1_Write(write);
     I2C1_Write(address);
     I2C1_Start();
     I2C1_Write(read);
     value = I2C1_Read(1u);
     I2C1_Stop();

     return value;
}

// ------------------------------ LMP91000 -------------------------------------

unsigned short LMP91000_status() {
      return I2C_read_byte(LMP91000_STATUS_REG);
}

void LMP91000_lock(){
      I2C_write_byte(LMP91000_LOCK_REG, LMP91000_WRITE_LOCK);
}

void LMP91000_unlock(){
      I2C_write_byte(LMP91000_LOCK_REG, LMP91000_WRITE_UNLOCK);
}

unsigned short LMP91000_configure(unsigned short _tiacn, unsigned short _refcn, unsigned short _modecn){
      if(LMP91000_status() == LMP91000_READY){
            LMP91000_unlock();
            I2C_write_byte(LMP91000_TIACN_REG, _tiacn);
            I2C_write_byte(LMP91000_REFCN_REG, _refcn);
            I2C_write_byte(LMP91000_MODECN_REG, _modecn);
            LMP91000_lock();
            return 1;
      }
      return 0;
}

void LMP91000_values() {
   if(cfgRes == 0x01){

       UART1_Write_Text("STAT:");
       ByteToStr(I2C_read_byte(LMP91000_STATUS_REG),cfg_txt);
       UART1_Write_Text(cfg_txt);

       UART1_Write_Text("TIACN:");
       ByteToStr(I2C_read_byte(LMP91000_TIACN_REG),cfg_txt);
       UART1_Write_Text(cfg_txt);

       UART1_Write_Text("REFCN:");
       ByteToStr(I2C_read_byte(LMP91000_REFCN_REG),cfg_txt);
       UART1_Write_Text(cfg_txt);

       UART1_Write_Text("MODECN:");
       ByteToStr(I2C_read_byte(LMP91000_MODECN_REG),cfg_txt);
       UART1_Write_Text(cfg_txt);
       
   }
}

// ---------------------------- Initialisation ---------------------------------
void ADC_Init() {

    TRISB = 0xFFFF;
    PORTB = 0x0000;

    ADC1_Init_Advanced(_ADC_12bit, _ADC_INTERNAL_REF);
    Delay_ms(50);                                                               //ADC Initialisation Delay
}

void LMP_Init() {

    TRISGbits.TRISG6 = 0;                                                       // MENB Pin is Enabled
    PORTG.B6 = 0;
    
    I2C1_Init(100000);
    delay_ms(100);
}


void UART_Init() {

     TRISDbits.TRISD1 = 1;                                                      // UART1.RX: RD1 as Input
     TRISDbits.TRISD2 = 0;                                                      // UART1.TX: RD2 as Output

     Unlock_IOLOCK();                                                           // Unlock
     PPS_Mapping_NoLock(24, _INPUT, _U1RX);                                     // Set UART1 Rx be pin RP24, RD1
     PPS_Mapping_NoLock(23, _OUTPUT, _U1TX);                                    // Set UART1 Tx be pin RP23, RD2
     Lock_IOLOCK();                                                             // Lock

     UART1_Init_Advanced(115200,_UART_8BIT_NOPARITY,_UART_ONE_STOPBIT,_UART_HI_SPEED);
     UART_Set_Active(&UART1_Read, &UART1_Write, &UART1_Data_Ready, &UART1_Tx_Idle);
}

// ------------------------------- Main ----------------------------------------
void main() {

  CLKDIVbits.RCDIV = 0;                    // FRCPLL : 8MHz x 4PLL = 32MHz

  ADC_Init();                                                                   // ADC  Initialisation
  LMP_Init();                                                                   // I2C  Initialisation
  UART_Init();                                                                  // UART Initialisation
  
  cfgRes = LMP91000_configure(LMP91000_TIA_GAIN_350K | LMP91000_RLOAD_10OHM,
                              LMP91000_REF_SOURCE_EXT | LMP91000_INT_Z_20PCT | LMP91000_BIAS_SIGN_POS | LMP91000_BIAS_1PCT,
                              LMP91000_FET_SHORT_DISABLED | LMP91000_OP_MODE_TIA_ON);

  while (1) {

    if (UART1_Data_Ready())
    {
      uart_rd = UART1_Read();
      
      if(uart_rd == '0')                                                        // LMP91000 Reconfiguration
      {
         cnt = 0x00;
         cfgRes = 0x00;
         
         UART1_Write_Text("Send Three Bytes");
         UART1_Write(0x0D);   //go to beginning
         UART1_Write(0x0A);   //of new line
         
         while(cnt < 3) {
            if (UART1_Data_Ready()) {
                 cfg_rd[cnt] = UART1_Read();
                 cnt++;
                 
                 switch(cnt){
                    case 1:
                      UART1_Write_Text("TIACN OK");
                      break;
                    case 2:
                      UART1_Write_Text("REFCN OK");
                      break;
                    case 3:
                      UART1_Write_Text("MODECN OK");
                      break;
                 }

                 UART1_Write(0x0D);   //go to beginning
                 UART1_Write(0x0A);   //of new line
            }
         }
         
         cfgRes = LMP91000_configure(cfg_rd[0], cfg_rd[1], cfg_rd[2]);          // TIACN, REFCN, MODECN
                                  
      }
      
      if(uart_rd == '1') {                                                      // Read register values
         LMP91000_values();
      }

      if(uart_rd == '2')                                                        // Read one sample
      {
         Send_Adc_Data(2);
         //Delay_us(10);
      }
      
      if(uart_rd == '3')                                                        // Read one sample
      {
         Send_Adc_Data(3);
         //Delay_us(10);
      }
      
      
    }
  }
}