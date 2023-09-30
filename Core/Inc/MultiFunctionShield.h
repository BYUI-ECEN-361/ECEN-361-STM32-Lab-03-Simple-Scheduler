/* Function prototypes of routines in MultiFUnctionShield.c */

void MultiFunctionShield_WriteNumberToSegment(uint8_t digit);
void MultiFunctionShield_Single_Digit_Display (int digit, int8_t value);
void MultiFunctionShield_Display (int16_t value);
void MultiFunctionShield__ISRFunc(void);
void MultiFunctionShield_Clear(void);
void Display_Waiting(void);
void Display_All(void);
void OneSecond_Show_Potentiometer__ISRFunc(void);
void Clear_LEDs(void);
