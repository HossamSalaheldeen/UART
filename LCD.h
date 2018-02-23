#ifndef LCD_H
#define LCD_H

#include <stdint.h>

typedef enum {LCD_OK =0, LCD_NOK}LCD_ChkType;
void LCD_Init(void);
LCD_ChkType LCD_DispChar(char Data);
LCD_ChkType LCD_SetCursPos(int x,int y);
LCD_ChkType LCD_Clear(void);
LCD_ChkType LCD_DispString(uint8_t* StrPtr);

#endif
