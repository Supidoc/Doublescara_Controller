/************************************************************
 * @file    display_helpers.c
 * @brief   Filedescription
 * @author  dg
 * @date    29 May 2026
 ************************************************************/


/********************
 *     Includes    *
 ********************/
#include <display/McuFontCour08Normal.h>
#include <display/McuFontDisplay.h>
#include <display/McuGDisplaySSD1306.h>

/************************************
 *     Private Macros / Defines    *
 ************************************/

/***************************
 *     Private Typedefs     *
 ***************************/

/*****************************************
 *     Private Function Declarations     *
 *****************************************/


/****************************
 *     Public Variables     *
 ****************************/

/*****************************
 *     Private Variables     *
 *****************************/

/*******************************************
 *     Public Function Implementations     *
 *******************************************/

void Show1Liner(const unsigned char* text0)
{
    /*! \todo Make sure things are reentrant! */
    PGFONT_Callbacks        font = McuFontCour08Normal_GetFont();
    McuFontDisplay_PixelDim x, y;
    McuFontDisplay_PixelDim charHeight, totalHeight;

    McuGDisplaySSD1306_Clear();
    McuFontDisplay_GetFontHeight(font, &charHeight, &totalHeight);

    x = McuGDisplaySSD1306_GetWidth() / 2 -
        McuFontDisplay_GetStringWidth((unsigned char*)text0, font, NULL) / 2;
    y = McuGDisplaySSD1306_GetHeight() / 2 - charHeight / 2; /* 1 line */
    McuFontDisplay_WriteString((unsigned char*)text0, McuGDisplaySSD1306_COLOR_BLUE, &x, &y, font);

    McuGDisplaySSD1306_DrawBox(0, 0, McuGDisplaySSD1306_GetWidth() - 1,
                               McuGDisplaySSD1306_GetHeight() - 1, 1,
                               McuGDisplaySSD1306_COLOR_BLUE);
    McuGDisplaySSD1306_DrawBox(2, 2, McuGDisplaySSD1306_GetWidth() - 1 - 4,
                               McuGDisplaySSD1306_GetHeight() - 1 - 4, 1,
                               McuGDisplaySSD1306_COLOR_BLUE);
    McuGDisplaySSD1306_UpdateFull();
}

void Show2Liner(const unsigned char *text0, const unsigned char *text1)
{
  /*! \todo Make sure things are reentrant! */
  PGFONT_Callbacks        font = McuFontCour08Normal_GetFont();
  McuFontDisplay_PixelDim x, y;
  McuFontDisplay_PixelDim charHeight, totalHeight;

  McuGDisplaySSD1306_Clear();
  McuFontDisplay_GetFontHeight(font, &charHeight, &totalHeight);

  x = McuGDisplaySSD1306_GetWidth() / 2 - McuFontDisplay_GetStringWidth((unsigned char *)text0, font, NULL) / 2;
  y = McuGDisplaySSD1306_GetHeight() / 2 - (1 * charHeight); /* 2 lines */
  McuFontDisplay_WriteString((unsigned char *)text0, McuGDisplaySSD1306_COLOR_BLUE, &x, &y, font);

  x = McuGDisplaySSD1306_GetWidth() / 2 - McuFontDisplay_GetStringWidth((unsigned char *)text1, font, NULL) / 2;
  y = McuGDisplaySSD1306_GetHeight() / 2 + ( 0.5*charHeight);
  McuFontDisplay_WriteString((unsigned char *)text1, McuGDisplaySSD1306_COLOR_BLUE, &x, &y, font);

  McuGDisplaySSD1306_DrawBox(0, 0, McuGDisplaySSD1306_GetWidth() - 1, McuGDisplaySSD1306_GetHeight() - 1, 1, McuGDisplaySSD1306_COLOR_BLUE);
  McuGDisplaySSD1306_DrawBox(2, 2, McuGDisplaySSD1306_GetWidth() - 1 - 4, McuGDisplaySSD1306_GetHeight() - 1 - 4, 1,
                             McuGDisplaySSD1306_COLOR_BLUE);
  McuGDisplaySSD1306_UpdateFull();
}

/********************************************
 *     Private Function Implementations     *
 ********************************************/
