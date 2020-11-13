/**
  * @brief  ST7735 Registers
  */


#define  ST7735_NOP				0x00 /* No Operation: NOP */
#define  ST7735_SWRESET			0x01 /* Software reset: SWRESET */
#define  ST7735_RDDID			0x04 /* Read Display ID: RDDID */
#define  ST7735_RDDST			0x09 /* Read Display Statu: RDDST */
#define  ST7735_RDDPM			0x0A /* Read Display Power: RDDPM */
#define  ST7735_RDDMADCTL		0x0B /* Read Display: RDDMADCTL */
#define  ST7735_RDDCOLMOD		0x0C /* Read Display Pixel: RDDCOLMOD */
#define  ST7735_RDDIM			0x0D /* Read Display Image: RDDIM */
#define  ST7735_RDDSM			0x0E /* Read Display Signal: RDDSM */
#define  ST7735_SLPIN			0x10 /* Sleep in & booster off: SLPIN */
#define  ST7735_SLPOUT			0x11 /* Sleep out & booster on: SLPOUT */
#define  ST7735_PTLON			0x12 /* Partial mode on: PTLON */
#define  ST7735_NORON			0x13 /* Partial off (Normal): NORON */
#define  ST7735_INVOFF			0x20 /* Display inversion off: INVOFF */
#define  ST7735_INVON			0x21 /* Display inversion on: INVON */
#define  ST7735_GAMSET			0x26 /* Gamma curve select: GAMSET */
#define  ST7735_DISPOFF			0x28 /* Display off: DISPOFF */
#define  ST7735_DISPON			0x29 /* Display on: DISPON */
#define  ST7735_CASET			0x2A /* Column address set: CASET */
#define  ST7735_RASET			0x2B /* Row address set: RASET */
#define  ST7735_RAMWR			0x2C /* Memory write: RAMWR */
#define  ST7735_RGBSET			0x2D /* LUT for 4k,65k,262k color: RGBSET */
#define  ST7735_RAMRD			0x2E /* Memory read: RAMRD*/
#define  ST7735_PTLAR			0x30 /* Partial start/end address set: PTLAR */
#define  ST7735_TEOFF			0x34 /* Tearing effect line off: TEOFF */
#define  ST7735_TEON			0x35 /* Tearing effect mode set & on: TEON */
#define  ST7735_MADCTL			0x36 /* Memory data access control: MADCTL */
#define  ST7735_IDMOFF			0x38 /* Idle mode off: IDMOFF */
#define  ST7735_IMDON			0x39 /* Idle mode on: IDMON */
#define  ST7735_COLMOD			0x3A /* Interface pixel format: COLMOD */
#define  ST7735_FRMCTR1			0xB1 /* In normal mode (Full colors): FRMCTR1 */
#define  ST7735_FRMCTR2			0xB2 /* In Idle mode (8-colors): FRMCTR2 */
#define  ST7735_FRMCTR3			0xB3 /* In partial mode + Full colors: FRMCTR3 */
#define  ST7735_INVCTR			0xB4 /* Display inversion control: INVCTR */
#define  ST7735_PWCTR1			0xC0 /* Power control setting: PWCTR1 */
#define  ST7735_PWCTR2			0xC1 /* Power control setting: PWCTR2 */
#define  ST7735_PWCTR3			0xC2 /* In normal mode (Full colors): PWCTR3 */
#define  ST7735_PWCTR4			0xC3 /* In Idle mode (8-colors): PWCTR4 */
#define  ST7735_PWCTR5			0xC4 /* In partial mode + Full colors: PWCTR5 */
#define  ST7735_VMCTR1			0xC5 /* VCOM control 1: VMCTR1 */
#define  ST7735_VMOFCTR			0xC7 /* Set VCOM offset control: VMOFCTR */
#define  ST7735_WRID2			0xD1 /* Set LCM version code: WRID2 */
#define  ST7735_WRID3			0xD2 /* Customer Project code: WRID3 */
#define  ST7735_NVCTR1			0xD9 /* NVM control status: NVCTR1 */
#define  ST7735_RDID1			0xDA /* Read ID1: RDID1 */
#define  ST7735_RDID2			0xDB /* Read ID2: RDID2 */
#define  ST7735_RDID3			0xDC /* Read ID3: RDID3 */
#define  ST7735_NVCTR2			0xDE /* NVM Read Command: NVCTR2 */
#define  ST7735_NVCTR3			0xDF /* NVM Write Command: NVCTR3 */
#define  ST7735_GAMCTRP1		0xE0 /* Set Gamma adjustment (+ polarity): GAMCTRP1 */
#define  ST7735_GAMCTRN2		0xE1 /* Set Gamma adjustment (- polarity): GAMCTRN1 */

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04
