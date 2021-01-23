


#include "main.h"
#include "stm32f0xx.h"
#include <stdbool.h>
#include "tsc_config.h"

void InitTscGpio(void);
void InitOutputGpio(void);
void InitTscModule(void);

bool Button1IsPressed(void);
bool Button2IsPressed(void);
bool Button3IsPressed(void);
bool Button4IsPressed(void);
