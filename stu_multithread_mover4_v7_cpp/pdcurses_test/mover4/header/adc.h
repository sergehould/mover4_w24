 /*
 *	adc.h
 *
 *	Author				Date			Version
 *	Serge Hould			7 Feb 2022		1.0.0
 *	SH					6 Mar. 2023				renamed adc_read()
 *												Remove ifdef	__cplusplus because
 *												was not linking! Don't know why
 */

#ifndef ADC_H
#define ADC_H
//#ifdef	__cplusplus
//extern "C" {
//#endif

#define readADC	adc_read
/*Prototype Area*/
int adc_read(unsigned int);
//#ifdef	__cplusplus
//}
//#endif

#endif
