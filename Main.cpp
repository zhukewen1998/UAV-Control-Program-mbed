#include "mbed.h"
// Author: Changgang Zheng
// E-mail: changgangzheng@std.uestc.edu.cn

Timer timerPA;
Timer timerPB;
Timer timerUA;
InterruptIn eventPA(p10);
InterruptIn eventPB(p11);
InterruptIn eventUA(p12);

InterruptIn eventIA(p13);

float   beginPA = 0.0f;
float   endPA = 0.0f;
float   beginPB = 0.0f;
float   endPB = 0.0f;

float   beginUA = 0.0f;
float   endUA = 0.0f;
float   RangeUA = 0.0f;   // Ranging date of Ultrasonic sensor A

int     NumRight = 0; // count times when it is in right rotate control            
int     FlagIA = 0;      // flag is to indicate whether infrared sensor get high level

float   PulseWidthPAin = 0.1f;
float   PulseWidthPBin = 0.9f;

float   PulseWidthLimit = 0.004f;  // it means true if rangeing date is less than 4ms, 
float   PulseWidthUA = 0.0f;

PwmOut  OutPwmA(p21);  // PWM output signal to control turning right or left
PwmOut  OutPwmB(p26);  // PWM output signal to control going forward or backward

Ticker  timerPutting;                           // interrupt time for changing PWM signal
float   timeTickerP = 0.1f;         //interrupt time timerOutput

float   OutPeriod = 0.015f;       // period of receiver's output signal 
float   DutyCycleStopPA = 0.1f;         // the value is to keep UAV not to turn right or left
float   DutyCycleLeftPA = 0.08f;        // the value is to let UAV turn rleft
float   DutyCycleRightCoarsePA = 0.120f;       // the value is to let UAV turn right
float   DutyCycleRightExactPA = 0.105f;       // the value is to let UAV turn right
float   DutyCycleStopPB = 0.9f;         // the value is to keep UAV not to go forward or backward
float   DutyCycleForwardPB = 0.89f;        // the value is to let UAV go forward
float   DutyCycleBackwardPB = 0.91f;        // the value is to let UAV go backward
float   PwmKeepStopPA = DutyCycleStopPA*OutPeriod;
float   PwmLeftRotate = DutyCycleLeftPA*OutPeriod;
float   PwmRightRotateCoarse = DutyCycleRightCoarsePA*OutPeriod;
float   PwmRightRotateExact = DutyCycleRightExactPA*OutPeriod;
float   PwmKeepStopPB = DutyCycleStopPB*OutPeriod;
float   PwmGoForward = DutyCycleForwardPB*OutPeriod;
float   PwmGoBackward = DutyCycleBackwardPB*OutPeriod;

int     RunMode = 0;        // run mode,1:control by UAV self,0:control by human
int     RunModeOld = 0;        // run mode,1:control by UAV self,0:control by human

int     FlagSearchExact = 0;      // flag is to indicate whether UAV is in searching condition
int     FlagTrack = 0;      // flag is to indicate whether UAV is in tracking condition
float   RangeTrackUA = 0.0f;   // range date of Ultrasonic Sensor A, be get in first into teacking condition 
float   RangeSafe = 0.15f;  // safe min Range
float   PulseWidthPAout = PwmKeepStopPA;
float   PulseWidthPBout = PwmKeepStopPB;

int     TimePAin = 0;         // Countting times for Control left rotate by human 
int     Ta = 100;       // the limit of interrupt' number for left rotate input,=10s/timeTickerP


PwmOut  TrigRange(p22);                     // PWM output signal to trig range
float   TrigRangePeriod = OutPeriod;    // period of range trig signal 
float   TrigRangePulseWidth = 0.0001f;    // Pulsewidth value is trig signal

float   PulseWidthPAoutOld = 0.0f;
float   PulseWidthPBoutOld = 0.0f;


int     CountSearchExact = 0;
int     MaxNumSE = 300;          // after MaxNumSE times interrupt, UAV quit exact search condition,30s/timeTickerP

int     NumNoRange = 0;            // number of counting times for no-range date
								   /************ test signal *******************/
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

DigitalOut TestOut(p30);

PwmOut TestPB(p23);  // PWM output signal B to test
PwmOut TestPC(p24);  // PWM output signal C to test
PwmOut TestPD(p25);  // PWM output signal D to test
					 // period of receiver's output test signal 
float TestDutyCyclePB = 0.07f;         // the DutyCycle value is test signal B
float TestDutyCyclePC = 0.5f;         // the DutyCycle value is test signal C
float TestDutyCyclePD = 0.95f;         // the DutyCycle value is test signal D
float TestPulseWidthPB = TestDutyCyclePB*OutPeriod;         // the DutyCycle value is test signal B
float TestPulseWidthPC = TestDutyCyclePC*OutPeriod;         // the DutyCycle value is test signal C
float TestPulseWidthPD = TestDutyCyclePD*OutPeriod;         // the DutyCycle value is test signal D
															/******************************************/


															/* interrupt */


void RiseTriggerPA() {
	timerPA.start();
	beginPA = timerPA.read();
	//    led1=1;
}

void FallTriggerPA() {
	endPA = timerPA.read();
	timerPA.stop();
	PulseWidthPAin = (endPA - beginPA);
	//    led2=1;
}

void RiseTriggerPB() {
	timerPB.start();
	beginPB = timerPB.read();
	//    led3=1;
}

void FallTriggerPB() {
	endPB = timerPB.read();
	timerPB.stop();
	PulseWidthPBin = (endPB - beginPB);
	//    led4=1;
}

void RiseTriggerUA() {
	timerUA.start();
	beginUA = timerUA.read();
}

void FallTriggerUA() {
	endUA = timerUA.read();
	timerUA.stop();
	PulseWidthUA = (endUA - beginUA);
	if (PulseWidthUA <= PulseWidthLimit) {
		RangeUA = PulseWidthUA*170.0;
	}
	else {
		PulseWidthUA = 0.0f;
		RangeUA = 0.0f;
	}
	//    led1=1;
}

void RiseTriggerIA() {
	FlagIA = 1;
}

void FallTriggerIA() {
	FlagIA = 0;
	//    led2=1;
}



void attimePutting() {
	//    led1 = 1;
	if (PulseWidthPAin >= PwmRightRotateCoarse) {  // is right rotate
		TimePAin = 0.0;     // set time of left rotate to zero for count again, if greater limit then put into UAV self control  
		if (NumRight == 50) {
			RunMode = 0;
		}
		else {
			NumRight = NumRight + 1;
		}
	}
	else {
		NumRight = 0;
	}
	if (RunMode == 1) {
		if (FlagTrack == 1) {
			if (RangeUA>0.005) {
				if (RangeUA>RangeTrackUA + 0.01f) {
					PulseWidthPAout = PwmKeepStopPA;     // set PWM_A to 10%
					PulseWidthPBout = PwmGoForward;     // set PWM_B to <90%
					led2 = 1;
				}
				else {
					if (RangeUA<RangeTrackUA - 0.01f) {
						PulseWidthPAout = PwmKeepStopPA; // set PWM_A to 10%
						PulseWidthPBout = PwmGoBackward;// set PWM_B to >90%
						led3 = 1;
					}
					else {
						PulseWidthPAout = PwmKeepStopPA; // set PWM_A to 10%
						PulseWidthPBout = PwmKeepStopPB; // set PWM_B to 90%
					}
				}
			}
			else {
				if (NumNoRange>30) {      //if it's no range date for 3s,than turn to Search 
					FlagTrack = 0;
					FlagSearchExact = 1;
					NumNoRange = 0;
					PulseWidthPAout = PwmRightRotateExact;
					PulseWidthPBout = PwmKeepStopPB;
				}
				else {
					NumNoRange = NumNoRange + 1;
					PulseWidthPAout = PwmKeepStopPA;
					PulseWidthPBout = PwmKeepStopPB;
				}
			}
		}
		else {
			if (FlagSearchExact == 1) {
				if (RangeUA>0.005) {
					FlagTrack = 1;
					FlagSearchExact = 0;
					RangeTrackUA = RangeUA - 0.025;
					if (RangeTrackUA<RangeSafe) {
						RangeTrackUA = RangeSafe;
					}
					PulseWidthPAout = PwmKeepStopPA; // set PWM_A to 10%
					PulseWidthPBout = PwmKeepStopPB; // set PWM_B to 90%
				}
				else {
					if (CountSearchExact >= MaxNumSE) {
						FlagSearchExact = 0;
						PulseWidthPAout = PwmRightRotateCoarse;
						PulseWidthPBout = PwmKeepStopPB;
					}
					else {
						CountSearchExact = CountSearchExact + 1;
						PulseWidthPAout = PwmRightRotateExact;
						PulseWidthPBout = PwmKeepStopPB;
					}
				}
			}
			else {
				if (FlagIA == 1) {
					if (RangeUA>0.005) {
						FlagTrack = 1;
						RangeTrackUA = RangeUA - 0.025;
						if (RangeTrackUA<RangeSafe) {
							RangeTrackUA = RangeSafe;
						}
						PulseWidthPAout = PwmKeepStopPA; // set PWM_A to 10%
						PulseWidthPBout = PwmKeepStopPB; // set PWM_B to 90%
					}
					else {
						FlagSearchExact = 1;
						PulseWidthPAout = PwmRightRotateExact;
						PulseWidthPBout = PwmKeepStopPB;
					}
				}
				else {
					PulseWidthPAout = PwmRightRotateCoarse;
					PulseWidthPBout = PwmKeepStopPB;
				}
			}
		}
	}
	else {
		if (PulseWidthPAin <= PwmLeftRotate) {  // is left rotate
			if (TimePAin >= Ta) {
				RunMode = 1;
				TimePAin = 0;
				FlagSearchExact = 0;
				FlagTrack = 0;
				PulseWidthPAout = PwmRightRotateCoarse;
				PulseWidthPBout = PwmKeepStopPB;
			}
			else {
				TimePAin = TimePAin + 1;
				PulseWidthPAout = PulseWidthPAin;
				PulseWidthPBout = PulseWidthPBin;
			}
		}
		else {
			TimePAin = 0;
			PulseWidthPAout = PulseWidthPAin;
			PulseWidthPBout = PulseWidthPBin;
		}
	}
}

int main() {

	TestPB.period(OutPeriod);
	TestPB.pulsewidth(TestPulseWidthPB);
	TestPC.pulsewidth(TestPulseWidthPC);
	TestPD.pulsewidth(TestPulseWidthPD);
	TrigRange.pulsewidth(TrigRangePulseWidth);

	RunMode = 0;

	timerPutting.attach(&attimePutting, timeTickerP);

	eventPA.rise(&RiseTriggerPA);
	eventPA.fall(&FallTriggerPA);
	eventPA.enable_irq();

	eventPB.rise(&RiseTriggerPB);
	eventPB.fall(&FallTriggerPB);
	eventPB.enable_irq();

	eventUA.rise(&RiseTriggerUA);
	eventUA.fall(&FallTriggerUA);
	eventUA.enable_irq();

	eventIA.rise(&RiseTriggerIA);
	eventIA.fall(&FallTriggerIA);
	eventIA.enable_irq();


	led1 = 0;
	led2 = 0;
	led3 = 0;
	led4 = 0;
	TestOut = 0;
	while (1) {
		if ((PulseWidthPAout != PulseWidthPAoutOld) or (PulseWidthPBout != PulseWidthPBoutOld)) {
			OutPwmA.period(OutPeriod);
			OutPwmA.pulsewidth(PulseWidthPAout);
			OutPwmB.pulsewidth(PulseWidthPBout);
			PulseWidthPAoutOld = PulseWidthPAout;
			PulseWidthPBoutOld = PulseWidthPBout;
			led4 = !led4;
		}

		wait(1);

		if (RangeUA>0.005f) {
			led1 = !led1;
		}
		led2 = 0;
		led3 = 0;
	}
}
