/**
 * Author: leert
 * Version: 2025.2.20
 */

#ifndef __LEERTHARDWARE_H
#define __LEERTHARDWARE_H

void pinMode(char c, uint16_t pin, GPIOMode_TypeDef mode);
uint8_t digitalRead(char c, uint8_t pinNum);
void digitalWrite(char c, uint8_t pinNum, uint8_t value);
int getKey(char c, uint8_t pinNum, uint8_t status);

void setInterruptGroup(uint8_t group);
void setEXTI(char c,uint8_t pinNum,EXTITrigger_TypeDef mode,uint8_t priority,uint8_t subPriority);

void timerInit(uint8_t timer, uint32_t period);
void setTimerInterrupt(uint8_t timer, uint32_t period, uint8_t priority, uint8_t subPriority);

void monitorFreq(char c, uint8_t pinNum, uint8_t channel);
uint32_t getFreq(char c, uint8_t pinNum, uint8_t channel);

void analogWrite(char c, uint8_t pinNum, uint16_t period, uint16_t high);
void analogModify(char c, uint8_t pinNum, uint16_t high);
void analogReadInit(char c, uint16_t pin, uint8_t isContinous);
uint16_t analogReadOnce(char c, uint8_t pinNum);
uint16_t analogRead(char c, uint8_t pinNum);
void measurePulse(char c, uint8_t pinNum, uint8_t priority, uint8_t subPriority);

void setDMA(uint32_t from, uint32_t to, uint8_t byteNum, uint16_t size, uint8_t priority);
void transmit(void);
#endif
