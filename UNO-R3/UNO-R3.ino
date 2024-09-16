
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// Copyright (c) 2024 Upside Down Labs - contact@upsidedownlabs.tech
// Author: Deepak Khatri
//
// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Arduino.h>

// Definitions
#define NUM_CHANNELS 6                                    // Number of channels supported
#define HEADER_LEN 3                                      // Header = SYNC_BYTE_1 + SYNC_BYTE_2 + Counter
#define PACKET_LEN (NUM_CHANNELS * 2 + HEADER_LEN + 1)    // Packet length = Header + Data + END_BYTE
#define SAMP_RATE 250.0                                   // Sampling rate (250/500 for UNO R4)
#define SYNC_BYTE_1 0xC7                                  // Packet first byte
#define SYNC_BYTE_2 0x7C                                  // Packet second byte
#define END_BYTE 0x01                                     // Packet last byte
#define CAL_SIG_DIVIDER 8                                 // Calibration signal divider value

// Global constants and variables
uint8_t PacketBuffer[PACKET_LEN];     // The transmission packet
uint8_t CurrentChannel;                    // Current channel being sampled
uint8_t PacketCounter = 0;	          // Counter for current packet
uint16_t ADCValue = 0;	              // ADC current value
bool STATUS = false;                  // STATUS bit

bool timerBegin(float sampling_rate) {
  cli();  // Disable global interrupts
  
  // Calculate the time interval in seconds from the sampling rate
  // Interval = 1 / sampleRateHz
  float intervalSeconds = 1.0 / sampling_rate;

  // Calculate OCR1A based on the interval
  // OCR1A = (16MHz / (Prescaler * Desired Time)) - 1
  // Prescaler options: 1, 8, 64, 256, 1024
  unsigned long ocrValue = (16000000 / (64 * sampling_rate)) - 1;

  // Configure Timer1 for CTC mode (Clear Timer on Compare Match)
  TCCR1A = 0;  // Clear control register A
  TCCR1B = 0;  // Clear control register B
  
  // Set CTC mode (WGM12 bit) and set the prescaler to 64
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);  // Prescaler = 64

  // Set the calculated value in OCR1A register
  OCR1A = ocrValue;

  sei();  // Enable global interrupts
}

// ISR for Timer1 Compare A match (called based on the sampling rate)
ISR(TIMER1_COMPA_vect) {
  //Read 6ch ADC inputs and store current values in PacketBuffer
  for(CurrentChannel=0;CurrentChannel<NUM_CHANNELS;CurrentChannel++){
    ADCValue = analogRead(CurrentChannel);
    delayMicroseconds(10);
    ADCValue = analogRead(CurrentChannel);
    ADCValue += analogRead(CurrentChannel);
    ADCValue = ADCValue/2;
    PacketBuffer[((2*CurrentChannel) + HEADER_LEN)] = highByte(ADCValue);      // Write High Byte
    PacketBuffer[((2*CurrentChannel) + HEADER_LEN + 1)] = lowByte(ADCValue);  // Write Low Byte
  }

  // Send Packet over serial
  Serial.write(PacketBuffer, PACKET_LEN);

  // Increment the packet counter
  PacketBuffer[2]++;

  // Status LED / Calibration signal
  if(PacketBuffer[2] % CAL_SIG_DIVIDER == 0) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

bool timerStart() {
  STATUS = true;
  digitalWrite(LED_BUILTIN,LOW);
  // Enable Timer1 Compare A interrupt
  return TIMSK1 |= (1 << OCIE1A);
}

bool timerStop() {
  STATUS = false;
  digitalWrite(LED_BUILTIN,LOW);
  // Disable Timer1 Compare A interrupt
  return TIMSK1 &= ~(1 << OCIE1A);;
}

void setup() {
  
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB
  }

  // Status LED 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  // Initialize PacketBuffer
  PacketBuffer[0]   = SYNC_BYTE_1;    //Sync 0
  PacketBuffer[1]   = SYNC_BYTE_2;    //Sync 1
  PacketBuffer[2]   = 0;              //Packet counter
  PacketBuffer[3]   = 0x02;           //CH1 High Byte
  PacketBuffer[4]   = 0x00;           //CH1 Low Byte
  PacketBuffer[5]   = 0x02;           //CH2 High Byte
  PacketBuffer[6]   = 0x00;           //CH2 Low Byte
  PacketBuffer[7]   = 0x02;           //CH3 High Byte
  PacketBuffer[8]   = 0x00;           //CH3 Low Byte
  PacketBuffer[9]   = 0x02;           //CH4 High Byte
  PacketBuffer[10]  = 0x00;           //CH4 Low Byte
  PacketBuffer[11]  = 0x02;           //CH5 High Byte
  PacketBuffer[12]  = 0x00;           //CH5 Low Byte
  PacketBuffer[13]  = 0x02;           //CH6 High Byte
  PacketBuffer[14]  = 0x00;           //CH6 Low Byte 
  PacketBuffer[15]  = END_BYTE;	      //End Byte

  timerBegin(SAMP_RATE);
}

void loop() {
  if(Serial.available()){
    // Read command
    String command = Serial.readString();
    command.trim();

    // Who are you?
    if(command == "WHORU") {
      Serial.println("UNO-R3");
    }

    // Start data acquisition
    if(command == "START"){
      STATUS = true;
      timerStart();
    } 

    // Stop data acquisition
    if(command == "STOP") {
      STATUS = false;
      timerStop();
    }

    // Get status
    if(command == "STATUS") {
      if(STATUS) {
        Serial.println("START");
      } else {
        Serial.println("STOP");
      }
    }
  }
}
