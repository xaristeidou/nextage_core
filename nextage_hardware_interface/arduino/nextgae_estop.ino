//
// Copyright (c) 2021, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

const byte pinEstop1  = 2;
const byte pinEstop2  = 3;
const byte pinReset1  = 4;
const byte pinReset2  = 5;
const byte pinPower  = 6;

int pingCount        = 0;
const int timeout    = 20;

void setup() {
  pinMode(pinEstop1, OUTPUT);
  pinMode(pinEstop2, OUTPUT);
  pinMode(pinReset1, OUTPUT);
  pinMode(pinReset2, OUTPUT);
  pinMode(pinPower, OUTPUT);
  // LOW = Relay ON, HIGH = Relay OFF
  digitalWrite(pinEstop1, LOW);
  digitalWrite(pinEstop2, LOW);
  digitalWrite(pinReset1, HIGH);
  digitalWrite(pinReset2, HIGH);
  digitalWrite(pinPower, HIGH);
  Serial.begin(9600);
}

void loop() {
  delay(100);
  if (pingCount < timeout) ++pingCount;
  
  while (Serial.available() > 0)
  {
    char ret;
    char val;
    Serial.readBytes(&ret, 1);
    switch (ret)
    {
      case '\n':
        // Ping
        Serial.println("OK");
        break;
      case 'S':
      case 's':
        // EStop
        Serial.readBytes(&val, 1);
        switch (val)
        {
          case '0':
            digitalWrite(pinEstop1, LOW);
            digitalWrite(pinEstop2, LOW);
            break;
          case '1':
            digitalWrite(pinEstop1, HIGH);
            digitalWrite(pinEstop2, HIGH);
            break;
          default:
            Serial.println("BAD");
        }
        Serial.readBytes(&ret, 1);
        break;
      case 'P':
      case 'p':
        // Power
        Serial.println("PWR");
        digitalWrite(pinPower, LOW);
        delay(500);
        digitalWrite(pinPower, HIGH);
        Serial.readBytes(&ret, 1);
        break;
      case 'R':
      case 'r':
        // Reset
        Serial.println("RST");
        digitalWrite(pinReset1, LOW);
        digitalWrite(pinReset2, LOW);
        delay(500);
        digitalWrite(pinReset1, HIGH);
        digitalWrite(pinReset2, HIGH);
        Serial.readBytes(&ret, 1);
        break;
      default:
        Serial.println("UNK");
        continue;
    }
    pingCount = 0;
  }

  if (pingCount >= timeout)
  {
    digitalWrite(pinEstop1, HIGH);
    digitalWrite(pinEstop2, HIGH);
  }
}
