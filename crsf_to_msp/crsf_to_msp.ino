/**
 * @file crsf_to_msp.ino
 * @author Frogmane + Cassandra "ZZ Cat" Robinson + Fabrizio Di Vittorio + yajo10
 * @brief Conversion of CRSF from RX to 
 * @version 1.0.2
 * @date 2024-3-19
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU General Public License v3.0
 * This example is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include "CRSFforArduino.hpp"
#include "MSP.h"


#define MSP_RX_PIN 16  
#define MSP_TX_PIN 17  

#define RX_UART_RX_PIN 23
#define RX_UART_TX_PIN 22

MSP msp;
CRSFforArduino *crsf = nullptr;

int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
    "A",
    "E",
    "T",
    "R",
    "Aux1",
    "Aux2",
    "Aux3",
    "Aux4",
    "Aux5",
    "Aux6",
    "Aux7",
    "Aux8",
    "Aux9",
    "Aux10",
    "Aux11",
    "Aux12"};

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    crsf = new CRSFforArduino(&Serial1, RX_UART_RX_PIN, RX_UART_TX_PIN);
    if (!crsf->begin())
    {
        crsf->end();

        delete crsf;
        crsf = nullptr;

        Serial.println("CRSF for Arduino initialisation failed!");
        while (1)
        {
            delay(10);
        }
    }
    Serial2.begin(115200, SERIAL_8N1, MSP_RX_PIN, MSP_TX_PIN);
    msp.begin(Serial2);
    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;
    crsf->setRcChannelsCallback(onReceiveRcChannels);
}

void loop()
{
    crsf->update();
}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (rcChannels->failsafe == false)
    {
      unsigned long thisTime = millis();
      static unsigned long lastTime = millis();

      if (thisTime < lastTime)
      {
          lastTime = thisTime;
      }

      if (thisTime - lastTime >= 100)
      {
        lastTime = thisTime;
        uint16_t rcValues[rcChannelCount];

        for (int i = 0; i < rcChannelCount; i++) {
            uint16_t us = crsf->rcToUs(crsf->getChannel(i + 1)); 
            us = constrain(us, 900, 2100); 
            rcValues[i] = us;
            //Serial.printf("CH %d: %d μs\n", i + 1, us);
        }

        // Cast the rcValues array to a uint8_t pointer
        uint8_t* rcPayload = (uint8_t*)rcValues;
        size_t payloadSize = rcChannelCount * sizeof(uint16_t);  // Each channel uses 2 bytes
        msp.command(MSP_SET_RAW_RC, rcPayload, payloadSize, false);


      }
    }
}
