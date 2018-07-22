#include "SPTPP.h"

SPTPP::SPTPP(uint8_t* obj, int size, unsigned long timeout)
{
    this->obj = obj;
    this->size = size;
    this->timeout = timeout;
    buffer = new uint8_t[size + 5];
    writeBuffer = new uint8_t[size + 5];
    pos = 0;
}

void SPTPP::GetedByte(uint8_t b)
{
    unsigned long act = millis();
    if (pos >= size + 5 || act - lastTime > timeout)
        pos = 0;
    lastTime = act;

    if (pos == 0)
    {
        if (b == 0xf0 || b == 0xaa)
        {
            buffer[0] = b;
            pos = 1;
        }
    }
    else if (buffer[0] == 0xaa)
    {
        buffer[pos++] = b;
        if (pos >= 5)
            RunRead();
    }
    else if (buffer[0] == 0xf0)
    {
        buffer[pos++] = b;

        if (pos == 2)
            actSize = b + 5;
        else if (pos >= actSize)
            RunWrite();
    }
}

int SPTPP::CheckFletcher16( uint8_t *data, int count )
{
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for(int index = 0; index < count; ++index )
    {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    return data[count] == (uint8_t)sum1 && data[count + 1] == (uint8_t)sum2;
}

uint8_t* SPTPP::Fletcher16( uint8_t *data, int count )
{
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;

    for(int index = 0; index < count; ++index )
    {
        sum1 = (sum1 + data[index]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    data[count] = (uint8_t)sum1;
    data[count + 1] = (uint8_t)sum2;
}

void SPTPP::RunRead()
{
    if (CheckFletcher16(buffer, 3))
    {
        int adr = buffer[1];
        int count = buffer[2];

        if (adr + count < size)
        {
            writeBuffer[0] = 0x55;
            writeBuffer[1] = buffer[2];

            for (int i = 0; i < count; ++i)
                writeBuffer[2 + i] = obj[adr + i];
            
            Fletcher16(writeBuffer, count + 2);
            Serial.write(writeBuffer, count + 4);
            Serial.flush();
        }
        else
            SendReadError();
        SetNewBuffer(5);
    }
    else
        CheckNextBuffPosition();
}

void SPTPP::RunWrite()
{
    if (CheckFletcher16(buffer, buffer[1] + 3))
    {
        int adr = buffer[1];
        int count = buffer[2];

        if (adr + count < size)
        {
            for (int i = 0; i < count; ++i)
                obj[adr + i] = buffer[3 + i];

            writeBuffer[0] = 0x0f;
            writeBuffer[1] = buffer[2];
            Fletcher16(writeBuffer, 2);

            Serial.write(writeBuffer, 4);
            Serial.flush();
        }
        else
            SendWriteError();

        SetNewBuffer(buffer[1] + 5);
    }
    else
        CheckNextBuffPosition();
}

void SPTPP::CheckNextBuffPosition()
{
    for (int i = 1; i < pos; ++i)
    {
        if (buffer[i] == 0xf0)
        {
            SetNewBuffer(i);
            if (pos > 1 && buffer[1] + 5 >= pos)
                RunWrite();
            return;
        }
        else if (i >= pos - 5 && buffer[i] == 0xaa)
        {
            SetNewBuffer(i);
            if (pos == 5)
                RunRead();
            return;
        }
    }
}

void SPTPP::SetNewBuffer(int p)
{
    for (int i = 0; i < pos - p; ++i)
        buffer[i] = buffer[p + i];
    pos -= p;
    
    if (pos < 0)
        pos = 0;
}

void SPTPP::SendReadError()
{
    writeBuffer[0] = 0x55;
    writeBuffer[1] = 0x00;
    writeBuffer[2] = 0x55;
    writeBuffer[3] = 0xAA;
    Serial.write(writeBuffer, 4);
    Serial.flush();
}

void SPTPP::SendWriteError()
{
    writeBuffer[0] = 0x0F;
    writeBuffer[1] = 0x00;
    writeBuffer[2] = 0x0F;
    writeBuffer[3] = 0x1E;
    Serial.write(writeBuffer, 4);
    Serial.flush();
}
