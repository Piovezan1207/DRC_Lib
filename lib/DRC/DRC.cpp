#include <Arduino.h>
#include <Coordinates.h>
#include <MPU9250.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "DRC.h"

MPU9250 IMU(Wire, 0x68);
Coordinates Angulo = Coordinates();

MAG::MAG(int NMedias, byte BotaoGrava)
{
    NMag = NMedias;
    ValoresMag[NMedias];

    btn = BotaoGrava;
}

void MAG::INICIAR(MOV *mov)
{
    pinMode(btn, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    Serial1.begin(9600); // Módulo bluetooth

    while (!Serial)
        digitalWrite(13, !digitalRead(13));

    status = IMU.begin();
    if (status < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1)
        {
            digitalWrite(13, !digitalRead(13));
            delay(100);
        }
    }
    if (digitalRead(btn) == LOW)
    {
        digitalWrite(13, HIGH);
        delay(500);
        digitalWrite(13, LOW);
        Calibra(mov);
        delay(2000);
    }
    else
    {
        Atualiza_Parametro_EEPROM();
    }
}

void MAG::Calibra(MOV *mov)
{
    float X, Y;

    //Direita('F');//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>VIRAR PARA DIREITA

    for (int i = 0; i < 20000; i++) //Parametrizaçã o do sensor
    {
        IMU.readSensor();
        X = (IMU.getMagX_uT() * 100);
        Y = (IMU.getMagY_uT() * 100);
        //800
        // 1400 < 800
        if (Xmax < X)
        {
            Xmax = X; // 1400
        }
        if (Ymax < Y)
        {
            Ymax = Y;
        }

        if (Xmin > X)
        {
            Xmin = X;
        }
        if (Ymin > Y)
        {
            Ymin = Y;
        }

        delay(1);
    }

    //Para();//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PARAR
    Serial1.print("X / max / min ");
    Serial1.print(Xmax);
    Serial1.print(" / ");
    Serial1.print(Xmin);
    Serial1.print("  Y / max / min ");
    Serial1.print(Ymax);
    Serial1.print(" / ");
    Serial1.println(Ymin);

    long Xmin2, Ymin2, Xmax2, Ymax2;
    Xmin2 = Xmin;
    Ymin2 = Ymin;
    Xmax2 = Xmax;
    Ymax2 = Ymax;

    if (Xmin2 < 0)
    {
        Xmin2 *= -1;
        EEPROM.write(0, 1);
    }
    else
    {
        EEPROM.write(0, 0);
    }
    EEPROM.write(1, Xmin2 >> 8);
    EEPROM.write(2, Xmin2 & 0XFF);

    if (Ymin2 < 0)
    {
        Ymin2 *= -1;
        EEPROM.write(3, 1);
    }
    else
    {
        EEPROM.write(3, 0);
    }
    EEPROM.write(4, Ymin2 >> 8);
    EEPROM.write(5, Ymin2 & 0XFF);

    if (Ymax2 < 0)
    {
        Ymax2 *= -1;
        EEPROM.write(6, 1);
    }
    else
    {
        EEPROM.write(6, 0);
    }
    EEPROM.write(7, Ymax2 >> 8);
    EEPROM.write(8, Ymax2 & 0XFF);

    if (Xmax2 < 0)
    {
        Xmax2 *= -1;
        EEPROM.write(9, 1);
    }
    else
    {
        EEPROM.write(9, 0);
    }
    EEPROM.write(10, Xmax2 >> 8);
    EEPROM.write(11, Xmax2 & 0XFF);
}

void MAG::Atualiza_Parametro_EEPROM()
{
    Xmin = (EEPROM.read(1) << 8) + EEPROM.read(2);
    if (EEPROM.read(0) == 1)
        Xmin *= -1;

    Ymin = (EEPROM.read(4) << 8) + EEPROM.read(5);
    if (EEPROM.read(3) == 1)
        Ymin *= -1;

    Ymax = (EEPROM.read(7) << 8) + EEPROM.read(8);
    if (EEPROM.read(6) == 1)
        Ymax *= -1;

    Xmax = (EEPROM.read(10) << 8) + EEPROM.read(11);
    if (EEPROM.read(9) == 1)
        Xmax *= -1;

    Serial1.print("X / max / min ");
    Serial1.print(Xmax);
    Serial1.print(" / ");
    Serial1.print(Xmin);
    Serial1.print("  Y / max / min ");
    Serial1.print(Ymax);
    Serial1.print(" / ");
    Serial1.println(Ymin);
}
long MAG::Medir()
{ //for(int i = N; i > 0; i--)
    //   {
    IMU.readSensor();

    Xmed = map((IMU.getMagX_uT() * 100), Xmin, Xmax, -1000, 1000);
    Ymed = map((IMU.getMagY_uT() * 100), Ymin, Ymax, -1000, 1000);
    //Serial.print("X = "); Serial.print(Xmed); Serial.print(" Y = "); Serial.println(Ymed);
    Calcular();
    //}
    long Dist_Filtrada = Filtro_Mag(ang);
    // Serial1.print(dir);Serial1.print(" ");
    //  Serial1.println(Dist_Filtrada);
    return (Dist_Filtrada);
}
void MAG::Calcular()
{
    Angulo.fromCartesian(Ymed, Xmed);
    ang = Angulo.getAngle(); //entr 0 e 6,28 -  628
    ang *= 100;
    ang2 = ang + Defasagem;
    ang += Defasagem;
    if (ang > 628)
        ang -= 628;
    if (ang < 0)
        ang += 628;
    if (ang <= 314)
    {
        ang = map(ang, 0, 314, 0, 180);
        dir = '+';
    } //Mapeamento do valor resultando do calculo, para o angulo de 0 a 360
    if (ang > 314)
    {
        ang = map(ang, 315, 628, 180, 0);
        dir = '-';
    } //Mapeamento do valor resultando do calculo, para o angulo de 0 a 360
}
long MAG::Filtro_Mag(long val)
{
    int i;
    SomatoriaMag = 0;

    for (i = NMag; i > 0; i--)
        ValoresMag[i] = ValoresMag[i - 1];

    ValoresMag[0] = val;

    for (i = 0; i < NMag; i++)
        SomatoriaMag = SomatoriaMag + ValoresMag[i];

    return (SomatoriaMag / NMag);
}