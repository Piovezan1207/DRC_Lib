#ifndef DRC
#define DRC

#include <Arduino.h>
#include <Coordinates.h>
#include <MPU9250.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

class MAG
{
private:
    float Xmin = 1000000;
    float Ymin = 1000000;
    float Ymax = -1000000;
    float Xmax = -1000000;
    float Xmed, Ymed;
    int Defasagem = 0;
    float ang;
    float ang2;
    char dir;

    int NMag;
    unsigned long ValoresMag[];
    unsigned long SomatoriaMag = 0;

    byte btn;
    int status;

    void Calibra(MOV *mov);
    void Atualiza_Parametro_EEPROM();
    long Medir();
    void Calcular();
    long Filtro_Mag(long val);

public:
    MAG(int NMedias, byte BotaoGrava);
    int LER();
    void INICIAR(MOV *mov);
};

class MOV //Movimentos do robô
{
private:
public:
    MOV(byte IN1, byte IN2, byte IN3, byte IN4, byte EN1, byte EN2);
    void MOVER(float Modo); //F/T/EA/EF/DA/DF
    void MOTOR(bool NMotor, char Mooo);
    void INICIAR();
    void VELOCIDADE(byte V1, byte V2);
};

class DIST //Sensor ultrassonico
{
private:
public:
    DIST(byte TRIG1, byte ECHO1, byte TRIG2, byte ECHO2, byte TRIG3, byte ECHO3, int NMedias);
    float LER(byte Nsensor);
    void INICIAR();
};

#endif