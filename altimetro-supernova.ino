#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <SysCall.h>
#include <Adafruit_BMP085.h>

//Definições default
#define SEA_LVL_PRESS 101500
#define PINO_BUZZER 4
#define PINO_BOTAO 5

#define ERRO_BMP 'a'
#define ERRO_GYRO 'b'
#define ERRO_SD 'c'

//Variáveis de bibliotecas
Adafruit_BMP085 bmp;
SdFat sd;
SdFile arquivo;

//Variáveis de timing
int     millisAtual   = 0;
int     millisUltimo  = 0;

//Variáveis de dados
int32_t pressaoAtual;
float   alturaAltual;
float   alturaInicial;
float   alturaMaxima =  0;

//variáveis de controle

bool    inicializado = false;
bool    terminou = false;
bool    rodando = false;
bool    apogeu = false;
char    erro = false;



void setup() {
  inicializa();

}

void loop() {

    //Sequência de verificação

    if(erro == 0){
    leBotoes();
    adquire();
    checaCondições();
    gravaDados();
    doVars();
    finaliza();
    recupera();
    }
    notifica();
    
    
}


void inicializa(){

  //Botão
  pinMode(PINO_BOTAO, INPUT);

  //Buzzer
  pinMode(PINO_BUZZER, OUTPUT);

  //LED vermelho
  pinMode(PINO_LED1, OUTPUT);

  //LED verde
  pinMode(PINO_LED2, OUTPUT);

  
  //incializar o altímetro
  if (!bmp.begin()) {
  erro = ERRO_BMP;
  }
  //iniciar o IMU
  //inicializar o cartão SD

  //caso erro bipe/led
  
}

void notifica (char codigo){

  switch codigo{

    case 
  }

}


