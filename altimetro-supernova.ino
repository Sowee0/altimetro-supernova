#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>
#include <SysCall.h>
#include <Adafruit_BMP085.h>

//Definições default
#define PRESSAO_MAR 101500
#define EIXO_X 0
#define EIXO_Y 1
#define EIXO_Z 2


//Definições de input
#define PINO_BUZZER 4
#define PINO_BOTAO 5
#define PINO_LED1 3
#define PINO_LED2 6

//definições de erros
#define ERRO_BMP 'b'
#define ERRO_MPU 'm'
#define ERRO_SD 's'

//definição de estados
#define ESTADO_GRAVANDO 'g'
#define ESTADO_FINALIZADO 'f'
#define ESTADO_ESPERA 'e'

//Variáveis de bibliotecas
Adafruit_BMP085 bmp;
SdFat sd;
SdFile arquivo;

//Variáveis de timing
int     millisAtual		= 0;
int     millisUltimo	= 0;
int		millisLed		= 0;
int 	millisBuzzer	= 0;
int		millisBotao		= 0;

//Variáveis de dados
int32_t pressaoAtual;
float   alturaAltual;
float   alturaInicial;
float   alturaMaxima =  0;
float 	aceleracaoAtual[3]; //em [x,y,z]
float	angulacaoAtual[3];	//em [x,y,z]

//variáveis de controle

bool    inicializado = false;
bool    terminou = false;
bool    rodando = false;
bool    apogeu = false;
char    erro = false;
char	statusAtual;



void setup() {
  inicializa();

}

void loop() {

    //Sequência de verificação, só é executada caso não existam erros
	//estes erros são notificados de acordo

    if(erro == 0){
    leBotoes();
    adquireDados();
	trataDados
    checaCondições();
    gravaDados();
    finaliza();
    recupera();
    }
    notifica();
	
	delay(100);
    
    
}


void inicializa(){

  //Inicializando as portas 
  pinMode(PINO_BOTAO, INPUT);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_LED1, OUTPUT);
  pinMode(PINO_LED2, OUTPUT);

  while(erro == 1){

  erro = 1;
  
  //Inicializando o Altímetro
  if (!bmp.begin()) {
  erro = ERRO_BMP;
  }
  
  //iniciar o IMU

  
  //inicializar o cartão SD

  
  }
  
}

void notifica (char codigo){

  switch codigo{

	//Problema com o BMP180
    case ERRO_BMP:
	if(millisAtual - millisLed	> 100)
		digitalWrite(PINO_LED1, !digitalRead(PINO_LED1));

	break;
	
	//Problema com o Gyro/Acelerômetro 
	case ERRO_MPU:
	if(millisAtual - millisLed	> 100)
		digitalWrite(PINO_LED1, !digitalRead(PINO_LED1));

	break;
	
	//Problema com o SD
	case ERRO_SD:
	if(millisAtual - millisLed	> 100)
		digitalWrite(PINO_LED1, !digitalRead(PINO_LED1));

	break;
	
	case ESTADO_FINALIZADO:
	if(millisAtual - millisLed	> 100)
		digitalWrite(PINO_LED1, !digitalRead(PINO_LED1));

	break;
	
	case ESTADO_GRAVANDO:
	if(millisAtual - millisLed	> 100)
		digitalWrite(PINO_LED1, !digitalRead(PINO_LED1));

	break;
	
	default:
	//led piscando devagar indicando espera
	if(millisAtual - millisLed	> 500)
		digitalWrite(PINO_LED1, !digitalRead(PINO_LED1));
		
	
	break;
	
	
  }

}

void recupera (){
	
	//verifica aqui se o foguete já atingiu o apogeu e se está descendo pelas
	//suas variáveis globais de controle e chama a função que faz o acionamento
	//do paraquedas
	
	if(apogeu && descendo){
	
	abreParaquedas();
	
	}
}

void adquire (){
	
	//todas as medidas são feitas aqui em sequeência de maneira que os valores
	//sejam temporalmente próximos
	pressaoAtual = bmp.readPressure();
    alturaAltual = bmp.readAltitude(PRESSAO_MAR);
	
	aceleracaoAtual[EIXO_X] = ;
	aceleracaoAtual[EIXO_Y] = ;
	aceleracaoAtual[EIXO_Z] = ;
	
	angulacaoAtual[EIXO_X] = ;
	angulacaoAtual[EIXO_Y] = ;
	angulacaoAtual[EIXO_Z] = ;
	
	
}

void leBotoes(){
	bool estado;
	millisAtual = millis();
	estado digitalRead(PINO_BOTAO);
	
	//Liga a gravação se em espera
	if(estado && (statusAtual == ESTADO_ESPERA)){
		statusAtual = ESTADO_GRAVANDO;
		
	}
	
	//Desliga a gravação se gravando e o botão for segurado por 2s
	if((statusAtual == ESTADO_GRAVANDO) && estado &&(millisBotao == 0){
	
	millisBotao = millis();
		
	}
	else if ((statusAtual == ESTADO_GRAVANDO) && !estado){
	millisBotao = 0;
	}
	
	if(millisAtual - millisBotao >= 2000)
		statusAtual = ESTADO_ESPERA;
	
	
}
	
	
	
	
