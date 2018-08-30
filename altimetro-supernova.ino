/*





*/

#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>

//Definições default
#define PRESSAO_MAR 101500
#define EIXO_X 0
#define EIXO_Y 1
#define EIXO_Z 2


#define TEMPO_ATUALIZACAO 50 //em milisegundos
#define THRESHOLD_DESCIDA 5  //em metros


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
Servo paraquedas;
const int chipSelect = 4;
string nomeBase = "dataLog";
string nomeConcat;

//Variáveis de timing
int     millisAtual		= 0;
int     millisUltimo	= 0;
int		millisLed		= 0;
int 	millisBuzzer	= 0;
int		millisBotao		= 0;
int 	millisGravacao	= 0;

//Variáveis de dados
int32_t pressaoAtual;
float   alturaAltual;
float   alturaInicial;
float   alturaMaxima =  0;
float 	aceleracaoAtual[3]; //em [x,y,z]
float	angulacaoAtual[3];	//em [x,y,z]

float	vetorAltura[10];
float	vetorAceleracao[3][10];
float	vetorAngulacao[3][10];

string stringDados;

//variáveis de controle

bool    inicializado = false;
bool    terminou = false;
bool    rodando = false;
bool 	emVoo = false;
bool    apogeu = false;
char    erro = false;
char	statusAtual;



void setup() {
	
	//Faz o setup inicial dos sensores de movimento e altura assim
	//como as portas 
	inicializa();

}

void loop() {
	
	//Recebendo o tempo atual de maneira a ter uma base de tempo
	//para uma taxa de atualização
	millisAtual = millis();
	
	
	
	
    if((atualizaMillis - millisAtual) >= TEMPO_ATUALIZACAO){
		
	//Se não existem erros no sistema relacionados a inicialização
	//dos dispositivos, fazer:
	
	if(!erro){
		
	//Verifica os botões e trata o clique simples e o clique longo
	//como controle de início/fim da gravação.
    leBotoes();
	
	//Recebe os dados dos sensores e os deixa salvo em variáveis
    adquireDados();
	
	//Trata os dados, fazendo filtragens e ajustes.
	trataDados();
	
	//Se a gravação estiver ligada, grava os dados.
	gravaDados();
	
	//De acordo com os dados recebidos, verifica condições como a
	//altura máxima atingida e seta variáveis de controle de modo
	//que ações consequintes sejam tomadas.
    checaCondicoes();
	
	//Faz ajustes finais necessários
    finaliza();
	
	//Caso o voo tenha chegado ao ápice, libera o sistema de recuperação
    recupera();
	}
	
	//Notifica via LEDs e buzzer problemas com o foguete 
    notifica();
	
	atualizaMillis = millisAtual;
    }
	
	
	
    
    
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
  if (!SD.begin(chipSelect)) {
    
    erro = ERRO_SD;
	
    return;
  }
  else{
	 int n = 1;
	 bool parar = FALSE;
	  
	  
	  
	  while(!parar)
	  {
		  sprintf(nomeConcat,"dataLog%d",n);
		  if(SD.exists(nomeConcat)
			  n++;
		  else
			  parar = TRUE;  
	  }
	  
	  File arquivoLog = SD.open(nomeConcat, FILE_WRITE);
		  
  }
  
  

  
  }
  
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

void adquireDados(){
	
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

void trataDados(){
	
	//o tratamento dos dados aqui é até o momento somente uma média rolante
	
	vetorAltura [n] = alturaAltual;
	
	vetorAceleracao [EIXO_X][n] = aceleracaoAtual[EIXO_X];
	vetorAceleracao [EIXO_Y][n] = aceleracaoAtual[EIXO_Y];
	vetorAceleracao [EIXO_Z][n] = aceleracaoAtual[EIXO_Z];
	
	vetorAngulacao [EIXO_X][n] = angulacaoAtual[EIXO_X];
	vetorAngulacao [EIXO_Y][n] = angulacaoAtual[EIXO_Y];
	vetorAngulacao [EIXO_Z][n] = angulacaoAtual[EIXO_Z];
	
	//otimizar isso aqui depois
	for(int i = 0; i<10;i++){
		
		if(i = 0){
			
		mediaAltura = 0;
		
		mediaAceleracao[EIXO_X] = 0;
		mediaAceleracao[EIXO_Y] = 0;
		mediaAceleracao[EIXO_Z] = 0;
		
		mediaAngulacao[EIXO_X] = 0;
		mediaAngulacao[EIXO_Y] = 0; 
		mediaAngulacao[EIXO_Z] = 0;
			
		}
		
		mediaAltura += vetorAltura[i];
		
		mediaAceleracao[EIXO_X] += vetorAceleracao[EIXO_X][i];
		mediaAceleracao[EIXO_Y] += vetorAceleracao[EIXO_Y][i];
		mediaAceleracao[EIXO_Z] += vetorAceleracao[EIXO_Z][i];
		
		mediaAngulacao[EIXO_X] += vetorAngulacao[EIXO_X][i];
		mediaAngulacao[EIXO_Y] += vetorAngulacao[EIXO_Y][i];
		mediaAngulacao[EIXO_Z] += vetorAngulacao[EIXO_Z][i];
		
		
	}
	
	//Variáveis finais prontas para serem salvas
	mediaAltura = mediaAltura/10;
	
	mediaAceleracao[EIXO_X] = mediaAceleracao[EIXO_X] /10;
	mediaAceleracao[EIXO_Y] = mediaAceleracao[EIXO_Y] /10;
	mediaAceleracao[EIXO_Z] = mediaAceleracao[EIXO_Z] /10;
	
	mediaAngulacao[EIXO_X] = mediaAngulacao[EIXO_X]/10;
	mediaAngulacao[EIXO_Y] = mediaAngulacao[EIXO_Y]/10;
	mediaAngulacao[EIXO_Z] = mediaAngulacao[EIXO_Z]/10;
	
	
	
	n++;
	
	if(n>9)
		n=0;
	
	
	
}

void gravaDados(){
	
	if((estado == ESTADO_GRAVANDO) && arquivoLog){
	String stringDados = "";
	
		millisGravacao = millis();
		stringDados += String(millisGravacao);
		stringDados += ",";
		stringDados += String(mediaAltura);
		stringDados += ",";
		stringDados += String(mediaAceleracao[EIXO_X]);
		stringDados += ",";
		stringDados += String(mediaAceleracao[EIXO_Y]);
		stringDados += ",";
		stringDados += String(mediaAceleracao[EIXO_Z]);
		stringDados += ",";
		stringDados += String(mediaAngulacao[EIXO_X]);
		stringDados += ",";
		stringDados += String(mediaAngulacao[EIXO_Y]);
		stringDados += ",";
		stringDados += String(mediaAngulacao[EIXO_Z]);
		
	arquivoLog.println(dstringDados);
    arquivoLog.close();	
	}
	
}

void checaCondicoes(){
	
	//verificar a altura máxima 
	if(mediaAltura > alturaMaxima)
		alturaMaxima =  mediaAltura;
	
	//Controle de descida, usando um threshold para evitar disparos não
	//intencionais
	if(mediaAltura + THRESHOLD_DESCIDA < alturaMaxima)
		descendo = true;
	
	if(descendo)

}

void finaliza(){
}

void recupera (){
	
	//verifica aqui se o foguete já atingiu o apogeu e se está descendo pelas
	//suas variáveis globais de controle e chama a função que faz o acionamento
	//do paraquedas
	
	if(apogeu && descendo){
	
	abreParaquedas();
	
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

void abreParaquedas(){
	
	paraquedas.write(SERVO_ABERTO);
	
}





	
	
	

