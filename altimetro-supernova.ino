/*###### ALTÍMETRO SUPERNOVA #######





*/

#include <Kalman.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//Definições de debug
#define DEBUG

//Definições de sensores

#define USANDO_BMP180
//#define	USANDO_IMU


//Definições default
#define PRESSAO_MAR 101500
#define EIXO_X 0
#define EIXO_Y 1
#define EIXO_Z 2

#define TAMANHO_MEDIA 10
#define SERVO_ABERTO 255


#define TEMPO_ATUALIZACAO 50 //em milisegundos
#define THRESHOLD_DESCIDA 5  //em metros


//Definições de input
#define PINO_BUZZER 3
#define PINO_BOTAO 5
#define PINO_LED_VERD 2
#define PINO_LED_VERM 6
#define PINO_SERVO 7
#define PINO_SD_CS 4

//definições de erros
#define ERRO_BMP 'b'
#define ERRO_MPU 'm'
#define ERRO_SD 's'

//definição de estados
#define ESTADO_GRAVANDO 'g'
#define ESTADO_FINALIZADO 'f'
#define ESTADO_RECUPERANDO 'r'
#define ESTADO_ESPERA 'e'

//Variáveis de bibliotecas
Adafruit_BMP085 bmp;
SdFile arquivo;
File arquivoLog;
Servo paraquedas;

char nomeBase[] = "dataLog";
char nomeConcat[10];

//Variáveis de timing
int     millisAtual		= 0;
int     millisUltimo	= 0;
int     atualizaMillis = 0;
int		millisLed		= 0;
int 	millisBuzzer	= 0;
int		millisBotao		= 0;
int 	millisGravacao	= 0;
int n = 0;
int IMU = 1;

//Variáveis de dados
int32_t pressaoAtual;
float   alturaAltual;
float   alturaInicial;
float   alturaMaxima =  0;
float   mediaAltura = 0;
float	vetorAltura[10];
float mediaAceleracao[3];
float mediaAngulacao[3];

//variaveis de kalman
uint32_t timer;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;
double KalAngleX;
double KalAngleY;
double KalAngleZ;

double gyroXangle;
double gyroYangle;
double gyroZangle;


#ifdef USANDO_IMU
float 	aceleracaoAtual[3]; //em [x,y,z]
float	angulacaoAtual[3];	//em [x,y,z]
float	vetorAceleracao[3][10];
float	vetorAngulacao[3][10];
#endif



char stringDados[50];

//variáveis de controle

bool    inicializado = false;
bool    terminou = false;
bool    rodando = false;
bool 	emVoo = false;
bool    apogeu = false;
bool	abriuParaquedas = false;
char    erro = false;
char	statusAtual;
char estado;
bool descendo = false;

//Arrays de som de erro;

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif
  //Faz o setup inicial dos sensores de movimento e altura assim
  //como as portas

#ifdef DEBUG
  Serial.println("Iniciando o altímetro");
#endif

  inicializa();

}

void inicializa() {

  //Inicializando as portas
  pinMode(PINO_BOTAO, INPUT);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_LED_VERD, OUTPUT);
  pinMode(PINO_LED_VERM, OUTPUT);
  
  //iniciando o servo
  paraquedas.attach(PINO_SERVO);

  erro = 0;

  //Inicializando o Altímetro
  if (!bmp.begin()) {
    erro = ERRO_BMP;
  }

  //iniciar o IMU

#ifdef USANDO_IMU
  if (IMU) {
	double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan(accZ/sqrt(accX * accX + accY * accY)) * RAD_TO_DEG;
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);
  KalmanZ.setAngle(yaw);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  timer = micros();
  
    erro = ERRO_MPU;
  }
#endif

  //inicializar o cartão SD
  if (!SD.begin(PINO_SD_CS)) {

    erro = ERRO_SD;

    return;
  }
  else if (!erro) {
    int n = 1;
    bool parar = false;


    while (!parar)
    {
      sprintf(nomeConcat, "dataLog%d", n);
      if (SD.exists(nomeConcat))
          n++;
          else
            parar = true;
      }

  arquivoLog = SD.open(nomeConcat, FILE_WRITE);
#ifdef DEBUG
    Serial.print("Salvando os dados no arquivo ");
    Serial.println(nomeConcat);
#endif

  }


if(!erro) {
#ifdef DEBUG
  Serial.println("Nenhum erro iniciando dispositivos, começando o loop do main");
#endif
  statusAtual = ESTADO_ESPERA;
}

else {
#ifdef DEBUG
  Serial.print("Altímetro com erro de inicialização código:");
  Serial.println(erro);
#endif
  statusAtual = erro;
  
  atualizaMillis = millis();
}

}

void loop() {

  //Recebendo o tempo atual de maneira a ter uma base de tempo
  //para uma taxa de atualização
  millisAtual = millis();
  
  

  if ((millisAtual - atualizaMillis) >= TEMPO_ATUALIZACAO) {

    //verifica se existem erros e mantém tentando inicializar
    if (erro)
      inicializa();

    //Se não existem erros no sistema relacionados a inicialização
    //dos dispositivos, fazer:

    if (!erro) {
		
		#ifdef DEBUG
		Serial.println("Rodando o loop de funções");
		#endif

      //Verifica os botões e trata o clique simples e o clique longo
      //como controle de início/fim da gravação.
      leBotoes();
	  
	  #ifdef DEBUG
	Serial.println("Li os botões");
	  #endif

      //Recebe os dados dos sensores e os deixa salvo em variáveis
      adquireDados();
		#ifdef DEBUG
		Serial.println("Adquiri os dados");
		#endif

      //Trata os dados, fazendo filtragens e ajustes.
      trataDados();
		#ifdef DEBUG
		Serial.println("Tratei os dados");
		#endif

      //Se a gravação estiver ligada, grava os dados.
      gravaDados();
		#ifdef DEBUG
		Serial.println("Gravei os dados");
		#endif

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
    notifica(statusAtual);

    atualizaMillis = millisAtual;
  }





}

void leBotoes() {
  bool estado;
  millisAtual = millis();
  estado = digitalRead(PINO_BOTAO);

  //Liga a gravação se em espera
  if (estado && (statusAtual == ESTADO_ESPERA)) {
    statusAtual = ESTADO_GRAVANDO;
#ifdef DEBUG
    Serial.println("Iniciando a gravação");
#endif

  }

  //Desliga a gravação se gravando e o botão for segurado por 2s
  if ((statusAtual == ESTADO_GRAVANDO) && (estado && millisBotao == 0) ){

  millisBotao = millis();

  }
  else if ((statusAtual == ESTADO_GRAVANDO) && !estado) {
  millisBotao = 0;
}

if (millisAtual - millisBotao >= 2000) {
#ifdef DEBUG
  Serial.println("Terminanado a gravação e voltando a espera");
#endif
    statusAtual = ESTADO_ESPERA;
  }


}

void adquireDados() {

  //todas as medidas são feitas aqui em sequeência de maneira que os valores
  //sejam temporalmente próximos
  pressaoAtual = bmp.readPressure();
  alturaAltual = bmp.readAltitude(PRESSAO_MAR);

#ifdef USANDO_IMU

double dt = (double)(micros() - timer)/1000000;

  timer = micros();
  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double yaw = atan(accZ/sqrt(accX * accX + accY * accY)) * RAD_TO_DEG;
  gyroXangle = gyroX / 131.0;
  gyroYangle = gyroY / 131.0;
  gyroZangle = gyroZ/131.0;
  
  // aceleracaoAtual[EIXO_X] = ;
  // aceleracaoAtual[EIXO_Y] = ;
  // aceleracaoAtual[EIXO_Z] = ;

  angulacaoAtual[EIXO_X] = KalmanX.getAngle(roll, gyroXangle, dt);
  angulacaoAtual[EIXO_Y] = KalmanY.getAngle(pitch, gyroYangle, dt);
  angulacaoAtual[EIXO_Z] = KalmanZ.getAngle(yaw, gyroZangle, dt);
#endif

}

void trataDados() {

  //o tratamento dos dados aqui é até o momento somente uma média rolante

  vetorAltura [n] = alturaAltual;

#ifdef USANDO_IMU
  vetorAceleracao [EIXO_X][n] = aceleracaoAtual[EIXO_X];
  vetorAceleracao [EIXO_Y][n] = aceleracaoAtual[EIXO_Y];
  vetorAceleracao [EIXO_Z][n] = aceleracaoAtual[EIXO_Z];

  vetorAngulacao [EIXO_X][n] = angulacaoAtual[EIXO_X];
  vetorAngulacao [EIXO_Y][n] = angulacaoAtual[EIXO_Y];
  vetorAngulacao [EIXO_Z][n] = angulacaoAtual[EIXO_Z];
#endif

  //otimizar isso aqui depois
  for (int i = 0; i < TAMANHO_MEDIA; i++) {

    if (i == 0) {

      mediaAltura = 0;
#ifdef USANDO_IMU
      mediaAceleracao[EIXO_X] = 0;
      mediaAceleracao[EIXO_Y] = 0;
      mediaAceleracao[EIXO_Z] = 0;

      mediaAngulacao[EIXO_X] = 0;
      mediaAngulacao[EIXO_Y] = 0;
      mediaAngulacao[EIXO_Z] = 0;
#endif

    }

    mediaAltura += vetorAltura[i];
#ifdef USANDO_IMU
    mediaAceleracao[EIXO_X] += vetorAceleracao[EIXO_X][i];
    mediaAceleracao[EIXO_Y] += vetorAceleracao[EIXO_Y][i];
    mediaAceleracao[EIXO_Z] += vetorAceleracao[EIXO_Z][i];

    mediaAngulacao[EIXO_X] += vetorAngulacao[EIXO_X][i];
    mediaAngulacao[EIXO_Y] += vetorAngulacao[EIXO_Y][i];
    mediaAngulacao[EIXO_Z] += vetorAngulacao[EIXO_Z][i];
#endif


  }

  //Variáveis finais prontas para serem salvas
  mediaAltura = mediaAltura / TAMANHO_MEDIA;

#ifdef USANDO_IMU
  mediaAceleracao[EIXO_X] = mediaAceleracao[EIXO_X] / TAMANHO_MEDIA;
  mediaAceleracao[EIXO_Y] = mediaAceleracao[EIXO_Y] / TAMANHO_MEDIA;
  mediaAceleracao[EIXO_Z] = mediaAceleracao[EIXO_Z] / TAMANHO_MEDIA;

  mediaAngulacao[EIXO_X] = mediaAngulacao[EIXO_X] / TAMANHO_MEDIA;
  mediaAngulacao[EIXO_Y] = mediaAngulacao[EIXO_Y] / TAMANHO_MEDIA;
  mediaAngulacao[EIXO_Z] = mediaAngulacao[EIXO_Z] / TAMANHO_MEDIA;
#endif



  n++;

  if (n >= TAMANHO_MEDIA)
    n = 0;



}

void gravaDados() {

  //verifica aqui o estado do foguete e também se o arquivo está aberto e pronto
  //para ser usado. Aqui, todos os dados são concatenados em uma string que dá
  //o formato das linhas do arquivo de log.

  if ((estado == ESTADO_GRAVANDO) && arquivoLog) {
    String stringDados = "";

    millisGravacao = millis();
    stringDados += String(millisGravacao);
    stringDados += ",";
    stringDados += String(mediaAltura);

#ifdef USANDO_IMU
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
#endif

    arquivoLog.println(stringDados);
    arquivoLog.close();
  }

}

void checaCondicoes() {

  //verificar a altura máxima
  if (mediaAltura > alturaMaxima)
    alturaMaxima =  mediaAltura;

  //Controle de descida, usando um threshold para evitar disparos não
  //intencionais
  if (mediaAltura + THRESHOLD_DESCIDA < alturaMaxima)
    descendo = true;

}

void finaliza() {
}

void recupera () {

  //verifica aqui se o foguete já atingiu o apogeu e se está descendo pelas
  //suas variáveis globais de controle e chama a função que faz o acionamento
  //do paraquedas
  if (descendo && !abriuParaquedas) {

    abreParaquedas();

  }


}

void notifica (char codigo) {
  unsigned int frequencia[10];
  //os tons aqui são tocados por um vetor que contem as frequências. Cada
  //slot do mesmo define um espaço de 100ms.
  
  #ifdef DEBUG
  Serial.print("Status atual do altímetro:");
  Serial.println(codigo);
#endif

  switch (codigo){

    //Problema com o BMP180
    //Ambos os leds piscando juntos + tom de erro
  case ERRO_BMP:
    
    frequencia[0] = 261;
      frequencia[1] = 261;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 220;
      frequencia[5] = 220;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 196;
      frequencia[9] = 196;
      if (millisAtual - millisLed	> 100) {
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERM));
		millisLed = millisAtual;
      }

      break;

      //Problema com o Gyro/Acelerômetro
      //Led1 (verde) piscando junto com o tom de erro
    case ERRO_MPU:
      frequencia[0] = 261;
      frequencia[1] = 261;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 220;
      frequencia[5] = 220;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 196;
      frequencia[9] = 196;
      if (millisAtual - millisLed	> 100){
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
		millisLed = millisAtual;
	  }

      break;

      //Problema com o SD
      //led vermelho piscando junto com o tom de erro
    case ERRO_SD:
    
      frequencia[0] = 261;
      frequencia[1] = 261;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 220;
      frequencia[5] = 220;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 196;
      frequencia[9] = 196;
      if (millisAtual - millisLed	> 100){
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERM));
		millisLed = millisAtual;
	  }

      break;

      //Estado onde o voo já terminou e faz um tom de recuperação
      //led verde pisca rápido também
    case ESTADO_RECUPERANDO:
      
      frequencia[0] = 4000;
      frequencia[1] = 4500;
      frequencia[2] = 4000;
      frequencia[3] = 0;
      frequencia[4] = 0;
      frequencia[5] = 0;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 0;
      frequencia[9] = 0;
      if (millisAtual - millisLed	> 100){
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
		millisLed = millisAtual;
	  }

      break;

      //Gravando, pisca um led vermelho como uma câmera e também faz
      //um tom simples.
    case ESTADO_GRAVANDO:
      
      frequencia[0] = 293;
      frequencia[1] = 293;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 0;
      frequencia[5] = 0;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 0;
      frequencia[9] = 0;
      if (millisAtual - millisLed	> 100){
        digitalWrite(PINO_LED_VERM, !digitalRead(PINO_LED_VERM));
		millisLed = millisAtual;
	  }

      break;

    default:
      //led verde piscando devagar indicando espera
      if (millisAtual - millisLed	> 500){
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
		millisLed = millisAtual;
	  }


      break;


  }

  //Lê o vetor de frequencias e toca a frequência na posição atual
  //voltando ao inicio do mesmo quando termina, assim tocando todos os tons

  if (codigo) {
    tone(PINO_BUZZER, frequencia[n], 100);
    n++;
    if (n > 9)
      n = 0;
  }
}


void abreParaquedas() {
#ifdef DEBUG
  Serial.println("Abrindo o paraquedas!");
#endif
  paraquedas.write(SERVO_ABERTO);
  abriuParaquedas = 1;

}









