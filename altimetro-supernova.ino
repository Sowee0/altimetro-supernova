#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "Adafruit_BMP085.h"
#include "MPU6050.h"

//Definições de debug
//#define DEBUG
//#define DEBUG_TEMP

//Definições de sensores

#define USANDO_BMP180
#define USANDO_IMU


//Definições default
#define PRESSAO_MAR 101500
#define EIXO_X 0
#define EIXO_Y 1
#define EIXO_Z 2

#define TAMANHO_MEDIA 10
#define SERVO_ABERTO 40
#define SERVO_FECHADO 0


#define TEMPO_ATUALIZACAO 50 //em milisegundos
#define THRESHOLD_DESCIDA 4  //em metros


//Definições de input
#define PINO_BUZZER 3
#define PINO_BOTAO 2
#define PINO_LED_VERD 5
#define PINO_LED_VERM 6
#define PINO_RELE 7
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
File arquivoLog;
MPU6050 mpu;

char nomeBase[] = "dataLog";
char nomeConcat[12];

//Variáveis de timing
unsigned long millisAtual   = 0;
unsigned long   millisUltimo  = 0;
unsigned long atualizaMillis = 0;
unsigned long millisLed   = 0;
unsigned long millisBuzzer  = 0;
unsigned long millisBotao   = 0;
unsigned long millisGravacao  = 0;
int n = 0;
int m = 0;
int o =  0;
int IMU = 1;

//Variáveis de dados
//int32_t pressaoAtual;
float   alturaAtual;
float   alturaInicial;
float   alturaMaxima =  0;
float   mediaAltura = 0;
float mediaPressao =0;
float pressaoAtual;
float temperatura;
float mediaTemperatura;
float temperaturaAtual;




#ifdef USANDO_IMU
float   aceleracaoAtual[3]; //em [x,y,z]
float angulacaoAtual[3];  //em [x,y,z]
#endif



String stringDados;

//variáveis de controle


bool    inicializado = false;
bool    terminou = false;
bool    rodando = false;
bool  emVoo = false;
bool    apogeu = false;
bool  abriuParaquedas = false;
char    erro = false;
char  statusAtual;
bool estado;
bool descendo = false;
bool salvarInicial = false;
//Arrays de som de erro;

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif

#ifdef DEBUG_TEMP
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
  pinMode(PINO_RELE, OUTPUT);
  digitalWrite(PINO_RELE, LOW);
  erro = 0;

  //Inicializando o Altímetro
  if (!bmp.begin()) {
    erro = ERRO_BMP;
  }
  
  alturaInicial =  bmp.readAltitude(PRESSAO_MAR);

  //iniciar o IMU

#ifdef USANDO_IMU
  if(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
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
    #ifdef DEBUG_TEMP
    Serial.println("não deveria estar aqui com o sd ligado");
    #endif
      sprintf(nomeConcat, "log%d.txt", n);
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
  #ifdef DEBUG_TEMP
    Serial.print("Status atual:");
    Serial.println(statusAtual);
    Serial.print("estado atual de erro:");
    Serial.println(erro);
    #endif
    //verifica se existem erros e mantém tentando inicializar
    if (erro){
      inicializa();
    notifica(erro);
  }

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
  
  millisAtual = millis();
  estado = digitalRead(PINO_BOTAO);

  //Liga a gravação se em espera
  if (estado && (statusAtual == ESTADO_ESPERA)) {
    statusAtual = ESTADO_GRAVANDO;


}
}

void adquireDados() {

  //todas as medidas são feitas aqui em sequeência de maneira que os valores
  //sejam temporalmente próximos
  pressaoAtual = bmp.readPressure();
  alturaAtual = bmp.readAltitude(PRESSAO_MAR);
  temperaturaAtual = bmp.readTemperature();

#ifdef USANDO_IMU

  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();
  
  aceleracaoAtual[EIXO_X] = normAccel.XAxis;
  aceleracaoAtual[EIXO_Y] = normAccel.YAxis;
  aceleracaoAtual[EIXO_Z] = normAccel.ZAxis;

  angulacaoAtual[EIXO_X] = normGyro.XAxis;
  angulacaoAtual[EIXO_Y] = normGyro.YAxis;
  angulacaoAtual[EIXO_Z] = normGyro.ZAxis;
#endif

}

void trataDados() {
}

void gravaDados() {

  //verifica aqui o estado do foguete e também se o arquivo está aberto e pronto
  //para ser usado. Aqui, todos os dados são concatenados em uma string que dá
  //o formato das linhas do arquivo de log.
  if(!salvarInicial){
    alturaInicial = mediaAltura;
    salvarInicial = 1;
  }
  if ((statusAtual == ESTADO_GRAVANDO) || (statusAtual == ESTADO_RECUPERANDO)) {
    arquivoLog = SD.open(nomeConcat, FILE_WRITE);
  #ifdef DEBUG_TEMP
  Serial.println("Estou gravando!");
  digitalWrite(PINO_RELE, HIGH);
  #endif
  stringDados = "";
    millisGravacao = millis();
    stringDados += millisGravacao;
    stringDados += ",";
    stringDados += abriuParaquedas;
    stringDados += ",";
    stringDados += alturaAtual;
    stringDados += ",";
    stringDados += alturaMaxima;
    stringDados += ",";
    stringDados += pressaoAtual;
    stringDados += ",";
    stringDados += temperaturaAtual;    
#ifdef USANDO_IMU
    stringDados += ",";
    stringDados += aceleracaoAtual[EIXO_X];
    stringDados += ",";
    stringDados += aceleracaoAtual[EIXO_Y];
    stringDados += ",";
    stringDados += aceleracaoAtual[EIXO_Z];
    stringDados += ",";
    stringDados += angulacaoAtual[EIXO_X];
    stringDados += ",";
    stringDados += angulacaoAtual[EIXO_Y];
    stringDados += ",";
    stringDados += angulacaoAtual[EIXO_Z];
#endif
  
    arquivoLog.println(stringDados);
  arquivoLog.close();
  }

  
}

void checaCondicoes() {

  //verificar a altura máxima
  if ((alturaAtual > alturaMaxima)&&(statusAtual==ESTADO_GRAVANDO)   )
    alturaMaxima =  alturaAtual;

  //Controle de descida, usando um threshold para evitar disparos não
  //intencionais
  if ((alturaAtual + THRESHOLD_DESCIDA < alturaMaxima)&&(statusAtual==ESTADO_GRAVANDO)){
    descendo = true;
  statusAtual = ESTADO_RECUPERANDO;
  }

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
      if (millisAtual - millisLed > 100) {
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
      if (millisAtual - millisLed > 100){
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
      if (millisAtual - millisLed > 100){
        digitalWrite(PINO_LED_VERM, !digitalRead(PINO_LED_VERM));
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
      if (millisAtual - millisLed > 100){
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
      if (millisAtual - millisLed > 100){
        digitalWrite(PINO_LED_VERM, !digitalRead(PINO_LED_VERM));
    digitalWrite(PINO_LED_VERD, LOW);
    millisLed = millisAtual;
    }

      break;

    case ESTADO_ESPERA:
      //led verde piscando devagar indicando espera
      if (millisAtual - millisLed > 500){
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
    millisLed = millisAtual;
    }


      break;


  }

  //Lê o vetor de frequencias e toca a frequência na posição atual
  //voltando ao inicio do mesmo quando termina, assim tocando todos os tons

   if (codigo) {
    if(frequencia[o]&&(statusAtual != ESTADO_ESPERA)){
  tone(PINO_BUZZER, frequencia[o], TEMPO_ATUALIZACAO);
  }
    o++;
    if (o > 9)
      o = 0; 
   
  }
 
  
  #ifdef DEBUG_TEMP
    Serial.print("tocando a posição do vetor:");
  Serial.println(o);

#endif
}


void abreParaquedas() {
#ifdef DEBUG
  Serial.println("Abrindo o paraquedas!");
#endif
  digitalWrite(PINO_RELE, HIGH);
  abriuParaquedas = 1;

}
