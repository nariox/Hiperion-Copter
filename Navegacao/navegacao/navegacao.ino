/*
===============================================================================

 Name        : navegacao.ino
 Author      : Bruno Pinho
 Revision    : Danilo Luvizotto e Pedro Nariyoshi
 Version     : 1.0
 Description : Navigation of Hiperion - Quadcopter

===============================================================================
*/

#include "Ultrasonic.h"
#include "TinyGPS.h"

//Define a pinagem
#define echoPin 52 //Pino 52 recebe o pulso do echo SENSOR DISTANCIA
#define trigPin 50 //Pino 50 envia o pulso para gerar o echo SENSOR DISTANCIA
//Constantes
#define CENTRADO 127 // Os comandos variam de 0 a 255 - 127 e' o centro
#define ANGULOGPS 5 // Valor do angulo usado no GPS
#define YAWPADRAO 30 // Velocidade padrao de rotacao
#define MARGEMGPS 10 // Margem de erro do GPS (0,0001 = 10m aproximadamente no equador)
#define MAXDIST 300 // Distancia maxima do sensor de distancia (em cm)
#define T_AMOSTRAGEM 100
//Modos de navegacao
#define DESLIGAR 0
#define POUSAR 1
#define MANUAL 2
#define LIVRE 3
#define GPS 4
//Controle de altura
#define altura_throttle_max 100
#define altura_throttle_min 1
#define altura_passo_max 10
#define ALTURA_POUSO 10
#define ki 1

Ultrasonic ultrasonic(50,52); //iniciando a função e passando os pinos SENSOR DISTANCIA
TinyGPS gps;
bool feedgps();

//Variaveis
int altura; //SENSOR DISTANCIA se altura = 0, sensor nao encontrou distancia.
int roll=0, pitch=0, yaw=0, throttle=0; //Dados para a camada de controle
int MODO = DESLIGAR; // Modo de navegacao
long lat, lon, alt; //Latitude, Longitude e altura(cm) (GPS)
unsigned long age, date, time, chars; // date(ddmmyy), time(hhmmsscc) GPS
unsigned short sentences, failed; // Informacoes GPS
long latdestino, londestino, latresultante, lonresultante, modlat, modlon, distgps = 0, distantgps; //Usado para navegacao GPS
bool gps_disponivel;
float angmag=500; // Angulo do magnetometro, valor inicial 500 que significa que o magnetometro nao esta conectado
float rollcontrol=0, pitchcontrol=0, yawcontrol=0, throttlecontrol=0; //Dados do controle , tambem usado como variavel auxiliar no gps
long ultima_execucao; // Tempo de execucao
//Objeto controlador proporcional, integral e derivativo.
int altura_alvo;
int altura_saida;
int altura_erro_acumulado = 0;
//Bluetooth
int Conectado = 0;

void manda_dados(int roll, int pitch, int yaw, int throttle) {
      roll =+ CENTRADO;
      pitch =+ CENTRADO;
      yaw =+ CENTRADO;
      Serial1.print("R");
      if(roll < 10)
        Serial1.print("0");
      if(roll < 100)
        Serial1.print("0");
      Serial1.print(roll);
      Serial1.print("P");
      if(pitch < 10)
        Serial1.print("0");
      if(pitch < 100)
        Serial1.print("");
      Serial1.print(pitch);
      Serial1.print("Y");
      if(yaw < 10)
        Serial1.print("0");
      if(yaw < 100)
        Serial1.print("0");
      Serial1.print(yaw);
      Serial1.print("T");
      if(throttle < 10)
        Serial1.print("0");
      if(throttle < 100)
        Serial1.print("0");
      Serial1.print(throttle);
}

void setup() {
    Serial.begin(115200); // Para debug
    Serial1.begin(115200); // Para a camada de Controle
    Serial2.begin(115200); // Para bluetooth
    Serial3.begin(57600); // Para GPS
    pinMode(echoPin, INPUT); // define o pino 52 como entrada (recebe) SENSOR DISTANCIA
    pinMode(trigPin, OUTPUT); // define o pino 48 como saida (envia) SENSOR DISTANCIA
    pinMode(13, OUTPUT); // O pino 13 é um led, que será usado para indicar erro.
    delay(3000);  // Espera 3 segundos para garantir que os periféricos e componentes externos estão prontos.
    ultima_execucao = millis();
}

bool le_gps() {
    if (feedgps()) {
        gps.get_position(&lat, &lon, &age);
        feedgps();
        gps.get_datetime(&date, &time, &age);
        feedgps();
        gps.stats(&chars, &sentences, &failed);
        alt = gps.altitude();
        return true;
    }
    return false;
}

void sinaliza_erro(char error_code) {
    switch (error_code) {
        case 0: // O loop demorou mais que T_AMOSTRAGEM para ser executado.
            while(1) {
                digitalWrite(13, HIGH);   // liga o led
                delay(100);               // espera
                digitalWrite(13, LOW);    // desliga o led
                delay(400);               // espera
            }
            break;
        default:
            break;
    }
}

int cont_altura() { // Controle de altura
  if(altura == 0) // usando o sensor de distancia
    return altura_erro_acumulado;
    
  int erro = altura_alvo - altura;
  altura_erro_acumulado += (ki * erro);
  if(altura_erro_acumulado > altura_throttle_max)
      altura_erro_acumulado = altura_throttle_max;
  else if(altura_erro_acumulado < altura_throttle_min)
      altura_erro_acumulado = altura_throttle_min;
  return altura_erro_acumulado;
}

void estavel() {   // Mantem o multirrotor estavel
  //Calcula o Roll e pitch caso seja necessario para ajudar na estabilizacao
  roll = 0;
  pitch = 0;
  yaw = 0;
  manda_dados(roll, pitch, yaw, cont_altura());
}

void bluetooth() {   // Tratamento dos dados do bluetooth
  int inByte;
  if(Conectado == 0 && Serial2.available()) {
    inByte = Serial2.read();    
    if (inByte = 'R') {
      inByte = Serial2.read();
      if (inByte = 'F') {
        inByte = Serial2.read();
        if (inByte = 'C') {
          inByte = Serial2.read();
          if (inByte = 'O') {
            inByte = Serial2.read();
            if (inByte = 'M') {
              inByte = Serial2.read();
              if (inByte = 'M') {
                Conectado = 1;
                delay(50);
                Serial2.flush();
              }
            }
          }
        }
      }
    }   
  }

  if(Conectado == 1) {
    for(int i = 1; i <= 8; i++) {
      inByte = Serial2.read();
      switch(inByte) {
        case 'M': //Modo sendo recebido
          inByte = Serial2.read();
          if(inByte == 'N') {
            inByte = Serial2.read();    
            if (inByte = 'O') {
              inByte = Serial2.read();
              if (inByte = ' ') {
                inByte = Serial2.read();
                if (inByte = 'C') {
                  inByte = Serial2.read();
                  if (inByte = 'A') {
                    inByte = Serial2.read();
                    if (inByte = 'R') {
                      Conectado = 0;
                      delay(50);
                      Serial2.flush();
                    }
                  }
                }
              }
            }
          }
          else
            MODO = inByte;          
          break;
        
        case 'R': //Roll sendo recebido
          inByte = Serial2.read();
          roll = (Serial2.read() - '0') * 100;
          roll =+ (Serial2.read() - '0') * 10;
          roll =+ (Serial2.read() - '0');// * 1;
          roll =- 127;
          break;

        case 'P': //Pitch sendo recebido
          pitch = (Serial2.read() - '0') * 100;
          pitch =+ (Serial2.read() - '0') * 10;
          pitch =+ (Serial2.read() - '0');// * 1;
          pitch =- 127;
          break;

        case 'Y': //Yaw sendo recebido
          yaw = (Serial2.read() - '0') * 100;
          yaw =+ (Serial2.read() - '0') * 10;
          yaw =+ (Serial2.read() - '0');// * 1;
          yaw =- 127;
          break;

        case 'T': //Throttle sendo recebido
          throttle = (Serial2.read() - '0') * 100;
          throttle =+ (Serial2.read() - '0') * 10;
          throttle =+ (Serial2.read() - '0');// * 1;
          break;
        
        case 'A': //Altura sendo recebida
          altura_alvo = (Serial2.read() - '0') * 1000;
          altura_alvo =+ (Serial2.read() - '0') * 100;
          altura_alvo =+ (Serial2.read() - '0') * 10;
          break;

        case 'K': //latitude sendo recebida
          inByte = Serial2.read();
          latdestino = (Serial2.read() - '0') * 1000000;
          latdestino =+ (Serial2.read() - '0') * 100000;
          latdestino =+ (Serial2.read() - '0') * 10000;
          latdestino =+ (Serial2.read() - '0') * 1000;
          latdestino =+ (Serial2.read() - '0') * 100;
          latdestino =+ (Serial2.read() - '0') * 10;
          latdestino =+ (Serial2.read() - '0');// * 1;
          if(inByte == '-')
            latdestino = 0 - latdestino;
          break;

        case 'L': //longitude sendo recebida
          inByte = Serial2.read();
          londestino = (Serial2.read() - '0') * 10000000;
          londestino =+ (Serial2.read() - '0') * 1000000;
          londestino =+ (Serial2.read() - '0') * 100000;
          londestino =+ (Serial2.read() - '0') * 10000;
          londestino =+ (Serial2.read() - '0') * 1000;
          londestino =+ (Serial2.read() - '0') * 100;
          londestino =+ (Serial2.read() - '0') * 10;
          londestino =+ (Serial2.read() - '0');// * 1;
          if(inByte == '-')
            londestino = 0 - londestino;
          break;
          
        case 'N': // Caso a conexao tenha sido perdida
          inByte = Serial2.read();    
          if (inByte = 'O') {
            inByte = Serial2.read();
            if (inByte = ' ') {
              inByte = Serial2.read();
              if (inByte = 'C') {
                inByte = Serial2.read();
                if (inByte = 'A') {
                  inByte = Serial2.read();
                  if (inByte = 'R') {
                    Conectado = 0;
                    delay(50);
                    Serial2.flush();
                  }
                }
              }
            }
          }   
          break;
          
        case 'O': // Caso a conexao tenha sido perdida
          inByte = Serial2.read();
          if (inByte = ' ') {
            inByte = Serial2.read();
            if (inByte = 'C') {
              inByte = Serial2.read();
              if (inByte = 'A') {
                inByte = Serial2.read();
                if (inByte = 'R') {
                  Conectado = 0;
                  delay(50);
                  Serial2.flush();
                }
              }
            }
          }
          break;

        default:
          break;
      }
    }
    Serial2.flush();
  }
}

void enviabluetooth() { // Envia dados de telemetria pelo bluetooth
  if(Conectado == 1 && gps_disponivel) {
    Serial2.print("LAT");
    Serial2.print(lat);
    Serial2.print("LON");
    Serial2.print(lon);
    Serial2.print("ALT");
    Serial2.print(alt);
  }
}

void loop() {
  if(millis() - ultima_execucao > T_AMOSTRAGEM ) {
      Serial.println("ERRO! A execuçao demorou mais que T_AMOSTRAGEM!");
      manda_dados(0, 0, 0, 0);   //Desliga os motores
      sinaliza_erro(0);
  }
   //Espera o tempo de amostragem
  while(millis() - ultima_execucao < T_AMOSTRAGEM);
  
  // Tratar dados do Controle remoto
  //bluetooth(); // Pegar dados pelo bluetooth
  //if(!Conectado) //Caso o controle bluetooth esteja desconectado ele entra no modo POUSAR
    //MODO = POUSAR;
  // Obter dados dos sensores
  altura = ultrasonic.Distancia(trigPin);   //Calcula a altura em centimetros atraves do sensor de distância
  gps_disponivel = le_gps();               //Lê os dados do GPS
  angmag = 500;           // Lê os dados do magnetometro (500 significa sem magnetômetro)
  if(MODO > GPS) // Caso tenha entrado em algum modo inexistente - GPS  o ultimo modo
    MODO = POUSAR;
  
  enviabluetooth(); // Envia dados do GPS pelo bluetooth
  
  switch(MODO) {
      case DESLIGAR:                //Fazer o multirrotor pousar em segurança
          manda_dados(0, 0, 0, 0);   //Desliga os motores
          break;
                  
      case POUSAR:                //Fazer o multirrotor pousar em segurança
          if(altura < ALTURA_POUSO)
              manda_dados(0, 0, 0, 0);   //Desliga os motores
          else {                  //Diminui o throtle gradativamente até o throttle mínimo permitido
              altura_alvo = 0;
              manda_dados(0, 0, 0, cont_altura());
          }
          break;

      case MANUAL:                //O multirrotor está no modo de navegação manual
           manda_dados(roll, pitch, yaw, cont_altura());
           break;
            
      case LIVRE:                 //O multirrotor está no modo de navegação livre
          manda_dados(roll, pitch, yaw, throttle);
          break;
            
      case GPS:                   //O multirrotor está navegando pelo GPS
          if (gps_disponivel) { //GPS esta com sinal
            if(angmag == 500) { // Magnetometro desconectado ou inexistente
              pitch = ANGULOGPS;
              latresultante = latdestino-lat;
              lonresultante = londestino-lon;
              distgps = latresultante*latresultante + lonresultante*lonresultante;
              if(distgps <= MARGEMGPS*MARGEMGPS) 
                estavel();
              else {
                if(distgps <= distantgps || distantgps == 0) //Se distancia estiver diminuindo ou quando iniciado(distantgps=0) ele segue em frente
                  yaw = 0;
                else //Caso contrario ele vira a direita.
                  yaw = YAWPADRAO;
              }
              manda_dados(0, pitch, yaw, cont_altura());
              distantgps = distgps; // distantgps guarda a distancia gps para usar na proxima iteracao
            }
            //Fazer a rotacao ate sincronizar com o norte utilizando o magnetometro
            else if(angmag > 3 && angmag <=180)  // Colocar uma margem de erro de 3 graus - Sentido horario
                manda_dados(0, 0, YAWPADRAO, cont_altura());

            else if(angmag > 180 && angmag < 357)  // Sentido anti-horario com uma margem de 3 graus
                manda_dados(0, 0, -YAWPADRAO, cont_altura());
            
            else { //Compara destino GPS
                latresultante = latdestino-lat;
                lonresultante = londestino-lon;
                modlat = abs(latresultante);
                modlon = abs(lonresultante);
                if(modlat <= MARGEMGPS && modlon <= MARGEMGPS) // Chegou na localizacao, mantem ele estavel
                    estavel();    

                else if(modlat <= MARGEMGPS) { // Latitude alcancada
                    if(lonresultante < 0) { // Esquerda
                        roll = - ANGULOGPS;
                        manda_dados(roll, 0, 0, cont_altura());
                    }
                    else { // Direita
                        roll = ANGULOGPS;
                        manda_dados(roll, 0, 0, cont_altura());
                    }            
                }

                else if(modlon <= MARGEMGPS) { // Longitude alcancada
                    if(latresultante < 0) { // Atras
                        pitch = - ANGULOGPS;
                        manda_dados(0, pitch, 0, cont_altura());
                    }
                    else { // Frente
                        pitch = ANGULOGPS;
                        manda_dados(0, pitch, 0, cont_altura());                            
                    }            
                }

                else if(modlat > modlon) { // Caso ele tenha que andar mais latitude que longitude
                    if(latresultante > 0)
                        pitch = ANGULOGPS;
                    else 
                        pitch = -ANGULOGPS;
                    if(lonresultante > 0)
                        roll = ANGULOGPS * (modlon / modlat);
                    else
                        roll = - ANGULOGPS * (modlon / modlat);
                    manda_dados(roll, pitch, 0, cont_altura());
                }
                    
                else if(modlon >= modlat) { // Caso ele tenha que andar mais lontitude que latitude
                    if(lonresultante > 0)
                        roll = ANGULOGPS;
                    else
                        roll = -ANGULOGPS;
                    if(latresultante > 0)
                        pitch = ANGULOGPS * (modlat / modlon);
                    else
                        pitch = - ANGULOGPS * (modlat / modlon);
                    manda_dados(roll, pitch, 0, cont_altura());
                }
            }
      }
      else //Sem sinal do GPS, mantem ele estavel
          estavel();
      break;
  }
  ultima_execucao = millis();
}

bool feedgps() {
  while (Serial3.available()) {
    if (gps.encode(Serial3.read()))
      return true;
  }
  return false;
}
