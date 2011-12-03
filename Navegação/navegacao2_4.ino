//Throttle e Roll vai de 0 a 99, seria em porcentagem da potencia maxima. Yaw de 0 a 49 horario, 50 a 99 anti horario
//Angulo vai de 45 a -45 graus
//Receber os dados por bluetooth semelhante ao carrinho. Para efetuar testes.
//  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): "); printFloat(gps.f_speed_kmph()); Serial.println();


#include "Ultrasonic.h"
#include "TinyGPS.h"

//Define a pinagem
#define echoPin 52 //Pino 52 recebe o pulso do echo SENSOR DISTANCIA
#define trigPin 50 //Pino 48 envia o pulso para gerar o echo SENSOR DISTANCIA
//Constantes
#define ANGULOMAX 30 // Valor do angulo max de Euler
#define ANGULOGPS 15 // Valor do angulo usado no GPS
#define VELPADRAO 2 // 0.2 m/s
#define VELPADRAOMAX 5 // 0.5 m/s
#define YAWPADRAO 25 // Velocidade padrao de rotacao
#define MARGEMGPS 0,02 // Margem de erro do GPS(Ainda alterar)
#define MAGTEMPO 10 // Auxiliar no GPS sem o magnetometro (.x segundos)
#define MAXDIST 300 // Distancia maxima do sensor de distancia (em cm)
//Modos de navegacao
#define DESLIGAR 0
#define LIGAR 1
#define DECOLAR 2
#define POUSAR 3
#define MANUAL 4
#define GPS 5
#define LIVRE 6

Ultrasonic ultrasonic(50,52); //iniciando a função e passando os pinos SENSOR DISTANCIA
TinyGPS gps;
bool feedgps();

//Variaveis
int distancia; //SENSOR DISTANCIA se distancia = 0, sensor nao encontrou distancia.
float roll=0, pitch=0, yaw=0, throttle=0; //Dados para a camada de controle
int MODO = DESLIGAR; // Modo de navegacao
int newmodo = -1; // Alterar o modo
int t_padrao = 1; //Throttle minimo para comecar a se mover verticalmente
long lat, lon, alt; //Latitude, Longitude e altura(cm) (GPS)
unsigned long age, date, time, chars; // date(ddmmyy), time(hhmmsscc) GPS
unsigned short sentences, failed; // Informacoes GPS
int distantiga=0; // Distancia anterior
long latdestino, londestino, latresultante, lonresultante, modlat, modlon, distgps = 0, distantgps; //Usado para navegacao GPS
float angmag=500; // Angulo do magnetometro, valor inicial 500 que significa que o magnetometro nao esta conectado
float rollcontrol=0, pitchcontrol=0, yawcontrol=0, throttlecontrol=0; //Dados do controle , tambem usado como variavel auxiliar no gps
long tempo; // teste

void setup() {
   Serial.begin(115200);
   Serial1.begin(57600);
   pinMode(echoPin, INPUT); // define o pino 52 como entrada (recebe) SENSOR DISTANCIA
   pinMode(trigPin, OUTPUT); // define o pino 48 como saida (envia) SENSOR DISTANCIA
}

void loop() {
  tempo = millis();
  //Tratar dados do Controle remoto
  
  
  //Sensor de distancia - distancia em cm 
  //seta o pino 48 com um pulso baixo "LOW" ou desligado ou ainda 0 SENSOR DISTANCIA
  digitalWrite(trigPin, LOW);
  // delay de 2 microssegundos SENSOR DISTANCIA
  delayMicroseconds(2);
  //seta o pino 48 com pulso alto "HIGH" ou ligado ou ainda 1 SENSOR DISTANCIA
  digitalWrite(trigPin, HIGH);
  //delay de 10 microssegundos SENSOR DISTANCIA
  delayMicroseconds(10);
  //seta o pino 48 com pulso baixo novamente SENSOR DISTANCIA
  digitalWrite(trigPin, LOW);
  // função Ranging, faz a conversão do tempo de SENSOR DISTANCIA resposta do echo em centimetros, e armazena na variavel distancia
  distancia = (ultrasonic.Ranging(CM));
  
  //Mexer com dados do GPS
  bool newdata = false;
  if (feedgps())
    newdata = true;
  
  if (newdata) {
    gps.get_position(&lat, &lon, &age);
    feedgps();
    gps.get_datetime(&date, &time, &age);
    feedgps();
    gps.stats(&chars, &sentences, &failed);
    alt = gps.altitude();
  }

  //Tratar dados do magnetometro
  angmag = 500; // Sem magnetometro
    
  //Alterar o modo
  if(newmodo != -1) {   //Alterar o modo
    if(newmodo != MODO) { 
      if(newmodo == DESLIGAR) { // Condicoes para alterar o modo para DESLIGAR
        if(MODO == POUSAR) // Ele so desliga se ja estiver no pousado 
          MODO = DESLIGAR;
        else
          MODO = POUSAR;
      }
      
      if(newmodo == LIGAR) { // Condicoes para alterar o modo para LIGAR
        if(MODO == DESLIGAR) // So entra no modo ligar se estiver desligado, caso contrario nao muda o modo
          MODO = LIGAR;
      }
      
      if(newmodo == DECOLAR) { // Condicoes para alterar o modo para DECOLAR
        if(MODO == LIGAR) // So entra no modo decolar se ele estiver ligado.
          MODO = DECOLAR;
      }
      
      if(newmodo == POUSAR) { // Condicoes para alterar o modo para POUSAR
        if(distancia < MAXDIST  && distancia != 0) // So entra no modo pousar se estiver ate 3m, diminuir altitude antes de entrar neste modo
          if(MODO == MANUAL || MODO == GPS) // So entra no modo pousar se ele estiver nos modos manual ou gps
            MODO = POUSAR;
      }
      
      if(newmodo == MANUAL) { // Condicoes para alterar o modo para MANUAL
        if(MODO == GPS) // Troca o modo gps pelo modo manual
          MODO = MANUAL;
        else if(MODO == LIGAR) //Decola e depois vai para o modo manual
          MODO = DECOLAR;
      }
      
      if(newmodo == GPS) { // Condicoes para alterar o modo para GPS
        if(MODO == LIGAR) { // So entra no modo gps se ele estiver no modo ligar
          MODO = GPS;
        }
      }

      if(newmodo == LIVRE) { // Condicoes para alterar o modo para LIVRE
        if(MODO == LIGAR) // So entra no modo gps se ele estiver no modo ligar
          MODO = LIGAR;
      }
    }
  }

  if(MODO == DESLIGAR) { //Desligar os motores
    if(distancia < 20  && distancia != 0) {
      Serial.print("R0P0Y0T0");
      roll = 0;
      pitch = 0;
      yaw = 0;
      throttle = 0;
    }
    else { //Impede ele de desligar no ar. Vai cair :P
      MODO == POUSAR;
    }
  }
  
  else if(MODO == LIGAR) { //Liga os motores em baixa potencia e aguarda nova ordem no solo
    Serial.print("R0P0Y0T1"); // Throttle = 1, valor apenas para ligar os motores
  }
  
  else if(MODO == DECOLAR) { //Decolar e subir at 1m 
    if(distancia >= 100) MODO == MANUAL; //Liberar o Controle apos atingir 1m de altura
    else if(distancia == 0) //Sensor de distancia nao encontrado
      MODO = DESLIGAR;
    else {
      if(distancia-distantiga<=VELPADRAO) // Velocidade padrao de subida
        t_padrao++;
      if(distancia-distantiga>=VELPADRAOMAX) // Velocidade Maxima Padrao
        t_padrao--;
      if(t_padrao < 1)
        t_padrao = 1;
      Serial.print("R0P0Y0T");
      Serial.print(t_padrao);
    }
  }
  
  else if(MODO == POUSAR) { //Fazer ele pousar em seguranca
    if(distancia == 0) { //Sensor de distancia nao encontrado, modo livre ativado
      MODO = LIVRE;
    }
    else {
      if(distantiga-distancia<=VELPADRAO) // Velocidade padrao de descida.
        t_padrao--;
      else if(distancia-distantiga>=VELPADRAOMAX) // Velocidade maxima de descida
        t_padrao--;
      Serial.print("R0P0Y0T");
      Serial.print(t_padrao);
    }
  }
  
  else if(MODO == MANUAL) { //Habilitar o controle manual
    if(distancia == 0) { //Sensor de distancia nao encontrado, modo livre ativado
      MODO = LIVRE;
    }
    else if(distancia <= 45 && distancia != 0) { // Altura minima para seguranca: 30cm
      if(distancia-distantiga<=VELPADRAO) // Velocidade padrao
        t_padrao++;
      if(distancia-distantiga>=VELPADRAOMAX) // Velocidade maxima padrao
        t_padrao--;
      Serial.print("R0P0Y0T");
      Serial.print(t_padrao);
    }
    else {
      //Obter os dados pelo controle - Fazer conversao dos dados
      Serial.print("R");
      Serial.print(roll);
      Serial.print("P");
      Serial.print(pitch);
      Serial.print("Y");
      Serial.print(yaw);
      Serial.print("T");
      Serial.print(throttle); 
    }
  }
  
  else if(MODO == GPS) { //Habilitar o controle GPS
    if(distancia < 100 && distancia != 0) {
      if(distancia-distantiga<=VELPADRAO) // Velocidade Padrao
        t_padrao++;
      if(distancia-distantiga>=VELPADRAOMAX) // Velocidade maxima padrao
        t_padrao--;
      if(t_padrao < 1)
        t_padrao = 1;
      Serial.print("R0P0Y0T");
      Serial.print(t_padrao);
    }
    else {
      if (newdata) { //GPS esta com sinal
        if(angmag == 500) { // Magnetometro desconectado ou inexistente
          roll = ANGULOGPS;
          latresultante = latdestino-lat;
          lonresultante = londestino-lon;
          distgps = latresultante*latresultante + lonresultante*lonresultante;
          if(distgps < MARGEMGPS) {
            roll =0;
            yaw = 0;
          }
          else {
            if(distgps <= distantgps || distantgps == 0) //Se distancia estiver diminuindo ou quando iniciado(distantgps=0) ele segue em frente
              yaw = 0;
            else //Caso contrario ele vira a direita.
              yaw = ANGULOSGPS;
          }
          Serial.print("R");
          Serial.print(roll);
          Serial.print("P0Y");
          Serial.print(yaw);
          Serial.print("T");
          Serial.print(throttle); // Calcular o throttle
          distantgps = distgps; // distantgps guarda a distancia gps para usar na proxima iteracao
        }
        //Fazer a rotacao ate sincronizar com o norte utilizando o magnetometro
        else if(angmag > 3 && angmag <=180) { // Colocar uma margem de erro de 3 graus - Sentido horario
          //Calcular o throttle.
          Serial.print("R0P0Y");
          Serial.print(YAWPADRAO);
          Serial.print("T");
          Serial.print(throttle);
        }       
        else if(angmag > 180 && angmag < 357) { // Sentido anti-horario com uma margem de 3 graus
          //Calcular o throttle.
          Serial.print("R0P0Y");
          Serial.print(50+YAWPADRAO);
          Serial.print("T");
          Serial.print(throttle);
        }
        else { //Compara destino GPS
          latresultante = latdestino-lat;
          lonresultante = londestino-lon;
          modlat = abs(latresultante);
          modlon = abs(lonresultante);
          if(modlat < MARGEMGPS && modlon < MARGEMGPS) { // Chegou na localizacao, mantem ele estavel
            if(distancia <= MAXDIST && distancia > 100) { // Altura maxima para o calculo do throttle, usando o sensor de distancia
              if(distancia-distantiga<0) // Descendo
                t_padrao++;
              if(distancia-distantiga>0) // Subindo
                t_padrao--;
            }
            else if(distancia < 100  && distancia != 0) { // Altura minima de seguranca
              if(distancia-distantiga<=VELPADRAO) // Velocidade Padrao
                t_padrao++;
              if(distancia-distantiga>=VELPADRAOMAX) // Velocidade maxima padrao
                t_padrao--;
            }
            else if(distancia > MAXDIST || distancia == 0) { //Utiliza a altitude para calcular o throtle
              //Utiliza a altitude para calcular o throtle
            }
            Serial.print("R0P0Y0T");
            Serial.print(t_padrao);
          }
          
          else if(modlat < MARGEMGPS) { // Latitude alcancada
            if(latresultante < 0) { // Esquerda
              //Calcular o throttle
              roll = ANGULOGPS;
              Serial.print("R");
              Serial.print(roll);
              Serial.print("P0Y0T");
              Serial.print(throttle);              
            }
            else { // Direita
              //Calcular o throttle
              roll = - ANGULOGPS;
              Serial.print("R");
              Serial.print(roll);
              Serial.print("P0Y0T");
              Serial.print(throttle);                            
            }            
          }
          
          else if(modlon < MARGEMGPS) { // Longitude alcancada
            if(lonresultante < 0) { // 
              //Calcular o throttle
              pitch = ANGULOGPS;
              Serial.print("R0P");
              Serial.print(pitch);
              Serial.print("Y0T");
              Serial.print(throttle);              
            }
            else { // Frente
              //Calcular o throttle
              pitch = - ANGULOGPS;
              Serial.print("ROP");
              Serial.print(pitch);
              Serial.print("Y0T");
              Serial.print(throttle);                            
            }            
          }
          
          else if(modlat > modlon) { // Caso ele tenha que andar mais latitude que longitude
            if(latresultante > 0)
              roll = ANGULOGPS;
            else 
              roll = -ANGULOGPS;
            if(lonresultante > 0)
              pitch = ANGULOGPS * (modlon / modlat);
            else
              pitch = - ANGULOGPS * (modlon / modlat);
            Serial.print("R");
            Serial.print(roll);
            Serial.print("P");
            Serial.print(pitch);
            Serial.print("Y0");
            Serial.print("T");
            Serial.print(throttle); //Falta calcular o throttle
          }
          
          else if(modlon >= modlat) { // Caso ele tenha que andar mais lontitude que latitude
            if(lonresultante > 0)
              pitch = ANGULOGPS;
            else
              pitch = -ANGULOGPS;
            if(latresultante > 0)
              roll = ANGULOGPS * (modlat / modlon);
            else
              roll = - ANGULOGPS * (modlat / modlon);
            Serial.print("R");
            Serial.print(roll);
            Serial.print("P");
            Serial.print(pitch);
            Serial.print("Y0");
            Serial.print("T");
            Serial.print(throttle); //Falta calcular o throttle            
          }
        }
      }
      else { //Sem sinal do GPS, mantem ele estavel
        if(distancia <= MAXDIST && distancia > 100 && distancia != 0) { // Altura maxima para o calculo do throttle, usando o sensor de distancia
          if(distancia-distantiga<0) // Descendo
            t_padrao++;
          if(distancia-distantiga>0) // Subindo
            t_padrao--;
          Serial.print("R0P0Y0T");
          Serial.print(t_padrao);
        }
        
        else if(distancia < 100 && distancia != 0) { // Altura minima de seguranca
          if(distancia-distantiga<=VELPADRAO) // Velocidade Padrao
            t_padrao++;
          if(distancia-distantiga>=VELPADRAOMAX) // Velocidade maxima padrao
            t_padrao--;
          Serial.print("R0P0Y0T");
          Serial.print(t_padrao);
        }
        
        else if(distancia > MAXDIST || distancia == 0) { //Necessario usar outros sensores
          MODO = LIVRE;
        }
      }
    }
  }
  
  else if(MODO == LIVRE) { //Habilitar o controle livre
    //Obter os dados pelo controle - Fazer conversao dos dados
    Serial.print("R");
    Serial.print(roll);
    Serial.print("P");
    Serial.print(pitch);
    Serial.print("Y");
    Serial.print(yaw);
    Serial.print("T");
    Serial.print(throttle); 
  }

  tempo = millis()-tempo;
  Serial.println(tempo);
  distantiga = distancia;
  newmodo = -1;
  delay(3000); //trocar este comando por millis()
}

bool feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
}
