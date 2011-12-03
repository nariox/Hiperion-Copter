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
#define CENTRADO 127 // Os comandos variam de 0 a 255 - 127 e' o centro
#define ANGULOMAX 30 // Valor do angulo max de Euler
#define ANGULOGPS 15 // Valor do angulo usado no GPS
#define VELPADRAO 2 // 0.2 m/s
#define VELPADRAOMAX 5 // 0.5 m/s
#define YAWPADRAO 25 // Velocidade padrao de rotacao
#define THROTTLE_MIN 3 // Valor do throttle ao ligar (Ainda alterar)
#define MARGEMGPS 0,02 // Margem de erro do GPS(Ainda alterar)
#define MAGTEMPO 10 // Auxiliar no GPS sem o magnetometro (.x segundos)
#define MAXDIST 300 // Distancia maxima do sensor de distancia (em cm)
#define T_AMOSTRAGEM 100
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
int altura; //SENSOR DISTANCIA se altura = 0, sensor nao encontrou distancia.
float roll=0, pitch=0, yaw=0, throttle=0; //Dados para a camada de controle
int MODO = DESLIGAR; // Modo de navegacao
int t_padrao = 1; //Throttle minimo para comecar a se mover verticalmente
long lat, lon, alt; //Latitude, Longitude e altura(cm) (GPS)
unsigned long age, date, time, chars; // date(ddmmyy), time(hhmmsscc) GPS
unsigned short sentences, failed; // Informacoes GPS
int altantiga=0; // Distancia anterior
long latdestino, londestino, latresultante, lonresultante, modlat, modlon, distgps = 0, distantgps; //Usado para navegacao GPS
float angmag=500; // Angulo do magnetometro, valor inicial 500 que significa que o magnetometro nao esta conectado
float rollcontrol=0, pitchcontrol=0, yawcontrol=0, throttlecontrol=0; //Dados do controle , tambem usado como variavel auxiliar no gps
long ultima_execucao; // teste

void manda_dados(byte yaw, byte roll, byte pitch, byte throttle) {
      Serial2.print("R");
      Serial2.print(roll);
      Serial2.print("P");
      Serial2.print(pitch);
      Serial2.print("Y");
      Serial2.print(yaw);
      Serial2.print("T");
      Serial2.print(throttle);
}

void setup() {
   Serial.begin(115200); // Para debug
   Serial1.begin(57600); // Para GPS
   Serial2.begin(115200); // Para a camada de Controle
   pinMode(echoPin, INPUT); // define o pino 52 como entrada (recebe) SENSOR DISTANCIA
   pinMode(trigPin, OUTPUT); // define o pino 48 como saida (envia) SENSOR DISTANCIA
   pinMode(13, OUTPUT); // O pino 13 é um led, que será usado para indicar erro.
   delay(3000);  // Espera 3 segundos para garantir que os periféricos e componentes externos estão prontos.
}

//Calcula a altura em centimetros atraves do sensor de distancia ultrassonico
int distancia() {
  //seta o pino 48 com um pulso baixo "LOW" (ou desligado ou ainda 0)
  digitalWrite(trigPin, LOW);
  // delay de 2 microssegundos
  delayMicroseconds(2);
  //seta o pino 48 com pulso alto "HIGH" (ou ligado ou ainda 1)
  digitalWrite(trigPin, HIGH);
  //delay de 10 microssegundos
  delayMicroseconds(10);
  //seta o pino 48 com pulso baixo novamente
  digitalWrite(trigPin, LOW);
  // função Ranging, faz a conversão do tempo de SENSOR DISTANCIA resposta do echo em centimetros
  return (ultrasonic.Ranging(CM));
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

void loop() {
  if(millis() - ultima_execucao > T_AMOSTRAGEM )
      sinaliza_erro(0);    // O último loop demorou mais que T_AMOSTRAGEM para ser executado.
      
  // Espera o tempo de amostragem
  while(millis() - ultima_execucao < T_AMOSTRAGEM);
  
  //TODO: Tratar dados do Controle remoto
    
  altura = distancia();   //Calcula a altura em centimetros atraves do sensor de distância
  bool gps_disponivel = le_gps();               //Lê os dados do GPS
  angmag = 500;           // Lê os dados do magnetometro (500 significa sem magnetômetro)

    switch(MODO) {
        //Desliga os motores
        case DESLIGAR:
            if(altura < 20  && altura != 0)
                manda_dados(CENTRADO, CENTRADO, CENTRADO, 0);
            else   //Impede o desligamento no ar (Senao iria  cair :P)
                MODO = POUSAR;
            break;
            
        //Decolar e subir até 1m
        case DECOLAR:
            if(altura >= 100)       //Liberar o Controle apos atingir 1m de altura
                MODO == MANUAL;
            else if(altura == 0)    //Erro: altura não encontrada pelo sensor
                MODO = DESLIGAR;  
            else {                  //Aumentar o throttle gradativamente até o máximo permitido
                if(altura-altantiga<=VELPADRAO) // Velocidade padrao de subida
                    t_padrao++;
                if(altura-altantiga>=VELPADRAOMAX) // Velocidade Maxima Padrao
                    t_padrao--;
                if(t_padrao < 1)
                    t_padrao = 1;
                Serial.print("R0P0Y0T");
                Serial.print(t_padrao);
            }
            break;

        case POUSAR:                //Fazer o multirrotor pousar em segurança
            if(altura == 0)         //Sensor de distancia nao encontrado, modo livre ativado
                MODO = LIVRE;
            else {                  //Diminui o throtle gradativamente até o throttle mínimo permitido
                if(altantiga-altura<=VELPADRAO) // Velocidade padrao de descida.
                    t_padrao--;
                else if(altura-altantiga>=VELPADRAOMAX) // Velocidade maxima de descida
                    t_padrao--;
                    Serial.print("R0P0Y0T");
                    Serial.print(t_padrao);
            }
            break;

        case MANUAL:                //O multirrotor está no modo de navegação manual
            if(altura == 0)         //Sensor de altura nao encontrado, modo livre ativado
                MODO = LIVRE;
            else if(altura <= 45) {  // Altura minima para seguranca: 30cm
                if(altura-altantiga<=VELPADRAO)     // Velocidade padrao
                    t_padrao++;
                if(altura-altantiga>=VELPADRAOMAX)  // Velocidade maxima padrao
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
            break;
            
        case LIVRE:                 //O multirrotor está no modo de navegação livre
            //Obter os dados pelo controle - Fazer conversao dos dados
            Serial.print("R");
            Serial.print(roll);
            Serial.print("P");
            Serial.print(pitch);
            Serial.print("Y");
            Serial.print(yaw);
            Serial.print("T");
            Serial.print(throttle); 
            break;
            
        case GPS:                   //O multirrotor está navegando pelo GPS
            if(altura < 100 && altura != 0) {
                if(altura-altantiga<=VELPADRAO) // Velocidade Padrao
                    t_padrao++;
                if(altura-altantiga>=VELPADRAOMAX) // Velocidade maxima padrao
                    t_padrao--;
                if(t_padrao < 1)
                    t_padrao = 1;
                Serial.print("R0P0Y0T");
                Serial.print(t_padrao);
                }
            else {
                if (gps_disponivel) { //GPS esta com sinal
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
                            yaw = ANGULOGPS;
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
                        if(altura <= MAXDIST && altura > 100) { // Altura maxima para o calculo do throttle, usando o sensor de distancia
                        if(altura-altantiga<0) // Descendo
                            t_padrao++;
                        if(altura-altantiga>0) // Subindo
                            t_padrao--;
                        }
                        else if(altura < 100  && altura != 0) { // Altura minima de seguranca
                        if(altura-altantiga<=VELPADRAO) // Velocidade Padrao
                            t_padrao++;
                        if(altura-altantiga>=VELPADRAOMAX) // Velocidade maxima padrao
                            t_padrao--;
                        }
                        else if(altura > MAXDIST || altura == 0) { //Utiliza a altitude para calcular o throtle
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
                if(altura <= MAXDIST && altura > 100 && altura != 0) { // Altura maxima para o calculo do throttle, usando o sensor de distancia
                    if(altura-altantiga<0) // Descendo
                        t_padrao++;
                    if(altura-altantiga>0) // Subindo
                        t_padrao--;
                    Serial.print("R0P0Y0T");
                    Serial.print(t_padrao);
                }
                
                else if(altura < 100 && altura != 0) { // Altura minima de seguranca
                    if(altura-altantiga<=VELPADRAO) // Velocidade Padrao
                        t_padrao++;
                    if(altura-altantiga>=VELPADRAOMAX) // Velocidade maxima padrao
                        t_padrao--;
                    Serial.print("R0P0Y0T");
                    Serial.print(t_padrao);
                }
                
                else if(altura > MAXDIST || altura == 0) { //Necessario usar outros sensores
                    MODO = LIVRE;
                }
            }
        }
        break;
    }

    altantiga = altura;

    ultima_execucao = millis();
}

bool feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
}
