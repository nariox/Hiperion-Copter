//Throttle e Roll vai de 0 a 99, seria em porcentagem da potencia maxima. Yaw de 0 a 49 horario, 50 a 99 anti horario
//Angulo vai de 45 a -45 graus
//Receber os dados por bluetooth semelhante ao carrinho. Para efetuar testes.
//  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): "); printFloat(gps.f_speed_kmph()); Serial.println();


#include "Ultrasonic.h"
#include "TinyGPS.h"

//Define a pinagem
#define echoPin 52 //Pino 52 recebe o pulso do echo SENSOR DISTANCIA
#define trigPin 50 //Pino 50 envia o pulso para gerar o echo SENSOR DISTANCIA
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
#define T_AMOSTRAGEM 1000
//Modos de navegacao
#define DESLIGAR 0
#define POUSAR 1
#define MANUAL 2
#define LIVRE 3
#define GPS 4

//Controle de altura
#define altura_throttle_max 100
#define altura_throttle_min 1
#define ALTURA_POUSO 5
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
long ultima_execucao; // teste
//Objeto controlador proporcional, integral e derivativo.
int altura_alvo;
int altura_saida;
int altura_erro_acumulado = 0;

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

void loop() {
  if(millis() - ultima_execucao > T_AMOSTRAGEM )
      //sinaliza_erro(0);    // O último loop demorou mais que T_AMOSTRAGEM para ser executado.
      Serial.println("ERRO!");
      
   //Espera o tempo de amostragem
  while(millis() - ultima_execucao < T_AMOSTRAGEM);
  
  //TODO: Tratar dados do Controle remoto
    
  altura = ultrasonic.Distancia(trigPin);   //Calcula a altura em centimetros atraves do sensor de distância
  gps_disponivel = le_gps();               //Lê os dados do GPS
  angmag = 500;           // Lê os dados do magnetometro (500 significa sem magnetômetro)

      Serial.print("Distancia: ");
      Serial.println(altura);
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
                //TODO: Obter os dados pelo controle - Fazer conversao dos dados
             manda_dados(roll, pitch, yaw, cont_altura());
            break;
            
        case LIVRE:                 //O multirrotor está no modo de navegação livre
            //TODO: Obter os dados pelo controle - Fazer conversao dos dados
            manda_dados(roll, pitch, yaw, throttle);
            break;
            
        case GPS:                   //O multirrotor está navegando pelo GPS
            if (gps_disponivel) { //GPS esta com sinal
              if(angmag == 500) { // Magnetometro desconectado ou inexistente
                roll = ANGULOGPS;
                latresultante = latdestino-lat;
                lonresultante = londestino-lon;
                distgps = latresultante*latresultante + lonresultante*lonresultante;
                if(distgps < MARGEMGPS) 
                  estavel();
                else {
                  if(distgps <= distantgps || distantgps == 0) //Se distancia estiver diminuindo ou quando iniciado(distantgps=0) ele segue em frente
                    yaw = 0;
                  else //Caso contrario ele vira a direita.
                    yaw = YAWPADRAO;
                }
                manda_dados(roll, 0, yaw, cont_altura());
                distantgps = distgps; // distantgps guarda a distancia gps para usar na proxima iteracao
              }
              //Fazer a rotacao ate sincronizar com o norte utilizando o magnetometro
              else if(angmag > 3 && angmag <=180)  // Colocar uma margem de erro de 3 graus - Sentido horario
                  manda_dados(0, 0, YAWPADRAO, cont_altura());
                     
              else if(angmag > 180 && angmag < 357)  // Sentido anti-horario com uma margem de 3 graus
                  manda_dados(0, 0, 50+YAWPADRAO, cont_altura());
              
              else { //Compara destino GPS
                  latresultante = latdestino-lat;
                  lonresultante = londestino-lon;
                  modlat = abs(latresultante);
                  modlon = abs(lonresultante);
                  if(modlat < MARGEMGPS && modlon < MARGEMGPS) // Chegou na localizacao, mantem ele estavel
                      estavel();    
                
                  else if(modlat < MARGEMGPS) { // Latitude alcancada
                      if(latresultante < 0) { // Esquerda
                          roll = ANGULOGPS;
                          manda_dados(roll, 0, 0, cont_altura());
                      }
                      else { // Direita
                          roll = - ANGULOGPS;
                          manda_dados(roll, 0, 0, cont_altura());
                      }            
                  }
                    
                  else if(modlon < MARGEMGPS) { // Longitude alcancada
                      if(lonresultante < 0) { // 
                          pitch = ANGULOGPS;
                          manda_dados(0, pitch, 0, cont_altura());
                      }
                      else { // Frente
                          pitch = - ANGULOGPS;
                          manda_dados(0, pitch, 0, cont_altura());                            
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
                      manda_dados(roll, pitch, 0, cont_altura());
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
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
  }
  return false;
}
