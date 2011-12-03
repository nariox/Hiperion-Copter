void modo_manual();
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
