/*
  -----------------------------------------------------------------------------
  Projeto:        Sistema de Controle Postural (SCP)
  Descrição:      O SCP é um projeto wearable que tem como objetivo alertar um 
                  usuário quando esse estiver com má postura, através de um buzzer
  Autor:          Luiz Henrique da Silva Amaral
  Data:           03/07/2025
  Versão:         2.0
  Plataforma:     Arduino IDE 2.3.6
  Contato:        luizhsamaral@gmail.com
  GitHub:         https://github.com/LuizAmaral19 
  -----------------------------------------------------------------------------

  Hardware:
  - Lilypad Arduino
  - MPU-6050
  - Módulo buzzer passivo Lilypad
  - Bateria de 9V
  - Módulo abaixador de tensão 3.3V DC-DC AMS1117 
  - Módulo conversor USB-Serial para upload de código CP2102

  Conexões:
  - Módulo buzzer passivo: Pino 9
  - MPU-6050:              Pino a5 Lilypad -> SCL MPU-6050 (Pino de Serial Clock)
                           Pino a4 Lilypad -> SDA MPU-6050 (Pino de Serial Data) 
  - O módulo conversor USB-Serial foi conectado no FTDI da Lilypad 
  de acordo com o pinout do microcontrolador, que pode ser visto na página web 
  <https://www.instructables.com/How-to-Program-a-LilyPad-Without-FTDI-Converter/>

  Notas:
  - Nunca alimentar a Lilypad com mais de 5.5V, pois isso a matará 
  - Fixe o MPU-6050 firmemente nas costas do usuário, com o chip do módulo virado para fora
  - Lembrar de configurar a sensibilidade de acordo com o caso de uso
  - O Serial Monitor com a Lilypad funciona de maneira estranha, então para poder usá-lo mude o baud rate no Serial Monitor até que sejam mostrados dados 
  coerentes. No caso usado neste código, o baud rate setado no Serial.begin foi de 9600, porém no Serial Monitor ele foi alterado para 19200 para que funcionasse
  - Setar o Kalman_angle_X e Kalman_angle_Y com os ângulos esperados dele na posição que o sistema é iniciado. No caso de um drone, por exemplo, esse valor
  é setado em 0 graus para os dois valores

  To-do list:
  - Fazer algo para corrigir o pequeno erro na leitura, usar o aparato da professora Claudia para medir o ângulo real e fazer os ajustes ✅
  - Talvez haja a necessidade de calibrar a unidade inercial com a postura ajustada do usuário no primeiro uso ✅
  - Colocar um modo de calibração que toca um som no buzzer enquanto o usuário se ajeita, que deve ser diferente do som para alertar a má postura ✅
  - Não é bom usar delay() para determinar por quanto tempo algo ocorre, seria bom usar millis() para isso, porém o buzzer age de maneira estranha quando
  faço isso
  - Colocar um sinal sonoro para alertar ao usuário que aperte o botão de reset da Lilypad por causa de leitura 'nan' (Not A Number) 
  e tentar fazer a leitura novamente ✅
  - Checar os valores em graus por segundo e graus dados pelo giroscópio ✅ 
  - Ler o tutorial para fazer o Filtro de Kalman
  - Setar o valor de Kalman_angle_X e Kalman_angle_Y como os valores calibrados pela função calibracao()
  - Fazer com que o buzzer apite com os valores de saída do Filtro de Kalman

  Referências:
  - How I2C Communication Works? Arduino and I2C Tutorial. Link: <https://howtomechatronics.com/tutorials/arduino/how-i2c-communication-works-and-how-to-use-it-with-arduino/>
  - Arduino and MPU6050 Accelerometer and Gyroscope Tutorial. Link: <https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/>
  - Datasheet MPU-6050. Link: <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf>
  - MPU-6050 Register Map. Link: <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf>
  - A KALMAN FILTERING TUTORIAL FOR UNDERGRADUATE STUDENTS. Link: <https://aircconline.com/ijcses/V8N1/8117ijcses01.pdf>
  - 4  | How to use the MPU6050 with Arduino and Teensy. Link: <https://www.youtube.com/watch?v=yhz3bRQLvBY>
  - 5  | How to calibrate the MPU6050 with Arduino and Teensy. Link: <https://www.youtube.com/watch?v=Yh6mYF3VdFQ>
  - 14 | Measure angles with the MPU6050 accelerometer. Link: <https://www.youtube.com/watch?v=7VW_XVbtu9k>
  - 15 | Combine a gyroscope and accelerometer to measure angles - precisely. Link: <https://www.youtube.com/watch?v=5HuN9iL-zxU&t=50s>
  - How To Use Arduino's Serial Plotter. Link: <https://www.youtube.com/watch?v=WnxBNxX_WDc&t=14s>
  - Manual-Quadcopter-Drone. Link: <https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone>



  Histórico de versões:
  - v1.0: Primeiro release
  - v1.1: Foi adicionada a calibração de postura correta 
  - v1.2: Foi adicionado o uso do buzzer para alertar quando o usuário estiver com má postura
  - v1.3: Foi implementado o uso do millis() no lugar do delay() na calibração
  - v1.4: Foi implementado uma função para calcular o erro na leitura da IMU e foi retirado o uso de millis() no lugar de delay() no acc_read()
  - v1.5: O erro na leitura do giroscópio foi consertado e o Filtro de Kalman foi implementado com sucesso aparente 
  - v2.0: O código foi melhor organizado e explicado, permitindo obter gráficos das leituras do acelerômetro, giroscópio e filtro de Kalman

  -----------------------------------------------------------------------------
*/

// --- Bibliotecas ---

#include <Wire.h> //Biblioteca para comunicação I2C

// --- Defines ---

#define alert_freq 75     //Frequência em Hz do alerta de má postura
#define bad_posture -65   //Ângulo de má postura 

// --- Mapeamento de Hardware ---

#define buzzer_pin 9 //Pino para ativar o módulo de buzzer passivo

// --- Variáveis Globais ---

const int MPU = 0x68;     //Endereço I2C do MPU-6050

float AccX;               //Variável global para armazenar a aceleração em X
float AccY;               //Variável global para armazenar a aceleração em Y
float AccZ;               //Variável global para armazenar a aceleração em Z
float accAngleX;          //Variável global para armazenar o ângulo medido pelo acelerômetro em X
float accAngleY;          //Variável global para armazenar o ângulo medido pelo acelerômetro em Y

float GyroX;              //Variável global para armazenar o giro em X
float GyroY;              //Variável global para armazenar o giro em Y
float GyroZ;              //Variável global para armazenar o giro em Z
float gyroAngleX;         //Variável global para armazenar o ângulo medido pelo giroscópio em X
float gyroAngleY;         //Variável global para armazenar o ângulo medido pelo giroscópio em Y
float gyroAngleZ;         //Variável global para armazenar o ângulo medido pelo giroscópio em Z
float previousTime;       //Variável global para armazenar o tempo anterior na medida do giroscópio
float currentTime;        //Variável global para armazenar o tempo atual na medida do giroscópio
float elapsedTime;        //Variável global para armazenar o tempo decorrido na medida do giroscópio

bool calibrated = false;  //Variável global que informa se a postura foi calibrada ou não 
float calibrated_pos = 0; //Variável global que informa o ângulo posturado em Y

float AccErrorX;          //Variável global para informar o erro no ângulo X do acelerômetro
float AccErrorY;          //Variável global para informar o erro no ângulo Y do acelerômetro
float GyroErrorX;         //Variável global para informar o erro no ângulo X do giroscópio
float GyroErrorY;         //Variável global para informar o erro no ângulo Y do giroscópio
float GyroErrorZ;         //Variável global para informar o erro no ângulo Z do giroscópio

float Kalman_angle_X = 0; //Variável global para determinar o ângulo X vindo do Filtro de Kalman. Atenção: inicialize ele com o valor esperado ao ligar o sistema
float Kalman_angle_Y = 0; //Variável global para determinar o ângulo Y vindo do Filtro de Kalman. Atenção: inicialize ele com o valor esperado ao ligar o sistema
float Kalman_uncertainty_X = 2*2; //Incerteza na medição do ângulo em X
float Kalman_uncertainty_Y = 2*2; //Incerteza na medição do ângulo em Y
float Kalman1dOutput[] = {0,0};   //Saída do filtro, sendo que o primeiro valor é a previsão do ângulo e o segundo é a incerteza nessa previsão, sendo atualizados a cada iteração
uint32_t LoopTimer;

// --- Protótipo das funções ---

void calibracao(); //Função para calibração da postura correta
void acc_read();   //Função de leitura do acelerômetro
void gyro_read();  //Função de leitura do giroscópio
void kalman_1d();  //Função para implementar o filtro de Kalman 

// --- Configurações Iniciais (Setup) ---

void setup() {

  //Inicializa o pino do buzzer como um OUTPUT
  pinMode(buzzer_pin, OUTPUT);  //Seta o pin do buzzer como um OUTPUT

  //Inicializa Serial
  Serial.begin(9600);

  Wire.setClock(400000);        //Seta a frequência de comunicação I2C com a MPU-6050 em 400kHz
  Wire.begin();                 //Inicia a comunicação I2C
  Wire.beginTransmission(MPU);  //Começa a comunicação com o MPU-6050
  Wire.write(0x6B);             //Fala com o registrador de endereço 0x6B do MPU
  Wire.write(0x00);             //Liga o dispositivo ao mandar um byte com somente zeros 
  Wire.endTransmission(true);   //Termina a transmissão

  //Configura a sensibilidade do acelerômetro - Default é de +/-2g

    /* Configuração em binário
      Wire.write(0b00000000); //Fundo de escala em +/-2g
      Wire.write(0b00001000); //Fundo de escala em +/-4g
      Wire.write(0b00010000); //Fundo de escala em +/-8g
      Wire.write(0b00011000); //Fundo de escala em +/-16g
    */

    /* Configuração em hexadecimal
      Wire.write(0x00); //Fundo de escala em +/-2g
      Wire.write(0x08); //Fundo de escala em +/-4g
      Wire.write(0x10); //Fundo de escala em +/-8g
      Wire.write(0x18); //Fundo de escala em +/-16g
    */
  
  Wire.beginTransmission(MPU); 
  Wire.write(0x1C);             //Fala com o registrador ACCEL_CONFIG para configurar o fundo de escala do acelerômetro
  Wire.write(0x00);             //Seleciona o fundo de escala +/-2g ao mandar 00000000 para o registrador
  Wire.endTransmission(true);

  //Configura a sensibilidade do giroscópio - Default é de +/-250deg/s

    /* Configuração em binário
      Wire.write(0b00000000); //Fundo de escala em +/-250deg/s
      Wire.write(0b00001000); //Fundo de escala em +/-500deg/s
      Wire.write(0b00010000); //Fundo de escala em +/-1000deg/s
      Wire.write(0b00011000); //Fundo de escala em +/-2000deg/s
    */

    /* Configuração em hexadecimal
      Wire.write(0x00); //Fundo de escala em +/-250deg/s
      Wire.write(0x08); //Fundo de escala em +/-500deg/s
      Wire.write(0x10); //Fundo de escala em +/-1000deg/s
      Wire.write(0x18); //Fundo de escala em +/-2000deg/s
    */

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);             //Fala com o registrador GYRO_CONFIG para configurar o fundo de escala do giroscópio
  Wire.write(0x00);             //Seleciona o fundo de escala 250deg/s ao mandar 00000000 para o registrador
  Wire.endTransmission(true);

  //calculate_IMU_error();        //Função para calcular o erro na leitura do IMU
  LoopTimer = micros();
  delay(250);

} //end setup

// --- Loop Principal ---

void loop() {

  //calibracao();
  acc_read();
  gyro_read();

  kalman_1d(Kalman_angle_X, Kalman_uncertainty_X, GyroX, accAngleX);
  Kalman_angle_X = Kalman1dOutput[0];
  Kalman_uncertainty_X = Kalman1dOutput[1];

  kalman_1d(Kalman_angle_Y, Kalman_uncertainty_Y, GyroY, accAngleY);
  Kalman_angle_Y = Kalman1dOutput[0];
  Kalman_uncertainty_Y = Kalman1dOutput[1];

  
  Serial.print("accAngleX:");      Serial.print(accAngleX);      Serial.print(",");
  Serial.print("gyroAngleX:");     Serial.print(gyroAngleX);     Serial.print(",");
  Serial.print("Kalman_angle_X:"); Serial.print(Kalman_angle_X); Serial.print(",");
  Serial.print("accAngleY:");      Serial.print(accAngleY);      Serial.print(",");
  Serial.print("gyroAngleY:");     Serial.print(gyroAngleY);     Serial.print(",");
  Serial.print("Kalman_angle_Y:"); Serial.println(Kalman_angle_Y); 
  
  
  while(micros() - LoopTimer < 4000){
    LoopTimer = micros();
  }

} //end loop

// ============= Cálculo de Erro da IMU =============

void calculate_IMU_error(){

  //Para calcular o erro do IMU deixe o sensor plano sobre uma mesa 

  int c = 0; //Contador

  //Lê o valor do acelerômetro 200 vezes
  while(c < 200){
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    //Soma todas as leituras
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  //Divide a soma por 200 para obter o valor de erro
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  //Verifica se o valor de erro é 'NAN' (Not A Number), o que ocorre quando há 0/0 ou sqrt(-1), emitindo um sinal sonoro para o usuário reiniciar o sistema
  if(isnan(AccErrorX) || isnan(AccErrorY)){
    tone(buzzer_pin, 300);
    delay(1000);
    tone(buzzer_pin, 450);
    delay(500);
    tone(buzzer_pin, 300);
    delay(1000);
    noTone(buzzer_pin);
    Serial.println("Nan identificado");
    return;
  }

  //Lê o valor do giroscópio 2000 vezes
  while(c < 2000){

    Wire.beginTransmission(MPU);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    //Soma todas as leituras
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ; 
    c++;
    delay(1);
  }

  //Divide a soma por 2000 para obter o valor do erro
  GyroErrorX /= 2000;
  GyroErrorY /= 2000;
  GyroErrorZ /= 2000;

  //Printa os valores dos erros no monitor serial
  
  Serial.println("============= ERRO DA IMU =============");
  Serial.print("Erro em X do acelerômetro: ");
  Serial.println(AccErrorX);
  Serial.print("Erro em Y do acelerômetro: ");
  Serial.println(AccErrorY);
  Serial.print("Erro em X do giroscópio: ");
  Serial.println(GyroErrorX);
  Serial.print("Erro em Y do giroscópio: ");
  Serial.println(GyroErrorY);
  Serial.print("Erro em Z do giroscópio: ");
  Serial.println(GyroErrorZ);
  

} //end calculate_IMU_error

// ============= Calibração da Postura =============

void calibracao(){

  float AccX_cal;       //Variável local para armazenar o valor lido em X na calibração
  float AccY_cal;       //Variável local para armazenar o valor lido em Y na calibração
  float AccZ_cal;       //Variável local para armazenar o valor lido em Z na calibração
  float accAngleX_cal;  //Variável local para armazenar o ângulo lido em X na calibração
  float accAngleY_cal;  //Variável local para armazenar o ângulo lido em Y na calibração
  float accAngleZ_cal;  //Variável local para armazenar o ângulo lido em Z na calibração

  if (!calibrated){
    tone(buzzer_pin,150);
    delay(3000);
    noTone(buzzer_pin);
    delay(100);
    for(int i = 0; i < 5; i++){
      tone(buzzer_pin, 210);
      delay(500);

      for (int i = 0; i < 40; i++){

        Wire.beginTransmission(MPU);
        Wire.write(0x3B);               
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);

        AccX_cal = (Wire.read() << 8 | Wire.read())/ 16384.0; 
        AccY_cal = (Wire.read() << 8 | Wire.read())/ 16384.0; 
        AccZ_cal = (Wire.read() << 8 | Wire.read())/ 16384.0;

        accAngleY_cal = (atan(-1 * AccX_cal / sqrt(pow(AccY_cal, 2) + pow(AccZ_cal, 2))) * 180 / PI) - (-3.446); //O valor do erro em Y calculado é de -3.446
        //Serial.print("Valores de accAngleY_cal: ");
        //Serial.print(accAngleY_cal);
        //Serial.println("° ||");
        calibrated_pos = calibrated_pos + accAngleY_cal;
        //Serial.print("Valores de calibrated_pos: ");
        //Serial.print(calibrated_pos);
        //Serial.println("° ||");
      }

      noTone(buzzer_pin);
      delay(500);
    }
    calibrated_pos = calibrated_pos / 200;
    calibrated = true;
    //Serial.print("Valor calibrado: ");
    //Serial.print(calibrated_pos);
    //Serial.println("° ||");
  }

} //end calibracao

// ============= Leitura do Acelerômetro =============

void acc_read(){

  unsigned long current_time_buzzer = millis();

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);               //Começa a leitura pelo primeiro registrador do acelerômetro, de endereço 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Requisição de 6 bytes contendo os valores das leituras dos três eixos, começando pelo endereço de memória 0x3B

  //Armazena o valor das palavras nas variáveis correspondentes
    //Lê os 8 primeiros bits mais significativos, desloca 8 para a esquerda e lê os outros 8 bits menos significativos
  AccX = (Wire.read() << 8 | Wire.read())/ 16384.0; //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  AccY = (Wire.read() << 8 | Wire.read())/ 16384.0; //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AccZ = (Wire.read() << 8 | Wire.read())/ 16384.0; //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  /* 
    Alterar divisão de acordo com fundo de escala escolhido
      +/-2g  = 16384.0
      +/-4g  = 8192.0
      +/-8g  = 4096.0
      +/-16g = 2048.0
  */

  //Calculando os ângulos à partir dos dados do acelerômetro

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - (-0.25);       //O valor do erro em X calculado é de -0.25
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - (-3.446);  //O valor do erro em Y calculado é de -3.446

/*
  Serial.println("========== ACC READ ==========");
  Serial.print("X angle acc: ");
  Serial.print(accAngleX);
  Serial.print("° ||");
  Serial.print(" Y angle acc: ");
  Serial.print(accAngleY);
  Serial.println("° ||");
  delay(500);
*/

} //end acc_read

// ============= Leitura do Giroscópio =============

void gyro_read(){

  previousTime = currentTime;                         //previousTime recebe o valor de currentTime
  currentTime = millis();                             //currentTime recebe o valor do tempo atual pela função millis()
  elapsedTime = (currentTime - previousTime) / 1000;  //elapsedTime é calculado pela diferença de currentTime e previousTime dividido por 1000, para obter o tempo em segundos

  Wire.beginTransmission(MPU);
  Wire.write(0x43);               //Começa a leitura pelo primeiro registrador do giroscópio, de endereço 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Requisição de 6 bytes contendo os valores das leituras dos três eixos, começando pelo endereço de memória 0x43

  //Armazena o valor das palavras nas variáveis correspondentes
    //Lê os 8 primeiros bits mais significativos, desloca 8 para a esquerda e lê os outros 8 bits menos significativos
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 

  GyroX = GyroX -(-2.16);   //O valor do erro em X calculado é de -2.16
  GyroY = GyroY -(1.27);    //O valor do erro em Y calculado é de 1.27
  GyroZ = GyroZ -(0.26);    //O valor do erro em Z calculado é de 0.26

  /* 
    Alterar divisão de acordo com fundo de escala escolhido
      +/-250deg/s  = 131.0
      +/-500deg/s  = 65.5
      +/-1000deg/s = 32.8
      +/-2000deg/s = 16.4
  */

  //No momento os valores dados pelo giroscópio estão em graus por segundo (deg/s), então precisamos multiplicar por segundos (s) para obter o ângulo em graus
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  gyroAngleZ = gyroAngleZ + GyroZ * elapsedTime;

/*
  Serial.println("========== GYRO READ =========");
  Serial.print("X: ");
  Serial.print(GyroX);
  Serial.print("°/s ");
  Serial.print(gyroAngleX);
  Serial.print("° ||");
  Serial.print(" Y: ");
  Serial.print(GyroY);
  Serial.print("°/s ");
  Serial.print(gyroAngleY);
  Serial.print("° ||");
  Serial.print(" Z: ");
  Serial.println(GyroZ);
  Serial.print("°/s ");
  Serial.print(gyroAngleZ);
  Serial.println("° ");
  delay(500);
*/

} //end gyro_read

// ============= Filtro de Kalman =============

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){

  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);

  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1dOutput[0] = KalmanState;
  Kalman1dOutput[1] = KalmanUncertainty;

} //end kalman_1d












