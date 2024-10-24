const int INPUT_PIN = A0;  // Pino de entrada analógico (sensor)
const int OUTPUT_PIN = 3;  // Pino de saída PWM (controlador)
const int POT_PIN = A1;    // Pino onde o potenciômetro está conectado

double dt, last_time;  // Variáveis para calcular o tempo entre iterações
double integral = 0, previous_error = 0, previous_filtered_derivative = 0, output = 0;  // Variáveis para controle PID
double kp, ki, kd;  // Ganhos do controlador PID
double setpoint = 200;  // Setpoint inicial (valor de referência para o controle)

unsigned long startTime;   // Variável para armazenar o tempo inicial do sistema
unsigned long currentTime; // Variável para calcular o tempo atual

bool use_derivative_filter = true;  // Ativa ou desativa o filtro no termo derivativo
bool use_anti_windup = true;  // Ativa ou desativa o anti-windup (para limitar a integral)
bool use_reference_weighting = true;  // Ativa ou desativa a ponderação no setpoint (beta)

double Kbc;  // Ganho para o mecanismo anti-windup
double beta;  // Fator de ponderação para o termo proporcional
double alpha; // Coeficiente do filtro derivativo
double K = 1.0;  // Ganho estático do sistema (ajustar conforme necessário)
double tau = 0.5;  // Constante de tempo do sistema (ajustar conforme necessário)

void setup() {
  // Aplicando a sintonia de Skogestad para os ganhos PID
  kp = 1.0 / (K * (tau + 0));  // Cálculo do ganho proporcional, assumindo que o atraso (theta) é 0
  ki = kp / tau;               // Cálculo do ganho integral
  kd = kp * tau / 2;           // Cálculo do ganho derivativo

  last_time = 0;  // Inicializa a variável de tempo
  Kbc = ki;       // Define o ganho para o anti-windup baseado no ganho integral
  beta = 0.5;     // Define o valor da ponderação proporcional (beta)
  alpha = 0.02;   // Coeficiente para o filtro derivativo

  Serial.begin(115200);  // Inicializa a comunicação serial
  analogWrite(OUTPUT_PIN, 0);  // Inicializa o pino de saída com valor 0 (desligado)
  startTime = millis();  // Armazena o tempo inicial

  // Loop para imprimir valores iniciais de tempo e saída zero
  for (int i = 0; i < 50; i++) {
    currentTime = (millis() - startTime) / 1000.0;  // Calcula o tempo decorrido em segundos
    Serial.print(currentTime);  // Imprime o tempo decorrido
    Serial.print(",");
    Serial.println(0);  // Imprime a saída inicial (0)
    delay(100);  // Atraso de 100 ms entre leituras
  }
  delay(100);  // Pequeno atraso para estabilizar o sistema
}

void loop() {
  // Lê o valor do potenciômetro e ajusta o setpoint dinamicamente (desativado no momento)
  // setpoint = map(analogRead(POT_PIN), 0, 1023, 0, 255);  // Mapeia o valor do potenciômetro para ajustar o setpoint

  double now = millis();  // Obtém o tempo atual
  dt = (now - last_time) / 1000.00;  // Calcula o intervalo de tempo (em segundos)
  last_time = now;  // Atualiza o último tempo

  // Lê o valor do sensor (0-1023) e mapeia para 0-255
  double actual = map(analogRead(INPUT_PIN), 0, 1023, 0, 255);
  double error = setpoint - actual;  // Calcula o erro (diferença entre setpoint e valor atual)

  // Calcula a saída PID com ponderação de setpoint
  output = pid_skogestad(error, actual, setpoint);  // Chama a função PID para calcular a saída

  analogWrite(OUTPUT_PIN, output);  // Aplica a saída calculada no pino de saída PWM

  // Envia os dados de tempo, setpoint, valor atual, erro e saída para o Serial Plotter
  currentTime = millis() - startTime;  // Calcula o tempo decorrido desde o início
  Serial.print(currentTime / 1000.0);  // Imprime o tempo em segundos
  Serial.print(",");  // Separador de valores
  Serial.print(setpoint);  // Imprime o valor do setpoint
  Serial.print(",");
  Serial.print(actual);  // Imprime o valor atual do sensor
  Serial.print(",");
  Serial.print(error);  // Imprime o erro
  Serial.print(",");
  Serial.println(output);  // Imprime a saída PID

  delay(12);  // Insere um atraso de 12 ms (opcional)
}

double pid_skogestad(double error, double actual, double setpoint) {
  // Termo proporcional com ponderação de setpoint
  double proportional;
  if (use_reference_weighting) {
    proportional = beta * (setpoint - actual);  // Pondera a influência do setpoint no termo proporcional
  } else {
    proportional = error;  // Sem ponderação, usa o erro diretamente
  }

  // Termo integral (acumulativo)
  integral += error * dt;  // Atualiza o termo integral

  // Termo derivativo com filtro
  double derivative;
  if (use_derivative_filter) {
    // Calcula o termo derivativo filtrado
    double raw_derivative = (error - previous_error) / dt;
    derivative = alpha * raw_derivative + (1 - alpha) * previous_filtered_derivative;
    previous_filtered_derivative = derivative;  // Atualiza o valor filtrado
  } else {
    // Sem filtro, usa a derivada bruta
    derivative = (error - previous_error) / dt;
  }
  
  previous_error = error;  // Atualiza o erro anterior

  // Calcula a saída PID combinando os termos proporcional, integral e derivativo
  double output = kp * (proportional + integral * ki + derivative * kd);

  // Aplica anti-windup (limitação do termo integral)
  if (use_anti_windup) {
    double max_output = 255;  // Limite superior da saída (PWM máximo)
    double min_output = 0;    // Limite inferior da saída (PWM mínimo)
    if (output > max_output) {
      output = max_output;  // Se a saída exceder o máximo, limita ao valor máximo
      integral -= Kbc * (output - max_output) * dt;  // Corrige a integral para evitar saturação
    } else if (output < min_output) {
      output = min_output;  // Se a saída for menor que o mínimo, limita ao valor mínimo
      integral -= Kbc * (output - min_output) * dt;  // Corrige a integral para evitar saturação
    }
  }

  return output;  // Retorna a saída calculada
}