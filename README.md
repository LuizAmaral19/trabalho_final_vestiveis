# trabalho_final_vestiveis
O SCP (Sistema de Controle Postural) é um projeto wearable que tem como objetivo alertar um usuário quando esse estiver com má postura, através de um buzzer.

# 🦾 LilyPad Postura – Sistema Vestível de Monitoramento Postural

Projeto de tecnologia vestível desenvolvido por **Luiz Henrique da Silva Amaral** para a disciplina de Tecnologias Vestíveis (2025).  
O sistema utiliza um microcontrolador LilyPad e um sensor inercial MPU-6050 para detectar desvios de postura em tempo real e alertar o usuário com um buzzer.

---

## 📌 Descrição Geral

Este projeto tem como objetivo auxiliar usuários a manterem uma boa postura, especialmente durante longos períodos em frente ao computador ou estudando.  
O sistema é posicionado nas costas e é capaz de **detectar a inclinação do tronco no plano sagital** (para frente ou para trás), emitindo um alerta sonoro sempre que uma má postura é identificada.

- Utiliza um **filtro de Kalman 1D** para suavizar os dados e melhorar a precisão da leitura.
- A operação é **não bloqueante** (sem `delay()`), garantindo resposta contínua.
- Todo o sistema é leve, discreto e adequado para uso em vestíveis.

---

## 🧩 Componentes Utilizados

- LilyPad Arduino
- MPU-6050 (acelerômetro + giroscópio)
- Buzzer passivo
- Jumpers ou fios de costura condutiva
- Software: Arduino IDE

---

## ⚙️ Funcionamento

1. **Calibração inicial**: o usuário senta com a postura correta e o sistema mede a inclinação média como referência.
2. **Leitura contínua**: a cada ciclo, o sistema lê os dados do acelerômetro e aplica o filtro de Kalman.
3. **Verificação de desvio**: se o ângulo ultrapassar o limite definido, o buzzer emite um som grave e contínuo por 2 segundos.
4. **Reset automático**: após o alerta, o sistema continua monitorando normalmente.

---

## 📍 Posicionamento da IMU

A IMU (MPU-6050) deve ser posicionada **entre as escápulas**, logo abaixo da linha dos ombros.  
Esse ponto é ideal porque representa bem a inclinação torácica do tronco no plano sagital, é relativamente estável e não interfere com articulações móveis como ombros e pescoço.  
Essa posição favorece a detecção precisa de **cifose ou curvatura excessiva para frente**.



