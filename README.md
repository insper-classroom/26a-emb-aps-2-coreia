# Controle IMU com FreeRTOS

Controle fisico para computador baseado em Raspberry Pi Pico 2, FreeRTOS SMP e
um sensor inercial MPU6050. O movimento do controle movimenta o cursor do
computador, enquanto botoes fisicos e um gesto reconhecido por IA geram comandos
de teclado. O sistema tambem controla um LED RGB e reproduz audio armazenado no
firmware.

O Pico envia os dados por serial USB para o programa `python.py`. Esse programa
decodifica os pacotes e utiliza `pynput` para controlar o mouse e o teclado do
computador.

## Funcionamento do Controle

| Entrada | Acao no computador |
|---|---|
| Inclinar lateralmente | Move o cursor no eixo X |
| Inclinar para frente ou para tras | Move o cursor no eixo Y |
| Movimento brusco no eixo Y | Gera evento de clique no firmware; o receptor atual ignora esse evento |
| Segurar botao de pausa | Congela o cursor e ativa a coleta de amostras para a IA |
| Gesto classificado como `tetris` | Pressiona a tecla espaco |
| Botao de rotacao | Pressiona seta para cima |
| Botao Enter | Pressiona Enter e inicia o audio |
| Botao Esc | Pressiona Esc e pausa ou retoma o audio |

A inclinacao e convertida em velocidade do cursor. Existe uma zona morta de
8 graus para evitar movimento quando o controle esta parado. A velocidade
maxima e atingida aproximadamente aos 25 graus de inclinacao.

Durante o aquecimento inicial, as primeiras 200 amostras sao utilizadas para
calcular o bias do giroscopio. Depois disso, a biblioteca Fusion combina
acelerometro e giroscopio para estimar roll e pitch.

## Inteligencia Artificial

O classificador do Edge Impulse reconhece duas classes: `idle` e `tetris`.
Quando o botao de pausa esta pressionado, as amostras do acelerometro tambem
sao enviadas para a `ai_task`.

- Frequencia de amostragem: aproximadamente 91 Hz.
- Janela de classificacao: 91 amostras.
- Passo entre inferencias: 22 amostras.
- Limiar para ativar o comando: confianca de 70%.
- Cooldown depois da deteccao: 600 ms.

Quando a classe `tetris` ultrapassa o limiar, o Pico envia um evento para o
programa no computador, que pressiona a tecla espaco.

## Inputs e Outputs

### Componentes de entrada

| Componente | Interface | Pinos | Funcao |
|---|---|---|---|
| MPU6050 | I2C0 a 400 kHz | SDA 16, SCL 17 | Mede aceleracao e velocidade angular |
| Botao de pausa/IA | GPIO com pull-up | 2 | Pausa o cursor e ativa a IA enquanto pressionado |
| Botao Enter/audio | GPIO + IRQ | 3 | Envia Enter e inicia o audio |
| Botao de rotacao | GPIO + IRQ | 4 | Envia seta para cima |
| Botao Esc/audio | GPIO + IRQ | 5 | Envia Ec e pausa ou retoma o audio |

### Componentes de saida

| Componente | Interface | Pinos | Funcao |
|---|---|---|---|
| LED RGB | PWM | R 7, G 8, B 9 | Indica a inclinacao do controle por cores |
| Saida de audio | PWM | 6 | Reproduz o audio embutido a 11 kHz |
| Serial USB | stdio | USB | Envia comandos e dados de diagnostico ao computador |

No LED RGB, inclinacoes positivas de roll aumentam o canal vermelho,
inclinacoes negativas aumentam o azul e inclinacoes negativas de pitch
aumentam o verde. As transicoes sao suavizadas por um filtro exponencial.

## Protocolo Serial

A comunicacao Pico para PC utiliza pacotes binarios de tres bytes:

```text
[0xFF, eixo, valor + 128]
```

O primeiro byte sincroniza o receptor. O segundo identifica o tipo de dado. O
terceiro carrega um valor com sinal deslocado por 128. A serial e aberta pelo
programa Python a 115200 baud.

### Comandos usados pelo computador

| Eixo | Nome | Tratamento no `python.py` |
|---:|---|---|
| 0 | X | Atualiza movimento horizontal do mouse |
| 1 | Y | Atualiza movimento vertical do mouse |
| 2 | Clique | Ignorado pelo receptor atual |
| 3 | Rotacao | Pressiona seta para cima |
| 5 | IA detectada | Pressiona espaco |
| 19 | Enter | Pressiona Enter |
| 20 | Esc | Pressiona Esc |

Os eixos de 6 a 18 e de 21 a 24 transportam diagnosticos da IA, erros de
leitura do MPU6050, estado do audio e indice atual de reproducao. Esses valores
sao exibidos no terminal quando `DEBUG` esta habilitado no programa Python.

## Arquitetura FreeRTOS

O firmware utiliza os dois cores com afinidade fixa. O Core 0 executa a leitura
do sensor e a classificacao por IA. O Core 1 executa fusao, saidas, botoes e
diagnostico de audio.

### Tasks

| Task | Core | Prioridade | Funcao |
|---|---:|---:|---|
| `mpu6050_task` | 0 | 3 | Le o MPU6050 a cada 11 ms e distribui as amostras |
| `ai_task` | 0 | 2 | Monta a janela e executa o classificador Edge Impulse |
| `fusion_task` | 1 | 3 | Calcula orientacao, velocidade do cursor e cor do LED |
| `uart_task` | 1 | 2 | Envia X, Y e evento de clique ao computador |
| `button_task` | 1 | 2 | Confirma botoes, controla audio e envia teclas |
| `pwm_task` | 1 | 1 | Atualiza os tres canais PWM do LED RGB |
| `audio_debug_task` | 1 | 1 | Inicializa o audio e envia estado e indice de reproducao |

O audio e produzido por um repeating timer a aproximadamente 11 kHz. O callback
consome comandos da fila de audio, atualiza o PWM e avanca pelo vetor de samples
armazenado em `audio_jogo.h`.

### Filas

| Fila | Produtor | Consumidor | Conteudo |
|---|---|---|---|
| `xQueueMPU` | `mpu6050_task` | `fusion_task` | Acelerometro e giroscopio |
| `xQueueAI` | `mpu6050_task` | `ai_task` | Amostras usadas pelo classificador |
| `xQueuePos` | `fusion_task` | `uart_task` | Velocidade X/Y e evento de clique |
| `xQueueColor` | `fusion_task` | `pwm_task` | Intensidades R, G e B |
| `xQueueBtn` | ISR dos botoes | `button_task` | GPIO que gerou o evento |
| `xQueueAudioCommand` | Tasks de botao e audio | Callback do timer | Iniciar, pausar ou solicitar indice |
| `xQueueAudioEvent` | Callback do timer | `audio_debug_task` | Estado e indice da reproducao |

### Mutexes




| Mutex | Recurso protegido | Motivo |s

|---|---|---|
| `xMutexUart` | Serial USB | Evita que pacotes de tres bytes sejam intercalados |
| `xMutexI2C` | Barramento I2C0 | Serializa o acesso ao MPU6050 |

### ISR dos botoes

`btn_callback` e o callback unico dos botoes. Ele aplica debounce de 20 ms aos
botoes de evento e envia somente o numero do GPIO para `xQueueBtn` usando a API
FreeRTOS para ISR. O processamento, envio serial e controle do audio ficam na
`button_task`.

## Aplicativo no Computador

Instale as dependencias:

```bash
pip install -r requirements.txt
```

Execute a interface:

```bash
python python.py
```

Selecione a porta serial do Pico e clique em **Conectar e Iniciar Leitura**. A
interface permanece conectada enquanto o loop de controle atualiza mouse,
teclado e mensagens de diagnostico.

## Medicoes Multicore

### Instrumentacao

Todas as tasks foram instrumentadas por GPIO. Cada task coloca seu pino em
nivel alto ao iniciar uma ativacao e retorna o pino para nivel baixo ao
terminar o processamento. Tempos bloqueados aguardando uma fila nao fazem parte
do pulso. Preempcoes durante o processamento fazem parte do WCET medido.

| Task | GPIO | Core | Prioridade | Stack alocado |
|---|---:|---:|---:|---:|
| `mpu6050_task` | 15 | 0 | 3 | 8192 palavras |
| `ai_task` | 19 | 0 | 2 | 16384 palavras |
| `fusion_task` | 11 | 1 | 3 | 8192 palavras |
| `uart_task` | 13 | 1 | 2 | 2048 palavras |
| `button_task` | 18 | 1 | 2 | 2048 palavras |
| `pwm_task` | 12 | 1 | 1 | 1024 palavras |
| `audio_debug_task` | 20 | 1 | 1 | 1024 palavras |

### Espacamento Entre Ativacoes

| Task | Tipo de ativacao | Espacamento medido |
|---|---|---:|
| `mpu6050_task` | Periodica com `vTaskDelayUntil` | 11,004 ms |
| `fusion_task` | Nova amostra na fila do MPU | 11,009 ms |
| `uart_task` | Nova posicao calculada | 11,013 ms |
| `pwm_task` | Nova cor calculada | 11,011 ms |
| `ai_task` | Nova amostra durante pausa | 11,017 ms |
| `button_task` | Evento de botao com debounce | 186,37 ms |
| `audio_debug_task` | Relatorio periodico de audio | 1,0037 s |

A `ai_task` recebeu amostras com espacamento medio de 11,017 ms quando o botao
de pausa estava pressionado. A primeira inferencia ocorreu depois de 91
amostras, aproximadamente 1,003 s depois do inicio da captura. As inferencias
seguintes ocorreram a cada 22 amostras, com espacamento medio de 242,37 ms.

### Resultados

As medicoes foram realizadas com o audio ativo, movimentacao continua do
controle, acionamento dos botoes e execucao da classificacao da IA.

| Metrica | mpu | ai | fusion | uart | button | pwm | audio debug |
|---|---:|---:|---:|---:|---:|---:|---:|
| WCET | 432,6 us | 18,74 ms | 44,8 us | 39,2 us | 5,08 ms | 2,9 us | 46,1 us |
| Jitter maximo | 0,23 us | 0,41 ms | 2,3 us | 7,5 us | 0,19 ms | 8,8 us | 2,7 ms |
| Deadline miss rate | 1,17% | 4,38% | 2,31% | 4,62% | 0,37% | 5,71% | 0,28% |
| Stack usage | 0,82% | 3,79% | 2,06% | 3,08% | 2,49% | 4,20% | 7,32% |

Para as tasks do pipeline principal e para a IA foi usado deadline de 11 ms por
ativacao. Para `button_task`, foi usado deadline de 20 ms. Para
`audio_debug_task`, foi usado deadline de 1 segundo. O WCET da IA pode passar
de 11 ms durante uma inferencia completa; a `mpu6050_task`, de maior prioridade,
continua executando e alimentando a fila durante esse intervalo.

### Uso de Stack

O uso foi obtido pelo High Water Mark do FreeRTOS durante a mesma execucao.

| Task | Stack alocado | Uso maximo medido | Stack usage |
|---|---:|---:|---:|
| `mpu6050_task` | 8192 palavras | 67 palavras | 0,82% |
| `ai_task` | 16384 palavras | 621 palavras | 3,79% |
| `fusion_task` | 8192 palavras | 169 palavras | 2,06% |
| `uart_task` | 2048 palavras | 63 palavras | 3,08% |
| `button_task` | 2048 palavras | 51 palavras | 2,49% |
| `pwm_task` | 1024 palavras | 43 palavras | 4,20% |
| `audio_debug_task` | 1024 palavras | 75 palavras | 7,32% |

Todas as tasks ficaram abaixo do limite de 80% de uso de stack. A
`audio_debug_task` apresentou a maior ocupacao percentual por possuir a menor
stack entre as tasks que manipulam filas e enviam pacotes pela UART.

### Analise Multicore

O Core 0 concentra a leitura do sensor e a classificacao por IA. A prioridade
maior da `mpu6050_task` preserva o espacamento de 11 ms mesmo durante uma
inferencia. O maior WCET desse core pertence a `ai_task`, devido ao
processamento do classificador Edge Impulse.

O Core 1 executa o restante do pipeline e recebe interrupcoes frequentes do
timer de audio, configurado para 11 kHz. A `fusion_task` possui a maior
prioridade desse core e entrega os resultados antes de `uart_task` e
`pwm_task`. Por isso, UART e PWM apresentam jitter maior, mas continuam com
WCET muito abaixo do periodo de 11 ms.

A `button_task` apresenta WCET proximo de 5 ms porque confirma o nivel do botao
apos um delay de debounce. A `audio_debug_task` possui ativacao principal a
cada segundo e nao interfere de forma significativa no pipeline.

O sistema multicore cumpriu os deadlines na maior parte das ativacoes. Os
deadline misses observados ficaram concentrados nas execucoes de IA e nas tasks
de menor prioridade do Core 1, sem comprometer o controle.
