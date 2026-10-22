# Demo_adrolab

## Descrição
Minha parte engloba essencialmente o robô, sua movimentação e fala. Adicinalmente tem que rodar também a câmera, que vai dizer onde ta os ursinhos pra ti poder pegar com o Kinova.
Uma informação importante, todos os códigos dependem do endereço de IP do servidor. A princípio ta 10.1.3.106 (pede pro Ervin colocar IP estático no servidor).

### Robô
1. Terminal 1: Ler as informações do robô (VOLTO e aconselho a abrir diferentes abas no mesmo terminal para não ter muitas janelas) - Todos os  comandos são dentro da pasta "ros2_ws"
```bash
# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

ros2 launch marta_nav robot_pub_laser_launch.py
```
2. Terminal 2: Navigation Stack - Todos os  comandos são dentro da pasta "ros2_ws"
```bash
# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

ros2 launch marta_nav navigation_launch.py
```
3. Terminal 3: Rviz (Visualização do mapa e comandos diretos para o robô) - Todos os  comandos são dentro da pasta "ros2_ws"
```bash
# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

ros2 launch marta_nav rviz_launch.py
```
4. Terminal 4: Espera por comandos para movimentar (Esse coloquei separado, pois se rodar ele novamente ele vai ler o arquivo alvo de novo caso dê problema) - Todos os  comandos são dentro da pasta "ros2_ws"
```bash
# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

ros2 run marta_nav go_to_more
```

5. Terminal 5: Atualizador de Status e Leitura do Servidor (Esse que lê o servidor e atualiza para busy e Success)
```bash
# Entrar na pasta demo_cnpem
cd demo_cnpem

python3 demo_part1.py
```
6. Terminal 6: Reconhecimento de fala:
```bash
# Entrar na pasta demo_cnpem
cd demo_cnpem

python3 stt.py  
```
### Câmera AXIS:

1. Terminal 1: Leitura dos dados da câmera (o IP da câmera é estático e já ta no código)
```bash
# Entrar na pasta de axis_ws
cd axis_ws

# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

ros2 launch axis_camera axis_demo_launch.py 
```
2. Terminal 2: Máquina de estados, leitura da câmera e movimentação:
   
**Método 1 (não sei se funciona)**
```bash
# Entrar na pasta de axis_ws
cd axis_ws

# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

ros2 run axis_camera camera_demo.py
```
**Método 2 (Se método 1 não funcionar) SIM**
```bash
# Entrar na pasta de axis_ws
cd axis_ws

# Atualizar as variáveis de ambiente para ser a da pasta
source install/setup.bash

# Entrar na pasta do arquivo

 cd src/axis_camera/scripts

python3 camera_demo.py
```

Acho que é tudo isso, imagino que tu esteja com todos códigos. Obrigado por me ajudar SEMPRE!!!

Abraços  😉 

