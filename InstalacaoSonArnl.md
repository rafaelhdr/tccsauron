# Introdução #

Este documento ensina a instalar e rodar os seguintes pacotes:
  * ARIA (código-fonte da biblioteca base)
  * SonARNL
  * MobileSim
  * MobileEyes
Todos eles são a versão mais recente., à exceção do MobileSim que por algum motivo é incrivelmente lento na versão 0.5 mas não na 0.4. Modificações precisam ser feitas na instalação do SonARNL.
# Instalação #

Todos os arquivos necessários estão neste projeto do Google Code, na aba "Downloads". Todos os caminhos são referentes a "\Arquivos de Programas\MobileRobots"
  1. Baixe e extraia o arquivo InstalaçãoMobileRobots.zip
  1. Baixe também os arquivos "sonarnlServer.exe" e "sonarnlServer.cpp"
  1. Instale todos os programas. A ordem não é importante.
  1. Substitua "ARNL\bin\sonarnlServer.exe" pelo sonarnlServer.exe que você baixou separadamente.
  1. Faça o mesmo para "ARNL\examples\sonarnlServer.cpp"
## Explicação sobre a mudança no sonarnlServer ##

O sonarnlServer é um programa de demonstração que exemplifica o uso do SonARNL e do ARIA. Por algum motivo, seus criadores decidiram que seria uma boa ideia fazer o programa registrar a última posição do robô, caso se esteja no simulador. Não foi uma boa ideia. Isso fez com que o robô se localizasse num ponto absurdo (algo como -12354123), o que faz com que nada funcione - nem o sonarnlServer, nem o MobileEyes, nem o MobileSim. Para corrigir isso, o seguinte bloco de código foi comentado:

```
  // create a pose storage class, this will let the program keep track
  // of where the robot is between runs...  after we try and restore
  // from this file it will start saving the robot's pose into the
  // file
ArPoseStorage poseStorage(&robot);
  /// if we could restore the pose from then set the sim there (this
  /// won't do anything to the real robot)... if we couldn't restore
  /// the pose then just reset the position of the robot (which again
  /// won't do anything to the real robot)
if (poseStorage.restorePose("robotPose"))
   serverLocHandler.setSimPose(robot.getPose());
else
   robot.com(ArCommands::SIM_RESET);
```

Note que **outros exemplos possuem o mesmo comportamento**. O sonarnlServer é o mais importante, portanto só ele foi corrigido.

# Rodando uma simulação #

Para testar que está tudo funcionando, faça uma simulação simples.
  1. Rode MobileSim\MobileSim.exe e escolha como mapa Arnl\examples\columbia.map
  1. Dentro de Arnl\bin, rode "sonarnlServer.exe -map columbia.map"
  1. Rode MobileEyes\bin\MobileEyes.exe
O MobileSim representa a posição "real" do robô. O MobileEyes exibe onde o robô _acha_ estar, o que pode ser diferente da posição real. Se tudo der certo, você deve ver o robô na mesma posição nos dois programas.