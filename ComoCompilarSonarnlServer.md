# Introdução #

O fabricante do robô Pioneer 2DX, a MobileRobots, desenvolveu duas bibliotecas que auxiliam o controle e a localização/navegação do robô: ARIA e (Son)Arnl, respectivamente.

Esta wiki ensina a compilar e rodar o programa de demonstração `sonarnlServer`, que utiliza ambas as bibliotecas.

# Compilando o sonarnlServer #
## Descrição ##
O `sonarnlServer` é um programa de exemplo da MobileRobots que realiza localização e navegação do robô. Quando ele é iniciado, tenta se conectar a um simulador, caso haja um ativo. Se não encontra-lo, ele tenta se conectar a COM1 para se comunicar com o robô.

## Como compilar ##
  * Na pasta `MobileRobots/Arnl/Examples`, abra `sonarnlExamples-vc2008.snl`.
  * Com o Visual Studio, compile o projeto "sonarnlServer".
  * Pronto. Ele gerará dentro da pasta `MobileRobots\bin` um executável com nome `sonarnlServerVC9.exe` ou `sonarnlServerDebugVC9.exe`, dependendo da configuração escolhida (Debug ou Release).

## Como rodar ##
Supondo que haja um robô ligado ao computador ou um simulador rodando, inicie o sonarnlServer pela linha de comando. Por exemplo:

`D:\Users\Pedro\Documents\Poli\TCC\MobileRobots\bin> sonarnlServerDebugVC9.exe -map columbia.map`

## Observações importantes ##

  * **Não é preciso compilar o ARIA**, pois suas DLLs estão na pasta MobileRobots/bin do repositório.
  * Contudo, **para debugar o código do ARIA ("step into"), é preciso recompilá-lo**. Para que o "step into" funcione, o depurador precisa de um arquivo `.pdb`, gerado quando um projeto é compilado. Esse arquivo não está no SVN, por ser grande demais.
  * **Não é _possível_ compilar o Arnl**, pois essa biblioteca não têm seu código-fonte aberto. É preciso se virar com o que temos, sem debugar as entranhas nem alterar o código.
  * **Só estão no SVN as versões para o Visual Studio 2008 (sufixo VC9)**, que é o padrão do projeto.