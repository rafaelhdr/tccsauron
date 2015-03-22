# Roteiro da Visão - Estudos Iniciais #

Para estruturarmos o trabalho com a visão computacional, iremos dividir o trabalho em etapas. Cada etapa pode ser desenvolvida integracionalmente, ou seja, iremos acrescentando novas funcionalidades a uma arquitetura que já esteja bem testada previamente.

## Definições ##
  * **Marco:** Reta vertical que possui uma representação no mapa.
  * **Projeção:** Representação de uma Reta vertical no plano.
  * **Reta vertical:** Elemento físico vertical.

## Etapas ##
  1. **Detecção das projeções de Retas Verticais**
    * _Pré-processamento_
> > > Etapa responsável por aumentar a visibilidade de retas na imagem. Utilizaremos um operador de sobel para isso.
    * _Parametrização_
> > > Parametrização matemática de cada reta. Calcúla-se o vetor uvert e o PMC.
  1. **Associação entre projeções e retas verticais**

> > Intenta associar as retas verticais às projeções geradas. Algumas técnicas são utilizadas:
    * _Restrição por janela de busca espacial_
> > > Usando-se informações do deslocamento estimado para gerar um quadro onde a probabilidade da projeção de uma reta vertical estar é grande.
    * _Restrição por PMC_
> > > Usando o PMC, é possível restringir ainda mais o espaço de busca, diminuindo a complexidade do algoritmo utilizado. Parte-se do princípio que uma projeção gerada por uma batente de porta com a parede(marrom e branco) seja diferente de uma outra gerada por uma quina de parede (branco e branco).
  1. **Associação entre marcos e retas verticais**
    * _Determinação de marcos no alcance da câmera_
    * _Geração de conjuntos de associação entre marcos e retas verticais_
    * _Seleção de um conjunto de associação, selecionando o que apresenta maior quantidade de associações válidas_

> > ` `
  1. **Obtenção das projeções esperadas para cada marco**

> Como ja associamos os marcos à retas (3), e a associação de retas e projeções já foi realizada, est etapa torna-se então trivial.
> Após isto, é necessário verificar possíveis variáveis que podem melhorar a efetividade do algoritmo. Um exemplo extraído da tese lida é a utilização de um período de quarentena para determinação de marcos, reduzindo a probabilidade de falsos positivos, altamente danosos ao sistema de localização.