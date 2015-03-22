

# Modelo de observação: sonares #

O principal sensor do Pioneer 2DX é um anel de oito sonares
localizados à frente do robô e espaçados segundo a figura a seguir.

![http://img36.imageshack.us/img36/6731/image002bw.jpg](http://img36.imageshack.us/img36/6731/image002bw.jpg)

Sonares, ao contrário de <i>lasers</i>, não geram mapas de
profundidades intuitivos. Pelo contrário: devido ao modo como o sensor opera,
pode-se chegar à conclusão (errônea) de que sonares não produzem informações
confiáveis. As figuras a seguir ilustram como mudanças leves no ângulo do robô
geram mapas de profundidade muito diferentes.

![http://img143.imageshack.us/img143/9812/image004o.jpg](http://img143.imageshack.us/img143/9812/image004o.jpg)

A explicação para esse comportamento vem da maneira como o
sonar atua. A explicação simples é que o sonar envia um pulso de som e espera o
seu retorno. Sabendo a velocidade do pulso e o tempo decorrido, o sensor sabe a
distância até o objeto. Naturalmente, para que isso ocorra com sucesso o feixe
emitido pelo sonar deve incidir perpendicularmente sobre a parede.

Essa simplificação, embora didática, não é uma aproximação
suficiente para o problema da localização robótica. Um modelo mais real
representa o pulso do sonar como um cone que “aponta” na mesma direção que o
sonar e cujo vértice é o próprio sensor. O ângulo de abertura desse cone é
chamado de β. Pode-se pensar no pulso do sonar como um
conjunto de feixes que começam no mesmo ponto e vão se distanciando: se pelo menos
um dos feixes incidir perpendicularmente sobre o obstáculo, ele será detectado.

A figura a seguir ilustra a resposta de um sonar rotatório
colocado à frente de um obstáculo, variando-se o ângulo de incidência dos
feixes. 0º significa que o sonar está perpendicular ao objeto. Nota-se que,
além do cone principal (entre –20 e 25 graus), há dois cones, menores e mais
ruidosos, que também enxergam o obstáculo.

![http://img22.imageshack.us/img22/3297/image006ph.jpg](http://img22.imageshack.us/img22/3297/image006ph.jpg)

## Modelos de observação ##

O modelo de observação proposto é voltado para a detecção de
paredes, e trata cada sonar como um sensor separado – efetivamente, o robô é
visto como possuindo oito sensores de distância. O processo pode ser separado
em três etapas:

  1. Validação das observações: Primeiramente, determina-se se o obstáculo visto é realmente uma parede.Isso é feito analisando as últimas <i>k</i> observações, que devem seguir um padrão similar ao da figura a seguir:

> ![http://img44.imageshack.us/img44/8012/image008sb.jpg](http://img44.imageshack.us/img44/8012/image008sb.jpg)

> Caso se chegue à conclusão de que uma parede está sendo observada, sua postura é estimada, em relação ao robô.
  1. Associação das observações com o mapa: Procura-se um elemento no mapa que corresponda à parede obtida no primeiro passo.
  1. Obtenção das observações esperadas: a função de um modelo de observação é retornar as observações esperadas para uma determinada postura estimada do robô. Conhecendo a posição global do elemento observado, a partir do passo anterior, retorna-se a observações esperada dessa parede.

As duas primeiras etapas são executadas até que se encontre um elemento mapeado que corresponda às últimas <i>k</i>observações recebidas. A partir desse
instante, as duas fases entram num estado de manutenção, enquanto a terceira
etapa continua gerando as observações esperadas, a partir da postura estimada
do robô.

### Validação das observações ###

Essa etapa só começa a ser executada quando um número <i>k<sub>min</sub></i> de observações estiver
disponível (não há sentido validar a consistência das medidas quando não as há
em quantidade suficiente). Quando isso ocorrer, o processo de validação tem
início. Ele consiste em verificar que as últimas <i>k</i> observações podem corresponder à mesma superfície plana,
valendo-se da propriedade de que tais observações tendem a se alinham num
segmento de reta. Formalmente, considere a seguinte figura, que ilustra um robô
em movimento retilíneo cujo sonar observa uma parede:

![http://img44.imageshack.us/img44/7646/image010up.jpg](http://img44.imageshack.us/img44/7646/image010up.jpg)

Seja y<sup>j </sup>– y<i><sup>i</sup></i>=y<sub>j,i</sub>,
então, por semelhança de triângulo:

y<sub>2,1</sub>/d<sub>2,1</sub>=y<sub>3,2</sub>/d<sub>3,2</sub>=y<sub>k,k-1</sub>/d<sub>k,k-1</sub>

Seja a razão y<sub>j,j-1</sub>/d<sub>j,j-1</sub>= γ<sub>j,j-1</sub>.
A idéia é que essas razões sejam constantes. Como há muitas fontes de
incertezas no processo, isso nunca acontecerá. Modela-se, então, cada γ<sub>j,j-1<br>
</sub>como uma amostra da mesma distribuição normal. A validação das
observações torna-se um teste estatístico para averiguar se, com um determinado
grau de certeza, as medidas podem corresponder à mesma amostra. Nas palavras de
Roberto Barra: “O teste consiste em verificar se é verdadeira a hipótese que a
variância das <i>k</i>-1 amostras, dada por
s²<sub>γ</sub>, é menor ou igual á variância da distribuição, representada
por σ²<sub>obsmedida</sub>. (...) O teste é realizado sobre a estatística χ²:

χ<sup>2 </sup><sub>k-2</sub>=(k-2) s²<sub>γ</sub>/
σ²<sub>obsmedida</sub>

Para que as observações sejam validadas, é necessário que:

χ<sup>2 </sup><sub>k-2</sub> &lt; χ<sup>2 </sup><sub>k-2,<br>
1-α</sub>

Onde α é o grau de confiança e χ<sup>2 </sup><sub>k-2,<br>
1-α </sub>é tabelado.

Naturalmente, para fazer isso é necessário conhecer s²<sub>γ<br>
</sub>(trivial) e σ²<sub>obsmedida</sub> (não-trivial).

#### Determinação de s²<sub>γ</sub> ####

É a variância comum, definida em função da média aritmética de
γ:

![http://img143.imageshack.us/img143/1466/image012p.jpg](http://img143.imageshack.us/img143/1466/image012p.jpg)

#### Determinação de σ²<sub>obsmedia</sub> ####

“Considerando que todos γ<sub>j,j-1</sub> seguem uma
mesma [distribuição de probabilidade] gaussiana, σ²<sub>obsmedia</sub> é a
variância dessa distribuição”. A distribuição é formada pelo componente da
medida do sonar, <i>y</i>, e pela distância
percorrida pelo robô, <i>d</i>. Portanto, a
variância da distribuição é na verdade função das incertezas nesses dois
processos.

Depois de umas contas malucas ele chega em um resultado
bizarro que está na página 79 do pdf.

#### Determinação da parede observada ####

Quando um conjunto de observações for validado, é possível
determinar a postura da parede enxergada. Na verdade, obtém-se a reta colinear
ao segmento que representa a parede. Essa reta é determinada pela sua distância
à origem, r<sub>wall</sub>, e sua inclinação, θ<sub>wall­.</sub>

θ<sub>wall</sub> pode ser estimado a partir da
orientação atual do robô, que é considerada invariante durante as observações.
Já r<sub>wall</sub> é obtido a partir da distância mais atual medida, r<sup>1</sup>,
e da posição estimada do sonar ao realizar essa observação, (x<sub>sonar</sub><sup>(1)</sup>,
y<sub>sonar</sub><sup>(2)</sup>).

### Associação das observações com o mapa ###

A saída da primeira etapa é a reta observada, determinada
por <i>r<sub>wall</sub> </i>e θ<sub>wall</sub>.
Esses números, mais a postura estimada do robô, são usados para encontrar uma
parede no mapa que possa corresponder às observações. Essa etapa é crítica, e
deve ser realizada de forma conservadora: falso-positivos têm efeitos
catastróficos, pois quase certamente causarão erros grosseiros de localização.

Dois filtros preliminares são aplicados ao conjunto de
paredes associáveis, tanto para aumentar a segurança do emparelhamento quando
para melhorar o desempenho ao reduzir o espaço de combinações. Primeiramente,
todas as paredes que não poderiam ser enxergadas pelo sonar, devido à abertura β
do cone de feixes, são excluídas. Depois disso, os segmentos mapeados que não
estão dentro do alcance do sonar também são eliminados. Por fim, os segmentos
restantes são testados um a um.

Para o teste, as duas observações mais recentes e a mais
antiga são usadas. Determina-se a diferença entre a observação obtida (r<sub>wall</sub>)
e a observação que seria esperada, dada a postura estimada do robô. O cálculo
da observação esperada é apresentado na próxima seção. Se a diferença for maior
do que um certo número (na verdade, a diferença sobre a variância), a reta
candidata é eliminada. Se mais de uma reta sobreviver ao teste, o modelo reage
conservadoramente e retorna que nenhuma parede pode ser associada. Se
exatamente uma reta passar pela validação, ela é passada para a próxima etapa.

### Obtenção das observações esperadas para a estimativa da postura ###

A resposta final do modelo de observação não é a observação
real do robô, e sim a observação <i>esperada</i>
para aquela postura estimada. Para isso, precisamos saber qual parede estamos
olhando, o que é trabalho da etapa anterior. Sabendo isso, a figura a seguir
mostra o ambiente do robô:

![http://img36.imageshack.us/img36/9894/image014bl.jpg](http://img36.imageshack.us/img36/9894/image014bl.jpg)

Dada a posição do robô e a localização global da parede, não
é difícil estimar o que deveria ser observado (i.e. o “y” esperado).

## Determinação dos parâmetros usados no modelo de observação do sonar ##

β = 27,5º

σ²<sub>obs</sub> = 625mm² (N.B. não é σ²<sub>obsmedia</sub>)

k<sub>min</sub> = 4

α<sub> </sub><span> (do<br>
limite de rejeição) = 0,5<br>
<br>
g² = 3,2<br>
<br>
<h2>Implementação do Barra</h2>

A classe que realiza os cálculos mencionados aqui é a <a href='http://code.google.com/p/tccsauron/source/browse/trunk/Barra/fusao_framework/sources/src/br/com/r4j/robosim/model/SonarModel.java'>SonarModel</a>