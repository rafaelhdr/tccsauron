

# Introdução #

Neste documento será descrito o funcionamento atual do sistema de visão, abordando tanto os aspectos de implementação e funcionamento geral quanto os conceitos gerais envolvidos.

# Convolução #

A convolução é aplicada sobre as imagens para que seja possível ressaltar os cantos de objetos contidos na imagem, de maneira análoga ao Filtro de Sobel, uma vez que a característica visual de interesse são retas verticais.

> ## Conceitos ##
A idéia básica da convolução de imagens, que é discreta e bidimensional, é a de uma janela que é deslizada sobre a imagem. O valor do pixel resultante é igual à soma ponderada dos pixels da imagem original que se encontram dentro da janela. Os pesos são os valores do filtro que foram estabelecidos para cada um dos pixels da janela. Tal janela é denomindada **kernel** da convolução.

> ![http://images.gamedev.net/features/programming/imageproc/image050.gif](http://images.gamedev.net/features/programming/imageproc/image050.gif)

> ![http://images.gamedev.net/features/programming/imageproc/image051.jpg](http://images.gamedev.net/features/programming/imageproc/image051.jpg)

(Note que o ponto (0,0) é o canto superior esquerdo da imagem.)

> ## Implementação ##

_Todo o algoritmo de convolução está centralizado na classe [VerticalLineConvolutionOperator](http://code.google.com/p/tccsauron/source/browse/trunk/Sauron/Vision/VerticalLineConvolutionOperator.h)_


O primeiro passo é aplicar um filtro de **blur** visando remover o máximo de ruído possível da image obtida da câmera. Em seguida, os valores dos pixel são reduzidos por um fator _s_. Tal redução se mostrou necessária para evitar a saturação nos níveis de intensidade dos pixels que serão ressaltados pelo filtro, o que dificultaria e reduziria a qualidade do algoritmo de detecção de projeções verticais.
Por fim, aplica-se a convolução sobre a imagem pré-processada, resultando em uma nova imagem, na qual as linhas vericais da imagem estarão demarcadas.

Para ganhos de desempenho, a convolução é feita somente em tons de cinza, exigindo um imagem entrada previamente convertida.

# Detecção de Projeções Verticais #

_Todo o algoritmo de detecção está centralizado na classe [VerticalProjectionDetector](http://code.google.com/p/tccsauron/source/browse/trunk/Sauron/Vision/VerticalProjectionDetector.h)_


A detecção de projeções é feita sobre a imagem em tons de cinza resultante da convolução do filtro para linhas verticais.

Antes de iniciar a detecção, aplica-se um procedimento que filtra pixels que não são máximos locais, isto é, dada uma pequena porção da imagem, a intensidade não é a maior dessa região. Já que a tendência do filtro de convolução usado é marcar os pixels mais perto do centro de uma linha vertical com valores maiores que os pixels periféricos da mesma linha, o procedimento aplicado faz com que uma possível projeção, antes representada na imagem como uma linha relativamente grossa (vários pixels), passe a ter largura de um único pixel.

Feito isso, a detecção é iniciada. Ela inicia-se em um estado no qual a imagem é varrida, da esquerda para a direita, de cima para baixo, buscando um pixel que tenha intensidade maior do que um limiar de início. Quando um pixel atende a esse requisito, passa-se a um estado secundário.

Neste novo estado, supoe-se que o pixel acima do limiar é um ponto de um nova projeção. A partir dele, a imagem passa a ser varrida verticalmente, da seguinte forma: compara-se o valor dos três pixels vizinhos que estão abaixo do pixel recém-inserido na nova possível projeção. O pixel que será adicionado na projeção será aquele com o maior valor, mas exitem algumas ressalvas:
  * se o de maior valor for um dos inferiores laterais, entra em ação um fator de inércia vertical cujo papel é forçar a projeção a ser o mais vertical possível. Enquanto este fator estiver ativo, é dada preferência para o pixel diretamente abaixo, mesmo que seu valor não seja o maior. Esse fator será desativado quando alguns pixels não máximos forem adicionados, evitando a formação de um projeção distorcida.
  * se o maior valor for menor que um limiar de término, o pixel é adicionado temporariamente à projeção e passa-se a contar quantos pixels abaixo do limiar foram inseridos em sequencia. Caso essa contagem atinja um valor máximo, todos os que foram adicionados temporariamente são removidos e a projeção é considerada completa e o algoritmo volta ao estado inicial. Caso após alguns pixels abaixo do limiar de término serem inseridos, mas com contagem menor do que a máxima permitida, a adição deles é confirmada e a contagem é zerada.

Após a detecção de uma projeção ter sido considerada finalizada, ela passa por algumas verificações que incluem o tamanho da projeção, eliminando, assim, projeções muito pequenas, e a inclinação, descartando aquelas que não são tão verticais quanto o desejado.

# Perfil de Cor #

O perfil de cor segue a hipótese que a distribuição de cores ao redor das retas verticais contém informação suficiente para ajudar a identificar projeções de uma mesma reta vertical em diferentes quadros do vídeo.

Ele considera uma região da imagem centrada na seqüência de pixels que pertencem a uma projeção identificada, estendendo-se por uma faixa de _n_ pixels para a direita e para a esquerda de cada pixel da projeção.

~~É representada como um conjunto de três vetores, um para cada componente de cor da imagem. Eles tem o mesmo tamanho da largura da região considerada (sempre tamanho ímpar para permitir a centralização). Cada elemento de cada vetor recebe o valor médio, da cor correpondente, dos pixels que estão à mesma distância do centro e estão do mesmo lado. No caso do elemento central, este recebe o valor médio dos pixels pertencentes à projeção.~~

As informações que se mostraram relevantes sobre o perfil de cor são os valores médios, para cada componente de cor, dos lados esquerdo e direito em relação ao centro da região.

A comparação de dois perfis de cor, logo, de duas projeções, resulta num fator de correlação normalizado que indica quão semelhantes são os perfis. O cálculo desse fator segue a seguinte equação:

![http://img132.imageshack.us/img132/4593/eqcp.png](http://img132.imageshack.us/img132/4593/eqcp.png)

onde δ é a maior diferença esperada entre os componentes. Essa equação não é a usada originalmente pelo Barra, pois a dele obteve resultados muito ruins nos testes realizados. O motivo de tal resultado parece decorrer do fato da equação dele utilizar diferenças entre os valores de cada lado, e empregar médias dessas diferenças. Os testes revelaram que isso levava vários perfis diferentes a possuírem valores próximos, e, consequentemente, a várias falsas correlações.

Esta nova equação foi escrita visando considerar diferenças de um mesmo lado da região, para cada componente de cor, aumentando o valor do fator de correlação quando as cores de cada lados dos perfis comparados são próximos.

# Rastreamento e Associação #

A associação de projeções à marcos foi dividida em duas etapas: rastrear projeções em seqüências contínuas de quadros e relacionar projeções rastreadas à marcos descritos no mapa. Essa abordagem não é a mesma seguida por Barra, que busca a melhor associação aos marcos usando todas as projeções observadas em um dado instante. Maiores detalhes sobre os motivos da divisão serão citados abaixo.

O rastreamento de projeções é responsável por identificar projeções em diferentes quadros que são, na verdade, a mesma projeção vista em um momento e/ou posição diferente. A importância dessa etapa é que ela realiza uma filtragem pelas projeções que têm maior probabilidade de serem marcos. Isso decorre das características desejadas em marcos: altamente visíveis e identificáveis. Se uma mesma projeção pode ser vista por muito tempo, ela pode ser um marco. Dessa forma, o rastreamento também age como o buffer adotado por Barra.

A implementação do rastreamento é iniciada com a comparação de todas as projeções que estão sendo rastreadas com todas as que acabaram de ser detectadas do último _frame_ obtido da câmera. Essa comparação, baseada no perfil de cor, resulta num valor de correlação, conforme explanado na seção anterior. Somente os valores acima de um dado limiar, determinado empiricamente, são aceitos. Em seguida, os valores são agrupados de acordo com a projeção rastreada que foi usada para gerá-lo, e ordenados, em ordem decrescente. Para cada grupo, calcula-se um fator de similaridade. O grupo que tiver o maior valor de similaridade, terá o direito de utilizar a projeção recém-observada, associada ao maior valor de correlação, para relacionar à sua projeção rastreada. Após isso, o grupo com maior similaridade é removido, assim como todos os valores de correlação, dos demais grupos, associados à projeção recém-observada que foi usada. O cáluclo de similaridade e a busca pelos grupos é repetida até que todas as projeções rastredas sejam relacionadas ou que ou o maior valor de similaridade encontrado esteja abaixo de um limiar.

O efeito de buffer se deve ao fato do rastreador poder ser configurado para considerar que uma dada projeção só está sendo devidamente rastreada se ela for vista pelos _n_ vezes nos últimos _i_ quadros, sendo _n_ e _i_ facilmente ajustáveis.

A associação de marcos é feita entre projeções rastreadas e marcos descritos num arquivo. Um marco foi modelado como um perfil de cor que se encontra em uma determinada coordenada do mundo real. O algoritmo de associação de marcos, segue, em essência, as mesmas idéias aplicadas no rastreamento de projeções, existindo apenas diferenças em estruturas de dados empregadas internamente.


# Próximos Passos #

A prioridade do componente de visão é a implementação do estimador que alimentará o Filtro de Kalman. Como o filtro ainda está para ser desenvolvido, decidiu-se aguardar até que sua interface esteja consolidada o suficiente para que se faça o estimador, evitando-se desperdício de esforço, numa eventual mudança nos planos.

Dentre melhorias a serem feitas, há a inclusão de um algoritmo que auxilia na associação de marcos por meio de estimativas da posição que um determinado marco deveria ocupar num frame, conhecendo a postura atual do robô (posição e orientação).


# Glossário #

  * **Kernel**
> > Matriz que contém os valores que serão empregados na convolução de imagens.

  * **Marco**
> > Reta vertical que possui uma representação no mapa

  * **Projeção**
> > Representação de uma reta vertical no plano de projeção da câmera

  * **Reta vertical**
> > Elemento físico vertical.

# Referências #

  * http://en.wikipedia.org/wiki/Convolution
  * http://en.wikipedia.org/wiki/Sobel_operator
  * http://www.gamedev.net/reference/programming/features/imageproc/page2.asp