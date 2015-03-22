# Introdução #

A idéia do programa é rastrear objetos em movimento usando a implementação nativa do Filtro de Kalman já existente no Opencv. Essa implementação foi feita para a versão convencional do filtro, embora a indicação informa que é possível adaptar para o EKF. (Ver referência 1)

# Modelo Considerado #

Nessa primeira versão, considera-se que o objeto reliza um movimento retilínio uniforme. A postura do objeto foi descrita como um vetor composto por quatro variáveis:
```
    X = [ posx  posy  velx  vely ] 
```

Montando o sistema de equações, tem-se:
```
    posx = posx + velx * delta
    posy = posy + vely * delta
    velx = velx
    vely = vely
```

Fixando o valor de delta = 1, pode-se representar o sistema usanda a seguinte matriz:
```
       | 1   0   1   0 |
   A = | 0   1   0   1 |
       | 0   0   1   0 |
       | 0   0   0   1 |
```

A matriz de controle B não precisou ser definida uma vez que não existem entrades de controle atuando sobre o objeto.

Quanto às medições, modelou-se uma medição capaz de obter valores para todos as variáveis de estado diretamente. Assim, a matriz H foi escrita como:
```
    H  = | 1   1   1   1 |
```


A covariância Q, referente ao ruído intrínseco do próprio processo, adotou um valor muito baixo, já que praticamente não há ruído no modelo de movimento uniforme. No entanto, a covariância do ruído da medida, R, foi "chutada", com um valor pouco maior do que um.

Todos esses parâmetros foram passados ao filtro de Kalman no seguinte trecho de código:
```
    m_kalman = cvCreateKalman( Object::getNumVars(), Measure::getNumVars(), 0 );

    m_delta = 1.0f;

    const float A[] = { 1.0f, 0.0f, m_delta, 0.0f,
                        0.0f, 1.0f, 0.0f, m_delta,
                        0.0f, 0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f };

    memcpy( m_kalman->transition_matrix->data.fl, A, sizeof(A) );
    cvSetIdentity( m_kalman->measurement_matrix, cvRealScalar( 1.0 ) );
    cvSetIdentity( m_kalman->process_noise_cov,  cvRealScalar( 1e-5 ) );
    cvSetIdentity( m_kalman->measurement_noise_cov, cvRealScalar( 10.0 ) );
    cvSetIdentity( m_kalman->error_cov_post, cvRealScalar( 1.0 ) );
```


# Geração de Estimativas #

(OBS: os códigos a seguir contém somente chamadas às classes _wrapper_ criadas para facilitar o uso do Opencv.)

A primeira etapa é obter uma estimativa primária, para a qual o filtro considera apenas  estado anterior.
```
    kalmans[i].predict( estimate ); // estimate é parâmetro de saída
```

Em seguida, é necessário fornecer a medida feita sobre acerca da postura real do objeto. Nesse programa, a medida é criada, adicionando um fator de erro variável:
```

    // Measurement generation
    mes.setPos( objs[i].getPosX() + (float)(rand() % 50) / (float)(rand() % 10 + 1) * (rand() % 2 > 0 ? 1.0f : -1.0f), 
    objs[i].getPosY() + (float)(rand() % 50) / (float)(rand() % 10 + 1) * (rand() % 2 > 0 ? 1.0f : -1.0f) );
    mes.setVel( objs[i].getVelX() + (float)(rand() % 50) / (float)(rand() % 10 + 1) * (rand() % 2 > 0 ? 1.0f : -1.0f), 
    objs[i].getVelY() + (float)(rand() % 50) / (float)(rand() % 10 + 1) * (rand() % 2 > 0 ? 1.0f : -1.0f ) );
```

Com a medida em mãos, dispara-se a segunda etapa do filtro, a de correção:
```
    kalmans[i].correct( estimate, mes ); // estimate é parâmetro de saída
                                         // mes é a medida feita (entrada)
```
Após a chamada acima, _estimate_ contém a estimativa final dada pelo Filtro de Kalman,

Nessa simulação, deve-se atualizar manualmente a posição real do objeto analisado. Isto é feito da seguinte maneira:
```
    objs[i].setPos( objs[i].getPosX() + objs[i].getVelX(), objs[i].getPosY() + objs[i].getVelY() );
```

# Próximos Passos #

  * Verificar como é a adaptação para o filtro EKF.


# Referências #

  1. http://opencv.willowgarage.com/documentation/motion_analysis_and_object_tracking_reference.html#estimators