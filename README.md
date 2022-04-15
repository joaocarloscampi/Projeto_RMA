# Comparação de métodos evolucionários para otimização de parâmetros de campos potênciais para robôs móveis

## Problema

No futebol de robôs, o jogador deve se mover pelo campo e desviar dos outros jogadores (tanto oponentes quanto parceiros de equipe) para atingir o seu alvo. Para isso é necessário um planejador de trajetória que atenda esses requisitos. Um método que se mostrou bastante eficiente é o *Univector Field Navigation* (UFN). Porém para sua utilização é necessário encontrar os parâmetros desse método para que tenha um desempenho ótimo. A partir disso, o nosso projeto tem como objetivo utilizar diferentes tipos de programação evolucionária (PE) para obtenção desses parâmetros.

Os parâmetros que precisam ser encontrados para o algoritmo são:

* **d_e** = Tamanho do raio da espiral;
* **K_r** = Suavização do campos;
* **K_o** = Constante proporcional relacionada a velocidade relativa entre o robô e o obstáculo;
* **d_min** = Limiar entre o campo totalmente repulsivo e a combinação entre repulsivo e o de movimento;
* **delta** = Espessura da composição entre o campo repulsivo e o de movimento.

Mais informações no [artigo](https://www.sciencedirect.com/science/article/pii/S1474667016410293)

## Método

Para obtenção dos parâmetros são implementados três tipos de métodos evolucionários:

* **Algoritmo Genético -> (Algoritmo de otimização bem conhecido)**
* **Optimização por enxame de partículas (PSO) -> (Algoritmo sugerido)** 
* **Covariance matrix adaptation evolution strategy (CMA-ES) -> (Algoritmo utilizado pelo ITA para resolver o mesmo problema)**

Cada método será implementado utilizando *Python*, *ROS* e a plataforma de simulação Gazebo, com o ambiente desenvolvido pelo time da USP [ThunderRatz](https://github.com/ThundeRatz/travesim). A otimização será realizada da seguinte forma:

* Otimização sem obstáculos;
* Otimização com obstáculos;
* Otimização com obstáculos em movimento. 
