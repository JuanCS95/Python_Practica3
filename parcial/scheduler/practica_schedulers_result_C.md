# Resultados del Ejercicio C


| Proceso  | Ráfaga de CPU | Tiempo de Llegada | Prioridad |
| -------- | ------------- | ----------------- | --------- |
| 1        | 5             | 0                 | 3         | 
| 2        | 3             | 2                 | 5         | 
| 3        | 6             | 1                 | 2         | 
| 4        | 4             | 4                 | 1         | 





## FCFS

| Proceso  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
| 1        |  5 |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
| 2        |    |    |  . |  . |  . |  . |  . |  . |  . |  . |  . |  3 |  2 |  1 |    |    |    |    |    |
| 3        |    |  . |  . |  . |  . |  6 |  5 |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |    |
| 4        |    |    |    |    |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  4 |  3 |  2 |  1 |    |


| Ready Q  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
|          |    |  3 |  3 |  3 |  3 |  2 |  2 |  2 |  2 |  2 |  2 |  4 |  4 |  4 |    |    |    |    |    |
|          |    |    |  2 |  2 |  2 |  4 |  4 |  4 |  4 |  4 |  4 |    |    |    |    |    |    |    |    |
|          |    |    |    |    |  4 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |


| Proceso  | T. Espera | T. Retorno | 
| -------- | --------- | ---------- | 
|  1       |      0    |  5         |
|  2       |      9    | 12         |
|  3       |      4    | 10         |
|  4       |     10    | 14         |
| -------- | --------- | ---------- |
| TOTALES  |   23      |    41      |
| PROMEDIO | **5.75**  | **10.25**  |




## SJF

| Proceso  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
| 1        |  5 |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
| 2        |    |    |  . |  . |  . |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |    |
| 3        |    |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  6 |  5 |  4 |  3 |  2 |  1 |    |
| 4        |    |    |    |    |  . |  . |  . |  . |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |


| Ready Q  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
|          |    |  3 | 2* |  2 |  2 |  4 |  4 |  4 |  3 |  3 |  3 |  3 |    |    |    |    |    |    |    |
|          |    |    |  3 |  3 |  4 |  3 |  3 |  3 |    |    |    |    |    |    |    |    |    |    |    |
|          |    |    |    |    |  3 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |

*: define FIFO


| Proceso  | T. Espera | T. Retorno | 
| -------- | --------- | ---------- | 
|  1       |      0    |  5         |
|  2       |      3    |  6         |
|  3       |     11    | 17         |
|  4       |      4    |  8         |
| -------- | --------- | ---------- |
| TOTALES  |   18      |    36      |
| PROMEDIO | **4.50**  |   **9**    |



## RR (q=3)

| Proceso  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
| 1        |  5 |  4 |  3 |  . |  . |  . |  . |  . |  . |  2 |  1 |    |    |    |    |    |    |    |    |
| 2        |    |    |  . |  . |  . |  . |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |
| 3        |    |  . |  . |  6 |  5 |  4 |  . |  . |  . |  . |  . |  . |  . |  . |  3 |  2 |  1 |    |    |
| 4        |    |    |    |    |  . |  . |  . |  . |  . |  . |  . |  4 |  3 |  2 |  . |  . |  . |  1 |    |


| Ready Q  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
|          |    |  3 |  3 |  2 |  2 |  2 |  1 |  1 |  1 |  4 |  4 |  3 |  3 |  3 |  4 |  4 |  4 |    |    |
|          |    |    |  2 |  1 |  1 |  1 |  4 |  4 |  4 |  3 |  3 |    |    |    |    |    |    |    |    |
|          |    |    |    |    |  4 |  4 |  3 |  3 |  3 |    |    |    |    |    |    |    |    |    |    |


| Proceso  | T. Espera | T. Retorno | 
| -------- | --------- | ---------- | 
|  1       |      6    | 11         |
|  2       |      4    |  7         |
|  3       |     10    | 16         |
|  4       |     10    | 14         |
| -------- | --------- | ---------- |
| TOTALES  |   30      |    48      |
| PROMEDIO | **7.50**  |  **12**    |





## Prio (non-preemptive)
| Proceso  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
| 1        |  5 |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
| 2        |    |    |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  3 |  2 |  1 |    |
| 3        |    |  . |  . |  . |  . |  . |  . |  . |  . |  6 |  5 |  4 |  3 |  2 |  1 |    |    |    |    |
| 4        |    |    |    |    |  . |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |


| Ready Q  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
|          |    |  3 |  3 |  3 |  4 |  3 |  3 |  3 |  3 |  2 |  2 |  2 |  2 |  2 |  2 |    |    |    |    |
|          |    |    |  2 |  2 |  3 |  2 |  2 |  2 |  2 |    |    |    |    |    |    |    |    |    |    |
|          |    |    |    |    |  2 |    |    |    |    |    |    |    |    |    |    |    |    |    |    |




| Proceso  | T. Espera | T. Retorno | 
| -------- | --------- | ---------- | 
|  1       |      0    |  5         |
|  2       |     13    | 16         |
|  3       |      8    | 14         |
|  4       |      1    |  5         |
| -------- | --------- | ---------- |
| TOTALES  |   22      |    40      |
| PROMEDIO | **5.50**  |  **10**    |


## Prio (preemptive)

| Proceso  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
| 1        |  5 |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  4 |  3 |  2 |  1 |    |    |    |    |
| 2        |    |    |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  . |  3 |  2 |  1 |    |
| 3        |    |  6 |  5 |  4 |  . |  . |  . |  . |  3 |  2 |  1 |    |    |    |    |    |    |    |    |
| 4        |    |    |    |    |  4 |  3 |  2 |  1 |    |    |    |    |    |    |    |    |    |    |    |

| Ready Q  |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 |
| -------- |  - |  - |  - |  - |  - |  - |  - |  - |  - |  - | -- | -- | -- | -- | -- | -- | -- | -- | -- |
|          |    |  1 |  1 |  1 |  3 |  3 |  3 |  3 |  1 |  1 |  1 |  2 |  2 |  2 |  2 |    |    |    |    |
|          |    |    |  2 |  2 |  1 |  1 |  1 |  1 |  2 |  2 |  2 |    |    |    |    |    |    |    |    |
|          |    |    |    |    |  2 |  2 |  2 |  2 |    |    |    |    |    |    |    |    |    |    |    |





| Proceso  | T. Espera | T. Retorno | 
| -------- | --------- | ---------- | 
|  1       |     10    | 15         |
|  2       |     13    | 16         |
|  3       |      4    | 10         |
|  4       |      0    | 14         |
| -------- | --------- | ---------- |
| TOTALES  |  27       |  45        |
| PROMEDIO | **6.75**  | **11.25**  |




## Comparación

| Algoritmo                 | T. Espera Promedio | T. Retorno  Promedio |
| ------------------------- | -------------------| -------------------- |
| FCFS                      |       5.75         |      10.25           |
| SJF                       |       4.50         |       9              |
| Round Robin (Quantum 3)   |       7.50         |      12              |
| Prioridad no-expropiativo |       5.50         |      10              |
| Prioridad expropiativo    |       6.75         |      11.25           |