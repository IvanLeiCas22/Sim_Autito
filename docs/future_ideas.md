# Ideas futuras no implementadas

Este documento lista trabajo futuro. No describe comportamiento garantizado salvo cuando
se indica explicitamente como estado actual.

## Retorno inteligente: mejoras futuras

Estado actual:

- `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION` esta implementado y es default.
- Evalua ruta optimista hacia inicio.
- Entra una celda desconocida por intento.
- Recalcula entre entradas.
- Mantiene fallback seguro.

Ideas pendientes:

- overlay especifico para goal-directed:
  - camino optimista;
  - primera frontera;
  - presupuesto desconocido;
  - intentos consumidos;
- exportar mas telemetria `goal_exec_*` al JSON de batch;
- Autocheck especifico:
  - `request_goal_enter_frontier -> nav_action` inicia;
  - entrada completada -> celda esperada;
  - fallback -> cola/plan desactivados;
- blacklist temporal de fronteras fallidas si aparece loop;
- soporte de entrada `BACK` con maniobra segura;
- penalizacion de smooth turns o pivots si se requiere scoring mas realista;
- presupuesto adaptativo segun distancia segura al inicio.

## Modo 2

Ideas:

- usar mapa completo;
- planificar ruta a objetivo;
- optimizar secuencia de acciones;
- elegir rutas por costo real y no solo BFS uniforme;
- usar celdas especiales como objetivos o checkpoints.

Estado:

- no implementado;
- el planner actual y `nav_goal_return_eval` son base util, pero siguen siendo modo 1.

## Planner con costos por accion

Actual:

- BFS orientado de costo uniforme;
- `nav_flood` por celda;
- `nav_goal_return_eval` usa flood optimista por celda;
- route eval por mascara de orientacion final.

Ideas futuras:

- Dijkstra/A* orientado;
- penalizar `CENTER_AND_PIVOT_180`;
- penalizar/bonificar smooths;
- costo por tiempo estimado, no solo cantidad de acciones;
- comparar ruta segura e intentos goal-directed con la misma metrica.

## Pivots 90 generales

Actualmente el planner no usa pivots 90 generales.

Ideas:

- `CENTER_AND_PIVOT_LEFT`;
- `CENTER_AND_PIVOT_RIGHT`;
- reorientacion in-cell sin avanzar;
- restricciones fisicas segun `rear_line_trusted_for_decision`.

Estado:

- no implementado.

## Recalibracion fisica avanzada

Ideas:

- acercarse a pared frontal;
- alinear con sensores frontales;
- pivotar hacia pared lateral util;
- alinear lateralmente;
- pivot final.

Objetivo:

- reducir error angular/posicional antes de rutas largas o modo 2.

Estado:

- no implementado como primitiva dedicada.

## Confianza de paredes mas avanzada

Actual:

- `Wall caution` existe para `ADVANCE_LINE`;
- las paredes conocidas presentes bloquean planners/evaluadores;
- paredes desconocidas pueden ser transitables optimistas solo en evaluacion
  goal-directed.

Ideas:

- confianza continua por lado;
- hysteresis por distancia;
- confirmacion temporal multi-tick;
- mapear incertidumbre en `nav_map`;
- usar confianza para scoring goal-directed.

## Filtros para sensores reales

Ideas:

- filtro mediana o IIR para IR;
- debouncing de piso;
- rechazo de saltos imposibles;
- calibracion por tabla;
- telemetria de ruido real.

Estado:

- no implementado en `nav_core`;
- necesario antes de firmware real.

## Mover mas adaptacion fuera de `MainWindow`

`MainWindow` sigue manejando:

- ejecucion fisica de cola;
- secuencias compuestas;
- seleccion `FRONT_LINE` vs `FRONT_WALL`;
- validaciones manuales de `J`;
- referencia de yaw del simulador;
- UI/overlay/telemetria.

Ideas:

- capa portable de ejecutor de cola;
- capa portable de secuencias compuestas;
- HAL comun para Qt y STM32.

Estado:

- pendiente; por ahora `MainWindow` funciona como adaptador.

## Persistencia de tuning

La UI `F3` permite editar valores en runtime.

Ideas:

- guardar perfiles;
- cargar defaults por mapa;
- exportar config para firmware;
- comparar perfiles simulador/robot real.

Estado:

- no implementado.

## Tests automaticos portables

Actual:

- `Shift+R`;
- `Shift+B`;
- Fast Batch Mode;
- Autocheck Monitor;
- export CSV/JSON.

Ideas:

- tests unitarios para `nav_map`;
- tests de planner BFS;
- tests para `nav_goal_return_eval`;
- simulaciones headless;
- fixtures de mapas con expected_fail fuera de `data/test_maps`.

## Telemetria firmware

Ideas:

- protocolo serial compacto;
- snapshots binarios por modulo;
- flags de debug compilables;
- muestreo reducido para no saturar UART.

Estado:

- no implementado para STM32.

## HAL STM32

Pendiente principal para robot real:

- sensores IR;
- sensores de piso;
- yaw/gyro;
- timers;
- PWM/motores;
- botones/comandos;
- telemetria serial;
- build separado sin Qt, `MainWindow`, `SimWorld` ni `SimRobot`.
