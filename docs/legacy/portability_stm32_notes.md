# Notas de portabilidad a STM32 Bluepill

Estado de portabilidad de la navegacion hacia STM32F103 Bluepill.

## Modulos portables actuales

La logica portable esta en:

- `nav/nav_core.h`
- `nav/nav_core.c`
- `nav/nav_map.h`
- `nav/nav_map.c`
- `nav/nav_types.h`
- `nav/nav_flood.h`
- `nav/nav_flood.c`
- `nav/nav_frontier_eval.h`
- `nav/nav_frontier_eval.c`
- `nav/nav_goal_return_eval.h`
- `nav/nav_goal_return_eval.c`
- `nav/nav_supervisor.h`
- `nav/nav_supervisor.c`
- `nav/pid_controller.h`
- `nav/pid_controller.c`

Caracteristicas:

- C/C++ portable sin Qt;
- sin `malloc/free`;
- sin `float/double` en `nav/`;
- arrays fijos;
- tipos de ancho fijo;
- fixed-point Q16.16 para PID;
- workspaces estaticos para planner/flood/evaluadores;
- supervisor portable para mision, SMART y retorno.

## Modulos no portables

No deben ir directo al firmware:

- `app/mainwindow.h`;
- `app/mainwindow.cpp`;
- `sim/sim_world.*`;
- `sim/sim_robot.*`;
- UI Qt;
- overlay;
- parsing JSON del simulador;
- teclado;
- telemetria UI.

`MainWindow` hoy es el adaptador Qt: aplica requests, arranca primitivas, ejecuta cola,
resetea yaw del simulador y corre tests/batch.

## Estado por modulo

### `nav_core`

Portable. Incluye:

- primitivas;
- deteccion de especiales;
- cola de plan;
- planner BFS orientado;
- `nav_core_route_eval_to_cell_with_dir_mask(...)`;
- `nav_core_route_plan_to_cell_with_dir_mask(...)`.

El workspace del planner es estatico y no reentrante. Es aceptable para control
single-thread, pero debe medirse en SRAM.

### `nav_map`

Portable. Mantiene:

- mapa `16 x 16` maximo;
- visitadas;
- paredes conocidas/presentes;
- celda/orientacion;
- especiales.

### `nav_flood`

Portable. Calcula costos por celda con arrays fijos.

### `nav_frontier_eval`

Portable. Evalua fronteras clasicas con flood para `Shift+F`/debug. No ejecuta
acciones.

### `nav_goal_return_eval`

Portable. Evalua retorno optimista hacia inicio:

- C puro;
- arrays fijos;
- sin heap;
- sin `float/double`;
- Dijkstra/flood optimista O(N^2) sobre grilla maxima `16 x 16`;
- revisa paredes compartidas desde ambos lados;
- bloquea paredes conocidas presentes;
- permite paredes/celdas desconocidas con penalty y presupuesto;
- distingue ruta optimista general de ruta que usa desconocido;
- devuelve primera frontera util.

Riesgo principal: costo CPU si se evalua demasiado seguido. En el flujo actual se evalua
en puntos de decision del retorno, no en cada tick de control.

### `nav_supervisor`

Portable. Controla:

- mision modo 1;
- SMART_RECOGNITION;
- retorno seguro;
- `FINAL_SAFE_SCAN_RETURN`;
- `GOAL_DIRECTED_RETURN_SHADOW`;
- `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`;
- requests y debug snapshots.

No llama Qt ni simulacion. En firmware una HAL/adaptador debe aplicar sus requests.

### `pid_controller`

Portable con Q16.16. Usa `int64_t` para multiplicacion/division intermedia.

## Riesgos para Bluepill

### SRAM

Vigilar:

- mapa;
- workspace BFS;
- workspaces flood/evaluadores;
- cola de plan;
- debug snapshots;
- buffers de firmware.

Recomendaciones:

- medir `.bss` y `.data` con linker map;
- compilar telemetria pesada solo en debug;
- bajar `NAV_MAP_MAX_WIDTH/HEIGHT` si el laberinto real lo permite.

### CPU

Riesgos:

- `int64_t` en PID;
- divisiones enteras;
- planner BFS;
- `nav_goal_return_eval` O(N^2);
- flood si se recalcula demasiado.

Recomendaciones:

- mantener `dt` fijo;
- medir peor caso;
- evaluar goal-directed solo en puntos de decision;
- precomputar escalas si tuning queda fijo.

### Sensores reales

El simulador es limpio. En robot real hacen falta:

- calibracion IR;
- filtros simples;
- debouncing de piso;
- validacion de yaw/gyro;
- thresholds revisados con datos reales.

## Que falta para firmware real

- HAL STM32 para sensores, yaw, tiempo y motores.
- Adaptador que aplique requests de `nav_supervisor`.
- Telemetria serial compacta.
- Config/tuning sin Qt.
- Tests unitarios fuera de Qt.
- Medicion de SRAM/CPU.
- Watchdog/timeouts reales.

## Orden recomendado de bring-up

1. Sensores de piso.
2. Yaw/gyro.
3. Motores/PWM.
4. `ADVANCE_LINE` yaw-only.
5. Wall assist lateral.
6. Smooth turns.
7. `nav_map`.
8. Deteccion de especiales.
9. Cola de plan.
10. Planner BFS.
11. `nav_supervisor` con SMART.
12. Retorno seguro.
13. `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION` con limites conservadores.

## Recomendacion actual

El codigo portable ya contiene la logica de decision principal. La parte pendiente no es
redisenar `nav/`, sino crear una HAL/adaptador STM32 que reemplace el trabajo que hoy
hace `MainWindow`.
</file>
