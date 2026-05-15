# Estado actual del modo 1 de navegacion

Este documento describe el estado real inspeccionado del modo 1. Separa lo que esta implementado de ideas futuras y usa los nombres actuales de archivos, enums, structs, funciones y telemetria cuando son relevantes.

## Objetivo del modo 1

El modo 1 busca reconocer el laberinto de forma incremental:

- navegar con percepcion actual de paredes;
- mantener un mapa logico sombra en `nav_map`;
- detectar celdas especiales por sensores de piso;
- explorar celdas no visitadas;
- volver automaticamente a una frontera de exploracion cuando queda rodeado de celdas ya visitadas;
- terminar cuando no quedan fronteras alcanzables (`NO_FRONTIER`).

El mapa logico ya se usa para algunas decisiones de exploracion y planificacion, pero no existe todavia flood fill ni modo 2.

## Arquitectura general

### Simulador Qt y MainWindow

Archivos principales:

- `app/mainwindow.h`
- `app/mainwindow.cpp`

`MainWindow` no es portable. Orquesta:

- ciclo de simulacion;
- lectura de `SimRobot` y `SimWorld`;
- construccion de `RobotSensors`;
- llamadas a `nav_core_update`;
- arranque de primitivas con `nav_core_start_*`;
- reseteo de referencia de yaw del simulador;
- ejecucion de planes;
- secuencias compuestas;
- autonomia basica;
- overlay del mapa;
- telemetria;
- ventana `Control Tuning`.

### nav_core portable

Archivos:

- `nav/nav_core.h`
- `nav/nav_core.c`
- `nav/nav_types.h`

`nav_core` contiene:

- maquina de estado de acciones;
- control de movimiento portable;
- percepcion de paredes;
- politicas de recomendacion;
- cola FIFO de acciones planificadas;
- planner BFS a celda;
- planner BFS a frontera;
- deteccion de celdas especiales;
- debug snapshots y telemetria portable.

No depende de Qt, no usa `malloc/free` y no usa `float/double` en `nav/*`.

### nav_map portable

Archivos:

- `nav/nav_map.h`
- `nav/nav_map.c`

`nav_map` mantiene:

- celda logica actual;
- orientacion discreta `NavMapDirection`;
- celdas visitadas;
- paredes conocidas;
- paredes presentes;
- celdas especiales detectadas;
- contadores de pose y paredes;
- debug snapshot del mapa.

### pid_controller portable

Archivos:

- `nav/pid_controller.h`
- `nav/pid_controller.c`

Usa fixed-point Q16.16 (`q16_16_t`) y operaciones con `int64_t` para multiplicacion/division intermedia.

### SimWorld y SimRobot

Archivos:

- `sim/sim_world.h`
- `sim/sim_world.cpp`
- `sim/sim_robot.h`
- `sim/sim_robot.cpp`

Son simulacion no portable:

- geometria del mundo;
- paredes y cintas;
- marcas especiales desde JSON;
- robot cinematico;
- sensores simulados;
- `double` para UI/simulacion.

## Flujo general de navegacion automatica

1. `MainWindow` lee el mundo y el robot simulado.
2. Construye `RobotSensors`.
3. Llama a `nav_core_update(...)`.
4. Si una accion termina, `nav_core` aplica la politica de actualizacion de mapa.
5. `MainWindow` mira estado/action y decide si debe arrancar otra accion.
6. En autonomia (`B`), `MainWindow::advanceBasicNavAutonomyIfNeeded()` usa la politica actual.
7. Antes de arrancar una primitiva, `MainWindow` ajusta la referencia de yaw del simulador.
8. `X` cancela autonomia, plan, secuencias compuestas y accion actual.

Atajos importantes:

- `B`: autonomia.
- `P`: cambia politica.
- `Y`: overlay del mapa sombra.
- `K`: plan dry-run a celda.
- `T`: plan dry-run a frontera.
- `J`: ejecutar ruta cargada, con validaciones fisicas.
- `U`: ruta de prueba.
- `F3`: `Control Tuning`.

## Politicas disponibles

Enum real: `NavPolicy`.

| Politica | Estado actual |
| --- | --- |
| `NAV_POLICY_RIGHT_HAND_RULE` | Implementada. Regla mano derecha con percepcion actual. |
| `NAV_POLICY_MAP_PREFER_UNVISITED` | Implementada. Prefiere vecinas libres no visitadas en orden derecha, frente, izquierda. Si no encuentra, cae a mano derecha. |
| `NAV_POLICY_SMART_RECOGNITION` | Implementada en orquestacion de `MainWindow`. Usa accion local hacia no visitada; si no hay, planifica a frontera y ejecuta la cola. |

### RIGHT_HAND_RULE

Si `floor_rear_black == false`:

- sin pared frontal: `NAV_RECOMMENDED_ACQUIRE_REAR_LINE`;
- con pared frontal: `NAV_RECOMMENDED_RECOVERY_PIVOT_180_FRONT_BLOCKED`.

Si `floor_rear_black == true`:

- derecha libre: `NAV_RECOMMENDED_SMOOTH_RIGHT`;
- frente libre: `NAV_RECOMMENDED_ADVANCE_LINE`;
- izquierda libre: `NAV_RECOMMENDED_SMOOTH_LEFT`;
- todo bloqueado: `NAV_RECOMMENDED_PIVOT_180`.

### MAP_PREFER_UNVISITED

Usa la pose logica del mapa para mirar vecinas:

1. derecha;
2. frente;
3. izquierda.

Solo elige una salida si esta libre, la vecina esta dentro del mapa y `visited == false`. No elige volver hacia atras por mapa. Si no encuentra vecina no visitada inmediata, usa fallback de mano derecha.

### SMART_RECOGNITION

Estado debug: `smart_recognition_state`.

Flujo:

1. Si hay plan en ejecucion, lo deja avanzar.
2. Si hay accion local hacia vecina no visitada, la ejecuta.
3. Si no hay vecina no visitada inmediata, llama a `nav_core_route_plan_to_nearest_frontier()`.
4. Si hay ruta, activa `plan_execution_enabled` y ejecuta la cola.
5. Al llegar a la frontera, vuelve a decidir localmente.
6. Si no hay frontera, detiene autonomia con estado `NO_FRONTIER`.

## Mapa logico

Structs principales:

- `NavMap`.
- `NavMapCell`.
- `NavMapDebugSnapshot`.

Campos relevantes de celda:

- `visited`.
- `special_detected`.
- `walls_known`.
- `walls_present`.

La orientacion discreta usa `NavMapDirection`:

- `NAV_MAP_DIR_NORTH`.
- `NAV_MAP_DIR_EAST`.
- `NAV_MAP_DIR_SOUTH`.
- `NAV_MAP_DIR_WEST`.

El mapa se inicializa con `nav_core_map_init(...)` y `nav_map_init(...)`:

- marca la celda inicial como visitada;
- habilita mapa;
- deja pendiente snapshot inicial de paredes;
- deja pendiente snapshot inicial de especial.

El mapa sombra se muestra en overlay desde `MainWindow`.

## Cola FIFO de acciones planificadas

Enum real: `NavPlanAction`.

Acciones actuales:

- `NAV_PLAN_ACTION_NONE`.
- `NAV_PLAN_ACTION_ADVANCE_LINE`.
- `NAV_PLAN_ACTION_SMOOTH_LEFT`.
- `NAV_PLAN_ACTION_SMOOTH_RIGHT`.
- `NAV_PLAN_ACTION_PIVOT_180`.
- `NAV_PLAN_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT`.
- `NAV_PLAN_ACTION_CENTER_AND_PIVOT_180`.

Funciones publicas:

- `nav_core_plan_clear()`.
- `nav_core_plan_push(...)`.
- `nav_core_plan_count()`.
- `nav_core_plan_is_empty()`.
- `nav_core_plan_peek_next()`.
- `nav_core_plan_pop_next()`.
- `nav_core_plan_debug_snapshot(...)`.

La cola es FIFO, fija y sin memoria dinamica. `NAV_PLAN_MAX_ACTIONS = 64`.

## Planner BFS

El planner esta en `nav_core.c`.

Estados:

```text
(cell_x, cell_y, dir)
```

Acciones permitidas por BFS:

- `ADVANCE_LINE`: avanza una celda en `dir`.
- `SMOOTH_RIGHT`: gira a la derecha y avanza una celda.
- `SMOOTH_LEFT`: gira a la izquierda y avanza una celda.
- `CENTER_AND_PIVOT_180`: no cambia celda y cambia `dir` a la opuesta.

Reglas de cruce:

- la pared en direccion de cruce debe ser conocida;
- la pared debe estar ausente;
- la celda destino debe estar dentro del mapa;
- la celda destino debe tener `visited == true`.

El target de `nav_core_route_plan_to_cell(...)` tambien debe estar visitado; si no, devuelve `NAV_ROUTE_STATUS_TARGET_NOT_VISITED`.

### Workspace estatico

El BFS usa `NavRouteWorkspace` estatico en `nav_core.c`:

- `visited[NAV_ROUTE_MAX_STATES]`;
- `parent[NAV_ROUTE_MAX_STATES]`;
- `parent_action[NAV_ROUTE_MAX_STATES]`;
- `queue[NAV_ROUTE_MAX_STATES]`;
- `reverse_actions[NAV_PLAN_MAX_ACTIONS]`.

`NAV_ROUTE_MAX_STATES = NAV_MAP_MAX_WIDTH * NAV_MAP_MAX_HEIGHT * 4`.

El workspace no es reentrante. Esto es intencional para evitar buffers grandes en stack y facilitar portabilidad a STM32.

### Planificacion a frontera

Funcion:

- `nav_core_route_plan_to_nearest_frontier()`.

Una frontera util es una celda visitada con una salida:

- pared conocida;
- pared ausente;
- vecina dentro del mapa;
- vecina no visitada.

El estado objetivo debe orientar esa salida hacia:

- frente;
- derecha;
- izquierda.

No se considera una salida hacia atras como frontera directamente util.

Limitaciones actuales:

- no hay costos distintos;
- no hay flood fill;
- no hay Dijkstra;
- no hay pivots 90 generales;
- `CENTER_AND_PIVOT_180` es la unica reorientacion general en ruta;
- despues de `CENTER_AND_PIVOT_180`, el planner evita `SMOOTH_LEFT/RIGHT` inmediatamente y permite `ADVANCE_LINE` o terminar.

## Acciones y primitivas

Enum real: `NavAction`.

| Accion | Estado |
| --- | --- |
| `NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK` | Implementada. Avance hasta cinta trasera. Tiene modos `NAV_ADVANCE_START_REAR_LINE` y `NAV_ADVANCE_START_CENTERED_POSE`. |
| `NAV_ACTION_SMOOTH_TURN_LEFT` | Implementada. Curva con yaw-rate PI y fase final recta. |
| `NAV_ACTION_SMOOTH_TURN_RIGHT` | Implementada. Curva con yaw-rate PI y fase final recta. |
| `NAV_ACTION_PIVOT_TURN_LEFT` | Implementada como pivot in-cell logico. |
| `NAV_ACTION_PIVOT_TURN_RIGHT` | Implementada como pivot in-cell logico. |
| `NAV_ACTION_PIVOT_TURN_180` | Implementada como pivot in-cell logico. |
| `NAV_ACTION_CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE` | Implementada. Avanza hasta que `floor_front_black` detecta la proxima cinta, con brake settle. |
| `NAV_ACTION_APPROACH_FRONT_WALL_FOR_PIVOT` | Implementada. Acerca a pared frontal hasta target o timeout, con brake settle. |

`CENTER_AND_PIVOT_180` no es `NavAction`; es `NavPlanAction` compuesta. `MainWindow` la ejecuta como:

- si hay pared frontal: `APPROACH_FRONT_WALL_FOR_PIVOT -> PIVOT_180`;
- si no hay pared frontal: `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE -> PIVOT_180`.

## Reglas de actualizacion del mapa por accion

`map_update_count` representa cambios logicos de pose/celda/orientacion. `map_wall_update_count` representa snapshots de paredes.

Reglas logicas actuales:

| Accion completada | Celda | Direccion | Paredes |
| --- | --- | --- | --- |
| `ADVANCE_UNTIL_REAR_BLACK` | `cell = cell + dir` | igual | registra paredes |
| `SMOOTH_TURN_RIGHT` | `cell = cell + right(dir)` | derecha | registra paredes |
| `SMOOTH_TURN_LEFT` | `cell = cell + left(dir)` | izquierda | registra paredes |
| `PIVOT_TURN_180` | no cambia | opuesta | no registra paredes |
| `PIVOT_TURN_RIGHT` | no cambia | derecha | no registra paredes |
| `PIVOT_TURN_LEFT` | no cambia | izquierda | no registra paredes |
| `APPROACH_FRONT_WALL_FOR_PIVOT` | no cambia | no cambia | no registra paredes |
| `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE` | no cambia | no cambia | no registra paredes |

La foto inicial de paredes se toma con `map_initial_wall_snapshot_pending`, sin exigir `floor_rear_black`.

## Deteccion de celdas especiales

Las marcas especiales son negras igual que las cintas. `nav_core` no recibe informacion magica del JSON. Detecta usando:

```text
floor_front_black && floor_rear_black
```

El JSON define marcas fisicas en `SimWorld`:

```json
"special_cells": [
  { "cell_x": 2, "cell_y": 3, "size_mm": 120 }
]
```

Si falta `size_mm`, el default actual es `120`.

### Deteccion normal

Contextos internos:

- `NAV_SPECIAL_DETECT_TRANSLATION_TO_NEXT_CELL`.
- `NAV_SPECIAL_DETECT_IN_CELL_AUX_TRANSLATION`.
- `NAV_SPECIAL_DETECT_DISABLED`.

Durante `ADVANCE_LINE` y `SMOOTH_LEFT/RIGHT`:

- no se confirma durante salida de la linea inicial;
- se requiere que la accion haya comenzado desde una linea trasera confiable;
- se usa ventana temporal de deteccion;
- `special_ignore_rear_until_white` evita que la marca central corte el avance como si fuera linea destino.

### Deteccion en smooth

Durante smooths se permite detectar especiales en traslacion. La telemetria expone:

- `special_mark_target_cell`;
- `special_mark_target_source`;
- `last_special_mark_action`.

Esto ayuda a distinguir si se marco la celda actual, destino de smooth o una fuente invalida.

### Deteccion en maniobras auxiliares

Esta habilitada para:

- `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE`;
- `APPROACH_FRONT_WALL_FOR_PIVOT`.

En estos casos se marca la celda logica actual y no se modifica el criterio de finalizacion de la maniobra.

No se detectan especiales durante pivots puros.

### Arranque ambiguo sobre marca especial

Existe snapshot inicial de especial:

- `initial_special_snapshot_pending`;
- `initial_special_snapshot_done`.

Si en el primer update ambos sensores de piso estan en negro, se marca la celda inicial como especial.

Para no tratar una marca especial como punto de decision se usa:

- `rear_line_trusted_for_decision`;
- `rear_line_trust_source`.

La autonomia no debe iniciar decisiones locales solo por `floor_rear_black == true` si la linea trasera no es confiable.

## Control de movimiento

### Smooth yaw-rate PI

La curva principal de `SMOOTH_LEFT/RIGHT` usa yaw-rate PI. La fase principal no usa wall assist ni diagonal guidance.

### Fase final del smooth

Fase real:

- `NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE`.

Prioridad actual:

1. Diagonales.
2. Wall hold lateral dinamico.
3. Yaw hold relativo.

Modo diagonal configurable:

- `NAV_SMOOTH_FINAL_DIAG_MODE_HOLD_RELATIVE`;
- `NAV_SMOOTH_FINAL_DIAG_MODE_SETPOINT`.

Default actual:

- `SETPOINT`.

### Diagonal guidance

Config:

- `NavDiagonalGuidanceConfig`.

En modo `SETPOINT`:

- `DIAG_CENTER`: `diag_right_mm - diag_left_mm`.
- `DIAG_LEFT`: `2 * (diag_target_mm - diag_left_mm)`.
- `DIAG_RIGHT`: `2 * (diag_right_mm - diag_target_mm)`.

En modo `HOLD_RELATIVE`:

- `DIAG_CENTER_HOLD`: mantiene diferencia capturada.
- `DIAG_LEFT_HOLD`: mantiene distancia izquierda capturada.
- `DIAG_RIGHT_HOLD`: mantiene distancia derecha capturada.

El control diagonal usa Kp/Kd/limite propios. El tuning actual es P-only.

### Advance yaw PD y wall PD

`ADVANCE_LINE` usa:

1. `WALL_CENTER`, `WALL_LEFT`, `WALL_RIGHT` si hay referencia lateral confirmada.
2. `WALL_LEFT_CAUTION` o `WALL_RIGHT_CAUTION` si no hay pared confirmada y hay cautela activa.
3. Preview diagonal si el latch frontal esta activo.
4. `YAW_ONLY` con yaw hold relativo.

El wall PD normal usa targets fijos `60 mm`.

### Preview diagonal de ADVANCE_LINE

Se aplica solo en `ADVANCE_LINE`, fase `SEEK_TARGET_LINE`.

Mecanica:

- `advance_front_diag_preview_armed` se arma despues de ver `floor_front_black == false`;
- `advance_front_diag_preview_latched` se activa cuando luego aparece `floor_front_black == true` con `floor_rear_black == false`;
- el latch queda activo hasta terminar/cancelar la accion;
- laterales confirmados o cautela tienen prioridad.

El armado evita falsos disparos con marcas especiales centrales o celdas especiales contiguas.

### Wall caution

Aplica solo en `ADVANCE_LINE`.

Estados por lado:

- `NAV_WALL_CAUTION_CONFIDENCE_LOST`;
- `NAV_WALL_CAUTION_CONFIDENCE_CONFIRMED`;
- `NAV_WALL_CAUTION_CONFIDENCE_CAUTION`.

Entrada a `CAUTION`:

- venia confirmada;
- se pierde la confirmacion diagonal;
- el lateral sigue valido.

En `CAUTION`:

- se mantiene la distancia lateral capturada;
- usa Kp/Kd/limite propios;
- no intenta recentrar a target absoluto.

Salida:

- vuelve a `CONFIRMED` si retorna la confirmacion diagonal;
- pasa a `LOST` si se pierde lateral, salta mas que `delta_max_mm`, vence timeout, termina accion o se deshabilita.

### Smooth yaw carry

Config:

- `NavSmoothYawCarryConfig`.

Fuentes de candidato:

- `NAV_YAW_CARRY_CANDIDATE_SMOOTH_FINAL_DIAG`;
- `NAV_YAW_CARRY_CANDIDATE_ADVANCE_FRONT_DIAG_PREVIEW`.

Si una fase con diagonales cambia la orientacion y la siguiente accion inmediata es `SMOOTH_LEFT` o `SMOOTH_RIGHT`, `MainWindow` puede iniciar el smooth con referencia corregida en vez de resetear el yaw actual como cero.

`MainWindow::resetNavigationYawReferenceForSmoothStart()` consume el offset pendiente y evita borrar la compensacion.

## Telemetria importante

Grupos utiles:

- `Pinned debug`.
- `Pose`.
- `Floor sensors`.
- `Nav perception`.
- `Advance debug`.
- `Smooth debug`.
- `Special detection`.
- `Logical map`.
- `Route planner`.
- `Frontier planner`.
- `Smart recognition`.
- `Plan executor`.
- `Motor command`.

Variables clave:

- `nav_policy`.
- `nav_action`.
- `nav_recommended_action`.
- `nav_last_decision`.
- `smart_recognition_state`.
- `logical_cell_x`, `logical_cell_y`, `logical_dir`.
- `map_update_count`, `map_wall_update_count`.
- `current_cell_visited`, `current_cell_special`.
- `special_candidate`, `special_confirmed`.
- `special_mark_target_cell`, `special_mark_target_source`.
- `rear_line_trusted_for_decision`.
- `route_status`, `frontier_route_status`.
- `plan_execution_enabled`, `plan_queue_count`.
- `plan_current_action`, `plan_next_action`.
- `plan_composite_prepare_method`.
- `advance_final_correction_source`.
- `advance_front_diag_preview_armed`.
- `advance_front_diag_preview_latched`.
- `smooth_final_guidance_source`.
- `smooth_final_diag_mode`.
- `smooth_yaw_carry_candidate_pending`.
- `smooth_yaw_carry_used`.
- `wall_left_confidence`, `wall_right_confidence`.

## Lo que sigue en MainWindow y podria portarse luego

Todavia esta en `MainWindow`:

- orquestacion completa de `SMART_RECOGNITION`;
- ejecucion fisica de la cola;
- secuencias compuestas;
- seleccion de preparacion para `CENTER_AND_PIVOT_180`;
- reseteo/aplicacion de referencia de yaw del simulador;
- ayuda de controles y telemetria UI.

Esto funciona para simulador, pero una parte deberia migrarse a una capa portable antes del firmware final.

## Limitaciones conocidas

- No hay flood fill.
- No hay modo 2 final.
- El planner usa BFS de costo uniforme.
- No hay pivots 90 generales en rutas.
- `CENTER_AND_PIVOT_180` es robusto, pero fisicamente mas lento que una ruta con turns suaves.
- Las decisiones siguen dependiendo de la confiabilidad de `rear_line_trusted_for_decision`.
- Los valores de sensores IR y controles estan tuneados para simulador.
- El planner no es reentrante por usar workspace estatico.
- El mapa maximo portable actual es `16 x 16`.
