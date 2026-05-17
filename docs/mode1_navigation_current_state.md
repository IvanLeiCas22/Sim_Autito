# Estado actual del modo 1 de navegacion

Este documento resume el estado real del modo 1 despues de integrar
`GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`.

## Objetivo del modo 1

El modo 1:

- explora incrementalmente el laberinto;
- mantiene mapa logico en `nav_map`;
- detecta celdas especiales con sensores de piso;
- usa `SMART_RECOGNITION` para explorar celdas no visitadas y volver a fronteras;
- cuando encuentra las especiales requeridas, vuelve al inicio;
- usa retorno inteligente limitado por defecto, con fallback seguro.

## Arquitectura actual

### `MainWindow`

`MainWindow` no decide la mision. Actua como adaptador Qt/UI/simulacion:

- lee `SimWorld` y `SimRobot`;
- calcula sensores simulados;
- construye `RobotSensors`;
- llama `nav_core_update(...)`;
- arranca primitivas;
- ejecuta la cola de planes;
- resetea yaw del simulador antes de acciones;
- carga JSON;
- dibuja overlay/telemetria;
- corre `Shift+R`, `Shift+B`, Fast Test Mode y Autocheck.

### `nav_core`

Contiene:

- `ADVANCE_LINE`;
- `SMOOTH_LEFT/RIGHT`;
- pivots;
- `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE`;
- `APPROACH_FRONT_WALL_FOR_PIVOT`;
- deteccion de especiales;
- cola FIFO de `NavPlanAction`;
- planner BFS orientado;
- `nav_core_route_eval_to_cell_with_dir_mask(...)`;
- `nav_core_route_plan_to_cell_with_dir_mask(...)`.

### `nav_map`

Mantiene:

- celda logica actual;
- orientacion discreta;
- visitadas;
- paredes conocidas/presentes;
- celdas especiales.

### `nav_flood`

Calcula costos por celda hacia un objetivo. Se usa para debug y como base de
evaluaciones portables.

### `nav_frontier_eval`

Evalua fronteras clasicas con flood. `Shift+F` lo usa para debug/overlay. No ejecuta
acciones.

### `nav_goal_return_eval`

Evalua retorno optimista hacia el inicio:

- C puro portable;
- arrays fijos;
- weighted/optimistic flood fill por celdas;
- permite paredes/celdas desconocidas con presupuesto;
- respeta paredes conocidas presentes;
- encuentra la primera frontera util del camino optimista.

### `nav_supervisor`

Controla:

- mision modo 1;
- SMART_RECOGNITION;
- retorno seguro;
- `FINAL_SAFE_SCAN_RETURN`;
- `GOAL_DIRECTED_RETURN_SHADOW`;
- `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`.

## Flujo de navegacion

1. `MainWindow` actualiza sensores y `nav_core`.
2. `nav_core` actualiza accion, mapa, paredes y especiales.
3. `MainWindow` arma inputs para `nav_supervisor`.
4. `nav_supervisor` decide mision/SMART/retorno.
5. `MainWindow` aplica requests como adaptador.
6. `SimRobot` avanza con el mismo `dt` logico.

## SMART_RECOGNITION

SMART esta controlado por `nav_supervisor`.

Flujo:

1. Si hay vecina local no visitada y accion viable, pide accion local.
2. Si no hay salida local, pide plan a frontera.
3. `MainWindow` llama `nav_core_route_plan_to_nearest_frontier()` solo por request.
4. Si hay plan, el supervisor pide ejecutar cola.
5. Si no hay frontera, queda en `NO_FRONTIER`.
6. La mision puede bloquear SMART durante retorno o final scan.

## Mision modo 1

Estados principales:

- `SEARCH_SPECIALS`;
- `FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE`;
- `FINAL_SAFE_SCAN_RETURN_PLAN`;
- `FINAL_SAFE_SCAN_RETURN_EXECUTE`;
- `RETURN_SMART_DECIDE`;
- `RETURN_FRONTIER_PLAN`;
- `RETURN_FRONTIER_EXECUTE`;
- `RETURN_FRONTIER_ENTER`;
- `RETURN_SAFE_PLAN`;
- `RETURN_SAFE_EXECUTE`;
- `DONE`;
- `ERROR`;
- `CANCELLED`.

Si SMART llega a `NO_FRONTIER` con `found < required`, la mision intenta una vez
`FINAL_SAFE_SCAN_RETURN`: vuelve al inicio por ruta conocida permitiendo detectar
especiales durante el retorno. Si llega al inicio sin completar required, falla con
`NO_FRONTIER_BEFORE_REQUIRED_SPECIALS`.

## Retornos

### Retorno seguro

Usa mapa conocido:

- planifica al inicio con BFS orientado;
- ejecuta cola;
- termina en `DONE` si llega a start;
- falla claro si no hay ruta.

### `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`

Es el default actual.

No planifica un camino desconocido completo. En cada decision:

1. calcula costo de retorno seguro conocido;
2. evalua una ruta optimista hacia inicio permitiendo desconocido;
3. identifica la primera frontera del camino optimista;
4. valida ruta conocida hasta `frontier_cell` con mascara de orientaciones;
5. ejecuta esa cola;
6. revalida:
   - celda actual == `frontier_cell`;
   - vecino dentro del mapa;
   - vecino no visitado;
   - pared compartida no conocida presente desde ningun lado;
   - accion de entrada recalculada con orientacion actual;
   - nav/cola libres;
7. entra una sola celda desconocida con `ADVANCE_LINE`, `SMOOTH_LEFT` o
   `SMOOTH_RIGHT`;
8. cuenta intento solo si la primitiva arranca correctamente;
9. al terminar, verifica que la celda actual sea `frontier_neighbor`;
10. recalcula si quedan intentos; si no, usa retorno seguro.

`BACK` no esta soportado por defecto (`goal_allow_back_entry = false`).

Fallback seguro ante:

- decision `FALLBACK_SAFE`;
- intentos agotados;
- ruta a frontera fallida;
- plan no cargado;
- mismatch de `frontier_cell`;
- vecino invalido/visitado;
- pared compartida conocida presente;
- accion `BACK`/unsupported;
- nav ocupado;
- primitiva no inicia;
- entrada no avanza o cae en otra celda;
- cancelacion.

## Configuracion F3 por defecto

- `return_strategy = GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`.
- `goal_required_improvement = 0`.
- `goal_unknown_cell_penalty = 0`.
- `goal_unknown_edge_penalty = 0`.
- `goal_max_unknown_cells = 32`.
- `goal_max_unknown_edges = 32`.
- `goal_min_safe_return_cost_to_try = 4`.
- `goal_max_shortcut_attempts = 32`.
- `goal_allow_back_entry = false`.
- `test_fast_mode_enabled = true`.
- `test_fast_ticks_per_ui_update = 10`.

`goal_max_unknown_cells/edges` afecta al evaluador optimista. `goal_max_shortcut_attempts`
limita entradas reales a celdas desconocidas; siempre se recalcula entre entradas.

## Planner y route eval

El planner BFS usa estados `(cell_x, cell_y, dir)` y acciones:

- `ADVANCE_LINE`;
- `SMOOTH_LEFT`;
- `SMOOTH_RIGHT`;
- `CENTER_AND_PIVOT_180`.

Las rutas reales solo atraviesan celdas visitadas y paredes conocidas ausentes.

APIs relevantes:

- `nav_core_route_plan_to_cell(...)`;
- `nav_core_route_plan_to_nearest_frontier()`;
- `nav_core_route_eval_to_cell_with_dir_mask(...)`;
- `nav_core_route_plan_to_cell_with_dir_mask(...)`.

La version `eval` no carga cola. La version `plan` carga cola si encuentra ruta.

## Acciones y mapa

Actualizacion logica:

- `ADVANCE_LINE`: avanza una celda en direccion actual.
- `SMOOTH_RIGHT`: avanza a la celda derecha y rota derecha.
- `SMOOTH_LEFT`: avanza a la celda izquierda y rota izquierda.
- pivots: cambian orientacion, no celda.
- `CENTER`/`APPROACH`: no cambian celda ni orientacion.

Las especiales se detectan durante avances, smooths y auxiliares habilitadas. Los pivots
puros no detectan especiales.

## Testing y herramientas

- `Shift+R`: test modo 1 del mapa actual.
- `Shift+B`: batch de `data/test_maps`.
- Fast Test Mode: activo por defecto para `Shift+R` y `Shift+B`; ejecuta multiples
  ticks logicos por refresh UI sin cambiar `kSimulationDtS`.
- Autocheck Monitor: corre durante tests y batch; puede fallar un mapa con
  `AUTOCHECK_FAIL`.
- Export CSV/JSON: `data/test_results/`, con wall time, sim time, speedup y failures
  de autocheck.
- `I`: flood hacia inicio.
- `Shift+F`: debug de fronteras flood via `nav_frontier_eval`.

## Telemetria util

Modo 1 y supervisor:

- `supervisor_state`;
- `supervisor_done_reason`;
- `mode1_found_special_count`;
- `mode1_return_route_status`;
- `supervisor_request_plan_return`;
- `supervisor_request_execute_return`.

Goal-directed:

- `goal_directed_shadow_decision`;
- `goal_directed_shadow_reason`;
- `goal_directed_score_improvement`;
- `goal_exec_attempt_count`;
- `goal_exec_max_attempts`;
- `goal_exec_attempts_remaining`;
- `goal_exec_frontier_cell`;
- `goal_exec_frontier_neighbor`;
- `goal_exec_entry_requested`;
- `goal_exec_entry_started`;
- `goal_exec_entry_completed`;
- `goal_exec_entry_action`;
- `goal_exec_revalidation_status`;
- `goal_exec_fallback_reason`.

## Limitaciones conocidas

- No hay modo 2 final.
- No hay blacklist temporal de fronteras fallidas.
- No hay soporte real de entrada `BACK`.
- El planner sigue siendo BFS de costo uniforme.
- No hay pivots 90 generales en rutas.
- `MainWindow` todavia ejecuta cola, secuencias compuestas y yaw del simulador.
- El planner no es reentrante por workspace estatico.
- El mapa maximo portable actual es `16 x 16`.
