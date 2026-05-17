# Diseno de la capa portable `nav_supervisor`

Este documento describe el estado actual de la supervision de navegacion. La regla
principal sigue siendo separar decisiones portables de la UI/simulacion Qt.

## Arquitectura por capas

### Qt / simulacion

`MainWindow` es adaptador, no fuente principal de decision de navegacion. Sus
responsabilidades actuales son:

- leer sensores simulados desde `SimWorld` y `SimRobot`;
- construir `RobotSensors`;
- llamar `nav_core_update(...)`;
- arrancar primitivas `nav_core_start_*`;
- ejecutar la cola de planes;
- resetear/aplicar referencia de yaw del simulador;
- cargar JSON;
- mantener UI, telemetria, overlay, test runner y batch runner.

### Modulos portables en `nav/`

- `nav_core`: primitivas fisicas, avance, smooth turns, pivots, cola de planes,
  planner BFS orientado, `route_eval_to_cell_with_dir_mask`.
- `nav_map`: mapa logico, celda/orientacion actual, visitadas, paredes
  conocidas/presentes y especiales.
- `nav_flood`: flood fill portable para costos/debug.
- `nav_frontier_eval`: evaluacion clasica de fronteras flood usada por `Shift+F`.
- `nav_goal_return_eval`: evaluador optimista de retorno hacia inicio. Usa
  weighted/optimistic flood fill por celdas, permite desconocido con presupuesto y
  devuelve la primera frontera util.
- `nav_supervisor`: mision modo 1, SMART_RECOGNITION, retorno seguro y
  `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`.
- `pid_controller`: PID/fixed-point portable.

## Responsabilidades de `nav_supervisor`

`nav_supervisor` coordina decisiones de alto nivel:

- mision modo 1 segura;
- latch al encontrar `required_special_count`;
- bloqueo de SMART cuando la mision toma control;
- retorno seguro conocido;
- acciones locales SMART;
- planificacion/ejecucion a frontera SMART;
- `FINAL_SAFE_SCAN_RETURN`;
- `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`;
- estados, reasons, pulsos de request y debug snapshots.

No debe depender de Qt, `SimWorld`, `SimRobot`, JSON, dibujo, teclado, PWM real,
`malloc/free`, `float/double` ni buffers grandes en stack.

## Entradas y salidas

Entradas principales:

- `NavSupervisorInput`;
- `NavSupervisorSmartInput`;
- `NavSupervisorConfig`;
- `nav_supervisor_set_start_cell(...)`;
- `nav_supervisor_notify_return_route_status(...)`;
- `nav_supervisor_notify_frontier_route_status(...)`;
- `nav_supervisor_notify_goal_frontier_route_status(...)`;
- `nav_supervisor_notify_goal_entry_started(...)`.

Salidas principales:

- `NavSupervisorOutput`;
- `NavSupervisorSmartOutput`;
- `NavSupervisorDebugSnapshot`.

`MainWindow` aplica esos outputs como adaptador: planifica con `nav_core`, activa la
cola, arranca primitivas o detiene autonomia/motores.

## Estados de mision

Estados activos principales:

- `IDLE`;
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

Estados SMART implementados:

- `IDLE`;
- `LOCAL_UNVISITED`;
- `PLAN_TO_FRONTIER`;
- `EXECUTING_FRONTIER_ROUTE`;
- `FRONTIER_ALREADY_HERE`;
- `NO_FRONTIER`;
- `ERROR`;
- `BLOCKED_BY_MISSION`;
- `WAIT_NAV_READY`.

## Estrategias de retorno

### `SAFE_KNOWN_RETURN`

Retorno conservador:

1. deja terminar la accion fisica actual;
2. limpia exploracion pendiente;
3. planifica a la celda inicial por mapa conocido;
4. ejecuta la cola;
5. termina en `DONE` al llegar al inicio.

### `GOAL_DIRECTED_RETURN_SHADOW`

Evalua que haria el retorno inteligente, pero no cambia el comportamiento real. Sirve
para comparar decision, frontera, costos y reasons.

### `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`

Es el default actual.

Objetivo: volver al inicio explorando de forma dirigida si puede descubrir un atajo,
sin planificar todo un camino desconocido de una vez.

Flujo:

1. calcula retorno seguro conocido;
2. calcula ruta optimista hacia inicio permitiendo desconocido con presupuesto;
3. elige la primera frontera util del camino optimista;
4. planifica por mapa conocido hasta `frontier_cell` con mascara de orientaciones;
5. ejecuta esa cola;
6. revalida la frontera;
7. entra una sola celda desconocida (`frontier_neighbor`) con:
   - `ADVANCE_LINE`;
   - `SMOOTH_LEFT`;
   - `SMOOTH_RIGHT`;
8. no usa `BACK` por defecto;
9. deja que `nav_core` actualice mapa/celda/orientacion normalmente;
10. recalcula desde la nueva situacion;
11. si ya no conviene o se agotan intentos, cae a retorno seguro.

El supervisor hace fallback seguro ante ruta no encontrada, orientacion no soportada,
pared compartida conocida presente, vecino invalido/visitado, nav ocupado, fallo de
arranque de primitiva, mismatch de celda o cancelacion.

## Defaults actuales de F3

- `return_strategy = GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`.
- `goal_required_improvement = 0`.
- `goal_unknown_cell_penalty = 0`.
- `goal_unknown_edge_penalty = 0`.
- `goal_max_unknown_cells = 32`.
- `goal_max_unknown_edges = 32`.
- `goal_min_safe_return_cost_to_try = 4`.
- `goal_max_shortcut_attempts = 32`.
- `goal_allow_back_entry = false`.
- `batch_fast_mode_enabled = true`.
- `batch_fast_ticks_per_ui_update = 10`.

Notas:

- `goal_max_unknown_cells/edges` limitan cuanto desconocido puede considerar el
  evaluador optimista.
- `goal_max_shortcut_attempts` limita entradas reales a celdas desconocidas durante
  el retorno. No significa planificar muchas celdas a ciegas.
- penalty `0` significa que desconocido cuesta lo mismo que un paso normal.
- pared conocida presente bloquea; pared conocida ausente es paso normal; pared
  desconocida es transitable optimista configurable.

## Relacion con STM32

`nav_supervisor` esta disenado como C portable:

- sin Qt;
- sin `malloc/free`;
- sin `float/double`;
- con arrays/workspaces fijos;
- con requests explicitos hacia una futura HAL/adaptador.

Pendiente para firmware real:

- HAL de sensores, yaw, tiempo y motores;
- telemetria serial liviana;
- medicion de SRAM/CPU;
- tuning con sensores reales.

## Riesgos actuales

- `MainWindow` todavia ejecuta cola, secuencias compuestas y yaw del simulador.
- `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION` puede gastar mas pasos que retorno seguro
  en mapas donde el atajo optimista no se materializa.
- No hay blacklist temporal de fronteras fallidas todavia.
- No hay soporte `BACK` para entrada a frontera.
- Planner y route eval usan workspace estatico no reentrante.

## Recomendacion

Mantener la direccion actual:

- decisiones en `nav/`;
- `MainWindow` como adaptador Qt/UI/simulacion;
- fallback seguro siempre disponible;
- batch + autocheck como validacion obligatoria despues de cambios.
