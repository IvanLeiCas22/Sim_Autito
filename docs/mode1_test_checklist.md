# Checklist manual del modo 1

Checklist para validar navegacion, batch y retorno inteligente.

## Teclas actuales

- `Shift+R`: corre/cancela el Mode 1 Test Runner sobre el mapa actual.
- `Shift+B`: corre/cancela el Batch Runner sobre `data/test_maps`.
- `X`: cancela autonomia, acciones, planes, runner y batch activo.
- `Shift+P`: performance debug.
- `Y`: overlay logico.
- `I`: flood hacia inicio.
- `Shift+F`: evaluacion debug de fronteras flood.
- `F3`: tuning, mision modo 1, goal-directed return y Fast Test Mode.

## Defaults actuales a verificar en F3

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

`goal_max_shortcut_attempts` cuenta entradas reales a celdas desconocidas. No significa
que el robot planifique muchas celdas desconocidas de una vez.

## Flujo recomendado despues de cambios de navegacion

1. Ejecutar `Shift+B`.
2. Verificar:
   - `batch_runner_state = BATCH_DONE`;
   - `batch_runner_fail_count = 0`;
   - `batch_runner_timeout_count = 0`;
   - `autocheck_failure_count = 0`.
3. Revisar CSV/JSON en `data/test_results/`.
4. Si falla un mapa, cargarlo y correr `Shift+R`.
5. Para debug visual, usar `Y`, `Shift+P`, `I` y `Shift+F`.

Los mapas en `data/test_maps` deben ser esperados a `PASS`. Mapas `expected_fail`,
experimentales o de desarrollo deben vivir en otra carpeta o estar documentados aparte.

## Fast Test Mode

Activo por defecto para `Shift+R` y `Shift+B`.

Propiedades:

- no cambia `kSimulationDtS`;
- no multiplica velocidades;
- no cambia control/PID;
- ejecuta varios ticks logicos por actualizacion UI mientras corre el test runner o el
  batch runner;
- sensores, `nav_core`, `nav_supervisor`, test runner, batch runner, autocheck y
  movimiento fisico siguen corriendo en cada tick logico.

Solo se reduce la frecuencia de UI/overlay/telemetria. No se aplica a navegacion manual
libre por ahora.

Si batch normal y fast producen resultados distintos para los mismos mapas, investigar
antes de confiar en el resultado fast.

## Navigation Autocheck Monitor

Corre durante `Shift+R` y `Shift+B`. Observa coherencia interna; no corrige la
navegacion.

Si detecta falla critica:

- el mapa actual termina `FAIL / AUTOCHECK_FAIL`;
- el batch sigue con el siguiente mapa.

Campos utiles:

- `autocheck_failure_count`;
- `autocheck_last_failure`;
- `autocheck_active_pending_count`.

Los failures se exportan en JSON.

## Export CSV/JSON

El Batch Runner exporta en `data/test_results/`.

CSV:

- una fila por mapa;
- result/reason/ticks/sim_time/especiales/estado final;
- `autocheck_failure_count`;
- `autocheck_last_failure`;
- `batch_fast_mode_enabled`;
- `batch_fast_ticks_per_ui_update`;
- `batch_wall_time_s`;
- `batch_sim_time_s`;
- `batch_speedup`.

Los nombres exportados siguen siendo `batch_fast_*` porque el speedup exportado es del
batch. En F3/telemetria visible se muestran como `test_fast_*`.

JSON:

- resumen del batch;
- resultados por mapa;
- failures de autocheck;
- `batch_wall_time_s`;
- `batch_sim_time_s`;
- `batch_speedup`;
- configuracion fast.

## Prueba especifica de `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`

En un mapa con atajo optimista:

1. Dejar defaults F3.
2. Ejecutar `Shift+R` o `Shift+B`.
3. Verificar que al alcanzar required specials:
   - `supervisor_state` pase por `RETURN_SMART_DECIDE`;
   - si decide intentar, pase por `RETURN_FRONTIER_PLAN`;
   - ejecute `RETURN_FRONTIER_EXECUTE`;
   - llegue a `RETURN_FRONTIER_ENTER`;
   - revalide la frontera;
   - emita entrada y arranque una primitiva.
4. Verificar:
   - `goal_exec_entry_started = true`;
   - `goal_exec_attempt_count` aumenta solo al arrancar la entrada;
   - `goal_exec_entry_completed = true` si llego a `frontier_neighbor`;
   - recalcula entre entradas;
   - si agota intentos o no conviene, pasa a `RETURN_SAFE_PLAN`;
   - termina `DONE`.

Fallas esperadas con fallback seguro:

- `GOAL_ROUTE_NOT_FOUND`;
- `GOAL_PLAN_NOT_LOADED`;
- `GOAL_FRONTIER_CELL_MISMATCH`;
- `GOAL_FRONTIER_NEIGHBOR_INVALID`;
- `GOAL_FRONTIER_NEIGHBOR_VISITED`;
- `GOAL_FRONTIER_WALL_BLOCKED`;
- `GOAL_ENTRY_UNSUPPORTED`;
- `GOAL_ENTRY_NAV_NOT_READY`;
- `GOAL_ENTRY_START_FAILED`;
- `GOAL_ENTRY_DID_NOT_ADVANCE`;
- `GOAL_ENTRY_CELL_MISMATCH`;
- `GOAL_ATTEMPTS_EXHAUSTED`.

## Cancelacion con `X`

Probar durante:

- exploracion SMART;
- plan a frontera SMART;
- retorno seguro;
- `RETURN_FRONTIER_PLAN`;
- `RETURN_FRONTIER_EXECUTE`;
- `RETURN_FRONTIER_ENTER`;
- entrada goal-directed;
- secuencias compuestas.

Esperado:

- accion actual cancelada;
- `plan_execution_enabled = false`;
- colas/secuencias canceladas;
- motores en cero;
- runner/batch cancelado si estaba activo.

## SMART_RECOGNITION

Con mision desactivada:

- SMART explora vecinas no visitadas;
- si no hay salida local, planifica a frontera;
- si no hay frontera, termina en `NO_FRONTIER`.

Con mision activada:

- SMART explora hasta encontrar required specials;
- mision bloquea SMART al tomar retorno;
- SMART no debe emitir nuevas fronteras mientras `block_smart_actions = true`.

Campos:

- `supervisor_smart_state`;
- `supervisor_smart_decision_reason`;
- `supervisor_request_plan_to_frontier`;
- `supervisor_request_execute_plan`;
- `smart_frontier_plan_requested_count`;
- `smart_frontier_routes_executed_count`;
- `smart_no_frontier_count`.

## Flood y fronteras debug

`I`:

- calcula flood hacia inicio;
- overlay muestra costos si esta activo.

`Shift+F`:

- llama `nav_frontier_eval`;
- muestra mejor frontera clasica flood;
- no carga plan;
- no ejecuta acciones;
- no cambia la mision.

## Validaciones generales

Considerar falla si:

- atraviesa una pared fisica;
- celda/orientacion logica no coincide con overlay;
- se marca una especial falsa;
- no detecta especiales alcanzables;
- SMART queda en loop;
- la mision no bloquea SMART al retornar;
- `goal_exec_attempt_count` aumenta antes de iniciar primitiva;
- goal-directed entra en una celda distinta al `frontier_neighbor`;
- fallback deja `plan_execution_enabled` activo;
- `X` no cancela seguro.
