# Handoff tecnico del proyecto

Resumen compacto del estado actual para retomar trabajo sin releer todo el historial.

## Estado actual

Proyecto Qt/C++ Widgets de simulador micromouse/autito.

Funciona actualmente:

- mision modo 1;
- `SMART_RECOGNITION`;
- retorno seguro conocido;
- `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`;
- `FINAL_SAFE_SCAN_RETURN`;
- Test Runner `Shift+R`;
- Batch Runner `Shift+B`;
- Fast Batch Mode;
- Navigation Autocheck Monitor;
- export CSV/JSON en `data/test_results`;
- `nav_goal_return_eval`;
- `nav_frontier_eval`;
- `nav_core_route_eval_to_cell_with_dir_mask(...)`.

## Arquitectura

### `MainWindow`

Adaptador Qt/UI/simulacion:

- sensores simulados;
- pipeline de simulacion;
- llamadas a `nav_core_update(...)`;
- arranque de primitivas;
- ejecucion de cola;
- reset de yaw del simulador;
- overlay y telemetria;
- carga JSON;
- test runner, batch runner, fast batch y autocheck.

No debe volver a concentrar logica de decision de mision/SMART/retorno.

### `nav/`

Logica portable:

- `nav_core`: primitivas, control, cola, planner BFS orientado, route eval.
- `nav_map`: mapa logico, visitadas, paredes, especiales.
- `nav_flood`: costos por celda.
- `nav_frontier_eval`: fronteras flood para `Shift+F`.
- `nav_goal_return_eval`: ruta optimista hacia inicio y primera frontera util.
- `nav_supervisor`: mision modo 1, SMART, retorno seguro y goal-directed.
- `pid_controller`: PID Q16.16.

## Retorno actual

Default:

```text
return_strategy = GOAL_DIRECTED_RETURN_LIMITED_EXECUTION
```

El retorno inteligente:

1. calcula retorno seguro conocido;
2. calcula ruta optimista hacia inicio permitiendo desconocido;
3. toma solo la primera frontera util;
4. planifica por mapa conocido hasta `frontier_cell`;
5. revalida pared compartida, vecino, orientacion y disponibilidad de nav;
6. entra una celda desconocida con `ADVANCE_LINE`, `SMOOTH_LEFT` o `SMOOTH_RIGHT`;
7. cuenta intento solo si la primitiva arranca;
8. recalcula entre entradas;
9. cae a retorno seguro ante cualquier inconsistencia.

`BACK` no esta soportado por defecto.

## Defaults relevantes

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

## Testing recomendado

Despues de cambios de navegacion:

1. Ejecutar `Shift+B`.
2. Esperar `batch_runner_state = BATCH_DONE`.
3. Verificar:
   - `batch_runner_fail_count = 0`;
   - `batch_runner_timeout_count = 0`;
   - `autocheck_failure_count = 0`.
4. Revisar export CSV/JSON si hubo cambios de runner o telemetria.

Para un mapa puntual, usar `Shift+R`.

## Portabilidad

`nav/` esta pensado para STM32:

- sin Qt;
- sin `malloc/free`;
- sin `float/double`;
- arrays fijos;
- mapa maximo `16 x 16`;
- workspaces estaticos.

Pendiente para firmware:

- HAL STM32;
- adaptador de requests de `nav_supervisor`;
- sensores reales, yaw, PWM y tiempo;
- telemetria serial compacta;
- medicion SRAM/CPU;
- tuning con robot real.

## Pendientes principales

- overlay especifico de goal-directed;
- Autocheck adicional para entrada goal-directed;
- blacklist de fronteras fallidas si aparece loop;
- soporte `BACK`;
- tests unitarios portables;
- HAL STM32;
- modo 2.
