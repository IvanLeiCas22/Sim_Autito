# Checklist manual del modo 1

Este checklist apunta a validar el modo 1 inteligente sin modificar codigo.

## Teclas y herramientas actuales

- `Shift+R`: corre/cancela el Mode 1 Test Runner sobre el mapa actual.
- `Shift+B`: corre/cancela el Batch Runner sobre todos los JSON de `data/test_maps`.
- `X`: cancela autonomia, acciones, planes, runner y batch activo.
- `Shift+P`: activa/desactiva telemetria de performance.
- `Y`: activa/desactiva overlay logico.
- `I`: calcula flood fill hacia la celda inicial.
- `Shift+F`: evalua fronteras candidatas con flood para debug.
- `F3`: abre tuning, mision modo 1 y configuracion de Fast Batch Mode.

## Flujo recomendado de pruebas automaticas

Despues de cambios de navegacion:

1. Ejecutar `Shift+B`.
2. Revisar `batch_runner_state`, `pass/fail/timeout/cancelled` y `autocheck_failure_count`.
3. Revisar los archivos exportados en `data/test_results/`.
4. Si falla un mapa, correrlo aislado con `Shift+R`.
5. Para debugging visual, activar `Y`, `Shift+P`, `I` o `Shift+F` segun corresponda.

Los mapas dentro de `data/test_maps` deben ser mapas esperados a `PASS`. Los casos
`expected_fail`, experimentales o de desarrollo deberian vivir en otra carpeta o estar
documentados explicitamente.

## Fast Batch Mode

Fast Batch Mode acelera solo `Shift+B`. No cambia `kSimulationDtS`, no multiplica
velocidades y no cambia control/PID. Ejecuta varios ticks logicos por cada actualizacion
visual/UI.

En cada tick logico siguen corriendo:

- sensores;
- `nav_core`;
- `nav_supervisor`;
- test runner;
- batch runner;
- autocheck;
- movimiento fisico con el mismo `dt` logico.

Solo se reduce la frecuencia de:

- escena/robot visual;
- overlay;
- labels de telemetria;
- pinned debug;
- performance UI.

Configuracion:

- `batch_fast_mode_enabled`.
- `batch_fast_ticks_per_ui_update`.

Si un batch normal y un batch fast producen resultados distintos para los mismos mapas,
investigar antes de confiar en el resultado fast.

## Navigation Autocheck Monitor

El autocheck corre durante `Shift+R` y `Shift+B`. Observa coherencia interna; no corrige
navegacion. Si detecta una falla critica, el mapa actual termina como
`FAIL / AUTOCHECK_FAIL` y el batch sigue con el siguiente mapa.

Revisar:

- `autocheck_failure_count`;
- `autocheck_last_failure`;
- `autocheck_active_pending_count`.

Los failures tambien quedan en el JSON exportado.

## Export de resultados

El Batch Runner exporta CSV y JSON en `data/test_results/`.

CSV:

- una fila por mapa;
- resultado, reason, ticks, `sim_time_s`, especiales encontradas y estado final;
- contadores basicos de autocheck;
- configuracion fast y resumen batch repetido por fila:
  `batch_fast_mode_enabled`, `batch_fast_ticks_per_ui_update`,
  `batch_wall_time_s`, `batch_sim_time_s`, `batch_speedup`.

JSON:

- resumen del batch;
- resultados por mapa;
- failures de autocheck;
- metricas de tiempo:
  `batch_wall_time_s`, `batch_sim_time_s`, `batch_speedup`;
- configuracion fast:
  `batch_fast_mode_enabled`, `batch_fast_ticks_per_ui_update`.

## Preparacion general

Para cada mapa:

- Cargar el JSON desde la UI.
- Activar overlay con `Y`.
- Abrir telemetria y revisar `Pinned debug`.
- Seleccionar politica con `P` hasta `NAV_POLICY_SMART_RECOGNITION`.
- Si hace falta, abrir `F3` y verificar tuning.
- Activar simulacion con `M` y `Space`.
- Activar autonomia con `B`.
- Dejar correr hasta terminar o detectar falla.

Resultado esperado general:

- con mision desactivada, `supervisor_smart_state = NO_FRONTIER` al completar lo alcanzable.
- con mision activada, `supervisor_state = DONE` al volver a la celda inicial.
- El overlay coincide con la pose logica.
- No atraviesa paredes.
- No queda en loop.
- No marca celdas especiales falsas.
- No pierde la pose logica despues de pivots.

## Carga de mapas JSON

Probar:

- mapas simples;
- mapas stress en `data/`;
- mapas con `special_cells`;
- mapas con regiones inaccesibles.

Verificar:

- dimensiones correctas;
- `cell_size_mm = 200`;
- start correcto;
- paredes visibles;
- marcas especiales centradas;
- marcas especiales default de `120 mm` si falta `size_mm`.

## Overlay de mapa logico

Pasos:

1. Activar `Y`.
2. Mover el robot por varias celdas.
3. Observar visitadas, celda actual, orientacion y paredes.

Esperado:

- celdas visitadas sombreadas;
- celda actual resaltada;
- flecha logica coherente;
- paredes conocidas/presentes dibujadas;
- celdas especiales detectadas con indicador visual.

Variables utiles:

- `logical_cell_x`.
- `logical_cell_y`.
- `logical_dir`.
- `current_cell_visited`.
- `current_cell_walls_known`.
- `current_cell_walls_present`.
- `current_cell_special`.

## SMART_RECOGNITION hasta NO_FRONTIER

Pasos:

1. Seleccionar `NAV_POLICY_SMART_RECOGNITION`.
2. Activar `B`.
3. Dejar explorar.

Esperado:

- primero entra a vecinas no visitadas inmediatas;
- si no hay salida local, planifica a frontera;
- ejecuta ruta automaticamente;
- al llegar a frontera vuelve a exploracion local;
- termina en `NO_FRONTIER`.

Variables utiles:

- `supervisor_smart_state`.
- `supervisor_smart_decision_reason`.
- `supervisor_smart_active_as_source`.
- `supervisor_request_plan_to_frontier`.
- `supervisor_request_execute_plan`.
- `smart_recognition_state`.
- `smart_frontier_plan_requested_count`.
- `smart_frontier_routes_executed_count`.
- `smart_no_frontier_count`.
- `frontier_route_status`.
- `plan_execution_enabled`.
- `plan_queue_count`.

## Supervisor modo 1

### Mision activada

Pasos:

1. Activar `mode1_mission_enabled`.
2. Dejar `mode1_required_special_count` en el valor deseado.
3. Seleccionar `NAV_POLICY_SMART_RECOGNITION`.
4. Activar `B`.

Esperado:

- `supervisor_active_as_source = true`;
- `supervisor_state = SEARCH_SPECIALS` mientras busca;
- SMART explora normalmente hasta encontrar N especiales;
- al encontrar N, `supervisor_state = FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE`;
- `supervisor_block_smart_actions = true`;
- no se planifican nuevas fronteras;
- la accion fisica actual termina sin ser cortada;
- luego pasa por `RETURN_SAFE_PLAN` y `RETURN_SAFE_EXECUTE`;
- termina en `DONE` al llegar a la celda inicial.

### Mision desactivada

Pasos:

1. Desactivar `mode1_mission_enabled`.
2. Activar SMART con `B`.

Esperado:

- `supervisor_active_as_source = false` para mision;
- SMART sigue controlado por `nav_supervisor`;
- explora hasta `NO_FRONTIER`;
- no intenta volver al inicio por mision.

### Bloqueo de SMART por mision

Esperado al alcanzar N especiales:

- `supervisor_smart_state = BLOCKED_BY_MISSION` o SMART sin nuevos requests;
- `supervisor_request_plan_to_frontier = false`;
- `plan_execution_enabled` de exploracion queda limpio;
- retorno seguro conserva prioridad.

## Planificacion manual K/J

Pasos:

1. Explorar parte del mapa.
2. Presionar `K`.
3. Ingresar celda visitada alcanzable.
4. Revisar la ruta.
5. Presionar `J` desde punto fisico valido.

Esperado:

- `route_status = FOUND`.
- `plan_queue_count` coincide con `route_length`.
- `J` no ejecuta si no hay `floor_rear_black` confiable.
- Ejecuta la cola sin recalcular.

Probar tambien:

- destino fuera de mapa: `TARGET_OUT_OF_BOUNDS`;
- destino no visitado: `TARGET_NOT_VISITED`;
- destino sin ruta: `NO_PATH`.

## Planificacion a frontera T/J

Pasos:

1. Detener el robot en zona explorada sin vecina no visitada inmediata.
2. Presionar `T`.
3. Verificar destino de frontera.
4. Presionar `J`.

Esperado:

- `frontier_route_status = FOUND` o `FRONTIER_ALREADY_HERE`;
- si hay ruta, la cola queda cargada;
- `J` ejecuta solo si el arranque fisico es valido;
- al llegar a frontera, `B` o SMART puede entrar a la celda no visitada.

## Celdas especiales

### En avance

Pasos:

1. Ejecutar `ADVANCE_LINE` atravesando una marca especial.
2. Observar sensores de piso.

Esperado:

- `special_candidate` aparece cuando ambos sensores pisan negro.
- `special_confirmed` confirma dentro de ventana valida.
- `special_ignore_rear_until_white` evita que la marca central corte el avance como si fuera linea destino.
- La accion termina en la cinta real de destino.

### En smooth

Pasos:

1. Ubicar una especial en celda de destino o inicio de smooth.
2. Ejecutar `SMOOTH_LEFT` y `SMOOTH_RIGHT`.

Esperado:

- deteccion durante smooth si se cumple la ventana;
- no corta mal el smooth;
- `special_mark_target_source` ayuda a verificar la celda marcada.

### Especiales contiguas

Pasos:

1. Usar mapa con dos especiales vecinas.
2. Ejecutar avance/smooth entre ellas.

Esperado:

- no hay falsos positivos de preview diagonal por marca central;
- `advance_front_diag_preview_armed` debe armarse solo despues de ver blanco.

### Arranque sobre especial

Pasos:

1. Cargar mapa con start sobre marca especial.
2. Activar SMART.

Esperado:

- snapshot inicial marca la celda actual;
- `rear_line_trusted_for_decision = false` si el negro es ambiguo;
- no inicia smooth inmediatamente desde marca central;
- adquiere una linea real antes de decisiones locales.

## Smooth turns

### Left/right basicos

Esperado:

- curva principal con yaw-rate PI;
- fase final `POST_YAW_SEEK_REAR_LINE`;
- terminacion por sensor trasero en cinta.

Variables:

- `smooth_phase`.
- `smooth_done_reason`.
- `smooth_final_guidance_source`.

### Smooth consecutivos

Pasos:

1. Usar pasillo serpenteante.
2. Activar SMART.

Esperado:

- no pierde pose logica;
- no sobrecorrige entre smooths;
- si hay diagonal y carry habilitado, se consume antes del siguiente smooth.

Variables:

- `smooth_yaw_carry_candidate_pending`.
- `smooth_yaw_carry_used`.
- `smooth_yaw_carry_offset_deg`.
- `smooth_yaw_carry_rejected_reason`.

### Smooth final con diagonal setpoint

Pasos:

1. En `F3`, dejar `smooth_final_mode = SETPOINT`.
2. Ejecutar smooth con paredes vistas por diagonales.

Esperado:

- `smooth_final_guidance_source = DIAG_CENTER`, `DIAG_LEFT` o `DIAG_RIGHT`;
- `diag_guidance_kd = 0`;
- correccion estable sin tembleque.

## ADVANCE_LINE

### Dos paredes

Esperado:

- `advance_final_correction_source = WALL_CENTER`;
- corrige hacia centro con targets de `60 mm`.

### Una pared

Esperado:

- `WALL_LEFT` o `WALL_RIGHT`;
- escala de error de una pared equivalente a x2.

### Sin paredes

Esperado:

- `YAW_ONLY`;
- yaw hold relativo si venia de perder pared.

### Preview diagonal

Pasos:

1. Avanzar sin paredes laterales.
2. Esperar que `floor_front_black` toque cinta frontal.

Esperado:

- `advance_front_diag_preview_armed = true` despues de blanco;
- `advance_front_diag_preview_latched = true` al tocar cinta;
- laterales confirmados tienen prioridad sobre diagonales;
- termina solo con `floor_rear_black`.

### Wall caution

Pasos:

1. Avanzar al borde de una pared.
2. Forzar perdida de diagonal mientras lateral sigue valido.

Esperado:

- lado pasa de `CONFIRMED` a `CAUTION`;
- captura `hold_mm`;
- usa correccion suave;
- vuelve a `CONFIRMED` si retorna diagonal o a `LOST` por timeout/delta/lateral lost.

## CENTER_AND_PIVOT_180

### Sin pared frontal

Esperado:

- `plan_composite_prepare_method = FRONT_LINE`;
- ejecuta `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE -> PIVOT_180`.

### Con pared frontal

Esperado:

- `plan_composite_prepare_method = FRONT_WALL`;
- ejecuta `APPROACH_FRONT_WALL_FOR_PIVOT -> PIVOT_180`;
- no busca una cinta frontal inexistente.

### Seguido de ADVANCE_LINE

Esperado:

- siguiente avance usa `NAV_ADVANCE_START_CENTERED_POSE`;
- no confirma especiales durante ese avance;
- no filtra la primera cinta real con `special_ignore_rear_until_white`.

Variables:

- `plan_next_advance_from_centered_pose`.
- `advance_start_mode`.
- `advance_from_centered_waiting_rear_white`.

## Cancelacion con X

Probar durante:

- autonomia local;
- ruta planificada;
- espera de retorno de mision;
- retorno seguro al inicio;
- secuencia compuesta;
- smooth;
- center;
- approach;
- pivot.

Esperado:

- se detiene accion actual;
- `plan_execution_enabled = false`;
- secuencias pendientes canceladas;
- `nav_supervisor` queda cancelado si habia mision activa;
- motores en cero;
- no queda cola ejecutandose.

## Reset y tuning F3

Pasos:

1. Abrir `F3`.
2. Cambiar un parametro superior.
3. Cambiar un parametro inferior usando scroll.
4. Aplicar.
5. Reset defaults.

Esperado:

- botones `Reset`, `Close`, `Apply` siempre visibles;
- scroll vertical permite llegar a todos los grupos;
- cambios se reflejan en telemetria/control;
- reset vuelve a defaults de codigo.

## Flood fill y fronteras debug

### Flood hacia inicio

Pasos:

1. Explorar algunas celdas.
2. Presionar `I`.

Esperado:

- `flood_status = OK`;
- el overlay muestra costos si `Y` esta activo;
- la celda inicial tiene costo `0`;
- celdas no visitadas no reciben costo util.

### Evaluacion de fronteras

Pasos:

1. Presionar `I`.
2. Presionar `Shift+F`.

Esperado:

- `flood_fr_candidate_edge_count` cuenta salidas candidatas;
- `flood_fr_candidate_cell_count` cuenta celdas visitadas frontera unicas;
- `flood_fr_candidate_neighbor_cell_count` cuenta vecinas no visitadas unicas;
- el overlay marca `best_cell` y `best_neighbor_cell`;
- no se carga plan ni se ejecuta accion.

## Criterios de falla

Considerar falla si:

- atraviesa una pared fisica;
- `logical_cell_x/y` no coincide con overlay;
- `logical_dir` queda incoherente despues de pivot;
- `map_update_count` cambia en acciones que no deben modificar pose;
- `map_wall_update_count` cambia en pivots o maniobras auxiliares;
- se marca una especial falsa;
- no detecta especiales alcanzables en avance/smooth/auxiliar;
- SMART queda en loop sin llegar a `NO_FRONTIER`;
- `J` ejecuta una ruta desde arranque fisico invalido;
- `supervisor_smart_active_as_source` queda falso durante SMART activo;
- la mision activa no bloquea SMART al encontrar N especiales;
- smooth consecutivos generan giro brusco por perdida de yaw carry;
- wall caution domina cuando deberia caer a `LOST`.
