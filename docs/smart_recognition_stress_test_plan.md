# Smart recognition stress test plan

Checkpoint actual para validar manualmente SMART y el modo 1 inteligente del simulador.

## Configuracion comun

Para cada mapa:

1. Cargar el JSON desde `File -> Load Maze...` o tecla `O`.
2. Activar overlay con `Y`.
3. Seleccionar `SMART_RECOGNITION` con `P`.
4. Para probar SMART puro hasta `NO_FRONTIER`, desactivar `mode1_mission_enabled` en
   `F3`.
5. Para probar modo 1 completo, dejar los defaults actuales:
   `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`, `goal_max_shortcut_attempts = 32` y
   Fast Test Mode activo.
6. Activar `auto_mode` con `M`.
7. Presionar `Space` para correr simulacion.
8. Activar autonomia con `B`.
9. Dejar correr hasta finalizar o hasta observar un fallo claro.

Checklist general:

- Con mision desactivada, `smart_recognition_state` debe terminar en `NO_FRONTIER`
  cuando no queden fronteras alcanzables.
- Con mision activada, al encontrar las especiales requeridas debe bloquear SMART y
  volver al inicio; por defecto puede usar `GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`.
- El overlay debe mantener pose logica coherente con el robot.
- El robot no debe atravesar paredes.
- Las celdas especiales alcanzables deben marcarse con el indicador visual del overlay.
- `special_cells_found_count` no debe aumentar por falsos positivos.
- `plan_execution_enabled` no debe quedar activo permanentemente en loop.
- `smart_frontier_plan_requested_count` puede aumentar cuando el robot queda rodeado de celdas visitadas.
- `smart_frontier_routes_executed_count` debe aumentar cuando se ejecuta una ruta automatica a frontera.
- Despues de explorar, probar `K/J` hacia al menos una celda especial detectada.
- En un punto intermedio, probar `T/J` para confirmar que la ruta manual a frontera sigue funcionando.

## Mapas

| Mapa | Objetivo | Configuracion inicial | Pasos especificos | Resultado esperado | Telemetria importante |
| --- | --- | --- | --- | --- | --- |
| `stress_open_adjacent_specials.json` | Probar especiales adyacentes, yaw PD y avance sin paredes laterales. | 7x6, inicio en `(0,0)` mirando East. Tres especiales, dos vecinas. | Dejar explorar completo; observar paso por `(2,2)` y `(3,2)`. | Detecta las especiales alcanzables sin cortar mal `ADVANCE_LINE`; no marca duplicados falsos. | `special_candidate`, `special_confirmed`, `special_cells_found_count`, `advance_final_correction_source`, `smart_recognition_state`. |
| `stress_snake_smooth_specials.json` | Forzar muchos `SMOOTH_LEFT/RIGHT` y deteccion especial durante smooth. | 7x7, pasillo serpenteante. | Observar cambios frecuentes de direccion y especiales ubicadas cerca de inicio de giros. | Smooth turns terminan en cinta correcta; especiales en celdas de giro se detectan si son alcanzables. | `smooth_phase`, `last_smooth_done_reason`, `special_detection_context/action` si existe, `special_confirmed`, overlay. |
| `stress_dead_ends_front_wall.json` | Probar callejones sin salida y `APPROACH_FRONT_WALL_FOR_PIVOT -> PIVOT_180`. | 7x7 con varios dead-ends internos y en limites. | Dejar que SMART explore ramas cerradas. | En dead-end usa aproximacion a pared frontal y pivotea sin irse de largo. | `dead_end_recovery_phase`, `approach_front_done_reason`, `plan_composite_prepare_method`, `last_approach_front_done_reason`. |
| `stress_center_pivot_cases.json` | Probar `CENTER_AND_PIVOT_180` con `FRONT_LINE` y `FRONT_WALL`. | 7x6, ramas que obligan a reorientarse. | Durante exploracion observar telemetria de accion compuesta; luego probar `K/J` hacia una celda detras. | Si hay pared frontal usa `FRONT_WALL`; si no, usa `FRONT_LINE`; luego `PIVOT_180` y continua. | `plan_composite_prepare_method`, `plan_composite_wall_front_at_start`, `plan_next_advance_from_centered_pose`, `advance_start_mode`. |
| `stress_frontier_backtracking.json` | Probar retorno por celdas visitadas hacia nuevas fronteras. | 8x8 con ramas y loops. | Dejar correr hasta que se aleje de una frontera y tenga que volver. | Planifica a frontera automaticamente y ejecuta cola sin requerir `T/J`. | `smart_frontier_plan_requested_count`, `smart_frontier_routes_executed_count`, `frontier_route_status`, `plan_queue_count`. |
| `stress_single_wall_guidance.json` | Probar `WALL_LEFT/WALL_RIGHT` y escalado de error x2. | 8x5 con tramos de una sola pared lateral. | Observar avance en tramos con una pared valida y sin pared opuesta. | Corrige con pared unica sin oscilacion excesiva y cae a `YAW_PD` cuando no hay pares lateral+diagonal validos. | `advance_final_correction_source`, `advance_follow_left_valid`, `advance_follow_right_valid`, `wall_single_side_error_scale`, `advance_wall_error_mm`. |
| `stress_unreachable_region.json` | Verificar region cerrada inaccesible y especial inaccesible. | 7x7 con bloque cerrado en la esquina inferior derecha. | Dejar correr hasta finalizar. | Explora todo lo alcanzable y termina en `NO_FRONTIER` sin intentar cruzar paredes hacia la region cerrada. | `smart_recognition_state`, `frontier_route_status`, `smart_no_frontier_count`, overlay, `special_cells_found_count`. |
| `stress_start_on_special_or_center.json` | Probar arranque incomodo desde una celda especial/centro de celda. | 6x6, inicio en `(1,1)`, sobre marca especial central. | Activar SMART desde el inicio; observar adquisicion de linea. | No confirma falsos positivos por el arranque; adquiere linea y explora normalmente. | `floor_rear_black_real`, `rear_black_for_line`, `advance_start_mode`, `special_detection_enabled_for_current_motion`, `special_cells_found_count`. |

## Criterios de fallo

- El robot atraviesa una pared visible o el overlay marca una celda imposible.
- `smart_recognition_state` queda alternando indefinidamente sin avanzar ni planificar.
- `plan_execution_enabled` queda activo con `plan_queue_count = 0`.
- `special_cells_found_count` aumenta al pasar por una cinta de frontera sin marca central.
- `last_smooth_done_reason` o `last_advance_done_reason` quedan en estados incoherentes mientras el robot sigue moviendose.
- En un dead-end o limite frontal, la accion compuesta busca cinta frontal inexistente en vez de usar `FRONT_WALL`.

## Notas

- Estos mapas son pruebas manuales; no agregan datos magicos ni cambian la logica de `nav_core`.
- Las paredes exteriores las impone `SimWorld`; los JSON solo declaran paredes internas.
- Las marcas especiales usan `size_mm = 120`.
- Con sensores de suelo separados 84 mm, `100x100 mm` deja una ventana de deteccion simultanea demasiado chica; `120x120 mm` aumenta la robustez y sigue dejando separacion razonable respecto de las cintas de frontera.
- Si un mapa termina en `NO_FRONTIER` sin visitar una region cerrada, eso es correcto si la region no es alcanzable.
