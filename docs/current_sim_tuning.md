# Current Simulator Tuning Checkpoint

Checkpoint actual del simulador Qt/C++ para el autito/micromouse.

Este documento registra una configuracion funcional validada para probar navegacion basica con regla de mano derecha, avance con yaw PD y wall assist, smooth turns, pivots, carga de laberintos JSON y tuning runtime.

## Estado validado

- `ADVANCE_LINE` funciona con bases separadas, yaw PD y wall assist.
- `SMOOTH_LEFT` y `SMOOTH_RIGHT` usan target de yaw-rate configurable, bases recalculadas automaticamente y fase `POST_YAW_SEEK_REAR_LINE`.
- `PIVOT_LEFT`, `PIVOT_RIGHT` y `PIVOT_180` usan correccion de centro de pivot cerca del sensor trasero en el simulador.
- La percepcion de paredes usa IR frontales, laterales y diagonales calibrados.
- La ventana de tuning runtime se abre con `F3`.
- La navegacion basica de prueba usa regla de mano derecha desde `MainWindow`.

## Parametros funcionales

| Area | Parametro | Valor |
| --- | --- | ---: |
| Turn yaw-rate PI | Kp | 8 |
| Turn yaw-rate PI | Ki | 0.5 |
| Turn yaw-rate PI | Kd | 0 |
| Turn yaw-rate PI | output_limit_pwm | 4000 |
| Advance yaw PD | Kp | 50 |
| Advance yaw PD | Ki | 0 |
| Advance yaw PD | Kd | 2 |
| Advance yaw PD | output_limit_pwm | 1200 |
| Advance wall PD | Kp | 12 |
| Advance wall PD | Kd | 600 |
| Advance wall PD | correction_limit_pwm | 4000 |
| Advance wall PD | error_deadband_mm | 0 |
| Advance wall PD | target_left_mm | 60 |
| Advance wall PD | target_right_mm | 60 |
| Advance base PWM | left | 3000 |
| Advance base PWM | right | 3370 |
| Sim motor gains | leftMotorGain | 1.0 |
| Sim motor gains | rightMotorGain | 0.89 |
| Wall perception | front threshold | 140 mm |
| Wall perception | side threshold | 100 mm |
| Wall perception | diagonal threshold | 145 mm |
| IR sensors | max range | 150 mm |
| Floor sensors | front local x | +42.0 mm |
| Floor sensors | rear local x | -42.0 mm |
| Floor sensors | separation | 84.0 mm |
| Special cell marker | default/recommended size | 120x120 mm |
| Smooth turn | target yaw-rate min | 60 deg/s |
| Smooth turn | target yaw-rate default | 120 deg/s |
| Smooth turn | target yaw-rate max | 120 deg/s |
| Pivot center correction | enabled | true |
| Pivot center correction | pivotCenterLocalXmm | -42.0 mm |
| Pivot center correction | pivotCenterLocalYmm | 0.0 mm |

## Tuning runtime

Abrir la ventana:

- Menu: `Tuning -> Control Tuning...`
- Tecla: `F3`

La ventana permite modificar en runtime:

- Turn yaw-rate PI: `Kp`, `Ki`, `Kd`, `output_limit_pwm`
- Advance yaw PD: `Kp`, `Ki`, `Kd`, `output_limit_pwm`
- Advance wall PD: `Kp`, `Kd`, `correction_limit_pwm`, `error_deadband_mm`, `target_left_mm`, `target_right_mm`

Los cambios no se guardan en archivo. Al reiniciar el programa vuelven los defaults del codigo.

## Pruebas manuales recomendadas

1. Iniciar el simulador y verificar en telemetria:
   - `smooth_target_yaw_rate_deg_s = 120 deg/s`
   - `wall_kp = 12`
   - `wall_kd = 600`
   - `wall_correction_limit_pwm = 4000`
   - `wall_error_deadband_mm = 0.0 mm`
   - `wall_front_threshold_mm = 140.0 mm`
   - `wall_side_threshold_mm = 100.0 mm`
   - `wall_diag_threshold_mm = 145.0 mm`
   - `sim_left_motor_gain = 1.000`
   - `sim_right_motor_gain = 0.890`
   - `sim_pivot_center_correction_enabled = true`
   - `sim_pivot_center_local_x_mm = -42.0 mm`
   - `sim_pivot_center_local_y_mm = 0.0 mm`

2. Probar `ADVANCE_LINE`:
   - Colocar el robot con `floor_rear` sobre una linea.
   - Usar `G` para iniciar avance.
   - Confirmar que termina al detectar la siguiente linea con el sensor trasero.
   - Con paredes laterales validas, confirmar que `advance_final_correction_source` pasa a `WALL_LEFT`, `WALL_RIGHT` o `WALL_CENTER`.

3. Probar smooth turns:
   - Usar `Q` y `E`.
   - Confirmar que terminan preferentemente por `REAR_SENSOR_TARGET_LINE`.
   - Confirmar que las bases smooth se recalculan segun el target.

4. Probar pivots:
   - Colocar `floor_rear` sobre cinta.
   - Usar `1`, `2` y `3`.
   - Confirmar que `sim_last_motion_was_pivot_like = true` durante el giro.
   - Confirmar que `floor_rear_global_x/y` se mantienen aproximadamente fijos durante el pivot.

5. Probar navegacion basica:
   - Activar auto mode y simulacion.
   - Activar navegacion basica con `B`.
   - Verificar que decide solo en punto valido de decision o adquiere linea si no esta sobre una.

## Parametros sensibles

- `Advance wall PD Kd`: valores altos amortiguan o corrigen rapido, pero pueden introducir picos grandes si el error cambia bruscamente.
- `Advance wall PD correction_limit_pwm`: valores altos dan autoridad al wall assist, pero pueden generar cambios de trayectoria fuertes.
- `wall_diag_threshold_mm`: al estar en 145 mm con IR max 150 mm, la confirmacion diagonal es permisiva.
- `smooth_target_yaw_rate_deg_s`: afecta radio de giro y bases PWM recalculadas.
- `rightMotorGain`: si cambia el desbalance del simulador, hay que recalibrar bases de avance y smooth turns.
- `pivotCenterLocalXmm/Ymm`: cambiarlo modifica donde queda fijo el robot durante pivots, aunque no cambia el origen visual.
- `Special cell marker size`: con sensores de suelo separados 84 mm, 100x100 mm deja una ventana de deteccion simultanea demasiado chica; 120x120 mm aumenta la robustez y sigue dejando separacion razonable respecto de las cintas de frontera.

## Notas

- `nav_core` sigue siendo portable y sin dependencias de Qt.
- La regla de mano derecha sigue en `MainWindow` como capa experimental.
- No hay mapa interno ni flood fill todavia.
- No hay persistencia de tuning runtime todavia.
