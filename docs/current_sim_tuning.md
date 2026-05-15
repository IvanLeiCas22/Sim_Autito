# Tuning actual del simulador

Estado documentado: modo 1 de navegacion en simulador Qt/C++ Widgets.

Estos valores son tuning validado en simulador. No deben asumirse como valores finales para el robot real ni para STM32 sin recalibracion con sensores, motores, bateria y `dt` reales.

## Geometria usada

- Celda del laberinto: `200 mm`.
- Sensores de piso:
  - `floor_front`: `+42 mm`.
  - `floor_rear`: `-42 mm`.
  - Separacion: `84 mm`.
- Centro de pivot simulado:
  - `pivotCenterLocalXmm = -42.0`.
  - `pivotCenterLocalYmm = 0.0`.
- Celdas especiales:
  - Tamano recomendado/default: `120 x 120 mm`.
  - Se definen en JSON con `special_cells`.
  - Si `size_mm` no esta presente, `SimWorld` usa el default de `120 mm`.

## Turn yaw-rate PI

Config portable usada por smooth turns y pivots.

| Parametro | Valor actual |
| --- | ---: |
| `Kp` | `8` |
| `Ki` | `0.5` |
| `Kd` | `0` |
| `output_limit_pwm` | `4000` |

En codigo, `Ki` se guarda como `NAV_SMOOTH_YAW_RATE_KI_X100 = 50`.

## Advance yaw PD

Config portable para avance sin referencia lateral suficiente.

| Parametro | Valor actual |
| --- | ---: |
| `Kp` | `50` |
| `Ki` | `0` |
| `Kd` | `2` |
| `output_limit_pwm` | `1200` |

El yaw hold de avance captura referencias relativas cuando cambia de pared a `YAW_ONLY`, para evitar que el robot intente volver a `0 deg` despues de perder una pared.

## Advance wall PD

Config portable para `ADVANCE_LINE` con paredes laterales.

| Parametro | Valor actual |
| --- | ---: |
| `Kp` | `12` |
| `Kd` | `600` |
| `correction_limit_pwm` | `4000` |
| `error_deadband_mm` | `0` |
| `target_left_mm` | `60` |
| `target_right_mm` | `60` |
| `single_side_error_scale` | `2` |

Reglas de error:

- `WALL_CENTER`: `right_distance_mm - left_distance_mm`.
- `WALL_LEFT`: `2 * (target_left_mm - left_distance_mm)`.
- `WALL_RIGHT`: `2 * (right_distance_mm - target_right_mm)`.

## Diagonal guidance

Config portable actual en `NavDiagonalGuidanceConfig`.

| Parametro | Valor actual |
| --- | ---: |
| `diag_kp` | `30` |
| `diag_kd` | `0` |
| `diag_correction_limit_pwm` | `1000` |
| `diag_error_scale_num` | `1` |
| `diag_error_scale_den` | `1` |
| `diag_error_scale` | `1/1` |
| `diag_target_mm` | `99` |
| `smooth_final_mode` | `SETPOINT` |

El control diagonal usa control propio separado del wall PD lateral. En el tuning actual queda en P-only (`diag_kd = 0`).

Uso actual:

- En `NAV_SMOOTH_PHASE_POST_YAW_SEEK_REAR_LINE`:
  - prioridad a diagonales;
  - modo configurable `HOLD_RELATIVE` o `SETPOINT`;
  - default actual: `SETPOINT`.
- En preview diagonal de `ADVANCE_LINE`:
  - se activa por latch cuando el sensor frontal detecta la cinta siguiente;
  - requiere armado previo para evitar que una marca especial central dispare el latch.

## Smooth yaw carry

Config portable actual en `NavSmoothYawCarryConfig`.

| Parametro | Valor actual |
| --- | ---: |
| `smooth_yaw_carry_enabled` | `true` |
| `smooth_yaw_carry_only_setpoint` | `true` |
| `smooth_yaw_carry_require_diag` | `true` |
| `smooth_yaw_carry_allow_advance_preview` | `true` |
| `smooth_yaw_carry_min_abs_deg` | `3` |
| `smooth_yaw_carry_max_abs_deg` | `15` |
| `smooth_yaw_carry_offset_scale_num` | `1` |
| `smooth_yaw_carry_offset_scale_den` | `1` |
| `smooth_yaw_carry_offset_scale` | `1/1` |

Fuentes actuales de candidato:

- `NAV_YAW_CARRY_CANDIDATE_SMOOTH_FINAL_DIAG`.
- `NAV_YAW_CARRY_CANDIDATE_ADVANCE_FRONT_DIAG_PREVIEW`.

La compensacion se consume solo antes de iniciar `SMOOTH_LEFT` o `SMOOTH_RIGHT`.

## Wall caution

Config portable actual en `NavWallCautionConfig`.

| Parametro | Valor actual del codigo |
| --- | ---: |
| `enabled` | `true` |
| `timeout_ms` | `400` |
| `delta_max_mm` | `10` |
| `kp` | `12` |
| `kd` | `600` |
| `correction_limit_pwm` | `1000` |

Nota de consistencia: el valor solicitado para documentacion era `correction_limit_pwm = 4000`, pero el default actual en codigo es `1000`. Este documento refleja el estado real del codigo. Si se quiere validar `4000` como tuning oficial, hay que cambiarlo desde F3 o ajustar el default portable.

Estados por lado:

- `NAV_WALL_CAUTION_CONFIDENCE_LOST`.
- `NAV_WALL_CAUTION_CONFIDENCE_CONFIRMED`.
- `NAV_WALL_CAUTION_CONFIDENCE_CAUTION`.

`CAUTION` aplica solo a `ADVANCE_LINE` y usa hold relativo de la distancia lateral capturada al perder la confirmacion diagonal.

## Umbrales de percepcion

| Umbral | Valor |
| --- | ---: |
| Pared frontal | `140 mm` |
| Pared lateral | `100 mm` |
| Pared diagonal | `145 mm` |
| Ventana minima de especial | `100 ms` |
| Ventana maxima de especial | `800 ms` |

## Ajuste desde UI

La ventana `Control Tuning` se abre con `F3`.

Permite editar:

- `Turn yaw-rate PI`.
- `Advance yaw PD`.
- `Advance wall PD`.
- `Diagonal guidance`.
- `Smooth yaw carry`.
- `Wall caution`.

Los valores son runtime y no quedan persistidos automaticamente en JSON ni en firmware.
