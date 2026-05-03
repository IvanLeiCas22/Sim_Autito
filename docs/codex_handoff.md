## Resumen Técnico Del Proyecto

### 1. Objetivo General

El proyecto es un simulador 2D en Qt C++ Widgets para un robot tipo micromouse/autito que navega un laberinto. El simulador busca validar gradualmente:

- Geometría del laberinto.
- Sensores simulados.
- Control manual.
- Pipeline `RobotSensors -> nav_core_update() -> RobotCommand`.
- Movimiento automático con cinemática diferencial.
- Acciones básicas de navegación como avance hasta línea, smooth turns y pivot turns.

La idea es mantener `nav_core` portable al STM32 Blue Pill, usando tipos enteros y Q16.16, sin depender de Qt ni de `float/double`.

---

### 2. Estructura Actual

Carpetas principales:

```text
app/
  main.cpp
  mainwindow.h
  mainwindow.cpp

sim/
  sim_world.h
  sim_world.cpp
  sim_robot.h
  sim_robot.cpp

nav/
  nav_types.h
  nav_core.h
  nav_core.c
  pid_controller.h
  pid_controller.c

data/
  maze_01.json

CMakeLists.txt
```

Archivos importantes:

- `app/mainwindow.*`: GUI, escena, dibujo, controles, telemetría, pipeline de simulación.
- `sim/sim_world.*`: laberinto, paredes, raycasting, cintas negras.
- `sim/sim_robot.*`: pose, yaw global, cinemática diferencial, yaw rate.
- `nav/nav_types.h`: tipos portables para sensores/comandos.
- `nav/nav_core.*`: lógica de navegación portable.
- `nav/pid_controller.*`: librería PID real del STM32 integrada al build.
- `CMakeLists.txt`: target Qt y fuentes C/C++.

---

### 3. Arquitectura General

#### MainWindow

Responsabilidades:

- Crear `QGraphicsScene` y `QGraphicsView`.
- Dibujar:
  - grilla 8x8
  - cintas negras
  - paredes
  - robot
  - rayos IR
  - sensores de piso
- Capturar teclado.
- Ejecutar loop de simulación con `QTimer` cada 10 ms.
- Construir `RobotSensors` en Q16.16.
- Llamar `nav_core_update()`.
- Guardar `lastNavCommand`.
- Aplicar movimiento automático solo si:
  - `simulationRunning == true`
  - `autoModeEnabled == true`
  - `motorTestModeEnabled == false`
- Mostrar telemetría en dock lateral.

#### SimWorld

Responsabilidades:

- Modelo interno del laberinto 8x8.
- Cada celda tiene paredes `North/East/South/West`.
- Mantiene consistencia de paredes vecinas.
- Define paredes exteriores e internas de prueba.
- Expone segmentos de paredes.
- Implementa `castRay()` contra paredes.
- Simula cintas negras:
  - cintas de límite entre celdas
  - cintas objetivo centrales
- Expone:
  - `isBlackTapeAt()`
  - `debugTapeKindAt()`
  - rectángulos de cinta para dibujo.

No depende de Qt.

#### SimRobot

Responsabilidades:

- Pose global:
  - `x_mm`
  - `y_mm`
  - `yaw_deg`
- Dimensiones:
  - largo 90 mm
  - ancho 80 mm
- Movimiento manual:
  - `moveForward()`
  - `rotate()`
  - `resetPose()`
- Cinemática diferencial:
  - `applyDifferentialDrive(left_pwm, right_pwm, dt_s)`
- Guarda `yawRateDegS_` calculado por la cinemática diferencial.
- No depende de Qt.

#### nav_core

Responsabilidades:

- Lógica portable de navegación.
- Usa `RobotSensors` y devuelve `RobotCommand`.
- No usa Qt.
- No usa `float` ni `double`.
- Usa Q16.16.
- Tiene acciones con estado:
  - avanzar hasta sensor trasero negro
  - smooth turn left/right
  - pivot turn left/right/180
- Usa `pid_controller` para control de velocidad angular de giros.

#### pid_controller

Librería PID real del STM32 integrada al build.

Responsabilidades:

- Control PID portable en C.
- Usa Q16.16.
- Mantiene estado:
  - `kp`, `ki`, `kd`
  - `setpoint`
  - `integral`
  - `prev_error`
  - límites de salida
- Tiene API compatible con enteros y API nueva Fixed para Q16.16 directo.

---

### 4. Convenciones Del Simulador

#### Coordenadas

- Origen: esquina superior izquierda del laberinto.
- X positivo: derecha.
- Y positivo: abajo.
- Unidad interna: milímetros.
- Celda: 200 mm.
- Laberinto base: 8x8.

#### Yaw Global

Usado por:

- dibujo del robot
- sensores en mundo
- raycasting
- movimiento
- cinemática diferencial

Convención:

- `yaw = 0°`: mira hacia +X
- `yaw = 90°`: mira hacia +Y
- `yaw = 180°`: mira hacia -X
- `yaw = 270°`: mira hacia -Y

Yaw positivo visualmente gira en sentido horario.

#### Yaw Relativo De Navegación

`MainWindow` mantiene:

```cpp
double navYawZeroDeg = 0.0;
```

Funciones:

```cpp
resetNavigationYawReference();
navigationYawDeg();
```

Antes de iniciar giros, `MainWindow` resetea la referencia:

```cpp
navYawZeroDeg = robot.yawDeg();
```

Luego `RobotSensors.yaw_deg_q16` recibe:

```cpp
navigationYawDeg() = robot.yawDeg() - navYawZeroDeg
```

Normalizado a `[-180, +180)`.

Este yaw relativo es el que usa `nav_core`.

#### Signo De Yaw/Yaw Rate

- Positivo: giro hacia la derecha.
- Negativo: giro hacia la izquierda.
- `SMOOTH_RIGHT`: target yaw `+90°`, yaw rate objetivo `+100 deg/s`.
- `SMOOTH_LEFT`: target yaw `-90°`, yaw rate objetivo `-100 deg/s`.
- `PIVOT_RIGHT`: target yaw `+90°`.
- `PIVOT_LEFT`: target yaw `-90°`.
- `PIVOT_180`: target yaw `180°`, por ahora gira hacia la derecha.

#### PWM Y Cinemática Diferencial

PWM simulado:

- Rango: `[-9999, +9999]`
- Positivo: motor hacia adelante.
- Negativo: motor hacia atrás.
- `3000 PWM` en ambas ruedas equivale aprox. a `200 mm/s`.
- Distancia entre ruedas: `73 mm`.

Modelo:

```cpp
v_left = left_pwm / 3000.0 * 200.0
v_right = right_pwm / 3000.0 * 200.0
linear_velocity = (v_left + v_right) / 2
angular_velocity = (v_left - v_right) / wheel_base
```

Con esta convención:

- izquierda más rápida que derecha -> yaw positivo -> giro derecho
- derecha más rápida que izquierda -> yaw negativo -> giro izquierdo

---

### 5. Sensores Simulados Actuales

#### IR De Pared

Seis sensores:

- `front_left`
- `front_right`
- `left`
- `right`
- `diag_left`
- `diag_right`

Cada uno tiene:

- posición local en el robot
- ángulo relativo
- rango máximo
- lectura `last_distance_mm`
- rayo visual
- punto de impacto si hay pared

Usan `SimWorld::castRay()`.

Rango actual: `130 mm`.

#### Sensores De Piso

Dos sensores:

- `floor_front`
  - local x = `+35 mm`
  - local y = `0`
- `floor_rear`
  - local x = `-35 mm`
  - local y = `0`

Detectan negro con:

```cpp
world.isBlackTapeAt(x, y)
```

También guardan debug:

```cpp
TapeDebugKind
```

Valores debug:

- `None`
- `Boundary`
- `Target`
- `BoundaryAndTarget`

Importante: `RobotSensors` solo recibe `BLACK/white`, no tipo debug.

#### Yaw

`RobotSensors.yaw_deg_q16` recibe yaw relativo de navegación en Q16.16.

#### Yaw Rate

`RobotSensors.yaw_rate_deg_s_q16` recibe `robot.yawRateDegS()` convertido a Q16.16.

Se calcula en `SimRobot::applyDifferentialDrive()`.

---

### 6. Telemetría Actual

Panel lateral derecho con secciones:

#### Pose

- `x`
- `y`
- `yaw`

Este yaw es global.

#### IR sensors

- `front_left`
- `front_right`
- `left`
- `right`
- `diag_left`
- `diag_right`

Distancias en mm.

#### Floor sensors

- `floor_front`
- `floor_rear`

Valores:

- `white`
- `BLACK (boundary)`
- `BLACK (target)`
- `BLACK (boundary+target)`

Esto es debug visual, no dato real de navegación.

#### Nav snapshot

- `floor_front_black`
- `floor_rear_black`
- `yaw_deg`
- `yaw_rate_deg_s`
- `yaw_zero_global`
- `nav_state`
- `nav_action`
- `action_start_yaw`
- `action_target_yaw`

#### Nav command

- `left_motor_pwm`
- `right_motor_pwm`

#### Simulation

- `running`
- `dt_ms`
- `auto_mode`
- `step_count`
- `sim_time_s`
- `motor_test`

#### Motor test command

- `test_left_pwm`
- `test_right_pwm`

---

### 7. Controles De Teclado Actuales

#### Manual

- `W` / Flecha arriba: avanzar 10 mm
- `S` / Flecha abajo: retroceder 10 mm
- `A` / Flecha izquierda: rotar -10°
- `D` / Flecha derecha: rotar +10°
- `R`: reset pose

#### Simulation

- `Space`: Play/Pause
- `N`: Step once si está pausado
- `M`: toggle auto mode

#### Motor test

- `T`: toggle motor test mode
- `I`: test PWM `{3000, 3000}`
- `K`: test PWM `{-3000, -3000}`
- `J`: test PWM `{-1500, 1500}`
- `L`: test PWM `{1500, -1500}`
- `U`: stop test motors `{0, 0}`

Motor test tiene prioridad sobre `nav_core` si `motor_test=true`.

#### Navigation test

- `G`: start `ADVANCE_LINE`
- `Q`: start `SMOOTH_LEFT`, resetea yaw relativo
- `E`: start `SMOOTH_RIGHT`, resetea yaw relativo
- `1`: start `PIVOT_LEFT`, resetea yaw relativo
- `2`: start `PIVOT_RIGHT`, resetea yaw relativo
- `3`: start `PIVOT_180`, resetea yaw relativo
- `X`: stop navigation action
- `Z`: reset navigation yaw reference

#### Help

- `H` o `F1`: abre ayuda
- menú `Help -> Controls`

---

### 8. Estado Actual De nav_core

#### NavState

```c
typedef enum NavState {
    NAV_STATE_IDLE = 0,
    NAV_STATE_ADVANCING_UNTIL_REAR_BLACK,
    NAV_STATE_SMOOTH_TURNING,
    NAV_STATE_PIVOT_TURNING,
    NAV_STATE_DONE
} NavState;
```

#### NavAction

```c
typedef enum NavAction {
    NAV_ACTION_NONE = 0,
    NAV_ACTION_ADVANCE_UNTIL_REAR_BLACK,
    NAV_ACTION_SMOOTH_TURN_LEFT,
    NAV_ACTION_SMOOTH_TURN_RIGHT,
    NAV_ACTION_PIVOT_TURN_LEFT,
    NAV_ACTION_PIVOT_TURN_RIGHT,
    NAV_ACTION_PIVOT_TURN_180
} NavAction;
```

#### Acciones Implementadas

##### ADVANCE_UNTIL_REAR_BLACK

- Si `floor_rear_black == false`: devuelve `{3000, 3000}`
- Si `floor_rear_black == true`: pasa a `DONE`, devuelve `{0, 0}`

##### SMOOTH_TURN_RIGHT

- Target yaw: `+90°`
- Target yaw rate: `+100 deg/s`
- Usa PI sobre yaw rate.
- PWM base:
  - left fast: `3500`
  - right slow: `1500`
- Finaliza por yaw relativo:
  - `yaw >= 90° - 3°`

##### SMOOTH_TURN_LEFT

- Target yaw: `-90°`
- Target yaw rate: `-100 deg/s`
- Usa PI sobre yaw rate.
- PWM base:
  - left slow: `1500`
  - right fast: `3500`
- Finaliza por yaw relativo:
  - `yaw <= -90° + 3°`

##### PIVOT_TURN_RIGHT

- Target yaw: `+90°`
- Target yaw rate: `+100 deg/s`
- Usa PI sobre yaw rate.
- Base:
  - left `+1000`
  - right `-1000`
- Finaliza por yaw relativo.

##### PIVOT_TURN_LEFT

- Target yaw: `-90°`
- Target yaw rate: `-100 deg/s`
- Usa PI sobre yaw rate.
- Base:
  - left `-1000`
  - right `+1000`
- Finaliza por yaw relativo.

##### PIVOT_TURN_180

- Target yaw: `180°`
- Por ahora gira hacia la derecha.
- Usa `abs(yaw)` para finalizar:
  - `abs(yaw) >= 180° - 3°`
- Base:
  - left `+1000`
  - right `-1000`

#### Acciones Pendientes

- Navegación completa.
- Decisión en intersecciones.
- Seguimiento de paredes con IR.
- Mantener recto con yaw o paredes.
- Flood fill / mapa.
- Calibraciones.
- Integrar reglas reales de cintas y celdas objetivo.
- Refinar smooth turn para condición de sensor trasero en línea objetivo.

---

### 9. PID

#### API Vieja

Sigue existiendo:

```c
void PID_Set_Setpoint(PID_Controller_t *pid, int32_t setpoint);
int32_t PID_Update(PID_Controller_t *pid, int32_t current_value, uint32_t dt_ms);
```

Estas funciones reciben enteros y convierten internamente a Q16.16.

#### API Fixed

Nueva API:

```c
void PID_Set_Setpoint_Fixed(PID_Controller_t *pid, int32_t setpoint_q16);
int32_t PID_Update_Fixed(PID_Controller_t *pid, int32_t current_value_q16, uint32_t dt_ms);
```

Permite trabajar directamente con Q16.16 sin perder decimales.

#### Uso Actual En Giros

`nav_core` usa un PID estático:

```c
static PID_Controller_t turn_yaw_rate_pid;
```

Configuración:

- `Kp = 8`
- `Ki = 0.50`
- `Kd = 0`
- salida limitada a `±4000 PWM`

Inicialización:

```c
PID_Init(&turn_yaw_rate_pid,
         INT_TO_FIXED(8),
         HUNDREDTHS_TO_FIXED(50),
         0);

PID_Set_Output_Limits(&turn_yaw_rate_pid,
                      INT_TO_FIXED(-4000),
                      INT_TO_FIXED(4000));
```

Uso:

```c
PID_Set_Setpoint_Fixed(..., ±INT_TO_FIXED(100));
PID_Update_Fixed(..., sensors->yaw_rate_deg_s_q16, 10);
```

La salida Q16.16 se convierte a PWM:

```c
correction_pwm = FIXED_TO_INT(pid_output_q16);
```

---

### 10. Reglas De Navegación Importantes

- Los giros izquierda/derecha en intersecciones deben ser smooth turns.
- Los pivot turns se reservan para:
  - calibraciones
  - callejones sin salida
  - media vuelta
- Antes de iniciar una maniobra de giro, `MainWindow` resetea la referencia de yaw de navegación.
- El yaw absoluto/global no se considera confiable a largo plazo para navegación.
- `nav_core` debe usar yaw relativo.
- Smooth turn real inicia con el sensor trasero sobre cinta.
- Smooth turn idealmente termina con el sensor trasero sobre la línea de la celda objetivo.
- Por ahora también puede terminar por yaw `±90°` con tolerancia.
- Pivot 180 termina con `abs(yaw_relativo)` cerca de `180°`.
- `RobotSensors` no debe incluir información debug como boundary/target.
- Debug de cinta es solo para GUI.

---

### 11. Últimos Cambios Realizados

Cambios recientes importantes:

- Integración de `pid_controller.h/.c` al build.
- `nav_core` usa `pid_controller` para smooth turns.
- PID pasó de P puro a PI:
  - `Kp = 8`
  - `Ki = 0.50`
  - `Kd = 0`
- Se agregó API Fixed al PID:
  - `PID_Set_Setpoint_Fixed`
  - `PID_Update_Fixed`
- `nav_core` usa `yaw_rate_deg_s_q16` directamente sin truncar decimales.
- Se agregaron pivot turns:
  - left
  - right
  - 180
- Se agregó `NAV_STATE_PIVOT_TURNING`.
- Se agregaron teclas `1`, `2`, `3`.
- El PID de giro se renombró internamente a `turn_yaw_rate_pid`.

---

### 12. Próximos Pasos Recomendados

1. Validar pivot turns en simulador:
   - dirección
   - yaw rate
   - corte en yaw
   - comportamiento con `motor_test=false`

2. Ajustar ganancias PI:
   - revisar estabilidad
   - revisar overshoot
   - revisar simetría izquierda/derecha

3. Agregar telemetría del PID:
   - setpoint yaw rate
   - measured yaw rate
   - correction PWM
   - integral opcional

4. Refinar smooth turn:
   - iniciar desde sensor trasero en línea
   - terminar con sensor trasero en línea de celda objetivo
   - mantener yaw como fallback o condición adicional

5. Implementar acción de mantener recto:
   - usando yaw relativo o sensores IR
   - probablemente PD

6. Implementar seguimiento de paredes:
   - usando IR laterales/diagonales
   - PD

7. Agregar lógica de intersecciones:
   - detectar línea/celda
   - decidir acción

8. Empezar estructura de mapa/flood fill cuando las primitivas estén robustas.

---

### 13. Riesgos Y Detalles A Cuidar

- `nav_core` debe seguir sin Qt, sin `float`, sin `double`.
- No pasar Q16.16 a funciones PID antiguas que convierten internamente.
- Para yaw rate, usar siempre `PID_Update_Fixed()`.
- Para setpoints Q16.16, usar `PID_Set_Setpoint_Fixed()`.
- Cuidar overflow en Q16.16.
- `abs_q16()` puede fallar si recibe `INT32_MIN`, aunque no debería ocurrir con yaw normalizado.
- `navigationYawDeg()` está normalizado a `[-180, +180)`, lo que afecta pivot 180.
- Pivot 180 cerca de `+180/-180` puede tener edge cases por la discontinuidad.
- `action_start_yaw_q16` y `action_target_yaw_q16` no se limpian al terminar, solo con `nav_core_stop()`.
- `ADVANCE_UNTIL_REAR_BLACK` sí limpia target/start actualmente al terminar.
- Motor test tiene prioridad si `motor_test=true`.
- Si `auto_mode=false`, `nav_core` puede generar comando pero no mueve el robot.
- Si `running=false`, tampoco hay movimiento automático.
- `MainWindow` hace reset de yaw relativo antes de Q/E/1/2/3.
- El yaw global sigue siendo el que usa la cinemática y el dibujo.
- La detección de cinta boundary/target es debug, no debe alimentar navegación real.
- El smooth turn real todavía no debería depender solo del yaw final; falta incorporar condición de sensor trasero/línea objetivo.