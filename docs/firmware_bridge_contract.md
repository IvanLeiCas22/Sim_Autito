# Contrato del Firmware Bridge

Este documento define el contrato de `FirmwareSimBridge`: conectar el mundo simulado Qt/C++ con el núcleo portable del firmware STM32 sin que el simulador implemente navegación propia.

## Estado actual

`FirmwareSimBridge` puede operar en dos situaciones:

1. **Sin `firmware_core` disponible**: modo seguro/stub, `TelemetryOnly`, PWM cero.
2. **Con `firmware_core` disponible**: el bridge inicializa y llama al núcleo portable real.

Cuando existe `firmware_core/app_nav.c`, CMake define `SIM_AUTITO_HAS_FIRMWARE_CORE=1`. Cuando además existe `app_nav_supervisor.c` y `app_nav_supervisor.h`, define `SIM_AUTITO_HAS_NAV_SUPERVISOR=1`.

## Rol del bridge

El bridge convierte:

```text
sensores simulados + dt + yaw/yaw-rate
        ↓
AppNavInput
        ↓
firmware_core portable
        ↓
AppNavOutput
        ↓
PWM para SimRobot + telemetría Qt
```

## Entrada al firmware portable

`FirmwareSimBridge::tick(...)` debe construir un `AppNavInput` coherente a partir de `SensorSnapshot`.

Entradas principales:

```text
dt_ms
ir_distance_mm[front_left]
ir_distance_mm[front_right]
ir_distance_mm[left]
ir_distance_mm[right]
ir_distance_mm[diag_left]
ir_distance_mm[diag_right]
floor_front_black
floor_rear_black
yaw_deg
yaw_rate_deg_s
```

El bridge es responsable de convertir esas señales al formato exacto esperado por `AppNavInput`, incluyendo canales ADC simulados cuando corresponda.

## Salida hacia el simulador

La salida pública del bridge hacia el robot simulado es:

```cpp
struct Command
{
    int left_pwm = 0;
    int right_pwm = 0;
};
```

El orden visible del simulador debe ser siempre:

```text
left_pwm
right_pwm
```

Si el firmware portable usa otra convención interna, la conversión debe quedar localizada en `FirmwareSimBridge`.

## Modos de operación

El bridge puede ejecutar modos de prueba de primitivas o el supervisor:

```text
TelemetryOnly
StraightYawHold
WallFollowAdvance
SmoothTurnLeft
SmoothTurnRight
PivotLeft90
PivotRight90
Pivot180
SupervisorV1
```

Los modos de primitivas sirven para depuración física/sensorial. La navegación de misión debe validarse usando `SupervisorV1`.

## SupervisorV1

Al iniciar `SupervisorV1`, el bridge debe:

1. asegurar que `firmware_core` esté inicializado;
2. detener primitivas directas activas;
3. resetear el supervisor o resetearlo con pose inicial explícita;
4. configurar la misión `APP_NAV_SUPERVISOR_MISSION_FIND_CELLS`;
5. llamar a `App_NavSupervisor_Start()`;
6. pasar a `ControlMode::SupervisorV1` solo si el arranque fue exitoso;
7. dejar `TelemetryOnly` y PWM cero si falla algún paso.

## GO_A_TO_B con aprendizaje entre runs

`GO_A_TO_B` debe usar el mismo supervisor portable y no debe implementar navegación propia en el simulador.

Al iniciar `GO_A_TO_B`, el bridge debe:

1. asegurar que `firmware_core` esté inicializado;
2. detener primitivas directas activas;
3. resetear runtime/pose del supervisor preservando el mapa aprendido mediante `App_NavSupervisor_ResetRunPreservingMapWithInitialPose(...)`;
4. configurar la celda objetivo `B`;
5. configurar la misión `APP_NAV_SUPERVISOR_MISSION_GO_A_TO_B`;
6. llamar a `App_NavSupervisor_Start()`.

La limpieza del mapa aprendido debe ser explícita mediante `FirmwareSimBridge::clearSupervisorLearnedMap()` o por reset completo de la simulación. No debe ocurrir automáticamente al iniciar una nueva run `GO_A_TO_B`.

## Configuración A/B desde HMI real

El simulador debe aceptar que la HMI Qt real sea la fuente de configuración de misión cuando está conectada por UNERBUS simulado.

`SimUnerbusLink` debe soportar estos comandos de supervisor:

```text
CMD_SET_SUPERVISOR_INITIAL_POSE = 0x98
CMD_GET_SUPERVISOR_INITIAL_POSE = 0x99
CMD_START_SUPERVISOR_RUN        = 0x9A
CMD_STOP_SUPERVISOR_RUN         = 0x9B
CMD_SET_SUPERVISOR_GOAL_CELL    = 0x9D
CMD_GET_SUPERVISOR_GOAL_CELL    = 0x9E
```

El estado configurado debe vivir en `FirmwareSimBridge`:

```text
pose inicial A: x, y, heading, valid
celda objetivo B: x, y, valid
```

Reglas:

- `CMD_GET_SUPERVISOR_INITIAL_POSE` devuelve la pose inicial configurada del simulador.
- `CMD_SET_SUPERVISOR_INITIAL_POSE` actualiza la pose inicial usada por el supervisor.
- `CMD_GET_SUPERVISOR_GOAL_CELL` devuelve la celda B configurada y su validez.
- `CMD_SET_SUPERVISOR_GOAL_CELL` actualiza B.
- `CMD_START_SUPERVISOR_RUN` arranca `FIND_CELLS` o `GO_A_TO_B` usando la pose/goal configurados, no un diálogo local ni otra fuente paralela.
- `CMD_STOP_SUPERVISOR_RUN` detiene el control supervisor y la simulación iniciada por HMI.

Las coordenadas expuestas por UNERBUS son coordenadas lógicas STM32. La conversión hacia filas visuales del simulador debe quedar localizada en el render/UI.

## Tick de SupervisorV1

En modo supervisor, el bridge debe llamar:

```cpp
AppNavOutput supervisor_output = {};
AppNavSupervisorState supervisor_state = App_NavSupervisor_Tick(&input, &supervisor_output);
```

Luego debe decidir si copia `supervisor_output` al comando del simulador.

## Gate de motores del supervisor

El bridge debe bloquear PWM en estados que no representen ejecución válida.

Estados que deben bloquear salida:

```text
APP_NAV_SUPERVISOR_IDLE
APP_NAV_SUPERVISOR_ERROR
```

Estados de ejecución o decisión conocidos deben permitir salida. Ejemplo actual importante:

```text
APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT
```

Este estado corresponde al centrado por cinta frontal antes de pivotar en backtracking abierto.

## Reglas al agregar un estado o acción nueva

Cada vez que se agregue un nuevo estado/acción en `app_nav_supervisor.h`, revisar obligatoriamente:

```text
app/firmware_sim_bridge.cpp
  supervisorStateText(...)
  supervisorActionText(...)
  supervisorStateAllowsMotorOutput(...)

app/sim_mainwindow.cpp
  Render de telemetría si se agregan campos de debug nuevos.

docs/
  Arquitectura y contrato del bridge si cambia el flujo.
```

Si no se actualiza el bridge, pueden aparecer síntomas como:

```text
supervisor: state=unknown action=unknown result=0
left_pwm=0
right_pwm=0
```

Ese patrón indica que el firmware puede estar funcionando, pero el adaptador del simulador no reconoce el estado/acción o bloquea el PWM.

## Debug y telemetría mínima

El bridge debe exponer al menos:

```text
state
reason
enabled
control_mode
advance_state
smooth_state
pivot_state
supervisor_state
supervisor_action
supervisor_result
left_pwm
right_pwm
pose lógica del mapa
paredes detectadas
sensores IR simulados
sensores de piso
```

Mejora recomendada para próximas etapas:

```text
supervisor_state_id
supervisor_action_id
supervisor_motor_output_allowed
supervisor_raw_left_pwm
supervisor_raw_right_pwm
```

Estos campos ayudan a separar tres problemas distintos:

1. el firmware genera PWM cero;
2. el bridge bloquea el PWM;
3. la UI está mostrando texto obsoleto.

## Responsabilidades permitidas

El bridge puede:

- convertir unidades;
- mapear sensores simulados a `AppNavInput`;
- llamar funciones del firmware portable;
- copiar `AppNavOutput` al simulador;
- proteger el simulador en estados `IDLE`/`ERROR`;
- exponer debug;
- ofrecer modos de prueba de primitivas.

## Responsabilidades prohibidas

El bridge no debe:

- decidir políticas de navegación;
- implementar flood fill propio;
- modificar el mapa lógico por fuera de `app_maze`;
- reemplazar decisiones del supervisor;
- traducir `BACKTRACK_REQUIRED` a una acción legacy distinta;
- ocultar clamps o decisiones de seguridad que pertenezcan al firmware portable.

## Criterio de aceptación

La integración bridge + firmware portable se considera correcta cuando:

1. el simulador compila sin HAL STM32;
2. `FirmwareSimBridge` llama al núcleo portable real;
3. Start/Stop/Reset controlan el firmware portable;
4. `SupervisorV1` arranca `FIND_CELLS`;
5. el robot se mueve solo por `AppNavOutput`;
6. la telemetría muestra estado/acción/result del supervisor;
7. los mapas de prueba reproducen casos de exploración, dead-end y backtracking abierto;
8. un nuevo estado/acción de supervisor no queda como `unknown` ni bloquea PWM por omisión.

## Telemetría extendida A/B

La telemetría pública de `FirmwareSimBridge::Debug` debe exponer los campos extendidos del supervisor:

```text
supervisor_mission
supervisor_go_to_b_phase
supervisor_go_to_b_outbound_steps
supervisor_go_to_b_optimistic_cost
supervisor_go_to_b_required_improvement
supervisor_go_to_b_improvement_detected
```

El puente UNERBUS simulado debe emitir el mismo payload compacto extendido que el STM32 real para `CMD_GET_SUPERVISOR_DEBUG_STATUS` y `CMD_SUPERVISOR_STATUS_UPDATE`.
