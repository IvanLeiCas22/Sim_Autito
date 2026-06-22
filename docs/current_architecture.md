# Arquitectura actual del simulador

Estado: simulador Qt/C++ usado como banco físico/sensorial para ejecutar y depurar el núcleo portable del firmware STM32 real.

Última actualización de este documento: integración del supervisor portable `FIND_CELLS`/`GO_A_TO_B`, sincronización A/B con la HMI real y enlace UDP estable para control desde la HMI.

## Objetivo

El simulador no debe contener una navegación paralela propia. Su función es modelar la planta física y adaptar esa planta al núcleo portable del firmware:

- mundo/laberinto cargado desde JSON;
- paredes físicas;
- cintas de frontera de celda;
- marcas especiales/target;
- robot diferencial simulado;
- sensores IR simulados por raycast;
- sensores de piso simulados;
- telemetría visual;
- puente Qt/C++ hacia `firmware_core/`.

La navegación activa debe venir del firmware portable copiado desde el proyecto STM32 real. El simulador puede tener modos manuales o de primitivas para depuración, pero no debe decidir por su cuenta la política de exploración.

## Estructura relevante

```text
app/
  main.cpp
  sim_mainwindow.cpp/.h          UI Qt, carga de mapas, render, telemetría y aplicación de pose física A
  firmware_sim_bridge.cpp/.h     Adaptador entre sensores simulados y firmware_core
  sim_unerbus_link.cpp/.h        Enlace UNERBUS/UDP para integración con la HMI real

sim/
  sim_world.cpp/.h               Geometría del mapa, paredes, cintas, targets, raycast
  sim_robot.cpp/.h               Cinemática diferencial y pose del robot

firmware_core/
  app_nav.*                      Percepción, controladores y primitivas portables
  app_nav_supervisor.*           Supervisor de misión FIND_CELLS / GO_A_TO_B
  app_find_cells_policy.*        Política de exploración/backtracking por flood/BFS
  app_go_to_b_policy.*           Política de ruta hacia B usando mapa aprendido
  app_route_planner.*            Planner BFS/optimista portable
  app_maze.*                     Mapa lógico portable
  pid_controller.*               PID/Q16 portable
  README.md                      Reglas de sincronización y ownership

tools/
  sync_firmware_core_from_stm32.cmd
```

## Flujo actual de simulación

```text
SimWorld + SimRobot
        ↓
actualización de sensores simulados
        ↓
FirmwareSimBridge::tick(...)
        ↓
AppNavInput portable
        ↓
app_nav / app_nav_supervisor / app_find_cells_policy / app_maze
        ↓
AppNavOutput portable
        ↓
PWM izquierdo/derecho del simulador
        ↓
SimRobot::applyDifferentialDrive(...)
        ↓
render + telemetría Qt
```

`FirmwareSimBridge` ya no es un stub cuando `firmware_core/app_nav.c` existe. En ese caso, CMake compila el núcleo portable y define `SIM_AUTITO_HAS_FIRMWARE_CORE=1`. Si también existe `firmware_core/app_nav_supervisor.c` junto con su header, se define `SIM_AUTITO_HAS_NAV_SUPERVISOR=1`.

Si falta el núcleo portable, el bridge debe permanecer seguro: modo `TelemetryOnly`, PWM en cero y telemetría indicando que el firmware no está disponible.

## Integración de CMake

`CMakeLists.txt` incluye siempre el simulador Qt y, si existen, agrega fuentes de `firmware_core`:

```text
firmware_core/app_nav.c
firmware_core/pid_controller.c
firmware_core/app_maze.c
firmware_core/app_find_cells_policy.c
firmware_core/app_nav_supervisor.c
```

Esto permite que el simulador compile tanto en modo stub como en modo firmware portable conectado.

## FirmwareSimBridge

`FirmwareSimBridge` es el único punto de contacto entre Qt/simulación y el firmware portable.

Responsabilidades principales:

- convertir distancias IR simuladas a campos de `AppNavInput`;
- convertir sensores de piso simulados a la representación esperada por el firmware;
- entregar `dt_ms`, yaw y yaw-rate;
- inicializar/configurar `app_nav` y `app_nav_supervisor`;
- ejecutar modos de prueba de primitivas cuando se seleccionan desde la UI;
- ejecutar `SupervisorV1` para las misiones `FIND_CELLS` y `GO_A_TO_B`;
- copiar `AppNavOutput.left_motor_pwm/right_motor_pwm` al comando del robot simulado;
- exponer snapshots de debug y mapa para telemetría.

El bridge no debe:

- implementar regla de mano derecha propia;
- implementar flood fill propio;
- modificar el mapa lógico por fuera de `app_maze`;
- decidir si avanzar, girar o pivotar en modo supervisor;
- duplicar máquinas de estado del firmware.

## Modos de control

El bridge mantiene varios modos para depuración:

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

`SupervisorV1` es el modo relevante para validar la navegación portable real. Al arrancar, configura el supervisor en misión `APP_NAV_SUPERVISOR_MISSION_FIND_CELLS` o `APP_NAV_SUPERVISOR_MISSION_GO_A_TO_B`, según la solicitud de la UI local o de la HMI real.

## Ownership de navegación portable

La separación funcional esperada es:

```text
app_nav
  Percepción, controladores y primitivas.
  No debe ser dueño de la misión FIND_CELLS.

app_nav_supervisor
  Secuencia de misión, estado global, arranque/parada de primitivas,
  actualización del mapa lógico y conteo de celdas especiales.

app_find_cells_policy
  Decisión de exploración: vecinos inmediatos no visitados,
  ruta a frontera y backtracking requerido.

app_go_to_b_policy / app_route_planner
  Decisión de ruta hacia B y comparación optimista para aprendizaje entre runs.

app_maze
  Pose lógica, paredes conocidas/presentes, celdas visitadas y especiales.
```

## Estados/acciones de supervisor y telemetría

Cada vez que se agregue un nuevo `AppNavSupervisorState` o `AppNavSupervisorAction`, se debe revisar el bridge:

1. `supervisorStateText(...)` para mostrar texto legible.
2. `supervisorActionText(...)` para mostrar texto legible.
3. `supervisorStateAllowsMotorOutput(...)` para decidir si el PWM del supervisor debe pasar al robot.
4. Telemetría Qt para que no aparezca `unknown` sin valor numérico útil.
5. Documentación y mapas de regresión si el nuevo estado corrige un caso físico concreto.

Caso ya integrado:

```text
APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT = 10
APP_NAV_SUPERVISOR_ACTION_CENTER_FRONT_TAPE_FOR_PIVOT = 7
```

Este estado/acción se usa para backtracking abierto: cuando el robot no está en dead-end pero debe girar 180° desde una celda abierta, primero se centra con cinta frontal y luego pivota.

## Integración con la HMI real

La HMI Qt real puede controlar el simulador por UNERBUS/UDP. El simulador debe comportarse como endpoint estable, no como emisor desde puerto efímero.

Puertos por defecto:

```text
HMI real local/listen: 30010
Simulador local/listen: 30011
```

Flujo esperado:

```text
Simulador -> HMI real: telemetría/alive desde 30011 hacia 30010
HMI real -> Simulador: comandos UNERBUS hacia 30011
```

El simulador debe soportar la configuración A/B de la HMI real:

```text
CMD_SET_SUPERVISOR_INITIAL_POSE
CMD_GET_SUPERVISOR_INITIAL_POSE
CMD_SET_SUPERVISOR_GOAL_CELL
CMD_GET_SUPERVISOR_GOAL_CELL
CMD_START_SUPERVISOR_RUN
CMD_STOP_SUPERVISOR_RUN
```

La pose inicial `A` tiene dos representaciones que deben mantenerse coherentes:

```text
A lógica STM32        estado de supervisor / UNERBUS
pose física SimRobot  posición real del robot virtual para sensores y render
```

Al cargar o resetear un mapa JSON, `A` se deriva de `startXMm`, `startYMm` y `startYawDeg`. Cuando la HMI real envía `SET A`, el simulador debe actualizar la `A` lógica y mover físicamente el `SimRobot` a la celda/orientación correspondiente antes de iniciar una run.

Los indicadores visuales `A/B` no forman parte del simulador. La visualización de inicio/meta queda en la HMI real; el simulador solo debe exponer y aplicar el estado lógico/físico correcto.

## Gate de salida PWM del supervisor

El bridge llama a `App_NavSupervisor_Tick(...)` y obtiene un `AppNavOutput`. Ese output solo debe pasar al robot si el estado del supervisor permite movimiento.

Estados seguros para bloquear motor:

```text
APP_NAV_SUPERVISOR_IDLE
APP_NAV_SUPERVISOR_ERROR
```

Estados de ejecución/decisión conocidos deben permitir pasar PWM, incluyendo:

```text
APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT
```

Este gate existe para que un estado desconocido o un error no mueva el robot por accidente, pero debe mantenerse sincronizado con los estados nuevos del supervisor.

## Mapas y pruebas

Los mapas JSON de `data/test_maps/` son casos de prueba manual/regresión. Actualmente no deben depender de navegación legacy del simulador. Deben validar el comportamiento del firmware portable a través del bridge.

Casos relevantes:

- geometría y sensores;
- detección de paredes;
- detección de cintas de frontera;
- detección de celdas especiales;
- smooth turns;
- dead-ends;
- backtracking abierto por cinta frontal;
- fronteras de exploración;
- runs `GO_A_TO_B` usando mapa aprendido y meta B configurada desde la HMI real.

## Regla de arquitectura principal

El simulador debe ser un banco de pruebas de la lógica portable STM32, no una segunda implementación de esa lógica.
