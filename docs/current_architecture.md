# Arquitectura actual del simulador

Estado: simulador Qt/C++ usado como banco físico/sensorial para ejecutar y depurar el núcleo portable del firmware STM32 real.

Última actualización de este documento: integración del supervisor portable `FIND_CELLS`, `FirmwareSimBridge` activo y soporte de `CENTER_BY_FRONT_TAPE_FOR_PIVOT` para backtracking abierto.

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
  sim_mainwindow.cpp/.h          UI Qt, carga de mapas, render y telemetría
  firmware_sim_bridge.cpp/.h     Adaptador entre sensores simulados y firmware_core

sim/
  sim_world.cpp/.h               Geometría del mapa, paredes, cintas, targets, raycast
  sim_robot.cpp/.h               Cinemática diferencial y pose del robot

firmware_core/
  app_nav.*                      Percepción, controladores y primitivas portables
  app_nav_supervisor.*           Supervisor de misión FIND_CELLS
  app_find_cells_policy.*        Política de exploración/backtracking por flood/BFS
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
- ejecutar `SupervisorV1` para la misión `FIND_CELLS`;
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

`SupervisorV1` es el modo relevante para validar la navegación portable real. Al arrancar, configura el supervisor en misión `APP_NAV_SUPERVISOR_MISSION_FIND_CELLS` y opcionalmente resetea la pose inicial del mapa lógico.

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
- fronteras de exploración.

## Regla de arquitectura principal

El simulador debe ser un banco de pruebas de la lógica portable STM32, no una segunda implementación de esa lógica.
