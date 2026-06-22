# firmware_core

Carpeta que contiene la copia del núcleo portable de navegación proveniente del proyecto STM32 real.

## Regla principal

Los archivos de esta carpeta no deben convertirse en una navegación alternativa del simulador. Deben ser copia directa o casi directa del núcleo portable del firmware real.

El simulador debe adaptar sensores y motores desde `FirmwareSimBridge`; la lógica de navegación debe permanecer en C portable dentro de `firmware_core/`.

## Estado actual

Esta carpeta ya no es solo una reserva futura. Actualmente el simulador puede compilar y ejecutar el núcleo portable si los archivos están presentes.

Archivos activos esperados:

```text
app_nav.c
app_nav.h
app_nav_types.h
app_nav_config.h

app_nav_supervisor.c
app_nav_supervisor.h

app_find_cells_policy.c
app_find_cells_policy.h

app_go_to_b_policy.c
app_go_to_b_policy.h
app_route_planner.c
app_route_planner.h

app_maze.c
app_maze.h
app_maze_types.h

pid_controller.c
pid_controller.h
```

## Ownership funcional

```text
app_nav
  Percepción, controladores y primitivas portables.
  Incluye avance, smooth, pivot, approach front wall y center front tape.

app_nav_supervisor
  Supervisor de misión.
  Ejecuta FIND_CELLS y GO_A_TO_B, arranca/detiene primitivas, actualiza mapa y reporta debug.

app_find_cells_policy
  Política de exploración.
  Decide vecino inmediato, ruta a frontera o BACKTRACK_REQUIRED usando BFS/flood conceptual.

app_go_to_b_policy / app_route_planner
  Política de ruta hacia B y planner BFS/optimista sobre el mapa aprendido.

app_maze
  Mapa lógico portable.
  Pose, heading, paredes conocidas/presentes, celdas visitadas y celdas especiales.

pid_controller
  Controladores enteros/Q16 portables.
```

## Flujo de sincronización

La copia desde el proyecto STM32 debe hacerse con:

```bat
tools\sync_firmware_core_from_stm32.cmd <STM32_REPO_ROOT>
```

Ejemplo:

```bat
tools\sync_firmware_core_from_stm32.cmd C:\Users\GAMING\Desktop\MICROCONTROLADORES\MICROCONTROLADORES-STM32
```

Después de sincronizar:

1. compilar el simulador en Qt;
2. revisar que CMake detecte `SIM_AUTITO_HAS_FIRMWARE_CORE=1`;
3. revisar que detecte `SIM_AUTITO_HAS_NAV_SUPERVISOR=1` si está el supervisor;
4. probar `SupervisorV1` desde la UI;
5. validar telemetría de estado/action/result.

## Integración con CMake

`CMakeLists.txt` agrega `firmware_core/app_nav.c` si existe. También agrega fuentes opcionales si están presentes:

```text
firmware_core/pid_controller.c
firmware_core/app_maze.c
firmware_core/app_find_cells_policy.c
firmware_core/app_go_to_b_policy.c
firmware_core/app_route_planner.c
firmware_core/app_nav_supervisor.c
```

Esto permite que el simulador siga compilando aunque la copia portable esté incompleta, pero una navegación real de `FIND_CELLS`/`GO_A_TO_B` requiere el set completo.

## Restricciones de portabilidad

El código dentro de esta carpeta no debe depender de:

- Qt;
- HAL STM32;
- UART/USB concretos;
- SSD1306/OLED;
- interrupciones específicas;
- rutas absolutas del proyecto STM32;
- archivos del simulador;
- heap dinámico para lógica crítica;
- `float`/`double` para navegación portable.

Si un archivo necesita HAL o Qt, todavía no está suficientemente extraído para vivir acá.

## Relación con FirmwareSimBridge

`FirmwareSimBridge` puede incluir headers de `firmware_core` dentro de `extern "C"`, construir `AppNavInput`, llamar funciones portables y copiar `AppNavOutput` al simulador.

El bridge no debe modificar reglas internas del firmware. Si se necesita cambiar navegación, hacerlo en el proyecto STM32 real y luego sincronizar `firmware_core` desde STM32.

## Agregar una nueva primitiva o estado de supervisor

Cuando se agregue una primitiva nueva o un estado/acción de supervisor:

1. actualizar headers y fuentes en el proyecto STM32 real;
2. compilar STM32 real;
3. sincronizar `firmware_core` hacia el simulador;
4. compilar Qt;
5. actualizar `FirmwareSimBridge` si hay enums nuevos;
6. actualizar telemetría si hay debug nuevo;
7. agregar o actualizar mapa de regresión manual;
8. actualizar documentación.

Ejemplo ya integrado:

```text
APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT = 10
APP_NAV_SUPERVISOR_ACTION_CENTER_FRONT_TAPE_FOR_PIVOT = 7
```

Este caso requiere que el bridge:

```text
supervisorStateText(...)       reconozca el estado
supervisorActionText(...)      reconozca la acción
supervisorStateAllowsMotorOutput(...) permita pasar PWM en ese estado
```

## Regla de mantenimiento

La fuente de verdad de navegación es el proyecto STM32 real. Esta carpeta es una copia portable para que el simulador pueda reproducir la lógica real con sensores simulados.
