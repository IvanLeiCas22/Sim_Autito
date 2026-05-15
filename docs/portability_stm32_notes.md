# Notas de portabilidad a STM32 Bluepill

Este documento resume el estado actual de portabilidad del codigo de navegacion hacia STM32F103 Bluepill.

## Archivos portables

La logica portable esta principalmente en:

- `nav/nav_core.h`
- `nav/nav_core.c`
- `nav/nav_map.h`
- `nav/nav_map.c`
- `nav/nav_types.h`
- `nav/pid_controller.h`
- `nav/pid_controller.c`

Caracteristicas actuales:

- sin Qt;
- sin `float`/`double`;
- sin `malloc/free`;
- sin `new/delete`;
- sin funciones matematicas pesadas tipo `sin`, `cos`, `sqrt`, `atan2`;
- arrays fijos;
- tipos de ancho fijo donde corresponde;
- fixed-point Q16.16 para PID.

## Archivos no portables

No deberian ir directo al firmware:

- `app/mainwindow.h`
- `app/mainwindow.cpp`
- `sim/sim_world.h`
- `sim/sim_world.cpp`
- `sim/sim_robot.h`
- `sim/sim_robot.cpp`
- UI Qt;
- dibujo del overlay;
- JSON parsing del simulador;
- telemetria UI;
- logica de teclado.

`MainWindow` aun contiene orquestacion importante que conviene migrar gradualmente a una capa portable antes del firmware final.

## Estado actual del codigo portable

### Numeros

`nav_types.h` define:

- `q16_16_t`;
- `RobotSensors`;
- `RobotCommand`.

`pid_controller` usa:

- Q16.16;
- `int64_t` para multiplicaciones y divisiones intermedias;
- salida limitada por enteros.

### Planner BFS

El planner usa `NavRouteWorkspace` estatico en `nav_core.c`.

Esto evita buffers grandes en stack:

- `visited[NAV_ROUTE_MAX_STATES]`;
- `parent[NAV_ROUTE_MAX_STATES]`;
- `parent_action[NAV_ROUTE_MAX_STATES]`;
- `queue[NAV_ROUTE_MAX_STATES]`;
- `reverse_actions[NAV_PLAN_MAX_ACTIONS]`.

El workspace es compartido por:

- `nav_core_route_plan_to_cell(...)`;
- `nav_core_route_plan_to_nearest_frontier()`.

No es reentrante. Es aceptable para el flujo actual de control single-thread y para STM32.

### Memoria maxima del mapa

`nav_map.h` define:

- `NAV_MAP_MAX_WIDTH = 16`;
- `NAV_MAP_MAX_HEIGHT = 16`.

El maximo teorico de estados BFS es:

```text
16 * 16 * 4 = 1024 estados
```

## Riesgos para Bluepill

### SRAM

Bluepill STM32F103 tiene SRAM limitada. Aunque los buffers grandes salieron del stack, siguen ocupando memoria estatica.

Riesgo:

- mapa;
- workspace BFS;
- cola de plan;
- debug snapshots;
- buffers propios del firmware.

Recomendacion:

- medir `.bss` y `.data` en el linker map;
- bajar `NAV_MAP_MAX_WIDTH/HEIGHT` si el laberinto real lo permite;
- compilar sin telemetria pesada en firmware final si hace falta.

### `int64_t`

El PID usa `int64_t` para operaciones Q16.16.

Riesgo:

- costo CPU mayor en Cortex-M3;
- latencia si se llama en loop rapido.

Recomendacion:

- mantener por seguridad numerica al inicio;
- medir tiempo real de loop;
- si hace falta, migrar a escalas Q mas chicas o ganancias preescaladas.

### Divisiones

Hay divisiones enteras en:

- PID/fixed-point;
- escalas de yaw carry;
- escalas de diagonal guidance;
- calculos de control.

Riesgo:

- costo CPU si se ejecutan a alta frecuencia.

Recomendacion:

- usar `dt` fijo;
- limitar divisiones en paths criticos;
- precomputar escalas si el tuning queda fijo.

### Tuning de sensores reales

El simulador usa thresholds y distancias ideales:

- pared frontal;
- pared lateral;
- diagonal;
- marcas de piso;
- velocidades de motor.

Riesgo:

- IR reales no lineales;
- ruido;
- saturacion;
- luz ambiente;
- diferencia mecanica del robot real.

Recomendacion:

- calibrar tablas o curvas de IR reales;
- agregar filtros simples;
- validar umbrales con datos reales antes de activar SMART.

### Orquestacion aun en MainWindow

Actualmente `MainWindow` conserva:

- ejecucion de cola;
- SMART_RECOGNITION;
- secuencias compuestas;
- seleccion `FRONT_LINE` vs `FRONT_WALL`;
- validaciones de `J`;
- referencias de yaw del simulador.

Riesgo:

- port directo incompleto si solo se copia `nav_core`.

Recomendacion:

- crear una capa portable de supervisor;
- mover gradualmente decisiones de ejecucion de plan;
- dejar HAL/firmware solo para sensores, motores y tiempo.

## Recomendaciones para firmware real

- Mantener fixed-point.
- Usar loop con `dt` fijo.
- Empezar con controles menos agresivos.
- Deshabilitar features experimentales si complican bring-up.
- Validar primero:
  - lectura de piso;
  - deteccion de cintas;
  - yaw;
  - control de motores;
  - distancias IR.
- Activar en orden:
  1. `ADVANCE_LINE` con yaw only.
  2. Wall assist lateral.
  3. Smooth turns.
  4. Mapa sombra.
  5. Deteccion de especiales.
  6. Plan queue.
  7. Planner BFS.
  8. SMART_RECOGNITION.
- Considerar compilacion condicional para telemetria pesada.
- Revisar watchdog y timeout de primitivas.

## Estado recomendado antes de portar

Antes del firmware:

- aislar una interfaz portable de supervisor;
- definir HAL minima para sensores y motores;
- definir formato de telemetria serial liviano;
- medir SRAM;
- medir peor caso de tiempo de `nav_core_update`;
- crear tests unitarios de `nav_map` y planner BFS fuera de Qt.
