# Diseno de la capa portable `nav_supervisor`

Este documento describe el estado actual y el diseno previsto de la capa portable de
supervision de navegacion. Distingue entre comportamiento implementado, herramientas de
debug y trabajo futuro.

## 1. Motivacion

El modo 1 combina primitivas de movimiento, mapa logico, planner orientado, cola FIFO,
flood fill, deteccion de celdas especiales y mision de retorno al inicio. La primera
version concentraba demasiada orquestacion en `MainWindow`.

`nav_supervisor` existe para:

- evitar que `MainWindow` siga creciendo con logica de mision y politica;
- separar UI/simulacion de decisiones de navegacion;
- permitir portar la misma orquestacion a STM32;
- mantener comportamiento reproducible entre Qt y firmware;
- conservar `nav_core`, `nav_map`, `nav_flood` y el planner ya validados.

## 2. Arquitectura por capas

### Nivel 0 - HAL / simulacion

Actual en Qt:

- `SimWorld`: paredes, cintas, marcas especiales y geometria del mundo.
- `SimRobot`: cinematica, motores simulados y sensores simulados.
- `MainWindow`: loop Qt, teclado, UI, overlay, telemetria y adaptacion hacia `nav_core`.

Futuro en STM32:

- sensores IR;
- sensores de piso;
- gyro/yaw;
- timers;
- PWM de motores;
- botones o comandos externos.

### Nivel 1 - Percepcion

Convierte lecturas crudas en datos de navegacion:

- `RobotSensors`;
- paredes frontal/lateral/diagonal;
- piso delantero/trasero negro o blanco;
- yaw relativo.

En Qt, esta adaptacion sigue viviendo en `MainWindow`.

### Nivel 2 - Primitivas

Viven en `nav_core`:

- `ADVANCE_LINE` / `ADVANCE_UNTIL_REAR_BLACK`;
- `SMOOTH_LEFT` / `SMOOTH_RIGHT`;
- `PIVOT_LEFT`, `PIVOT_RIGHT`, `PIVOT_180`;
- `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE`;
- `APPROACH_FRONT_WALL_FOR_PIVOT`.

Estas primitivas son portables y no dependen de Qt.

### Nivel 3 - Mapa

`nav_map` mantiene:

- celda logica actual;
- orientacion discreta;
- celdas visitadas;
- paredes conocidas/presentes;
- celdas especiales detectadas.

### Nivel 4 - Planificacion

Componentes actuales:

- planner BFS orientado sobre `(cell_x, cell_y, dir)`;
- cola FIFO portable de `NavPlanAction`;
- `nav_flood` como capa portable de costos por celda;
- evaluacion debug de fronteras con flood en `MainWindow`.

El BFS orientado sigue siendo el que genera acciones fisicas ejecutables. `nav_flood`
calcula costos y por ahora no cambia el comportamiento de navegacion.

### Nivel 5 - Supervisor / mision

Actual:

- `nav/nav_supervisor.h`;
- `nav/nav_supervisor.c`.

`nav_supervisor` ya controla:

- mision modo 1 segura;
- latch de busqueda completa al encontrar N celdas especiales;
- bloqueo de SMART durante retorno;
- planificacion de retorno seguro al inicio mediante request hacia `MainWindow`;
- acciones locales de `SMART_RECOGNITION`;
- planificacion a frontera de `SMART_RECOGNITION`;
- estados principales de SMART y telemetria portable.

`MainWindow` aplica los requests del supervisor y sigue ejecutando las primitivas reales.

## 3. Estado actual real

### Implementado y validado

- `nav_supervisor` fue creado como modulo portable.
- C1: shadow/debug de mision modo 1 validado.
- C2: mision modo 1 segura migrada al supervisor.
- C3: `MainWindow` limpiado como adaptador de mision.
- C4B: shadow/debug de SMART agregado.
- C4C: acciones locales SMART migradas al supervisor.
- C4D: planificacion a frontera SMART migrada al supervisor.
- C4E: limpieza de integracion SMART; se elimino comparacion shadow como fuente activa.

### MainWindow hoy

`MainWindow` ya no es la fuente principal de decision para mision modo 1 segura ni para
SMART. Sigue siendo adaptador Qt/fisico:

- lee sensores simulados y arma snapshots;
- llama `nav_core_update(...)`;
- arranca primitivas `nav_core_start_*`;
- llama al planner real cuando `nav_supervisor` lo pide;
- activa y consume la cola con `advancePlanExecutionIfNeeded()`;
- maneja secuencias compuestas como `CENTER_AND_PIVOT_180`;
- resetea/aplica referencia de yaw del simulador;
- dibuja UI, overlay y telemetria;
- mantiene herramientas debug `K/J`, `T/J`, `I`, `Shift+F`.

### Debug experimental

La evaluacion de fronteras con flood (`Shift+F`) todavia vive en `MainWindow`. Es una
herramienta de debug: calcula mejor frontera, score, decision tentativa y accion de
entrada, pero no ejecuta acciones ni modifica la mision.

## 4. Responsabilidades actuales de `nav_supervisor`

`nav_supervisor` debe:

- mantener estado de mision;
- aplicar config de mision;
- detectar que se alcanzo `required_special_count`;
- pedir limpiar exploracion pendiente;
- esperar que termine la accion fisica actual;
- pedir plan seguro al inicio;
- pedir ejecucion de cola de retorno;
- producir `DONE`, `ERROR` o `CANCELLED`;
- decidir acciones locales SMART;
- pedir planificacion a frontera SMART;
- pedir ejecucion de plan de frontera;
- emitir estados, razones y flags de debug.

El supervisor coordina modulos existentes. No duplica primitivas ni planner.

## 5. Que no debe hacer `nav_supervisor`

`nav_supervisor` no debe:

- dibujar overlay;
- leer teclado;
- depender de Qt;
- acceder a `SimWorld` o `SimRobot`;
- parsear JSON;
- manejar PWM directamente;
- resetear yaw del simulador;
- ejecutar cinematicas;
- usar `malloc/free`;
- usar `float/double`;
- usar buffers grandes en stack.

## 6. Entradas y salidas

### Entradas actuales

El supervisor recibe snapshots portables como:

- `NavSupervisorInput` para mision;
- `NavSupervisorSmartInput` para SMART;
- config `NavSupervisorConfig`;
- celda inicial mediante `nav_supervisor_set_start_cell(...)`;
- notificaciones de rutas:
  - `nav_supervisor_notify_return_route_status(...)`;
  - `nav_supervisor_notify_frontier_route_status(...)`.

### Salidas actuales

Produce:

- `NavSupervisorOutput`;
- `NavSupervisorSmartOutput`;
- `NavSupervisorDebugSnapshot`.

`MainWindow` traduce esas salidas a:

- `nav_core_plan_clear()`;
- `nav_core_route_plan_to_cell(...)`;
- `nav_core_route_plan_to_nearest_frontier()`;
- `planExecutionEnabled = true`;
- `startBasicNavRecommendedAction(...)`;
- detener autonomia cuando corresponde.

## 7. Estados del supervisor

Estados de mision implementados:

- `NAV_SUPERVISOR_STATE_IDLE`;
- `NAV_SUPERVISOR_STATE_SEARCH_SPECIALS`;
- `NAV_SUPERVISOR_STATE_FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE`;
- `NAV_SUPERVISOR_STATE_RETURN_SAFE_PLAN`;
- `NAV_SUPERVISOR_STATE_RETURN_SAFE_EXECUTE`;
- `NAV_SUPERVISOR_STATE_DONE`;
- `NAV_SUPERVISOR_STATE_ERROR`;
- `NAV_SUPERVISOR_STATE_CANCELLED`.

Estados reservados/futuros:

- `NAV_SUPERVISOR_STATE_RETURN_SMART_DECIDE`;
- `NAV_SUPERVISOR_STATE_RETURN_FRONTIER_PLAN`;
- `NAV_SUPERVISOR_STATE_RETURN_FRONTIER_EXECUTE`;
- `NAV_SUPERVISOR_STATE_RETURN_FRONTIER_ENTER`.

Estados SMART implementados:

- `NAV_SUPERVISOR_SMART_STATE_IDLE`;
- `NAV_SUPERVISOR_SMART_STATE_LOCAL_UNVISITED`;
- `NAV_SUPERVISOR_SMART_STATE_PLAN_TO_FRONTIER`;
- `NAV_SUPERVISOR_SMART_STATE_EXECUTING_FRONTIER_ROUTE`;
- `NAV_SUPERVISOR_SMART_STATE_FRONTIER_ALREADY_HERE`;
- `NAV_SUPERVISOR_SMART_STATE_NO_FRONTIER`;
- `NAV_SUPERVISOR_SMART_STATE_ERROR`;
- `NAV_SUPERVISOR_SMART_STATE_BLOCKED_BY_MISSION`;
- `NAV_SUPERVISOR_SMART_STATE_WAIT_NAV_READY`.

## 8. Estrategias de retorno

### `SAFE_KNOWN_RETURN`

Actual e implementada:

- al encontrar N especiales, deja terminar la accion fisica actual;
- limpia exploracion pendiente;
- planifica a la celda inicial con `nav_core_route_plan_to_cell(...)`;
- ejecuta la cola;
- termina al llegar a la celda inicial.

### `GOAL_DIRECTED_RETURN`

Futuro:

- intentaria descubrir atajos hacia el inicio;
- usaria flood fill para evaluar fronteras;
- deberia mantener fallback seguro;
- necesita presupuesto, limite de intentos y blacklist de fronteras;
- no esta conectado al control real.

## 9. Uso de flood fill

Actual:

- `nav_flood` calcula costos por celda hacia un objetivo;
- la tecla `I` calcula flood hacia la celda inicial;
- el overlay muestra costos;
- `Shift+F` evalua fronteras candidatas como debug.

Rol correcto:

- flood fill sirve para costos globales y scoring;
- no reemplaza al planner orientado;
- no arranca primitivas;
- no cambia la mision actual.

Integracion futura:

1. `nav_flood` calcula costo hacia inicio.
2. El supervisor evalua si conviene probar una frontera.
3. El BFS orientado genera la cola hacia esa frontera.
4. El supervisor decide entrar o hacer fallback seguro.

## 10. Relacion con STM32

`nav_supervisor` esta disenado como portable:

- sin Qt;
- sin `malloc/free`;
- sin `float/double`;
- sin dependencia de `SimWorld`/`SimRobot`;
- con inputs/outputs explicitos.

Antes de firmware real falta:

- definir HAL para sensores, yaw, tiempo y motores;
- decidir que telemetria queda en firmware;
- medir SRAM y tiempo de loop;
- validar sensores reales con ruido.

## 11. Plan de migracion actualizado

- Etapa A - documentar diseno: completada.
- Etapa B - crear `nav_supervisor.h/c`: completada.
- Etapa C - migrar mision modo 1 segura: completada.
- Etapa D - migrar SMART/orquestacion local y planificacion a frontera: completada para SMART actual.
- Etapa E - integrar retorno inteligente con flood: pendiente.
- Etapa F - preparar HAL STM32: pendiente.

## 12. Riesgos actuales

- `MainWindow` aun ejecuta cola, secuencias compuestas y yaw del simulador.
- `GOAL_DIRECTED_RETURN` podria gastar mas tiempo que el retorno seguro si se implementa sin presupuesto.
- Flood frontier debug puede parecer decision real, pero todavia no ejecuta nada.
- Portar a STM32 requiere HAL y tuning real de sensores.
- La cola y planner siguen usando workspace estatico no reentrante, aceptable para el flujo actual.

## 13. Recomendacion

No volver a agregar logica de mision o SMART en `MainWindow`. La direccion actual es:

- mantener `MainWindow` como adaptador Qt;
- conservar primitivas y planner actuales;
- usar `nav_flood` como capa de costos;
- mover la evaluacion de fronteras y retorno inteligente a `nav_supervisor` cuando se active;
- preparar luego una HAL limpia para STM32.
