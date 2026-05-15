# Diseño de una capa portable de supervisor de navegación

## 1. Motivación

El modo 1 de navegación ya combina exploración local, mapa lógico, cola de acciones,
planner orientado, flood fill, detección de celdas especiales y retorno al inicio. Hoy
esa orquestación funciona, pero una parte importante vive en `MainWindow`.

El objetivo de una capa portable de supervisor es:

- Evitar que `MainWindow` siga creciendo con lógica de misión.
- Separar UI/simulación de decisiones de navegación.
- Hacer que el mismo comportamiento pueda ejecutarse en Qt y en STM32.
- Mantener decisiones reproducibles entre simulador y firmware.
- Conservar las primitivas y el planner actuales, que ya están validados.

## 2. Arquitectura por capas

### Nivel 0 - HAL / Simulación

**Actual en Qt**

- `SimWorld`: mapa físico, paredes, cintas y marcas especiales.
- `SimRobot`: cinemática, motores simulados y geometría.
- `MainWindow`: loop de simulación, sensores, UI, teclado y telemetría.

**Futuro en STM32**

- Lectura de sensores IR.
- Sensores de piso.
- Gyro/yaw.
- Timers.
- PWM de motores.
- Botones o comandos externos.

### Nivel 1 - Percepción

Convierte lecturas crudas en señales usadas por navegación:

- `RobotSensors`.
- Pared frontal/lateral/diagonal.
- Piso delantero/trasero negro o blanco.
- Yaw relativo.

En Qt, esta capa hoy se arma principalmente desde `MainWindow`.

### Nivel 2 - Primitivas

Viven en `nav_core` y generan comandos de movimiento:

- `ADVANCE_LINE` / `ADVANCE_UNTIL_REAR_BLACK`.
- `SMOOTH_LEFT`.
- `SMOOTH_RIGHT`.
- `PIVOT_LEFT`, `PIVOT_RIGHT`, `PIVOT_180`.
- `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE`.
- `APPROACH_FRONT_WALL_FOR_PIVOT`.

Estas primitivas no deberían depender de Qt ni de la UI.

### Nivel 3 - Mapa

`nav_map` mantiene el estado lógico:

- Celda lógica actual.
- Orientación discreta.
- Celdas visitadas.
- Paredes conocidas/presentes.
- Celdas especiales detectadas.
- Pose lógica y acciones que actualizaron el mapa.

### Nivel 4 - Planificación

Componentes actuales:

- Planner BFS orientado sobre `(cell_x, cell_y, dir)`.
- Cola FIFO portable de `NavPlanAction`.
- `nav_flood` como capa de costos por celda.
- Evaluación debug de fronteras usando flood fill.

El BFS orientado genera acciones físicas ejecutables. El flood fill calcula costos
globales por celda, pero no reemplaza a las primitivas.

### Nivel 5 - Supervisor / Misión

Capa propuesta:

- Estado de misión.
- Política de exploración.
- Decisión de retorno.
- Estrategia de retorno.
- Manejo de planes.
- Manejo de errores.
- Selección de próxima acción.

Esta capa debería ser portable y no conocer Qt.

## 3. Qué Existe Hoy

### Actual

- `nav_core` contiene primitivas, PID/control, detección de especiales, planner
  orientado, cola FIFO y debug portable.
- `nav_map` contiene el mapa lógico.
- `nav_flood` calcula costos por celda hacia un objetivo.
- `pid_controller` contiene control portable.
- `MainWindow` todavía orquesta:
  - `SMART_RECOGNITION`.
  - Ejecución de cola planificada.
  - Acción compuesta `CENTER_AND_PIVOT_180`.
  - Selección `FRONT_LINE` / `FRONT_WALL`.
  - Misión modo 1.
  - Retorno al inicio.
  - Teclas, overlay y telemetría.
  - Referencias de yaw del simulador.

### Importante

El sistema actual es estable y no conviene reemplazarlo de golpe. La migración debe
copiar comportamiento validado, no rediseñarlo durante el traslado.

## 4. Qué Debería Hacer `nav_supervisor`

Módulo futuro propuesto:

- `nav/nav_supervisor.h`
- `nav/nav_supervisor.c`

Responsabilidades:

- Mantener estado de misión.
- Implementar política de exploración de modo 1.
- Decidir cuándo buscar especiales y cuándo volver.
- Gestionar `SAFE_KNOWN_RETURN` y, más adelante, `GOAL_DIRECTED_RETURN`.
- Pedir planes al planner orientado.
- Usar `nav_flood` para evaluar costos.
- Decidir cuándo ejecutar cola.
- Manejar estados de error/cancelación.
- Exponer snapshot de debug portable.

El supervisor debería coordinar módulos existentes, no duplicar primitivas.

## 5. Qué No Debería Hacer `nav_supervisor`

`nav_supervisor` no debería:

- Dibujar overlay.
- Leer teclado.
- Depender de Qt.
- Acceder a `SimWorld` o `SimRobot`.
- Parsear JSON.
- Manejar PWM directamente.
- Implementar cinemática.
- Usar `malloc/free`.
- Usar `float/double`.
- Hacer operaciones pesadas no acotadas en el loop de control.

## 6. Entradas del Supervisor

Entradas conceptuales:

- `RobotSensors` o snapshot procesado equivalente.
- Estado actual de `nav_core`.
- Acción actual y último resultado de acción.
- Snapshot de `nav_map`.
- Estado de cola de plan.
- Configuración de misión:
  - misión habilitada;
  - cantidad requerida de celdas especiales;
  - estrategia de retorno.
- Eventos externos:
  - start autonomy;
  - stop/cancel;
  - reset;
  - mission enabled/disabled.

## 7. Salidas del Supervisor

Salidas conceptuales:

- Próxima acción recomendada.
- Solicitud de iniciar primitiva.
- Solicitud de planificar ruta.
- Solicitud de ejecutar cola.
- Solicitud de limpiar cola.
- Estado de misión.
- Motivo de done/error.
- Debug/telemetría portable.

Una integración Qt podría traducir estas salidas a llamadas actuales como
`nav_core_start_*`, `nav_core_route_plan_to_cell(...)` o ejecución de cola.

## 8. Estados Propuestos

Estados para modo 1:

- `IDLE`
- `SEARCH_SPECIALS`
- `FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE`
- `RETURN_SAFE_PLAN`
- `RETURN_SAFE_EXECUTE`
- `RETURN_SMART_DECIDE`
- `RETURN_FRONTIER_PLAN`
- `RETURN_FRONTIER_EXECUTE`
- `RETURN_FRONTIER_ENTER`
- `DONE`
- `ERROR`
- `CANCELLED`

### Actual

En `MainWindow` ya existen estados equivalentes a:

- `SEARCH_SPECIALS`
- `FOUND_REQUIRED_SPECIALS_WAIT_ACTION_DONE`
- `RETURN_TO_START_PLAN`
- `RETURN_TO_START_EXECUTE`
- `DONE`
- `ERROR`

### Futuro

Los estados `RETURN_SMART_DECIDE`, `RETURN_FRONTIER_PLAN`,
`RETURN_FRONTIER_EXECUTE` y `RETURN_FRONTIER_ENTER` pertenecen al retorno
inteligente y todavía no son comportamiento de misión activo.

## 9. Estrategias de Retorno

### `SAFE_KNOWN_RETURN`

Actual:

- Usa planner orientado hacia la celda inicial.
- Solo atraviesa mapa conocido/visitado según reglas actuales.
- Ejecuta la cola resultante.
- Es seguro y conservador.

### `GOAL_DIRECTED_RETURN`

Futuro:

- Intenta descubrir atajos hacia el inicio.
- Usa `nav_flood` para comparar costos.
- Evalúa fronteras candidatas.
- Usa BFS orientado para moverse hasta la frontera.
- Entra a la celda no visitada si la entrada es físicamente viable.
- Debe tener fallback a `SAFE_KNOWN_RETURN`.
- Debe usar presupuesto e intentos máximos para evitar explorar todo.

## 10. Uso de Flood Fill

Rol correcto de `nav_flood`:

- Capa de costos globales por celda.
- Útil para retorno, evaluación de fronteras y futuro modo 2.
- No reemplaza primitivas.
- No reemplaza necesariamente al BFS orientado.

Comparación:

- Flood fill por celda es liviano y bueno para costos globales.
- BFS orientado considera orientación y produce acciones físicas.
- Para ejecutar una ruta real, el planner orientado sigue siendo necesario.

Uso recomendado:

1. Flood calcula costo hacia inicio o hacia objetivos.
2. Supervisor elige una intención: volver seguro, probar frontera, terminar.
3. BFS orientado genera la cola de acciones.
4. `nav_core` ejecuta primitivas.

## 11. Relación con STM32

Restricciones para Bluepill/STM32F103:

- Sin Qt.
- Sin `malloc/free`.
- Sin `float/double`.
- Arrays fijos.
- Evitar stack grande.
- Trabajo acotado por tick.
- Telemetría liviana.
- HAL separado de navegación.

El supervisor debería usar tipos enteros y snapshots compactos. Si necesita buffers,
deberían ser estáticos o provistos por workspace explícito, como ya se hizo con el
planner BFS y `nav_flood`.

## 12. Plan de Migración Propuesto

### Etapa A - Documentar diseño

- Crear este documento.
- No tocar comportamiento.

### Etapa B - Crear módulo vacío/controlado

- Agregar `nav_supervisor.h/c`.
- Definir enums, config y snapshot debug.
- No integrarlo todavía al loop.

### Etapa C - Migrar misión modo 1

- Mover estados de misión desde `MainWindow` al supervisor.
- Mantener comportamiento igual.
- Comparar telemetría antes/después.

### Etapa D - Migrar SMART/orquestación local

- Mover decisión local de exploración.
- Mantener `MAP_PREFER_UNVISITED` como comportamiento de base.
- Evitar duplicar decisiones entre `MainWindow` y supervisor.

### Etapa E - Integrar retorno inteligente

- Usar flood fill y evaluación de fronteras.
- Agregar presupuesto de exploración de retorno.
- Mantener fallback seguro.

### Etapa F - Preparar HAL STM32

- Definir interfaz mínima para sensores, yaw, tiempo y motores.
- Mantener simulador como una implementación de HAL.
- Portar supervisor sin dependencias Qt.

## 13. Riesgos

- Migrar demasiado de golpe y romper navegación estable.
- Duplicar lógica entre `MainWindow` y `nav_supervisor`.
- Mezclar debug experimental con control real.
- Depender indirectamente de telemetría Qt.
- Subestimar ruido y latencias de sensores reales.
- Introducir buffers grandes en stack.
- Hacer que el supervisor conozca detalles de UI o simulación.
- Perder trazabilidad de razones de decisión.

## 14. Recomendación Final

Recomendación:

- No seguir agregando lógica de misión en `MainWindow` más allá de debug temporal.
- Conservar primitivas, planner orientado y cola FIFO actuales.
- Usar `nav_flood` como capa de costos, no como reemplazo inmediato del planner.
- Crear `nav_supervisor` de forma gradual y portable.
- Migrar primero la misión modo 1 segura, después SMART, y recién luego retorno
  inteligente.

La prioridad es preservar el comportamiento validado mientras se reduce la dependencia
de `MainWindow`.
