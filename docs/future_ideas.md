# Ideas futuras no implementadas

Este documento lista ideas discutidas o utiles para etapas posteriores. No describe comportamiento actual garantizado.

## Pivot 180 preciso como recalibracion

Idea para maniobra de recalibracion mas robusta:

1. Acercarse a una pared frontal.
2. Alinear con sensores frontales.
3. Hacer pivot 90 hacia una pared lateral util.
4. Acercarse/alinearse contra esa pared.
5. Hacer pivot 90 final.

Objetivo:

- reducir acumulacion de error angular;
- aprovechar paredes reales como referencia;
- mejorar posicion antes de modo 2.

Estado:

- no implementado;
- no integrado al planner;
- no hay primitive dedicada para alineacion frontal fina.

## Confianza de paredes mas avanzada

Actualmente existe `Wall caution` para `ADVANCE_LINE` con estados:

- `LOST`;
- `CONFIRMED`;
- `CAUTION`.

Ideas:

- confianza continua por lado;
- hysteresis por distancia;
- confirmacion temporal multi-tick;
- mezcla controlada de pared confirmada + pared en caution;
- mapear incertidumbre de paredes en `nav_map`.

Estado:

- no implementado.

## Filtros para sensores reales

Ideas:

- filtro mediana o IIR simple para IR;
- debouncing para sensores de piso;
- validacion por ventana temporal;
- rechazo de saltos imposibles;
- calibracion por tabla.

Motivo:

- el simulador es limpio;
- los sensores reales tendran ruido, saturacion y no linealidad.

Estado:

- no implementado en `nav_core`.

## Migrar mas orquestacion desde MainWindow

Hoy `MainWindow` maneja:

- ejecucion de cola;
- SMART_RECOGNITION;
- secuencias compuestas;
- seleccion `FRONT_LINE` vs `FRONT_WALL`;
- validaciones de `J`;
- referencias de yaw del simulador.

Idea:

- crear un supervisor portable de alto nivel;
- dejar a la app Qt solo UI/sim;
- permitir reusar ese supervisor en STM32.

Estado:

- pendiente.

## Modo 2

Ideas:

- usar mapa completo;
- planificar ruta a objetivo;
- optimizar secuencia de acciones;
- elegir rutas por costo real y no solo BFS uniforme;
- usar celdas especiales como objetivos o checkpoints.

Estado:

- no implementado;
- el planner actual es base para esto, pero todavia es modo 1.

## Flood fill o Dijkstra

Ideas:

- flood fill para micromouse clasico;
- Dijkstra/A* con costos por tipo de accion;
- penalizar `CENTER_AND_PIVOT_180` por duracion;
- preferir smooths consecutivos cuando sean fisicamente estables.

Estado:

- no implementado.

## Pivots 90 generales

Actualmente el planner no usa pivots 90 generales.

Ideas:

- `CENTER_AND_PIVOT_LEFT`;
- `CENTER_AND_PIVOT_RIGHT`;
- reorientacion in-cell sin avanzar;
- restricciones fisicas segun `rear_line_trusted_for_decision`.

Estado:

- no implementado.

## Centering para pivots fuera de callejon

La primitiva `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE` existe y funciona como base.

Ideas:

- variantes con sensor delantero y estimacion de centro;
- variantes con distancia frontal si hay pared;
- validacion por velocidad/odometria;
- freno ajustable segun velocidad.

Estado:

- parcialmente cubierto por `CENTER_IN_CELL_FOR_PIVOT_BY_FRONT_LINE` y `APPROACH_FRONT_WALL_FOR_PIVOT`;
- no hay familia completa de primitives para pivots 90.

## Persistencia de tuning

La UI `Control Tuning` permite editar valores en runtime.

Ideas:

- guardar perfiles de tuning;
- cargar defaults por mapa;
- exportar configuracion para firmware;
- comparar perfiles para simulador y robot real.

Estado:

- no implementado.

## Tests automaticos

Ideas:

- tests unitarios para `nav_map`;
- tests de planner BFS;
- simulaciones headless de mapas stress;
- assertions de `NO_FRONTIER`;
- metricas de cobertura de celdas visitadas/especiales.

Estado:

- existe checklist manual y mapas stress;
- no hay suite automatica completa.

## Telemetria firmware

Ideas:

- protocolo serial compacto;
- snapshot binario de `nav_core`;
- flags de debug por modulo;
- muestreo reducido para no saturar UART.

Estado:

- no implementado para STM32.
