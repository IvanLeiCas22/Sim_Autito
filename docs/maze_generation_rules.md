# Reglas para generar mapas JSON

Reglas para crear mapas compatibles con el simulador Qt/C++ de micromouse/autito.

## Estado actual del simulador

El simulador es un banco físico/sensorial conectado al núcleo portable del firmware STM32 mediante `FirmwareSimBridge`.

Actualmente puede ejecutar `SupervisorV1` con misión `FIND_CELLS` cuando `firmware_core` está presente. No debe usarse navegación legacy propia del simulador como fuente de verdad.

Los mapas JSON deben servir para validar:

- geometría física;
- sensores IR por raycast;
- sensores de piso;
- paredes;
- cintas de frontera;
- marcas de celda especial;
- primitivas portables;
- supervisor portable;
- política `FIND_CELLS`;
- backtracking abierto y dead-ends.

## Carpetas

- `data/test_maps/`: mapas útiles para pruebas manuales y regresiones del simulador actual.
- `data/stress_maps/` o `data/dev_maps/`: opcionales para mapas extremos o experimentales.
- Mapas sueltos en `data/`: pruebas rápidas o compatibilidad.

Los mapas en `data/test_maps/` deben ser físicamente válidos y reproducibles. No deben depender de estados legacy eliminados del simulador.

## Parser real

El parser usado por el simulador es:

```cpp
SimWorld::loadFromJsonFile(...)
```

Por lo tanto, las reglas de este documento se basan en lo que acepta `SimWorld`.

## Formato JSON recomendado

```json
{
  "name": "nombre_del_mapa",
  "cells": {
    "width": 8,
    "height": 8,
    "cell_size_mm": 200
  },
  "start": {
    "x_mm": 100,
    "y_mm": 100,
    "yaw_deg": 0
  },
  "special_cells": [
    { "cell_x": 3, "cell_y": 2, "size_mm": 120 }
  ],
  "walls": [
    { "cell_x": 1, "cell_y": 0, "dir": "E" }
  ]
}
```

Campos principales:

- `name`: nombre visible del mapa.
- `cells.width`: cantidad de columnas.
- `cells.height`: cantidad de filas.
- `cells.cell_size_mm`: tamaño de celda en milímetros.
- `start.x_mm`: posición inicial física X.
- `start.y_mm`: posición inicial física Y.
- `start.yaw_deg`: yaw inicial físico.
- `walls`: paredes internas.
- `special_cells`: marcas especiales/target.

Alias aceptados por compatibilidad:

- raíz `cols`, `rows`, `cell_size_mm`;
- `col`/`row` como alias de `cell_x`/`cell_y`.

Para mapas nuevos, usar siempre el formato recomendado.

## Coordenadas

Sistema lógico:

```text
cell_x = columna
cell_y = fila
origen lógico = esquina superior izquierda (0, 0)
X crece hacia Este
Y crece hacia Sur
```

Sistema físico:

```text
x_mm = coordenada horizontal en mm
y_mm = coordenada vertical en mm
cell_size_mm recomendado = 200
centro de celda (x, y) = ((cell_x + 0.5) * cell_size_mm, (cell_y + 0.5) * cell_size_mm)
```

Para una celda de 200 mm, el centro de `(0, 0)` es:

```text
x_mm = 100
y_mm = 100
```

## Yaw inicial

Convención esperada:

```text
0°   = Este
90°  = Sur
180° = Oeste
270° / -90° = Norte
```

Usar valores simples y cardinales cuando el mapa se use para probar navegación.

## Paredes

Formato:

```json
{ "cell_x": 1, "cell_y": 0, "dir": "E" }
```

Direcciones aceptadas:

```text
N / NORTH
E / EAST
S / SOUTH
W / WEST
```

Reglas:

1. Definir solo paredes internas necesarias.
2. Las paredes exteriores del laberinto son agregadas automáticamente por `SimWorld::addBoundaryWalls()`.
3. Al agregar una pared interna, `SimWorld::setWall(...)` refleja la pared opuesta en la celda vecina si existe.
4. No duplicar paredes internas salvo que sea intencional por legibilidad.
5. No crear paredes fuera del rango del mapa.

## Cintas de frontera

Las cintas de frontera entre celdas no se definen manualmente en JSON. El simulador las genera según las divisiones de la grilla.

Para `cell_size_mm = 200`, existen líneas de frontera en:

```text
x = 200, 400, 600, ...
y = 200, 400, 600, ...
```

Estas cintas son detectadas por los sensores de piso simulados y se usan para confirmar ingreso/salida de celda.

## Celdas especiales

Formato:

```json
{ "cell_x": 3, "cell_y": 2, "size_mm": 120 }
```

Reglas:

1. La celda especial se representa como un cuadrado centrado dentro de la celda.
2. `size_mm` recomendado: `120`.
3. La marca especial no reemplaza la cinta de frontera.
4. Los mapas de `FIND_CELLS` normalmente deben tener tres celdas especiales si se quiere validar misión completa.
5. Evitar colocar marcas especiales ambiguas si el objetivo del test no es justamente validar ese borde.

## Nombres recomendados

Usar nombres descriptivos:

```text
normal_pass_open_3_specials.json
stress_pass_dead_ends_3_specials.json
stress_frontier_backtracking.json
stress_open_adjacent_specials.json
stress_center_pivot_cases_v2.json
stress_open_backtracking_front_tape.json
```

Prefijos sugeridos:

```text
normal_    caso representativo esperado
stress_    caso extremo o de regresión
shortcut_  casos de retorno/ruta alternativa
frontier_  casos de frontera de exploración
```

## Mapa de regresión para backtracking abierto

Cuando se quiera probar `CENTER_BY_FRONT_TAPE_FOR_PIVOT`, el mapa debe generar una situación donde:

1. el robot esté en una celda ya visitada;
2. no haya vecino no visitado inmediato útil;
3. la política `FIND_CELLS` necesite volver hacia atrás;
4. la celda actual no sea un dead-end físico;
5. exista salida frontal abierta para poder avanzar hasta la cinta frontal;
6. el frente no tenga pared conocida/presente;
7. el supervisor devuelva `BACKTRACK_REQUIRED`;
8. el supervisor entre en `APP_NAV_SUPERVISOR_RUN_CENTER_FRONT_TAPE_FOR_PIVOT`;
9. el bridge deje pasar PWM distinto de cero.

Telemetría esperada:

```text
supervisor: state=run_center_front_tape_for_pivot action=center_front_tape_for_pivot result=0
left_pwm != 0
right_pwm != 0
```

## Dead-end vs backtracking abierto

No confundir estos dos casos:

### Dead-end

```text
frente bloqueado
izquierda bloqueada
derecha bloqueada
solo queda volver por atrás
```

Preparación esperada:

```text
APPROACH_FRONT_WALL_FOR_PIVOT
PIVOT_180
```

### Backtracking abierto

```text
no hay vecino no visitado inmediato conveniente
existe una ruta a frontera que requiere volver hacia atrás
la celda no es un dead-end físico
el frente está abierto para buscar cinta frontal
```

Preparación esperada:

```text
CENTER_BY_FRONT_TAPE_FOR_PIVOT
PIVOT_180
```

## Checklist antes de agregar un mapa

Revisar:

- `width` y `height` correctos;
- `cell_size_mm = 200`, salvo test específico;
- pose inicial centrada en celda;
- yaw inicial cardinal;
- paredes internas dentro de rango;
- no duplicar innecesariamente paredes reflejadas;
- celdas especiales dentro de rango;
- nombre descriptivo;
- el caso físico reproduce una situación concreta;
- si es regresión, anotar qué telemetría se espera.

## Regla final

Un mapa válido debe probar al firmware portable a través del simulador, no a una lógica alternativa implementada en Qt.
