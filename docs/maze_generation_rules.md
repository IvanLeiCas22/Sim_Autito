# Reglas para generar mapas JSON

Reglas para crear mapas compatibles con el simulador Qt/C++ de micromouse/autito.

## Estado actual del simulador

El simulador actual es un banco físico/sensorial con `FirmwareSimBridge` stub. No tiene navegación propia activa, no tiene batch runner y no ejecuta todavía el core real STM32.

Por lo tanto, estas reglas se enfocan en geometría física, sensores y representación visual. Las reglas de `PASS/FAIL`, `Shift+B`, flood, smart recognition y retorno inteligente pertenecen a la navegación legacy y no aplican al estado actual.

## Carpetas

- `data/test_maps/`: mapas útiles para pruebas manuales de geometría, sensores IR, sensores de piso, paredes y celdas especiales.
- `data/stress_maps/` o `data/dev_maps/`: sugeridas para mapas extremos, experimentales o casos conflictivos.
- Mapas sueltos en `data/`: pruebas manuales rápidas o compatibilidad.

`data/test_maps` debe contener mapas físicamente válidos y útiles para validar el simulador. No implica que exista un runner automático activo.

## Formato JSON

Parser real: `SimWorld::loadFromJsonFile(...)`.

Formato recomendado:

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

Campos:

- `name`: opcional.
- `cells.width`, `cells.height`, `cells.cell_size_mm`.
- `start.x_mm`, `start.y_mm`, `start.yaw_deg`.
- `walls`.
- `special_cells`.

Alias aceptados por compatibilidad:

- raíz `cols`, `rows`, `cell_size_mm`;
- `col`/`row` como alias de `cell_x`/`cell_y`.

Para mapas nuevos, preferir siempre el formato recomendado.

## Coordenadas

- `cell_x`: columna.
- `cell_y`: fila.
- origen lógico: esquina superior izquierda `(0, 0)`.
- X crece hacia Este.
- Y crece hacia Sur.
- `cell_size_mm` recomendado: `200`.

Direcciones válidas:

- `N` / `NORTH`;
- `E` / `EAST`;
- `S` / `SOUTH`;
- `W` / `WEST`.

Preferir `N`, `E`, `S`, `W`.

## Paredes

- Todas las paredes deben referenciar celdas dentro del mapa.
- No declarar paredes fuera de límites.
- Evitar duplicados conflictivos.
- Si se declara una pared compartida, no hace falta declarar la opuesta.
- `SimWorld::setWall(...)` propaga la pared al vecino cuando corresponde.
- `SimWorld::addBoundaryWalls()` genera el perímetro exterior automáticamente.

## Cintas y celdas especiales

`SimWorld` distingue internamente:

- cinta de frontera de celda: `boundary`;
- celda especial/target: `target`;
- superposición: `boundary+target`.

Visualmente:

- las cintas de frontera se dibujan en gris translúcido;
- las celdas especiales se dibujan como cuadrados gris oscuro;
- las paredes se dibujan como líneas negras gruesas por encima de las cintas.

## Celdas especiales

- Cada especial debe estar dentro del mapa.
- `size_mm` usado normalmente: `120`.
- Si falta `size_mm`, el simulador usa su default.
- Para pruebas normales, evitar ambigüedades innecesarias:
  - start sobre especial;
  - especiales contiguas si no se busca probar ese caso;
  - especiales pegadas a paredes que impidan pisarlas.

Casos ambiguos son válidos como stress si están documentados.

## Start

- `start.x_mm` y `start.y_mm` deben caer dentro del mapa.
- `yaw_deg` recomendado: `0`, `90`, `180` o `270`.
- Start no debe quedar encerrado.
- Start debe permitir verificar sensores sin colisión visual inmediata.
- Start sobre especial debe tratarse como stress, salvo que se busque probar explícitamente la detección inicial de especial.

## Clasificación sugerida

- `normal_sensor`: mapa simple para validar sensores y representación.
- `stress_sensor`: mapa difícil o ambiguo para sensores/geometría.
- `debug`: prueba una función puntual.
- `legacy_nav`: mapa creado para navegación antigua; conservar solo si todavía sirve como geometría física.

## Checklist para generar un mapa

- Elegir `width`, `height` y `cell_size_mm`.
- Definir start dentro del mapa.
- Usar yaw cardinal.
- Generar paredes interiores válidas.
- No declarar paredes exteriores.
- Colocar celdas especiales solo si son necesarias para la prueba.
- Usar JSON puro, sin comentarios.
- Usar nombre descriptivo.
- Cargar el mapa y verificar visualmente paredes/cintas/especiales.
- Usar movimiento manual para verificar IR y sensores de piso.

## Validación manual actual

1. Cargar el mapa.
2. Usar `Fit map` si hace falta.
3. Mover el robot con `W/S/A/D` o flechas.
4. Verificar telemetría de sensores IR.
5. Verificar sensores de piso:
   - `kind=none` fuera de cinta;
   - `kind=boundary` sobre cinta de frontera;
   - `kind=target` sobre celda especial;
   - `kind=boundary+target` si hay superposición.
6. Verificar que las paredes negras coincidan con el JSON.
7. Verificar que el raycast IR corte contra paredes físicas, no contra cintas.
