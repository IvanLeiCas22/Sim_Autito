# Reglas para generar mapas JSON

## Objetivo

Este documento define reglas para crear mapas JSON compatibles con el simulador Qt/C++ de micromouse/autito. Está pensado especialmente para que una IA pueda generar mapas nuevos sin romper el formato actual.

Separación recomendada:

- Mapas de test automático: deben ser estables y pasar el Mode 1 Test Runner.
- Mapas stress: pueden incluir casos extremos, ambiguos o difíciles.
- Mapas experimentales/debug: sirven para probar una función puntual y no necesariamente deben pasar.

## Carpetas recomendadas

- `data/test_maps/`: mapas que deben pasar el batch runner de modo 1. Resultado esperado: `PASS`.
- `data/stress_maps/` o `data/dev_maps/`: sugeridas para mapas extremos o experimentales. No tienen obligación de pasar.

Los mapas sueltos en `data/` pueden seguir usándose para pruebas manuales o compatibilidad.

## Formato JSON actual

El parser real está en `SimWorld::loadFromJsonFile(...)`. El formato usado actualmente es:

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

Campos actuales:

- `name`: opcional; si falta, se usa un nombre por defecto.
- `cells.width`: cantidad de columnas.
- `cells.height`: cantidad de filas.
- `cells.cell_size_mm`: tamaño de celda en milímetros.
- `start.x_mm`: posición inicial X física en milímetros.
- `start.y_mm`: posición inicial Y física en milímetros.
- `start.yaw_deg`: yaw inicial en grados.
- `walls`: lista de paredes internas o explícitas.
- `special_cells`: lista de celdas especiales.

Compatibilidad aceptada por el parser:

- También existen alias raíz `cols`, `rows` y `cell_size_mm`.
- En `walls` y `special_cells`, también se aceptan `col`/`row` como alias de `cell_x`/`cell_y`.
- Para mapas nuevos, preferir siempre `cells.width`, `cells.height`, `cells.cell_size_mm`, `cell_x` y `cell_y`.

Campos futuros deben marcarse como opcionales/futuros y no deben ser necesarios para cargar el mapa.

## Sistema de coordenadas

- `cell_x` es la columna.
- `cell_y` es la fila.
- El origen lógico es la esquina superior izquierda del mapa: `(0, 0)`.
- `x` crece hacia el Este.
- `y` crece hacia el Sur.
- La celda `(cell_x, cell_y)` ocupa:
  - `x_mm = cell_x * cell_size_mm ... (cell_x + 1) * cell_size_mm`
  - `y_mm = cell_y * cell_size_mm ... (cell_y + 1) * cell_size_mm`

Direcciones válidas para paredes:

- `N` o `NORTH`
- `E` o `EAST`
- `S` o `SOUTH`
- `W` o `WEST`

Para mapas nuevos, preferir las formas cortas `N`, `E`, `S`, `W`.

## Reglas geométricas obligatorias

- `width` y `height` deben ser positivos.
- `cell_size_mm` debe ser positivo.
- Todas las celdas referenciadas por paredes o especiales deben estar dentro del mapa.
- Las paredes deben pertenecer a una celda válida y a una dirección válida.
- No agregar paredes fuera de límites.
- Evitar duplicados conflictivos. Si se declara una pared compartida, no hace falta declarar también la opuesta en la celda vecina.
- `SimWorld::setWall(...)` propaga la pared a la celda vecina cuando corresponde.
- `SimWorld::addBoundaryWalls()` genera el borde exterior automáticamente. No es necesario declarar el perímetro exterior en JSON.

## Reglas de conectividad para `data/test_maps/`

Para mapas de test automático:

- Debe existir un camino desde `start` hasta todas las celdas especiales requeridas.
- Debe existir camino de regreso al inicio.
- El mapa debe tener al menos `required_special_count` celdas especiales.
- El default actual recomendado es al menos 3 especiales.
- Evitar regiones inalcanzables salvo que no afecten el `PASS` esperado.
- Si el objetivo del mapa es probar regiones inalcanzables, ubicarlo como stress/dev o documentarlo como `expected_fail`, no como test normal.

## Reglas para celdas especiales

- Cada celda especial debe estar dentro del mapa.
- No debe superponerse con una pared de forma que impida físicamente detectarla.
- El tamaño usado en mapas actuales suele ser `size_mm: 120`.
- Si falta `size_mm`, el simulador usa su tamaño especial por defecto.
- Para mapas `normal_pass`, evitar casos ambiguos:
  - especial muy pegado al inicio;
  - especiales contiguas;
  - start sobre especial;
  - marcas especiales en trayectorias físicamente raras.
- Esos casos son válidos para mapas stress si se documenta la intención.

## Reglas de start

- `start.x_mm` y `start.y_mm` deben caer dentro del mapa.
- La orientación inicial debe ser una orientación cardinal compatible con la navegación esperada. Recomendado: `0`, `90`, `180` o `270`.
- En mapas `normal_pass`, el start no debe quedar encerrado por paredes.
- Start sobre especial debe tratarse como caso stress, no como mapa normal de batch.

## Clasificación sugerida de mapas

- `normal_pass`: debería pasar siempre.
- `stress_pass`: desafiante, pero debería pasar.
- `expected_fail`: diseñado para fallar por falta de especiales, regiones inaccesibles u otra condición esperada.
- `debug`: prueba una función específica.

Solo `normal_pass` y `stress_pass` deberían entrar en `data/test_maps/` si el batch espera `PASS`.

## Checklist para IA generadora

Antes de entregar un mapa:

- Elegir `width` y `height`.
- Elegir `cell_size_mm`, normalmente `200`.
- Definir `start` dentro del mapa.
- Usar orientación inicial cardinal.
- Generar paredes interiores válidas.
- No declarar paredes fuera de límites.
- Recordar que el borde exterior se genera automáticamente.
- Garantizar conectividad desde start hacia las especiales.
- Colocar al menos 3 especiales alcanzables para el modo 1 default.
- Usar `size_mm: 120` salvo que se quiera probar otra cosa.
- Mantener JSON puro, sin comentarios.
- Usar nombres descriptivos.
- Guardar en `data/test_maps/` solo si se espera `PASS`.

## Plantilla de prompt para generar mapas

```text
Generá un mapa JSON para el simulador de micromouse usando el formato existente del proyecto.

Requisitos:
- JSON puro, sin comentarios.
- No inventes campos nuevos.
- Usá:
  - name
  - cells.width
  - cells.height
  - cells.cell_size_mm
  - start.x_mm
  - start.y_mm
  - start.yaw_deg
  - walls con cell_x, cell_y, dir
  - special_cells con cell_x, cell_y, size_mm
- Usá cell_size_mm = 200.
- Asegurá al menos 3 special_cells alcanzables desde start.
- Asegurá que existe camino de regreso al inicio.
- No declares paredes exteriores; el simulador las genera.
- Las direcciones de pared deben ser N, E, S o W.
- Todas las celdas referenciadas deben estar dentro del mapa.

Objetivo del mapa:
[describir si es normal_pass, stress_pass, expected_fail o debug]
```

## Checklist de validación manual

1. Cargar el mapa desde la UI.
2. Activar overlay lógico si ayuda (`Y`).
3. Ejecutar `Shift+R` para correr el Mode 1 Test Runner en el mapa actual.
4. Si pasa, puede entrar a `data/test_maps/`.
5. Ejecutar `Shift+B` cuando haya varios mapas.
6. Revisar `batch_runner_state`, `pass_count`, `fail_count`, `timeout_count` y `cancelled_count`.
7. Si falla, moverlo a stress/dev o corregir conectividad/especiales.
