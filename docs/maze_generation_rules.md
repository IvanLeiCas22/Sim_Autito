# Reglas para generar mapas JSON

Reglas para crear mapas compatibles con el simulador Qt/C++ de micromouse/autito.

## Carpetas

- `data/test_maps/`: mapas esperados a `PASS` con `Shift+B`.
- `data/stress_maps/` o `data/dev_maps/`: sugeridas para mapas extremos,
  experimentales o `expected_fail`.
- Mapas sueltos en `data/`: pruebas manuales o compatibilidad.

`data/test_maps` debe contener solo mapas que el batch debe pasar. Si un mapa esta
disenado para fallar, usar otra carpeta o documentarlo explicitamente.

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

- raiz `cols`, `rows`, `cell_size_mm`;
- `col`/`row` como alias de `cell_x`/`cell_y`.

Para mapas nuevos, preferir siempre el formato recomendado.

## Coordenadas

- `cell_x`: columna.
- `cell_y`: fila.
- origen logico: esquina superior izquierda `(0, 0)`.
- X crece hacia Este.
- Y crece hacia Sur.
- `cell_size_mm` recomendado: `200`.

Direcciones validas:

- `N` / `NORTH`;
- `E` / `EAST`;
- `S` / `SOUTH`;
- `W` / `WEST`.

Preferir `N`, `E`, `S`, `W`.

## Paredes

- Todas las paredes deben referenciar celdas dentro del mapa.
- No declarar paredes fuera de limites.
- Evitar duplicados conflictivos.
- Si se declara una pared compartida, no hace falta declarar la opuesta.
- `SimWorld::setWall(...)` propaga la pared al vecino cuando corresponde.
- `SimWorld::addBoundaryWalls()` genera el perimetro exterior automaticamente.

## Reglas para `data/test_maps`

Con defaults actuales:

- `return_strategy = GOAL_DIRECTED_RETURN_LIMITED_EXECUTION`;
- `goal_max_shortcut_attempts = 32`;
- `batch_fast_mode_enabled = true`;
- `required_special_count` recomendado: `3`.

Para que un mapa entre en `data/test_maps`:

- debe haber al menos `required_special_count` especiales alcanzables;
- debe existir camino fisico desde start hacia esas especiales;
- debe existir alguna forma valida de volver al inicio;
- no debe depender de comportamiento indefinido;
- debe terminar `PASS` en `Shift+R` y `Shift+B`.

El retorno inteligente puede explorar atajos desconocidos, pero el mapa no debe exigir
atravesar paredes conocidas presentes ni inconsistencias geometricas.

## Celdas especiales

- Cada especial debe estar dentro del mapa.
- `size_mm` usado normalmente: `120`.
- Si falta `size_mm`, el simulador usa su default.
- Para mapas `normal_pass`, evitar ambiguedades innecesarias:
  - start sobre especial;
  - especiales contiguas si no se busca probar ese caso;
  - especiales pegadas a paredes que impidan pisarlas.

Casos ambiguos son validos como stress si estan documentados.

## Start

- `start.x_mm` y `start.y_mm` deben caer dentro del mapa.
- `yaw_deg` recomendado: `0`, `90`, `180` o `270`.
- Start no debe quedar encerrado.
- Start sobre especial debe tratarse como stress, salvo que el caso ya este validado.

## Clasificacion sugerida

- `normal_pass`: debe pasar siempre.
- `stress_pass`: dificil, pero debe pasar.
- `expected_fail`: disenado para fallar por una razon esperada.
- `debug`: prueba una funcion puntual.

Solo `normal_pass` y `stress_pass` deberian entrar en `data/test_maps`.

## Checklist para generar un mapa

- Elegir `width`, `height` y `cell_size_mm`.
- Definir start dentro del mapa.
- Usar yaw cardinal.
- Generar paredes interiores validas.
- No declarar paredes exteriores.
- Colocar al menos 3 especiales alcanzables.
- Garantizar regreso al inicio.
- Usar JSON puro, sin comentarios.
- Usar nombre descriptivo.
- Guardar en `data/test_maps` solo si se espera `PASS`.

## Validacion

1. Cargar el mapa.
2. Activar overlay con `Y` si ayuda.
3. Ejecutar `Shift+R`.
4. Si pasa, ejecutar `Shift+B`.
5. Revisar:
   - `batch_runner_state = BATCH_DONE`;
   - `batch_runner_fail_count = 0`;
   - `batch_runner_timeout_count = 0`;
   - `autocheck_failure_count = 0`.
6. Revisar CSV/JSON exportado en `data/test_results`.
