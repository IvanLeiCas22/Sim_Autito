# firmware_core

Carpeta reservada para el núcleo portable copiado desde el proyecto STM32 real.

## Regla principal

Los archivos de esta carpeta no deben convertirse en una navegación alternativa del simulador. Deben ser copia directa o casi directa del núcleo portable del firmware real.

## Flujo previsto

1. Extraer en el repo STM32 un núcleo C portable sin HAL.
2. Copiar esos archivos a esta carpeta mediante `tools/sync_firmware_core_from_stm32.cmd`.
3. Compilar esos archivos dentro del simulador.
4. Hacer que `FirmwareSimBridge` sea el único adaptador entre Qt/simulación y el núcleo portable.

## Archivos esperados a futuro

La lista exacta todavía puede cambiar, pero el objetivo es algo de este estilo:

```text
app_nav.c
app_nav.h
app_nav_types.h
app_nav_config.h
app_nav_debug.h
app_maze.c
app_maze.h
pid_controller.c
pid_controller.h
```

## Restricciones

El código dentro de esta carpeta no debería depender de:

- Qt;
- HAL STM32;
- UART/USB concretos;
- SSD1306/OLED;
- interrupciones específicas;
- rutas absolutas del proyecto STM32;
- archivos del simulador.

Si un archivo necesita HAL, todavía no está suficientemente extraído para vivir acá.
