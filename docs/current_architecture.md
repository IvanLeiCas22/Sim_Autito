# Arquitectura actual del simulador

Estado: simulador Qt/C++ convertido en banco de pruebas para un futuro núcleo portable del firmware STM32.

## Objetivo

El simulador ya no debe contener una navegación propia independiente. Su función es modelar la planta física:

- mundo/laberinto cargado desde JSON;
- paredes físicas;
- cintas de frontera de celda;
- marcas especiales/target;
- robot diferencial simulado;
- sensores IR simulados por raycast;
- sensores de piso simulados;
- telemetría visual.

La navegación debe venir de un núcleo portable compartible con el proyecto STM32 real. En este momento ese núcleo todavía no está conectado; `FirmwareSimBridge` es un stub.

## Estructura relevante

```text
app/
  main.cpp
  sim_mainwindow.cpp/.h          UI mínima del simulador actual
  firmware_sim_bridge.cpp/.h     Stub e interfaz futura hacia firmware_core

sim/
  sim_world.cpp/.h               Geometría del mapa, paredes, cintas, targets, raycast
  sim_robot.cpp/.h               Cinemática diferencial y pose del robot

firmware_core/
  README.md                      Carpeta reservada para el núcleo portable copiado desde STM32

tools/
  sync_firmware_core_from_stm32.cmd
```

## Flujo actual de simulación

```text
SimWorld + SimRobot
        ↓
actualización de sensores simulados
        ↓
FirmwareSimBridge::tick(...)
        ↓
PWM izquierdo/derecho
        ↓
SimRobot::stepDifferential(...)
        ↓
render + telemetría
```

Actualmente `FirmwareSimBridge` devuelve `left_pwm = 0` y `right_pwm = 0`, por lo que el robot no se mueve por navegación automática. El movimiento manual existe solo para validar sensores y geometría.

## Reglas de arquitectura

1. El simulador no debe volver a implementar políticas de navegación propias.
2. El simulador no debe decidir avanzar/girar/frenar en modo automático.
3. El simulador puede tener controles manuales de debug, pero no deben confundirse con navegación.
4. La lógica de navegación real debe vivir en `firmware_core/` cuando se extraiga desde el proyecto STM32.
5. `app/` debe actuar como adaptador Qt/UI.
6. `sim/` debe mantenerse como modelo físico/sensorial, sin dependencia del firmware.
7. El bridge debe ser el único punto de contacto entre la simulación y el núcleo portable.

## Qué quedó fuera a propósito

La navegación legacy del simulador fue eliminada del build y luego removida del repo. La documentación antigua se conserva en `docs/legacy/` solo como referencia histórica y está excluida de Repomix.

No se deben reintroducir directamente conceptos legacy como:

- `nav_core` del simulador;
- `nav_supervisor`;
- `SMART_RECOGNITION` del simulador;
- `MODE1_FLOOD_SAFE` del simulador;
- runners/autochecks dependientes de esa navegación;
- overlays de mapa lógico basados en la navegación vieja.

Si más adelante se necesitan runners, deben validar el comportamiento del firmware portable conectado por `FirmwareSimBridge`, no una navegación paralela del simulador.
