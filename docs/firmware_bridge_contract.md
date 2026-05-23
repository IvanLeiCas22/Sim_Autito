# Contrato del Firmware Bridge

Este documento define la intención de `FirmwareSimBridge`: conectar el mundo simulado con un núcleo portable del firmware STM32 sin que el simulador implemente navegación propia.

## Rol del bridge

`FirmwareSimBridge` debe convertir:

```text
sensores simulados + tiempo + pose/yaw
        ↓
entrada portable del firmware
        ↓
núcleo de navegación portable
        ↓
salida de motores/debug
        ↓
SimRobot + telemetría Qt
```

En la etapa actual el bridge es un stub. Su salida debe permanecer en PWM cero hasta conectar `firmware_core`.

## Responsabilidades permitidas

El bridge puede:

- convertir unidades del simulador a unidades del firmware;
- convertir sensores IR simulados a distancias en mm;
- convertir sensores de piso a lecturas equivalentes o booleanos según la API portable final;
- entregar `dt_ms`;
- pasar yaw/yaw-rate si el núcleo los requiere;
- llamar funciones del núcleo portable;
- convertir PWM de salida al formato usado por `SimRobot`;
- exponer debug para telemetría.

## Responsabilidades prohibidas

El bridge no debe:

- decidir si avanzar, girar o frenar;
- implementar reglas de mano derecha;
- implementar flood fill propio;
- modificar el mapa lógico por su cuenta;
- esconder clamps/configuraciones que el firmware debería controlar;
- duplicar la máquina de estados del STM32.

## API futura esperada

La API final puede variar, pero debe tender a una forma similar a esta:

```c
typedef struct
{
    uint32_t dt_ms;

    uint16_t dist_front_left_mm;
    uint16_t dist_front_right_mm;
    uint16_t dist_left_mm;
    uint16_t dist_right_mm;
    uint16_t dist_diag_left_mm;
    uint16_t dist_diag_right_mm;

    uint16_t adc_floor_front;
    uint16_t adc_floor_rear;

    int16_t yaw_deg;
    int16_t yaw_rate_dps;
} AppNavInput;

typedef struct
{
    int16_t left_motor_pwm;
    int16_t right_motor_pwm;
} AppNavOutput;

void App_Nav_Init(const AppNavConfig *config);
void App_Nav_Reset(void);
void App_Nav_Start(void);
void App_Nav_Stop(void);
void App_Nav_Tick(const AppNavInput *input, AppNavOutput *output);
void App_Nav_GetDebug(AppNavDebug *debug);
```

## Convención de motores

El firmware real suele trabajar con comandos de motor izquierdo/derecho o derecho/izquierdo según la capa. Para evitar ambigüedad, el bridge debe documentar explícitamente la conversión final.

En el lado del simulador, la salida visible debe quedar como:

```text
left_pwm
right_pwm
```

Si el firmware entrega otro orden, la conversión debe hacerse en `FirmwareSimBridge` y no dispersarse por la UI.

## Criterio de aceptación al conectar firmware_core

Una conexión inicial se considera válida cuando:

1. el simulador compila sin HAL STM32;
2. `FirmwareSimBridge` llama al núcleo portable real;
3. Start/Stop/Reset controlan el núcleo portable;
4. la telemetría muestra estado/debug del firmware;
5. el robot simulado se mueve solo por comandos del firmware;
6. los controles manuales siguen existiendo solo como modo de prueba sensorial.
