# TPF — Firmware PSoC Creator

Proyectos PSoC Creator para el firmware embebido del helicóptero vertical. El firmware implementa el bucle de control en tiempo real sobre el PSoC 5LP a `Ts = 1 ms`.

## Versión activa recomendada

> **`FinalV2.2.rebuild.cydsn`** — última versión estable y reconstruida.  
> En caso de duda, usar esta o `FinalV2.0.cydsn`.

---

## Historial de versiones `FinalVx.x`

| Proyecto | Estado / Notas |
|----------|----------------|
| `FinalV1.1` – `FinalV1.4` | Versiones iniciales, iteraciones de desarrollo |
| `FinalV1.5` | Versión de desarrollo |
| `FinalV1.5SiFunciona` | Primera versión verificada funcional |
| `FinalV1.6` | Mejoras sobre V1.5 |
| `FinalV1.6b_solo_sensor` | Solo sensor activo, sin controlador (debug de sensor) |
| `FinalV1.7.1` | Iteración intermedia |
| `FinalV1.8` – `FinalV1.9` | Refinamientos |
| `FinalV1NoFunciona` | Versión defectuosa — no usar |
| `FinalV2.0` | Salto mayor: protocolo UART robusto implementado |
| `FinalV2.1XD` | Pruebas experimentales |
| `FinalV2.2.rebuild` | Reconstrucción limpia — versión final entregada |

---

## Otros proyectos

| Proyecto | Descripción |
|----------|-------------|
| `LazoAbierto.cydsn` | Control en lazo abierto (caracterización del sistema) |
| `PruebasComunicacionMatlab.cydsn` | Pruebas del protocolo UART con MATLAB |
| `PruebasControlLed.cydsn` | Pruebas básicas de I/O con LED |
| `PruebasControlTF.cydsn` | Pruebas de controlador por función de transferencia |
| `PruebasControlTodo.cydsn` | Banco de pruebas integrado |
| `TFminiPlusLib.cydsn` | Biblioteca del sensor TFmini Plus |
| `TFminiPlusLib_elias_pruebas.cydsn` | Fork de pruebas del sensor |
| `TFminiPlusLibPro.cydsn` | Versión mejorada de la biblioteca del sensor |
| `Basura.cydsn` | Proyecto descartado — no usar |
| `ControladoresMotorHechosPorTania/` | Controladores de motor desarrollados por Tania |
| `Workspace03/` | Workspace de pruebas tempranas |

---

## Modos de control del firmware (FinalV2.x)

El firmware soporta 5 modos configurables via protocolo UART desde MATLAB:

| Modo | Descripción |
|------|-------------|
| `0` | TF — compensador en función de transferencia |
| `1` | SS + observador predictor, sin integrador |
| `2` | SS + observador actual, sin integrador |
| `3` | SS + observador predictor, con integrador |
| `4` | SS + observador actual, con integrador |

Los coeficientes se cargan en tiempo real desde MATLAB usando la API en [`../Matlab/ProtocoloDeComunicacion/`](../Matlab/ProtocoloDeComunicacion/).

---

## Notas de hardware

- **Microcontrolador:** PSoC 5LP (CY8C5888LTI-LP097)
- **Periodo de muestreo:** `Ts = 1 ms` (ISR del timer)
- **Comunicación:** UART 115200 bps con MATLAB/PC
- **Actuador:** señal PWM al ESC del motor brushless
- **Sensor:** TFmini Plus por UART dedicado (DMA o polling)
- **Entorno de desarrollo:** PSoC Creator 4.x

## Cómo abrir

Abrir `Workspace03` o el `.cydsn` deseado directamente con **PSoC Creator**. Programar con **MiniProg3** o **KitProg**.
