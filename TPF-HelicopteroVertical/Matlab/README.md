# TPF — Scripts MATLAB

Scripts y datos MATLAB del Trabajo Práctico Final (Helicóptero Vertical).

## Estructura

```
Matlab/
├── Compensadores/          # Datos experimentales y scripts de graficado por controlador
├── PruebasControl/         # Scripts de validación y simulación de todos los controladores
├── PruebasEmpiricas/       # Datos crudos de experimentos en planta real
├── SensorCalibracion/      # Calibración del sensor TFmini Plus
├── ProtocoloDeComunicacion/ # API MATLAB para comunicación UART con PSoC (ver README propio)
├── FuncionesTania/         # Funciones auxiliares
├── PruebasV1/              # Pruebas tempranas de comunicación con PSoC
├── FaltaOrdenar/           # Scripts de diseño en desarrollo / sin clasificar
├── Recuperado.backup/      # Backup de scripts recuperados
├── planta.mat              # Modelo identificado de la planta (último válido)
├── plantaElias.mat         # Variante del modelo identificado
├── planta (1).mat
├── pruebaSS.mat
└── *.mat                   # Ganancias y modelos guardados (Kalman, LQR, etc.)
```

---

## Carpetas principales

### `Compensadores/`
Datos `.mat` de respuestas experimentales y teóricas de cada estrategia de control:
- `PID_practico.mat` / `PID_teorico.mat`
- `LQR_act_practico.mat`, `LQR_pred_practico.mat`, `LQR_teorico.mat`
- `LQGi_practico.mat`, `LQGi_teorico.mat`
- `SS_actual_practico.mat`, `SS_predictor_practico.mat`, `SS_teorico.mat`
- `SSi_act_practico.mat`, `SSi_pred_practico.mat`, `SSi_teorico.mat`
- `BODE_practico.mat`, `rlocus_practico_*.mat`

Usar `graficarPracticos.m` para visualizar comparaciones.

### `PruebasControl/`
- `simulacion_todos_controladores.m` — simulación comparativa de todos los métodos
- `calculo_coeficientes_todo.m` — cálculo de ganancias (K, L, Ki, Kalman)
- `pid_digital_from_mat.m` — PID digital desde modelo `.mat`
- `simulate_control_app.m` — app de simulación interactiva

### `PruebasEmpiricas/`
Datos de experimentos en lazo abierto realizados el 08/02/2026:
- `StepLazoAbierto08-02-26_*.mat` — respuestas al escalón en lazo abierto

### `SensorCalibracion/`
Scripts para calibrar el sensor TFmini Plus:
- `calib_train_from_excel.m` / `calib_train_simple.m` — entrenamiento de la curva de calibración
- `calib_poly3_plot.m`, `calib_poly4_plot.m` — ajuste polinomial grado 3/4
- `calib_quad_lnq_plot_byD.m` — ajuste cuadrático-logarítmico por distancia
- `calib_make_ln1p_lut.m` — generación de LUT para firmware
- `calib_export_header.m` / `calib_export_simple_h.m` — exportar calibración a `.h` para PSoC

### `ProtocoloDeComunicacion/`
API MATLAB completa para el protocolo UART con el PSoC. Ver [`README.md`](ProtocoloDeComunicacion/README.md) propio con documentación detallada del protocolo.

### `FaltaOrdenar/`
Scripts de diseño en progreso:
- `disenar_control.m` — script principal de diseño de controladores
- `pruebas_system_identification.m` — identificación del sistema
- `pruebaModelo.m` — validación del modelo
