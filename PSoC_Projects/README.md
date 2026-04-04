# PSoC Projects

Proyectos **PSoC Creator** con el firmware de implementación de cada controlador sobre el microcontrolador PSoC 5LP.

## Estructura

```
PSoC_Projects/
├── lab2.cydsn              # Lab 2 — Lugar de raíces
├── Lab3.cydsn              # Lab 3 — Bode
├── Lab4.cydsn              # Lab 4 — Truxal–Ragazzini
├── Lab5.cydsn              # Lab 5 — Ubicación de polos
├── Lab5_editado.cydsn      # Lab 5 — versión editada/corregida
├── Lab6.current.cydsn      # Lab 6 — Estimador de actualización
├── Lab6.predict.cydsn      # Lab 6 — Estimador de predicción
├── Lab6.predict_Simple.cydsn
├── Lab7.current.cydsn      # Lab 7 — Con integrador (actualización)
├── Lab7.current_Copy_01.cydsn
├── Lab7.predict.cydsn      # Lab 7 — Con integrador (predicción)
├── Lab8.cydsn              # Lab 8 — LQI
├── Lab9.cydsn              # Lab 9 — LQG / Kalman
├── Labo9.cydsn             # Lab 9 — versión alternativa
├── template.cydsn          # Plantilla base para nuevos proyectos
└── template_Copy_01.cydsn
```

## Notas

- Todos los proyectos implementan la **ecuación en diferencias** del controlador a `Ts = 1 ms`
- La comunicación con MATLAB se realiza por **UART** para envío de datos y recepción de referencia
- Los ADC/DAC del PSoC se usan para interfazar con la planta analógica
- Los archivos `.cydsn` son carpetas de proyecto de PSoC Creator — abrir con PSoC Creator 3.x o superior
