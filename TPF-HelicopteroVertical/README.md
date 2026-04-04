# TPF — Helicóptero Vertical de un Solo Eje

**Trabajo Práctico Final — Automatización / Control Digital**  
**Autores:** Elías Álvarez · Tania Romero  
**Institución:** Universidad Católica Nuestra Señora de la Asunción — Ing. Electrónica

---

## Descripción del sistema

Planta experimental de **helicóptero vertical de un solo eje**: un motor brushless montado sobre un riel vertical mueve un vehículo cuya altura se controla. El sistema presenta:

- **Orden:** 3 (tercer orden)
- **Comportamiento en lazo abierto:** inestable
- **Polos dominantes:** complejos con parte real positiva
- **Actuador:** motor brushless + ESC (saturación estricta)
- **Sensor:** LiDAR ToF TFmini Plus (ruido de medición significativo)
- **No idealidades:** fricción variable en riel, descarga de batería, dinámica del ESC

---

## Hardware

| Componente | Descripción |
|------------|-------------|
| Motor + ESC | Brushless, controlado por señal PWM |
| Sensor | TFmini Plus (LiDAR ToF, comunicación UART) |
| Controlador embebido | PSoC 5LP |
| GUI de monitoreo | Arduino (visualización y envío de referencia) |
| Alimentación | Batería LiPo |

---

## Estructura del proyecto

```
TPF-HelicopteroVertical/
├── documentacion/         # Informe técnico completo en LaTeX
│   ├── document.tex       # Documento principal (formato IEEEtran)
│   ├── document.pdf       # Informe compilado
│   ├── intro.tex
│   ├── modeladoPlanta.tex
│   ├── caracPlanta.tex
│   ├── PID.tex
│   ├── ubiArbPolos.tex    # Ubicación arbitraria de polos
│   ├── LQR.tex
│   ├── ControlOptimo.tex
│   ├── ControlOptimoIntegrador.tex
│   ├── Kalman.tex
│   ├── Estimadores.tex
│   ├── LQGi.tex           # LQG con integrador
│   ├── SistSeguimiento.tex
│   ├── conclusiones.tex
│   ├── refs.bib
│   └── img/               # Figuras del informe
│
├── Matlab/                # Scripts y funciones de MATLAB
│   ├── Compensadores/     # Diseño de compensadores clásicos
│   ├── PruebasControl/    # Scripts de validación de controladores
│   ├── PruebasEmpiricas/  # Análisis de datos experimentales
│   ├── SensorCalibracion/ # Calibración del sensor TFmini
│   ├── FuncionesTania/    # Funciones auxiliares
│   ├── ProtocoloDeComunicacion/ # Comunicación PSoC–PC
│   ├── FaltaOrdenar/      # Scripts en revisión
│   ├── planta.mat         # Modelo identificado de la planta
│   └── *.mat              # Modelos y ganancias guardados
│
├── PSoC Creator/          # Proyecto PSoC Creator (firmware)
│   └── [workspace]        # Implementación del controlador embebido
│
├── Arduino/               # Código Arduino (GUI de monitoreo)
│   └── PruebasSensoresAuto/
│
├── Planta/                # Documentación física de la planta
│   ├── imagenes/          # Fotos del sistema físico
│   └── README.md
│
├── DatosMotor.xlsx        # Caracterización motor/ESC (PWM vs. empuje)
├── DatosLeidos.xlsx       # Datos de experimentos
├── pruebasPeso-Motor.xlsx # Ensayos de peso vs. esfuerzo de control
└── modeloSystemIdentification.mat  # Modelo por System Identification
```

---

## Estrategias de control implementadas

### 1. PID Digital
Control clásico como línea de base. Sintonía manual sobre la planta real.

### 2. Ubicación Arbitraria de Polos (SS)
Realimentación de estados con estimador. Polos de lazo cerrado elegidos para cumplir especificaciones de tiempo de establecimiento y amortiguamiento.

### 3. LQR — Regulador Cuadrático Lineal
Minimización del criterio `J = Σ(xᵀQx + uᵀRu)`. Diseñado con `dlqr()` en MATLAB.

### 4. LQR + Integrador (LQI)
Estado aumentado con integrador del error para garantizar seguimiento con `ESS = 0`.

### 5. LQG — Linear Quadratic Gaussian
Combinación de LQR + filtro de Kalman. Diseño óptimo bajo la hipótesis de ruido gaussiano en proceso y medición.

---

## Flujo de trabajo

```
Modelado experimental
        ↓
Identificación (System Identification Toolbox)
        ↓
Diseño del controlador en MATLAB (simulación)
        ↓
Implementación en PSoC 5LP (ecuación en diferencias)
        ↓
Validación experimental en planta real
        ↓
Comparación y análisis de resultados
```

---

## Cómo compilar el informe

```bash
cd documentacion/
pdflatex document.tex
bibtex document
pdflatex document.tex
pdflatex document.tex
```

Requiere: `IEEEtran`, `amsmath`, `graphicx`, `siunitx`, `listings`, `subfig`, `natbib`, `babel` (español).

---

## Observaciones del sistema real

- El ESC requiere calibración previa (rango PWM mínimo/máximo)
- La batería descargada desplaza el punto de operación — los modelos son válidos en un rango limitado
- El sensor TFmini introduce un retardo de ~10 ms adicional al lazo de control
- La fricción del riel es altamente no lineal y dificulta el modelado exacto
