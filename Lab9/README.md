# Lab 9 — Filtro de Kalman y Regulador LQG

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar un **regulador LQG** completo (Linear Quadratic Gaussian), que combina el regulador óptimo LQR (Lab 8) con un **filtro de Kalman** como estimador óptimo en presencia de ruido de proceso y de medición.

Se comparan dos estimadores:
- **Filtro de Kalman** — ganancia óptima `L` calculada a partir de las covarianzas reales `Q_ruido / R_ruido` (`dlqe`)
- **Observador de Luenberger rápido** — polos fijados 5 veces más rápido que los del lazo cerrado

---

## Contenido

```
Lab9/
├── EA_TR_lab9.tex         # Documento principal LaTeX
├── EA_TR_lab9.pdf         # Informe compilado
├── estimador.tex          # Diseño del filtro de Kalman
├── rql.tex                # Diseño del regulador LQR
├── resultados.tex
├── anexo.tex
├── Exp/                   # Datos experimentales
├── Sim/                   # Resultados de simulación
└── Otros/                 # Material adicional
```

---

## Metodología

1. **Estimación de covarianzas** — caracterización del ruido de proceso `Q` y medición `R` desde datos experimentales
2. **Diseño del filtro de Kalman** — `L = dlqe(A, G, C, Q, R)` → ganancia óptima de Kalman
3. **Observador de Luenberger rápido** — polos `p_obs = p_cl^5`
4. **Regulador LQR** — `K = dlqr(A, B, Q1, Q2)` (del Lab 8)
5. **Regulador LQG** — combinación LQR + Kalman → controlador por principio de separación
6. **Simulación en MATLAB** — comparación con/sin ruido, Kalman vs. Luenberger
7. **Implementación en PSoC** — bucle de control LQG completo

---

## Resultados clave

- Comparación Kalman vs. Luenberger rápido: desempeño con y sin ruido
- Validación del principio de separación en el diseño LQG
- Comportamiento experimental con ruido real de sensor
- Trade-off entre velocidad del estimador y amplificación de ruido
