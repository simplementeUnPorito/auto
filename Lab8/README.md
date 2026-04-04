# Lab 8 — Control Óptimo con Integrador (LQI)

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar un **regulador cuadrático lineal con integrador (LQI)** para la planta analógica. En lugar de ubicar los polos manualmente, se minimiza un criterio de costo cuadrático que pondera el error de estado y el esfuerzo de control, obteniendo un controlador óptimo en el sentido cuadrático.

---

## Contenido

```
Lab8/
├── Auto_Lab8_EA y TR.tex  # Documento principal LaTeX
├── Auto_Lab8_EA y TR.pdf  # Informe compilado
├── desarrollo.tex
├── dinamica.tex           # Modificación de la dinámica con LQI
├── Exp/                   # Datos experimentales
├── Sim/                   # Resultados de simulación
└── Otros/                 # Material adicional
```

---

## Metodología

1. **Modelo aumentado** — estado integrador `ξ` agregado al modelo discreto
   ```
   A_ad = [A_d,  0 ]    B_ad = [B_d]
          [-Ts·C, I]            [ 0 ]
   ```
2. **Cálculo LQI** — `K_a = dlqr(A_ad, B_ad, Q1, Q2)`, con `K_a = [K | Ki]`
3. **Variación de matrices de peso** — se diseñan 3 controladores con distintas relaciones `Q1/Q2`:
   - Mayor peso en estado → respuesta más rápida, mayor esfuerzo
   - Mayor peso en control → respuesta lenta, esfuerzo reducido
4. **Ley de control:** `u(k) = -K·x̂(k) - Ki·ξ(k)`
5. **Simulación en MATLAB** — comparación de los 3 diseños
6. **Implementación en PSoC** — bucle de control con `dlqr`
7. **Comparación** simulación vs. experimento

---

## Resultados clave

- Tres reguladores LQI con distintas matrices de peso `Q1` comparados
- Trade-off entre velocidad de respuesta y esfuerzo de control
- Validación experimental: comportamiento en planta real vs. simulación
