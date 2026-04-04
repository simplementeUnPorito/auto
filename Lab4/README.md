# Lab 4 — Método de Truxal–Ragazzini / Dead-beat

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Diseñar un controlador digital mediante el **método analítico de Truxal–Ragazzini**, que especifica directamente la función de transferencia de lazo cerrado deseada. Se comparan dos variantes:

- **Controlador con oscilaciones entre muestras** — tiempo de establecimiento mínimo sin restricciones
- **Controlador Dead-beat** — sin oscilaciones entre muestras (intersample behavior)

---

## Contenido

```
Lab4/
├── Lab4Auto_EAyTR.tex     # Documento principal LaTeX
├── Lab4Auto_EAyTR.pdf     # Informe compilado
├── intro.tex
├── funtrans.tex           # Funciones de transferencia
├── discplant.tex          # Discretización de la planta
├── tiempodemuestreo.tex   # Selección del tiempo de muestreo
├── disecontr.tex          # Diseño del controlador
├── impPrac.tex            # Implementación práctica
├── resultados.tex / desarrollo.tex
├── conclusiones.tex
├── anexo.tex
├── referencias.bib
├── Codigo/                # Código C para PSoC
└── Img/                   # Figuras del informe
```

---

## Metodología

1. **Selección del tiempo de muestreo** — criterios de ingeniería (dinámica del sistema)
2. **Discretización por ZOH** para distintos `Ts`
3. **Diseño Truxal–Ragazzini** — se especifica `T(z)` deseada, se despeja `C(z)` del lazo
4. **Variante Dead-beat** — restricciones adicionales para eliminar oscilaciones entre muestras
5. **Simulación en MATLAB** — comparación de ambas variantes
6. **Implementación en PSoC** — ecuación en diferencias
7. **Comparación** simulación vs. experimento

> **Referencia teórica:** M. Fadali, A. Visioli, *Digital Control Engineering*, Sección 6.6.

---

## Resultados clave

- Comparación del controlador estándar vs. Dead-beat: sobreimpulso, Ts, esfuerzo de control
- Comportamiento entre muestras (intersample ripple) en ambas variantes
- Discrepancias simulación / experimento debidas a no idealidades de la planta
