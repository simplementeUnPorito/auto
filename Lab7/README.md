# Lab 7 — Realimentación de Estados con Acción Integral

**Materia:** Automatización / Control Digital  
**Autores:** Elías Álvarez · Tania Romero

---

## Objetivo

Extender el controlador por realimentación de estados del Lab 6 incorporando una **acción integral** sobre el error de seguimiento. Esto garantiza **error en estado estacionario nulo** ante referencias escalón y mejora la robustez frente a perturbaciones constantes, a diferencia del sistema en lazo abierto del laboratorio anterior.

---

## Contenido

```
Lab7/
├── lab7_Tania_Elias.tex   # Documento principal LaTeX
├── lab7_Tania_Elias.pdf   # Informe compilado
├── intro.tex
├── calculos.tex           # Diseño del controlador aumentado
├── implementacion.tex
├── resultados.tex
├── anexo.tex
├── matlab/                # Scripts MATLAB de diseño
├── c/                     # Código C para PSoC
├── exp/                   # Datos experimentales
└── sim/                   # Resultados de simulación
```

---

## Metodología

1. **Modelo aumentado** — se agrega estado integrador `ξ(k+1) = ξ(k) + e(k)` al sistema original
2. **Diseño del controlador aumentado** — ganancia `[K | Ki]` por ubicación de polos sobre el sistema aumentado
3. **Ley de control:** `u(k) = -K·x̂(k) - Ki·ξ(k)` (sin pre-filtro)
4. **Estimador** — se mantiene el observador del Lab 6
5. **Simulación en MATLAB** — respuesta ante escalón y perturbaciones
6. **Implementación en PSoC** — integración del estado `ξ` en el bucle de control
7. **Comparación** con el controlador sin integrador (Lab 6)

---

## Resultados clave

- Error estacionario nulo frente a referencia escalón (verificado en experimento)
- Comparación con Lab 6 (sin integrador): mejora de seguimiento y robustez
- Respuesta ante perturbaciones constantes
- Sensibilidad del diseño a la ubicación de los polos del integrador
