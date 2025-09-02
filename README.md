# Proyecto RCAM en Simulink

Este proyecto implementa el *Research Civil Aircraft Model (RCAM)* en un entorno *MATLAB/Simulink*, con el objetivo de disponer de una herramienta flexible para:

- Obtener *puntos de trimado* de la aeronave en condiciones de vuelo recto y nivelado u otras configuraciones de interés.  
- *Linealizar* el modelo en torno al trimado para analizar la *estabilidad dinámica* mediante autovalores, modos propios y funciones de transferencia.  
- Simular la *respuesta a maniobras* (dobletes, impulsos, OEI, etc.) tanto en modelos no lineales como lineales.  
- Integrar control *manual mediante joystick*.  
- Visualizar los resultados en *FlightGear*, aportando un entorno inmersivo de simulación.  

De este modo, se construye una base robusta para estudiar la estabilidad de aeronaves comerciales representativas y experimentar con técnicas de control.

## Funcionalidades clave

- Herramienta modular y extensible para análisis de dinámica de vuelo.  
- Comparación entre *modelo linealizado (3DOF/6DOF)* y simulación *no lineal*.  
- Simulación de *maniobras clásicas* de estabilidad.  
- Posibilidad de incorporar *bloques adicionales de control* (PID, SAS, autopiloto).  
- *Visualización 3D en FlightGear* para validar trayectorias y actitudes.  

---

## Requisitos

- MATLAB/Simulink 
- FlightGear (para visualización externa).  
- Joystick compatible (para control manual).  

---

El proyecto proporciona una *guía práctica* para construir, trimar y analizar un modelo dinámico completo de una aeronave civil. Sirve como base para:  
- Evaluar la *estabilidad* de configuraciones de vuelo.  
-Introducir *capaidad de control*.  
- Implementar extensiones como *modelos de motor completos, **variación de CG por consumo de combustible, o la **integración con otros sistemas aeronáuticos*.
