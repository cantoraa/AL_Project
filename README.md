# Proyecto final de vida artificial - Simulador
## Andrés Felipe Cantor Albarracín

El presente proyecto simula la interacción presa(zebra) - depredador(guepardo) en un ambiente artificial usando distintos conceptos
de vida artificial para hacer la simulación más cercana a la realidad.

### Tecnologías

* Allegro.CC para la construcción de elementos gráficos y el manejo de frames.
* Python para el tablero de control, la generación de pieles y de transformaciones afines
  * Scikit Image
  * Python Image Library (PIL)
  * Numpy
  * Matplotlib
* OpenMP para la paralelización del procesamiento
* JsonCPP para el manejo de archivos JSON de configuración

### Conceptos de Vida Artificial

* Sistemas de Lindenmayer para la construcción de plantas.
* Generación por pila de arena para la producción de comida.
* Transformaciones afines sinuidales para la generación de nuevas formas.
* Autómata celular para la generación de nuevas pieles.
* Ley de potencias para la producción de poblaciones en donde los individuos más favorecidos sean menos.
* BOIDS para el movimiento en manada de las presas.
* Segregación para el movimiento de los depredadores.
* Código Genético para la codificación de los factores que modulan la adaptabilidad del individuo al ambiente.
* Evolución del Código Genético por medio de algoritmos genéticos.

### Dependencias

Para funcionar, el simulador necesita las siguientes librerias

* Allegro5
* OpenMP
* JsonCPP

Además, deben ser creadas las siguiente carpetas para el guardado de las pieles y transformaciones creadas en tiempo de ejecución:

* new_transforms
* new_skins
* new_zebra_images

### Compilación

El archivo AL_Project.cpp puede ser compilado por medio de g++ usando el siguiente comando: 

g++ -std=c++11 -I/usr/include/allegro5 -o "AL_Project" "AL_Project.cpp" -lallegro_image -lallegro_primitives -lallegro_font -lallegro -ljsoncpp -fopenmp

### Configuración

La simulación se puede configurar con múltiples parámetros que se pueden encontrar en los siguientes archivos:

* [configuracion ambiente](Environment.json)
* [configuracion presa](Prey.json)
* [configuracion depredador](Predator.json)

#### El archivo [video](demonstration_video.mp4) contiene un video con una demostración de la ejecución del simulador.
#### Para una descripción detallada de los conceptos de vida artificial usados para la simulación, por favor remítase al archivo 'presentación_final.pdf'

