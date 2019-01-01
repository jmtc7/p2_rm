# p2_rm
Este repositorio contiene los archivos de la 2ª práctica de la asignatura "Robots Móviles", del 4º año del grado de Ingeniería Robótica (Universidad de Alicante). Todo este contenido ha sido desarrollado por:
* NOVELO TÉLLEZ, Yoinel (https://github.com/yoinelnovelotellez)
* TORRES CÁMARA, José Miguel (https://github.com/jmtc7)

# Descripción de la aplicación
Implementremos una sistema robótico de logística interna. Se le pedirá un objeto definido por dos variables (pequeño/mediano/grande y rojo/verde/azul). El robot entrará al almacén, irá a donde se almacenen los del tamaño pedido y, mediante un escaneo basado en visión artificial, cogerá el del color solicitado.

La clasificación por tamaño serán varios grupos de mesas (cada uno formado por una roja, otra verde y otra azul), un grupo para objetos pequeños, otro para objetos medianos y otro para objetos grandes.

Por ejemplo, en el grupo de mesas de objetos pequeños, sobre la mesa roja habrán objetos pequeños de tipo 1, en la azul, objetos peq. de tipo 2 y en la verde, los peq. de tipo 3.

# Componentes del sistema
* Tendremos un **mapa** que contará con un pasillo en L con una puerta que accedía a una sala grande donde estarán las mesas a un lado y unas ventanas donde el robot entregará los objetos en el lado opuesto
* Un nodo de ROS que se suscriba a un *topic* donde se publiquen las imágenes captadas por el robot, la pase a HSV (para aumentar robustez frente a cambios de iluminación), umbralice por colores (rojo, verde y azul) y **publique** en otros *topics* las **áreas detectadas de cada color** en todo momento
* Otro nodo de ROS que se suscriba a un *topic* donde se publiquen mensajes Int32. Según el número que lea (0, 1, 2 o 3), se **publicará un u otro objetivo** (fuera del almacén, mesas pequeñas, mesas medianas o mesas grandes)
* Otro nodo que suponga el **control principal**. Se encargará de recibir las órdenes de un usuario que solicite un objeto y publicarlas en los *topics* que convenga para, gracias a los otros nodos, lograr el funcionamiento descrito
