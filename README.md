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
* Otro nodo de ROS que se suscriba a un *topic* donde se publiquen mensajes Int32. Según el número que lea (0, 1, 2, 3, 4, 5 o 6), se **publicará un u otro objetivo** (fuera del almacén, mesas pequeñas, mesas medianas, mesas grandes, ventana 1, ventana 2, ventana 3 o escaneo del entorno)
* Otro nodo que suponga el **control principal**. Se encargará de recibir las órdenes de un usuario que solicite un objeto y publicarlas en los *topics* que convenga para, gracias a los otros nodos, lograr el funcionamiento descrito

# Launching instructions
In order to launch the system, assuming you have got a working ROS distribution instaled, the **first** thing to do is, obviously, to **download the package** (p2_robots_moviles). You can download it by using *git clone [repository_URL]* via commandline or using a browser plugin to download just the needed folder (avoiding download the multimedia one and the documentation).

Next thing to do is to **put it inside a catkin workspace** (if not downloaded in it). Specifically, you will have to introduce it in the *[catkin_workspace_name]/src/* folder. Once there, the next step is to **compile** it, by typing *catkin_make* in a terminal emulator, being at the catkin workspace folder.

Now everything is ready to run (assuming you have sourced your catkin workspace). You will need two terminal emulators:
* In one you will launch a *launchfile* that runs the simulation (in Gazebo 7), the ROS' navigation stack, the visualization (RViz) and the nodes *area_publisher* and *goal_publisher*. To do this, you will type ***roslaunch p2_robots_moviles p2_launcher.launch***
* In the other terminal emulator, you will manually run the *main_process* node by typing ***rosrun p2_robots_moviles main_process***. We implemented it this way so the user could easily manage when he wants to run or stop this node. Each time it is run, the node will ask the user to give the robot a target (writing it in the command line), will make the robot go for it, search ir over a place, deliver the object to the user and, finally, go back to the home possition and end its execution. This makes it possible to upgrade the working flow and include a web or app interface, buttons or a higher level program that manages all this making it more user-friendly
