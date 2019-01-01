#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int selection = 0;

void selectionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int sel_aux = msg->data;
  
  if(sel_aux==0 or sel_aux==1 or sel_aux==2 or sel_aux==3) //Si se ha publicado una seleccion de pose valida
  {
    selection = sel_aux;
  }
  else
  {
    std::cout << "[!] ERROR! Se ha introducido un identificador de pose ilegal. Opciones:" << std::endl;
    std::cout << "  '1': Salir del almacen" << std::endl;
    std::cout << "  '2': Ir a por objetos S" << std::endl;
    std::cout << "  '1': Ir a por objetos M" << std::endl;
    std::cout << "  '1': Ir a por objetos L" << std::endl;

  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ubiReader");

  //Suscriptor a topic para seleccionar goal
  ros::NodeHandle nh;
  ros::Subscriber action_getter = nh.subscribe("/actionSelect", 1, selectionCallback);

  //Configurar el cliente de accion
  MoveBaseClient ac("move_base", true);

  //Esperar a que el servidor de accion se inicie
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Configuracion general del goal
  move_base_msgs::MoveBaseGoal goal; //Mensaje a enviar
  goal.target_pose.header.frame_id = "/map"; //Indicar que el sistema de referencia sera el del mapa
  goal.target_pose.header.stamp = ros::Time::now();

  //Declarar posibles goals
  const float pose_home[3] = {2, 2, 1}; //Home (fuera del almacen)
  const float pose_s[3] = {5, 5, 1}; //Ir a los objetos pequenos
  const float pose_m[3] = {2, 8, 1}; //Ir a los objetos medianos
  const float pose_l[3] = {7, 9, 1}; //Ir a los objetos grandes


  while (ros::ok())
  {
      //Moverse al goal seleccionado
      switch (selection)
      {
        case 0:
          goal.target_pose.pose.position.x = pose_home[0];
          goal.target_pose.pose.position.y = pose_home[1];
          goal.target_pose.pose.orientation.w = pose_home[2];
        break;
        case 1:
          goal.target_pose.pose.position.x = pose_s[0];
          goal.target_pose.pose.position.y = pose_s[1];
          goal.target_pose.pose.orientation.w = pose_s[2];
        break;
        case 2:
          goal.target_pose.pose.position.x = pose_m[0];
          goal.target_pose.pose.position.y = pose_m[1];
          goal.target_pose.pose.orientation.w = pose_m[2];
        break;
        case 3:
          goal.target_pose.pose.position.x = pose_l[0];
          goal.target_pose.pose.position.y = pose_l[1];
          goal.target_pose.pose.orientation.w = pose_l[2];
        break;
        default:
          std::cout << "[!] ERROR: Opcion ilegal introducida. Opciones validas: '0'. 1', '2' y '3'" << std::endl;
          return 0;
        break;
      }
      

      ROS_INFO("[*] Enviando goal...");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("[*] Exito! Goal alcanzado");
      else
        ROS_INFO("[!] ERROR! No ha sido posible alcanzar el goal");
      

      //Configurar la pose "home" (salir del almacen) como siguiente destino
      selection = 0;

      ros::spinOnce();
  }

  return 0;
}

