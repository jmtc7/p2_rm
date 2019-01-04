/*
***************************
***************************
*******  S E T U P  *******
***************************
***************************
*/

//Include dependencies
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//Declarations
void scanAreas(); //Function to rotate the turtlebot scanning the areas of the desired color (R/G/B) and pick the object from the orientation in which the area is the biggest
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool error = 0;


/*
***************************
***************************
**** C A L L B A C K S ****
***************************
***************************
*/

int selection = 0; //Received action (move to the poses 0, 1, 2 or 3 or perform the color area scanning)ç
int color = 0; //Received desired color (0, 1 or 2 -> red, green or blue)
std::vector<float> areas; //Received areas of each color (R, G and B)
std::vector<float> current_position(2); //Readed position (X and Y obtained thanks to AMCL)

void selectionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int sel_aux = msg->data;
  
  if(sel_aux>0 and sel_aux<7) //Si se ha publicado una seleccion de pose valida
  {
    selection = sel_aux;
  }
  else
  {
    std::cout << "[!] ERROR! Se ha introducido un identificador de pose ilegal. Opciones:" << std::endl;
    std::cout << "  '0': Salir del almacen" << std::endl;
    std::cout << "  '1': Ir a por objetos S" << std::endl;
    std::cout << "  '2': Ir a por objetos M" << std::endl;
    std::cout << "  '3': Ir a por objetos L" << std::endl;
    std::cout << "  '4': Dejar objeto S en ventana 1" << std::endl;
    std::cout << "  '5': Dejar objeto M en ventana 2" << std::endl;
    std::cout << "  '6': Dejar objeto L en ventana 3" << std::endl;
    std::cout << "  '7': Escaneo de areas" << std::endl;

  }
}


void colorCallback(const std_msgs::Int32::ConstPtr& msg)
{
  color = msg->data;

  return;
}


void areaCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    areas[0] = msg->data[0];
    areas[1] = msg->data[1];
    areas[2] = msg->data[2];

    return;
}


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  current_position[0] = msg->pose.pose.position.x; //Store current X coordinate
  current_position[1] = msg->pose.pose.position.y; //Store current Y coordinate

  return;
}



/*
***************************
***************************
***** M A I N   F N C *****
***************************
***************************
*/

int main(int argc, char** argv)
{
  //ROS setup
    ros::init(argc, argv, "goal_publisher");
    std_msgs::Bool goalReached;
    ros::NodeHandle nh;

    //Subscribers
    ros::Subscriber action_getter = nh.subscribe("/desiredAction", 1, selectionCallback); //Action selection subscriber
    ros::Subscriber color_getter = nh.subscribe("/desiredColor", 1, colorCallback); //AMCL pose subscriber
    ros::Subscriber pose_getter = nh.subscribe("/amcl_pose", 1, poseCallback); //AMCL pose subscriber
    ros::Subscriber area_subscriber = nh.subscribe("/areas", 1, areaCallback); //Color areas subscriber
    ros::Publisher goalReached_publisher = nh.advertise<std_msgs::Bool>("/goal_reached", 1); //Publisher to tell if the robot is moving or not
    MoveBaseClient ac("move_base", true); //Action client

    //Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

  //General goal configuration
  move_base_msgs::MoveBaseGoal goal; //Mensaje a enviar
  goal.target_pose.header.frame_id = "/map"; //Indicar que el sistema de referencia sera el del mapa
  goal.target_pose.header.stamp = ros::Time::now();

  //Goals declarations
  const float pose_home[3] = {2, 2, 1}; //Home (Outside the warehouse)
  const float pose_s[3] = {5, 5, 1}; //Go to the small objects
  const float pose_m[3] = {2, 8, 1}; //Medium objects
  const float pose_l[3] = {7, 9, 1}; //Large objs.
  const float pose_ws[3] = {7, 5, 1}; //Go to the small objects window
  const float pose_wm[3] = {4, 8, 1}; //Medium objects window
  const float pose_wl[3] = {5, 9, 1}; //Large objs. w.


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
        case 4:
          goal.target_pose.pose.position.x = pose_ws[0];
          goal.target_pose.pose.position.y = pose_ws[1];
          goal.target_pose.pose.orientation.w = pose_ws[2];
        break;
        case 5:
          goal.target_pose.pose.position.x = pose_wm[0];
          goal.target_pose.pose.position.y = pose_wm[1];
          goal.target_pose.pose.orientation.w = pose_wm[2];
        break;
        case 6:
          goal.target_pose.pose.position.x = pose_wl[0];
          goal.target_pose.pose.position.y = pose_wl[1];
          goal.target_pose.pose.orientation.w = pose_wl[2];
        break;
        case 7:
          scanAreas();
        break;
        default:
          std::cout << "[!] ERROR: Opcion ilegal introducida. Opciones validas: '0', '1', '2', '3', '4', '5', '6' y '7'" << std::endl;
          return 0;
        break;
      }
      
      goalReached.data = false;
      goalReached_publisher.publish(goalReached); //Indicate the robot is bussy traveling

      ROS_INFO("[*] Enviando goal...");
      ac.sendGoal(goal);
      ac.waitForResult();

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("[*] Exito! Goal alcanzado");
      else
        ROS_INFO("[!] ERROR! No ha sido posible alcanzar el goal");

      ros::spinOnce();

      goalReached.data = true;
      goalReached_publisher.publish(goalReached); //Indicate the robot has finished the job
  }

  return 0;
}



/*
***************************
***************************
***** A U X.  F N C S *****
***************************
***************************
*/

void scanAreas()
{
  float theta, theta_maxArea;//Robot orientation and orientation in which the maximum area was perceived
  float max_area; //Maximum perceived area so far


  MoveBaseClient ac("move_base", true); //Action client
    
  //Wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Configure current pose
    //General goal configuration
      move_base_msgs::MoveBaseGoal goal; //ROS message to send
      goal.target_pose.header.frame_id = "/map"; //Indicar que el sistema de referencia sera el del mapa
      goal.target_pose.header.stamp = ros::Time::now();
    //Set current coordinates X and Y)
      goal.target_pose.pose.position.x = current_position[0];
      goal.target_pose.pose.position.y = current_position[1];
  
  //Turn in sections of 0.02 rad (1º aprox.)
  ROS_INFO("[*] Escaneando colores...");
  for (theta=0; theta<3.142; theta+=0.02)
  {
    //Move the robot to the new orientation
    goal.target_pose.pose.orientation.w = theta; //Update theta
    ac.sendGoal(goal);
    ac.waitForResult();

    //Check if the actual area is bigger than the maximum perceived so far
    if (max_area < areas[color])
    {
        max_area = areas[color];
        theta_maxArea = theta; //Store actual orientation
    }
  }

  //Check if no area was detected
  if (max_area == 0)
  {
      error = 1;
      std::cout << "[!] ERROR: No se ha detectado ningun area del color buscado" << std::endl;
  }
  else //Rotate and "pick" the object
  {
    //Move the robot to the orientation in which the biggest area of the desired color was registered
    goal.target_pose.pose.orientation.w = theta; //Update theta
    ac.sendGoal(goal);
    ac.waitForResult();
  }

  return;
}
