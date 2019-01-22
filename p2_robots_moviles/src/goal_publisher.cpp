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
  #include <tf/LinearMath/Quaternion.h> //To create and use quaternions
  #include <tf/transform_datatypes.h> //Convert from tf quaternions to msg quat.

//Declarations
  void scanAreas(); //Function to rotate the turtlebot scanning the areas of the desired color (R/G/B) and pick the object from the orientation in which the area is the biggest
  geometry_msgs::Quaternion getQuatMsgFromTheta(float theta); //Returns a normalized quaternion given an orientation (Yaw angle)
  void updateGoal(move_base_msgs::MoveBaseGoal& goal, const float poses[7][4], int selectedGoal); //Load the "pose" number "selectedGoal" into the "goal" variable
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  bool error = 0;
  bool big_box_exception = false;


/*
***************************
***************************
**** C A L L B A C K S ****
***************************
***************************
*/

int selection = 0; //Received action (move to the poses 0, 1, 2 or 3 or perform the color area scanning)
int color = 0; //Received desired color (0, 1 or 2 -> red, green or blue)
std::vector<float> areas; //Received areas of each color (R, G and B)
std::vector<float> current_position(2); //Readed position (X and Y obtained thanks to AMCL)

void selectionCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int sel_aux = msg->data;
  
  if(sel_aux>=0 and sel_aux<=7) //Si se ha publicado una seleccion de pose valida
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
      ros::Subscriber area_subscriber = nh.subscribe("/rgb_areas", 1, areaCallback); //Color areas subscriber
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
    const float pose_home[3] = {22, 25, 1}; //Home (Outside the warehouse) //[0, 0, 0, 1]
    const float pose_s[3] = {19.5, 16.5, 1}; //Go to the small objects
    const float pose_m[3] = {19.5, 11.8, 1}; //Medium objects - 19.5; 11.8
    const float pose_l[3] = {16.5, 11.5, 1}; //Large objs - 16.5; 11.5
    const float pose_ws[3] = {14, 16, 1}; //Go to the small objects window //[0, 0, 1, 0]
    const float pose_wm[3] = {15, 13, 1}; //Medium objects window
    const float pose_wl[3] = {15, 10, 1}; //Large objs. w.

  //Create a vector with the poses (helps to access to them in a more elegant way)
    const float poses[7][4] = {{22, 25, 0, 1}, {19.5, 16.5, 0, 1}, {19.5, 11.8, 0, 1}, {16.5, 11.5, 1, 0}, {14.5, 16, 1, 0}, {14.5, 13, 1, 0}, {14.5, 10, 1, 0}};

  //Initialize areas vector
    areas.push_back(0.0);
    areas.push_back(0.0);
    areas.push_back(0.0);

  //MAIN LOOP
    while (ros::ok())
    {
      //Indicar que no se ha procesado ni alcanzado el nuevo objetivo
        goalReached.data = false;
        goalReached_publisher.publish(goalReached); //Indicate the robot is bussy traveling
  
      //Evaluate the selected action
        if(selection>=0 and selection<7)
        {
          updateGoal(goal, poses, selection); //Update the goal
  
      	  if(selection == 3)
      	    big_box_exception = true;
      	  else
      	    big_box_exception = false;
        }	
        else if (selection == 7)
        {
          scanAreas(); //Scan areas
        }
        else
        {
          std::cout << "[!] ERROR: Opcion ilegal introducida. Opciones validas: '0', '1', '2', '3', '4', '5', '6' y '7'" << std::endl;
          return 0;
        }
  
  
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
  float theta_ini, theta, theta_maxArea, theta_multiplier;//Robot initial orientation, orientation and orientation in which the maximum area was perceived
  float max_area; //Maximum perceived area so far
  float increment_multiplier;

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
    if(big_box_exception)
    {
      theta_multiplier = 3;
      increment_multiplier = -1;
    }
    else
    {
      theta_multiplier = 1;
      increment_multiplier = 1;
    }
    for (theta=-increment_multiplier*3.142/2; theta<theta_multiplier*3.142/2; theta+=0.02) //Ir desde -90 a 90 grados
    {
      //Update the areas (attending the pending callbacks
        ros::spinOnce();
  
      //Move the robot to the orientation in which the biggest area of the desired color was registered
        goal.target_pose.pose.orientation = getQuatMsgFromTheta(theta);
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
        goal.target_pose.pose.orientation = getQuatMsgFromTheta(theta_maxArea);
        ac.sendGoal(goal);
        ac.waitForResult();
    }

  return;
}


geometry_msgs::Quaternion getQuatMsgFromTheta(float theta)
{
  //Quaternion variables
    tf::Quaternion quat; //TF quaternion
    geometry_msgs::Quaternion quat_msg; //Msg quaternion

  //Generate quaternion
    quat.setRPY( 0, 0, theta);  //Create the quaternion from roll/pitch/yaw (in radians)
    quat.normalize();
    tf::quaternionTFToMsg(quat, quat_msg);

  return quat_msg;
}


void updateGoal(move_base_msgs::MoveBaseGoal& goal, const float poses[7][4], int selectedGoal)
{
  //Update goal
    goal.target_pose.pose.position.x = poses[selectedGoal][0];
    goal.target_pose.pose.position.y = poses[selectedGoal][1];
    goal.target_pose.pose.orientation.z = poses[selectedGoal][2];
    goal.target_pose.pose.orientation.w = poses[selectedGoal][3];

  return;
}
