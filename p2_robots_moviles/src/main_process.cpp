/*
***************************
***************************
*******  S E T U P  *******
***************************
***************************
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdlib>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

std::vector<int> analyzePetition(std::string petition);



/*
***************************
***************************
**** C A L L B A C K S ****
***************************
***************************
*/

bool error; //Variable to flag when an error occures
bool goal_reached; //Variable to flag if the turtlebot has or hasnt arrived to the goal

void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    goal_reached = msg->data;

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
    ros::init(argc, argv, "main_process");
    ros::NodeHandle nh;

    //Publishers and subscribers
      ros::Publisher desiredAction_publisher = nh.advertise<std_msgs::Int32>("/desiredAction", 1); //Publisher to set a goal for the robot
      ros::Publisher color_publisher = nh.advertise<std_msgs::Int32>("/desiredColor", 1); //Publisher to set a goal for the robot

  //Get order
    std::string petition; //Store the command
    int size, color; //Store the asked size and color
    std::vector<int> analysys_result; //Store the result of the analysys

    std::cout << "¿Qué objeto desea? (Caja [Pequeña/mediana/grande] [roja/verde/azul]):"
    std::cin >> petition;

    //Analyze petition
    analysys_result = analyzePetition(petition);
    size = analysys_result[0]; //Extract the desired size
    color = analysys_result[1]; //Extract the desired color

  //Select goal (Small, medium or large tables)
    desiredAction_publisher.publish(size);

  //Wait for the robot to arrive
    //TODO!!!! Como esperar a que llegue al objetivo (llegar a entre las cajas)

  //Turn around searching the desired color box
    color_publisher.publish(color); //Publish the desired color
    desiredAction_publisher.publish(7); //Scan environment and pick object

  //Wait 1 sec (simulate that the robot is picking the object)

  //Select goal (window 1 (S), window 2 (M) or window 3 (L))
    desiredAction_publisher.publish(0); //size+3

  //Wait for the robot to arrive and 1 sec (simulate that the robot is giving the object)
    //TODO!!!! Como esperar a que llegue al objetivo (llegar a entre las cajas)

  //Leave the warehouse
desiredAction_publisher.publish(size+3);


  return 0;
}



/*
***************************
***************************
***** A U X.  F N C S *****
***************************
***************************
*/

std::vector<int> analyzePetition(std::string petition)
{
    std::vector<int> ret;

    //Find the desired size
    std::size_t found;
    if (petition.find("pequeña"))
        ret[0] = 1;
    else if (petition.find("mediana"))
        ret[0] = 2;
    else if (petition.find("grande"))
        ret[0] = 3;
    else
    {
        error = true;
        std::cout << "[!] ERROR: No se ha detectado un tamaño válido en la petición" << std::endl;
        std::cout << "    Tamaños válidos: pequeña, mediana, grande" << std::endl;
    }

    //Find the desired colour
    if (petition.find("roja"))
        ret[1] = 0;
    else if (petition.find("verde"))
        ret[1] = 1;
    else if (petition.find("azul"))
        ret[1] = 2;
    else
    {
        error = true;
        std::cout << "[!] ERROR: No se ha detectado un color válido en la petición" << std::endl;
        std::cout << "    Colores válidos: roja, verde, azul" << std::endl;
    }

    return ret;
}
