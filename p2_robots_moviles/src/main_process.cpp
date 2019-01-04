/*
***************************
***************************
*******  S E T U P  *******
***************************
***************************
*/

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

std::vector<int> analyzePetition(std::string petition);
void waitUntil_GoalReached(); //Function that blocks the program until the setted goal is reached


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
      ros::Subscriber goalReached_subscriber = nh.subscribe("/goal_reached", 1, goalReachedCallback); //Goal reaching subscriber

  //Get order
    std::string petition; //Store the command
    std_msgs::Int32 size, color; //Store the asked size and color
    std::vector<int> analysys_result; //Store the result of the analysys

    std::cout << "¿Qué objeto desea? (Caja [pequeña/mediana/grande] [roja/verde/azul]):" << std::endl;
    std::getline(std::cin, petition); //Store the input in "petition"

    //Analyze petition
    analysys_result = analyzePetition(petition);
    size.data = analysys_result.at(0); //Extract the desired size
    color.data = analysys_result.at(1); //Extract the desired color

  //Select goal (Small, medium or large tables)
    desiredAction_publisher.publish(size);

  //Wait for the robot to arrive
    waitUntil_GoalReached();

  //Turn around searching the desired color box and pick the object
    color_publisher.publish(color); //Publish the desired color
    size.data = 7;
    desiredAction_publisher.publish(size); //Scan environment and pick object

  //Wait until the object is picked
    waitUntil_GoalReached();

  //Select goal (window 1 (S), window 2 (M) or window 3 (L))
    size.data = analysys_result.at(0)+3;
    desiredAction_publisher.publish(size);

  //Wait for the robot to arrive and 1 sec (simulate that the robot is giving the object)
    waitUntil_GoalReached();

  //Leave the warehouse
    size.data = 0;
    desiredAction_publisher.publish(size);

  //Wait for the robot to leave
    waitUntil_GoalReached();

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
    if (petition.find("pequeña") != std::string::npos)
        ret.push_back(1);
    else if (petition.find("mediana") != std::string::npos)
        ret.push_back(2);
    else if (petition.find("grande") != std::string::npos)
        ret.push_back(3);
    else
    {
        error = true;
        ret.push_back(0); //Keep being outside the warehouse
        std::cout << "[!] ERROR: No se ha detectado un tamaño válido en la petición" << std::endl;
        std::cout << "    Tamaños válidos: pequeña, mediana, grande" << std::endl;
    }

    //Find the desired colour
    if (petition.find("roja") != std::string::npos)
        ret.push_back(0);
    else if (petition.find("verde") != std::string::npos)
        ret.push_back(1);
    else if (petition.find("azul") != std::string::npos)
        ret.push_back(2);
    else
    {
        error = true;
        ret.push_back(0); //Give a value for the color (doesnt mind)
        std::cout << "[!] ERROR: No se ha detectado un color válido en la petición" << std::endl;
        std::cout << "    Colores válidos: roja, verde, azul" << std::endl;
    }

    return ret;
}



void waitUntil_GoalReached()
{
    ros::spinOnce(); //Attend pending callbacks
    
    while(!goal_reached)
    {
        ros::spinOnce(); //Attend pending callbacks
    }

    return;
}