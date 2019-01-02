#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdlib>
#include <std_msgs/Float64MultiArray.h>


bool error; //Variable to flag when an error occures
std::vector<float> areas;

std::vector<int> analyzePetition(std::string petition);

void areaCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    areas = msg.data;

    return;
}

int main(int argc, char** argv)
{
  //ROS setup
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    //Publisher to set a goal for the robot
    ros::Publisher size_publisher = nh.advertise<std_msgs::Float64>("/actionSelect", 1);

    //Subscriber to the color areas
    ros::Subscriber area_subscriber = nh.subscribe("/areas", 1, areaCallback);


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
    size_publisher.publish(size);

  //Wait for the robot to arrive
    //TODO!!!! Como esperar a que llegue al objetivo (llegar a entre las cajas)

  //Turn around searching the desired color box
    float theta, theta_maxArea; //Robot orientation and orientation in which the maximum area was perceived
    float max_area; //Maximum perceived area so far

    for (theta=0; theta<3.14159265359; theta+=0.02) //Turn in sections of 0.02 rad (1º aprox.)
    {
        //Move the robot to the new orientation
            //TODO

        //Check if the actual area is bigger than the maximum perceived so far
        if (max_area < areas[color])
        {
            max_area = areas[color];
            theta_maxArea = theta; //Store actual orientation
        }
    }

    if (max_area == 0)
    {
        error = 1;
        std::cout << "[!] ERROR: No se ha detectado el color buscado"
    }

  //Wait 1 sec (simulate that the robot is picking the object)

  //Select goal (window 1 (S), window 2 (M) or window 3 (L))
    size_publisher.publish(size+3);

  //Wait for the robot to arrive and 1 sec (simulate that the robot is giving the object)
    //TODO!!!! Como esperar a que llegue al objetivo (llegar a entre las cajas)

  //Leave the warehouse
size_publisher.publish(size+3);


  return 0;
}



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
        ret[1] = 1;
    else if (petition.find("verde"))
        ret[1] = 2;
    else if (petition.find("azul"))
        ret[1] = 3;
    else
    {
        error = true;
        std::cout << "[!] ERROR: No se ha detectado un color válido en la petición" << std::endl;
        std::cout << "    Colores válidos: roja, verde, azul" << std::endl;
    }

    return ret;
}
