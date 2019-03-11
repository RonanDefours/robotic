#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>

/**
  * MD - Ce programme permet de gérer l'aspect décisionnel du follow_me
  * S'effectue sous forme d'automate à 3 états
  *
  * Etat 1 : Attente du message goal_to_reach (lorsque programme moving_persons_detector_node
  *          Repère une personne, publie ce message)
  * - Calcul de rotation_to_do et translation_to_do grâce aux coordonnées de la personne (x,y)
  *   et aux formules fournies (coord. polaires).
  * - Publication de la rotation_to_do
  * - Entrée dans l'état 2 (attente de la fin de la rotation effectuée par programme rotation_node)
  *
  *
  * Etat 2 :  Attente du message rotation_done (lorsque programme rotation_node
  *           a terminé la rotation, publie ce message)
  * - Publication de la translation_to_do
  * - Entrée dans l'état 3 (attente de la fin de la translation effectuée par programme translation_node)
  *
  *
  * Etat 3 :  Attente du message translation_done (lorsque programme translation_node
  *           a terminé la translation, publie ce message)
  *
  * - Publication du goal_reached
  * - Retour a l'état 1
  *
  */
class decision {
private:

    ros::NodeHandle n;

    // communication with one_moving_person_detector or person_tracker
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;

    // communication with rotation
    ros::Publisher pub_rotation_to_do;
    ros::Subscriber sub_rotation_done;

    // communication with translation
    ros::Publisher pub_translation_to_do;
    ros::Subscriber sub_translation_done;

    float rotation_to_do;
    float rotation_done;
    float translation_to_do;
    float translation_done;

    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not
    bool new_rotation_done;//to check if a new /rotation_done is available or not
    bool new_translation_done;//to check if a new /translation_done is available or not

    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point goal_reached;

    int state;
    bool display_state;

public:

decision() {

    // communication with moving_persons_detector or person_tracker
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &decision::goal_to_reachCallback, this);

    // communication with rotation_action
    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
    sub_rotation_done = n.subscribe("rotation_done", 1, &decision::rotation_doneCallback, this);

    // communication with translation_action
    pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
    sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);

    state = 1;
    display_state = false;
    new_goal_to_reach = false;
    new_rotation_done = false;
    new_translation_done = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( !display_state ) {
        display_state = true;
        ROS_INFO("state: %i", state);
    }

    // we receive a new /goal_to_reach and robair is not doing a translation or a rotation
    /**
      * MD - Etat 1
      */

    if ( ( new_goal_to_reach ) && ( state == 1 ) ) {

        ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);
        new_goal_to_reach = false;

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        /**
          * MD - 1.1 Calcul de la rotation/translation
          */
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        if ( translation_to_do ) {
            //we compute the /rotation_to_do
            rotation_to_do = acos( goal_to_reach.x / translation_to_do );

            if ( goal_to_reach.y < 0 )
                rotation_to_do *=-1;

            display_state = false;
            //we first perform the /rotation_to_do
            ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
            std_msgs::Float32 msg_rotation_to_do;
            //to complete
            /**
              * MD - 1.2 Publication de la rotation_to_do
              */
              msg_rotation_to_do.data = rotation_to_do;
              pub_rotation_to_do.publish(msg_rotation_to_do);
              /**
                * MD - Passage dans l'état 2 (attente fin rotation)
                */
              state = 2;


        }
        else {
            geometry_msgs::Point msg_goal_reached;
            msg_goal_reached.x = 0;
            msg_goal_reached.y = 0;

            ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
            pub_goal_reached.publish(msg_goal_reached);
        }
    }

    //we receive an ack from rotation_action_node. So, we perform the /translation_to_do
    /**
      * MD - Etat 2
      */
    if ( ( new_rotation_done ) && ( state == 2 ) ) {
        ROS_INFO("(decision_node) /rotation_done : %f", rotation_done*180/M_PI);
        new_rotation_done = false;

        display_state = false;
        //the rotation_to_do is done so we perform the translation_to_do
        ROS_INFO("(decision_node) /translation_to_do: %f", translation_to_do);
        std_msgs::Float32 msg_translation_to_do;
        //to complete
        /**
          * MD - 2.1 Publication de la translation_to_do
          */

          msg_translation_to_do.data = translation_to_do;
          pub_translation_to_do.publish(msg_translation_to_do);
          /**
            * MD - Passage dans l'état 2 (attente fin rotation)
            */
          state = 3;
    }

    //we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
    /**
      * MD - Etat 3
      */
    if ( ( new_translation_done ) && ( state == 3 ) ) {
        ROS_INFO("(decision_node) /translation_done : %f\n", translation_done);
        new_translation_done = false;

        display_state = false;
        //the translation_to_do is done so we send the goal_reached to the detector/tracker node
        geometry_msgs::Point msg_goal_reached;
        ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
        //to complete

        /**
          * MD - 3.1 Publication de la destination atteinte
          * //TODO Si ne fonctionne pas, remplacer msg_goal_reached par goal_reached
          */
        msg_goal_reached.x = goal_reached.x;
        msg_goal_reached.y = goal_reached.y;
        pub_goal_reached.publish(msg_goal_reached);
        new_goal_to_reach = false;

        /**
          * MD - 3.2 retour à l'état initial (en attente de goal to reach)
          */

        state = 1;
        ROS_INFO(" ");
        ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;

}

void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
// process the angle received from the rotation node

    new_rotation_done = true;
    rotation_done = a->data;

}

void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
// process the range received from the translation node

    new_translation_done = true;
    translation_done = r->data;

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
