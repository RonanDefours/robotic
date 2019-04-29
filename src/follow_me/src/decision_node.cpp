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
  * MD - Ce programme permet de gérer l'aspect décisionnel du rob_and_seek
  * S'effectue sous forme d'automate à 3 états.
  *
  * Etat 1 : Attente du message goal_to_reach (lorsque programme object_detector_node
  *          Repère l'objet, publie ce message)
  * Si objet trouvé
  * - Calcul de rotation_to_do et translation_to_do grâce aux coordonnées de l'objet (x,y)
  *   et aux formules fournies (coord. polaires).
  * - Publication de la rotation_to_do
  * - Entrée dans l'état 2 (attente de la fin de la rotation effectuée par programme rotation_node)
  *
  * Sinon
  *   On avance et tourne à gauche en cas d'obstacle
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

    /**
      * MD - Ces deux points permettent d'évaluer en mode recherche s'il faut effectuer une rotation (obstacle rencontré)
      */
    geometry_msgs::Point start_position;
    geometry_msgs::Point current_position;

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

    sub_odometry = n.subscribe("odom", 1, &decision::odomCallback, this);

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
  * MD - Si on est en mode "recherche" (état 1) et que l'objet a été trouvé,
  * alors on se dirige vers lui (comme pour le follow_me)
  */

    if ( ( new_goal_to_reach ) && ( state == 1) ) {

        ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);
        new_goal_to_reach = false;

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        /**
          * MD - Calcul de la rotation/translation
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
              * MD - Publication de la rotation_to_do
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
            * MD - Passage dans l'état 3 (attente fin rotation)
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

        msg_goal_reached.x = goal_reached.x;
        msg_goal_reached.y = goal_reached.y;
        pub_goal_reached.publish(msg_goal_reached);
        new_goal_to_reach = false;

        /**
          * MD - Etat 1 : Recherche de l'objet
          */

        state = 1;
        ROS_INFO(" ");
        ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    }

    /**
      * État 1
      */
    if (state == 1){
      /**
        * MD - Mode recherche de l'objet
        * - Se déplace en ligne droite
        * - Si obstacle, tourne à gauche
        */

        start_position.x = current_position.x;
        start_position.y = current_position.y;

        /**
          * MD - On déplace le robot par pas de 1
          */

          translation_to_do = 1.0;
          std_msgs::Float32 msg_translation_to_do;
          ROS_INFO("Etat 1, effectue translation %f", translation_to_do);
          msg_translation_to_do.data = translation_to_do;
          pub_translation_to_do.publish(msg_translation_to_do);
          ROS_INFO("Dodo 7 secondes...");
          ros::Duration(7).sleep();
          ROS_INFO("Reveil");
          ROS_INFO("translation terminée");

          /**
            * MD -Lorsque l'on rencontre un obstacle, alors le robot tourche à gauche
            * Pour cela, on regarde si le robot a bougé ou s'il fait face à un obstacle
            */
            float distance_parcourue = distancePoints(start_position, current_position);
            if (distance_parcourue < 0.2){
              //TODO A SUPPRIMER (sert juste à vérifier passage dans condition)
              ROS_INFO("OBSTACLE RENCONTRE");
              // Effectue rotation à 90 degrés (idéalement)
              rotation_to_do = M_PI/2;
              ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do);
              /**
                * MD - Publication de la rotation_to_do
                */
                std_msgs::Float32 msg_rotation_to_do;
                msg_rotation_to_do.data = rotation_to_do;
                pub_rotation_to_do.publish(msg_rotation_to_do);
                ROS_INFO("Dodo 7 secondes...");
                ros::Duration(7).sleep();
                ROS_INFO("Reveil");
                ROS_INFO("rotation terminée");
            }


      //    state = 4; //Déplacement du robot
        }
    /**  if ( state == 4 && new_translation_done){ //On attend que la translation se termine
      //ROS_INFO("Attente fin translation");
      ROS_INFO("translation terminée, retour à l'état 1");
      state = 1;

    }*/

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

//Pour vérifier si le robot s'est suffisamment déplacé lors de la recherche
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
    //TODO A SUPPRIMER (sert juste à vérifier passage dans fonction + tester fréquence publication de l'odometre)
    ROS_INFO("PASSAGE ODOMCALLBACK DECISION NODE");
    current_position.x = o->pose.pose.position.x;
    current_position.y = o->pose.pose.position.y;
    current_position.z = o->pose.pose.position.z;
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
