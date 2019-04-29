// moving persons detector using lidar data
// written by O. Aycard

/**
  * MD -
  * Résumé du programme :
  * 1- Détection mouvements de l'environnement (comparaison avant/après)
  * 2- Création + config des clusters (id, taille, centre, % points dynamiques, coloration, nombre..)
  * 3- Détection des jambes (filtrage clusters qui correspondent à des jambes)
  * 4- Détection des personnes (comparaison des jambes) + publication goal_to_reach
  *
  */

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

//used for clustering
#define cluster_threshold 0.2 //threshold for clustering

//used for detection of motion
#define detection_threshold 0.2 //threshold for motion detection

//used for detection of moving legs
#define object_size_min 0.05
#define object_size_max 0.25

//used for detection of moving persons
#define legs_distance_max 0.7

using namespace std;

class object_detector_node {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;


    ros::Publisher pub_moving_persons_detector;
    ros::Publisher pub_moving_persons_detector_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    float background[1000];//to store the background
    //bool dynamic[1000];//to store if the current is dynamic or not

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    //int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000], cluster_end[1000];

    //to perform detection of object detected and to store them
    int nb_object_detected;
    geometry_msgs::Point object_detected[1000];// to store the middle of each object detected.
    int id_object_detected [1000];

    //to store the goal to reach that we will be published
    geometry_msgs::Point goal_to_reach;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

    //to check if the robot is moving or not
    bool previous_robot_moving;
    bool current_robot_moving;

    bool init_laser;//to check if new data of laser is available or not
    bool init_robot;//to check if new data of robot_moving is available or not

    bool display_laser;
    bool display_robot;

public:

object_detector_node() {

    sub_scan = n.subscribe("scan", 1, &object_detector_node::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &object_detector_node::robot_movingCallback, this);

    pub_moving_persons_detector_marker = n.advertise<visualization_msgs::Marker>("moving_person_detector", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_moving_persons_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    current_robot_moving = true;
    init_laser = false;
    init_robot = false;
    display_laser = false;

    display_robot = false;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data and robot_moving
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( init_laser && init_robot ) {
        nb_pts = 0;

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_pts = 0;
        //if the robot is not moving then we can perform moving persons detection
        if ( !current_robot_moving ) {

            ROS_INFO("robot is not moving");
                // if the robot was moving previously and now it is not moving now then we store the background
            if ( previous_robot_moving && !current_robot_moving )
                store_background();

            //we search for moving persons in 4 steps
            //detect_motion();//to classify each hit of the laser as dynamic or not
            perform_clustering();//to perform clustering
            detect_object();//to detect objects using cluster
            //detect_moving_persons();//to detect moving_persons using moving legs detected

            //graphical display of the results
            populateMarkerTopic();

            //to publish the goal_to_reach
            if ( nb_object_detected )
                pub_moving_persons_detector.publish(goal_to_reach);
        }
        else
            ROS_INFO("robot is moving");
    }
    else {
        if ( !display_laser && !init_laser ) {
            ROS_INFO("wait for laser data");
            display_laser = true;
        }
        if ( display_laser && init_laser )  {
            ROS_INFO("laser data are ok");
            display_laser = false;
        }
        if ( !display_robot && !init_robot ) {
            ROS_INFO("wait for robot_moving_node");
            display_robot = true;
        }
        if ( display_robot && init_robot ) {
            ROS_INFO("robot_moving_node is ok");
            display_robot = true;
        }
    }

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {

  /**
    * MD : Permet de stocker le balayage laser précédent
    * Nécessaire pour comparer s'il y a un mouvement au prochain balayage laser
    */
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = range[loop];

    ROS_INFO("background stored");

}//init_background

void detect_motion() {



}//detect_motion

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
  /**
    * MD : Permet de constituer les groupes de points (cluster)
    * Premier point cluster : vert (initialisation)
    * Dernier point cluster : rouge (lorsque seuil dépassé)
    */
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is lower than "cluster_threshold"
//then the current hit belongs to the current cluster
//else we start a new cluster with the current hit and end the current cluster

    ROS_INFO("performing clustering");

    nb_cluster = 0;//to count the number of cluster

    //initialization of the first cluster
    cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    //graphical display of the start of the current cluster in green

    /**
      * MD : Point vert = début du cluster
      */
    display[nb_pts].x = current_scan[cluster_start[nb_cluster]].x;
    display[nb_pts].y = current_scan[cluster_start[nb_cluster]].y;
    display[nb_pts].z = current_scan[cluster_start[nb_cluster]].z;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    /**
      * MD : Permet de stocker les groupes de cluster
      * Ex : cluster[0] = 0 ; cluster[1] = 0 ; cluster[2] = 0 ; cluster[3] = 1
      * Il a le cluster 0 (resp 1) qui contient 3 points (resp 1 point)
      */
    for( int loop=1; loop<nb_beams; loop++ )//loop over all the hits
        //if distance between (the previous hit and the current one) is lower than "cluster_threshold"

        /**
          * MD - Point supplémentaire pour ce cluster
          */
        if (distancePoints(current_scan[loop-1],current_scan[loop])< cluster_threshold){
            cluster[loop]=nb_cluster;   //the current hit belongs to the current cluster
            /**
              * MD : Compte le nombre de Point dynamique (tab dynamic initialisé par méthode detect_motion)
              * Réinitialisé à chaque nouveau cluster
              */
          /*  if (dynamic[loop]==1)
            {
                nb_dynamic++;
            }*/
        }

        /**
          * MD - Seuil dépassé, on termine donc le cluster précédent et on
          * en créé un nouveau à partir du point actuel
          */
        else {//the current hit doesnt belong to the same hit
              /*1/ we end the current cluster, so we update:
              - cluster-end to store the last hit of the current cluster
              - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic
              - cluster_size to store the size of the cluster ie, the distance between the first hit of the cluster and the last one
              - cluster_middle to store the middle of the cluster*/

          /**
            * MD - cluster_end : Tableau où l'on stocke le dernier point de chaque cluster
            *       Indice = n°grp cluster
            *      Valeur = n°dernier point
            */
            cluster_end[nb_cluster]=loop-1;
            /**
              * MD - cluster_dynamic : Tableau où l'on stocke le % de points dynamiques de chaque
              *      cluster
              *      Indice = n°grp cluster
              *      Valeur = % points dynamiques
              */
              /*
            if (loop-cluster_start[nb_cluster] > 0) {

              cluster_dynamic[nb_cluster] = (nb_dynamic/(loop-cluster_start[nb_cluster]))*100;
            }else{
                cluster_dynamic[nb_cluster] =0;
            }*/
            /**
              * MD - cluster_size : Tableau où l'on stocke la distance entre le
              *      premier et le dernier point de chaque cluster
              *      Indice = n°grp cluster
              *      Valeur = distance entre les extrémités du cluster
              */
            cluster_size[nb_cluster]=distancePoints(current_scan[cluster_end[nb_cluster]],current_scan[cluster_start[nb_cluster]]);

            /**
              * MD - cluster_middle : Tableau où l'on stocke la coordonnée x et y
              *      moyenne du cluster
              *      Indice = n°grp cluster
              *      Valeur = Point au centre du cluster
              */
            cluster_middle[nb_cluster].x= (current_scan[cluster_start[nb_cluster]].x + current_scan[cluster_end[nb_cluster]].x)/2;
            cluster_middle[nb_cluster].y= (current_scan[cluster_start[nb_cluster]].y + current_scan[cluster_end[nb_cluster]].y)/2;

            //graphical display of the end of the current cluster in red
            /**
              * MD : Point rouge = fin du cluster
              */
            display[nb_pts].x = current_scan[cluster_end[nb_cluster]].x;
            display[nb_pts].y = current_scan[cluster_end[nb_cluster]].y;
            display[nb_pts].z = current_scan[cluster_end[nb_cluster]].z;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

            //textual display
            ROS_INFO("cluster[%i]: [%i](%f, %f) -> [%i](%f, %f), size: %f", nb_cluster, cluster_start[nb_cluster], current_scan[cluster_start[nb_cluster]].x, current_scan[cluster_start[nb_cluster]].y, cluster_end[nb_cluster], current_scan[cluster_end[nb_cluster]].x, current_scan[cluster_end[nb_cluster]].y, cluster_size[nb_cluster] );

            //2/ we starta new cluster with the current hit
            /**
              * MD - Début du nouveau cluster (similaire à init premier cluster cf début méthode)
              */
            nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
            nb_cluster++;
            cluster_start[nb_cluster] = loop;
            cluster[loop] = nb_cluster;
          /*  if ( dynamic[loop] )
                nb_dynamic++;*/

            //graphical display of the start of the current cluster in green
            display[nb_pts].x = current_scan[cluster_start[nb_cluster]].x;
            display[nb_pts].y = current_scan[cluster_start[nb_cluster]].y;
            display[nb_pts].z = current_scan[cluster_start[nb_cluster]].z;

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            nb_pts++;

        }

    //Dont forget to update the different information for the last cluster
    //...
    cluster_end[nb_cluster]=nb_beams-1;/*
    if (nb_beams-cluster_start[nb_cluster] >0) {

      cluster_dynamic[nb_cluster] = (nb_dynamic/(nb_beams-cluster_start[nb_cluster]))*100;
    }else{
      cluster_dynamic[nb_cluster] =0;
    }*/
    cluster_size[nb_cluster]=(nb_beams -1)-cluster_start[nb_cluster];
    cluster_middle[nb_cluster].x= (current_scan[cluster_start[nb_cluster]].x + current_scan[cluster_end[nb_cluster]].x)/2;
    cluster_middle[nb_cluster].y= (current_scan[cluster_start[nb_cluster]].y + current_scan[cluster_end[nb_cluster]].y)/2;
    nb_cluster++;

    ROS_INFO("clustering performed");

}//perfor_clustering

// DETECTION OF MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*
	fonction qui teste si le cluster est un arc de cercle
	on teste la symetrie du cluster courant
*/
bool detect_circular(int current_cluster){

	float threshold= 0.01; // accuracy
	int cmp=0;
  geometry_msgs::Point point_start = current_scan[cluster_start[current_cluster]];
  geometry_msgs::Point point_end = current_scan[cluster_end[current_cluster]];
  geometry_msgs::Point point_median = current_scan[cluster_start[current_cluster+(int)(cluster_size[current_cluster]/2)]];
  geometry_msgs::Point point_middle = cluster_middle[current_cluster];
  int ratio = 0;

  if(distancePoints(point_median,point_middle)>(0.6*distancePoints(point_middle,point_start)) && distancePoints(point_middle,point_start)>=distancePoints(point_middle, point_median)){
  	for (int i = 0; i < cluster_size[current_cluster]/2; ++i)
  	{
  		if( (distancePoints(point_start,point_middle) <= distancePoints(point_middle,point_end) + threshold) && (distancePoints(point_start,point_middle) >= distancePoints(point_middle,point_end) - threshold)){
  			ratio++;
  		}
  		cmp++;
  		point_start = current_scan[cluster_start[current_cluster]+cmp];
  		point_end = current_scan[cluster_end[current_cluster]-cmp];
  	}
  }
	return ((ratio/(cluster_size[current_cluster]/2)) >= 0.95); // on teste s'il y a plus de 95% de symetrie

}


// DETECTION OF MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_object() {
	// un cylindre :
	// - un cluster entre 20-40 cm
	// - 2 points en partant des extrémités symétrique p/r au centre du cluster

	float cylinder_size_min = 0.20;
	float cylinder_size_max = 0.40;

    ROS_INFO("detecting cylinder");
    nb_object_detected = 0;

    /**
      * MD - On parcourt chaque cluster (données fournies par méthode perform_clustering)
      * S'il correspond aux critères pour en faire un cylindre
      */
    for (int loop=0; loop<nb_cluster; loop++){//loop over all the clusters
        //if the size of the current cluster is higher than "leg_size_min" and lower than "leg_size_max" and it has "dynamic_threshold"% of its hits that are dynamic
        //then the current cluster is a moving leg

        /**
          * MD - Si le cluster correspond à un cylindre de 20-30 cm
          */
        if (cluster_size[loop] > cylinder_size_min && cluster_size[loop] < cylinder_size_max && detect_circular(loop)){
            // we update the cylinder_detected table to store the middle of the moving leg
            nb_object_detected++;
            /**
              * MD - On l'ajoute au tableau nb_cylinder_detected
              *      Index : Nombre de cylindre
              *      Valeur : Point moyen du cluster actuel (milieu = centre cylindre)
              */
            object_detected[nb_object_detected] =cluster_middle[loop];
            //textual display
            ROS_INFO("cylinder detected[%i]: cluster[%i]", nb_object_detected, loop);
            /**
              * MD - Affichage de toutes les cylindre détectés
              */
            //graphical display
            for(int loop2=cluster_start[loop]; loop2<=cluster_end[loop]; loop2++) {
                // moving legs are white
                display[nb_pts].x = current_scan[loop2].x;
                display[nb_pts].y = current_scan[loop2].y;
                display[nb_pts].z = current_scan[loop2].z;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }
            //update of the goal and publish of the goal
                display[nb_pts].x = cluster_middle[loop].x;
                display[nb_pts].y = cluster_middle[loop].y;
                display[nb_pts].z = cluster_middle[loop].z;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;
                goal_to_reach.x = object_detected[nb_object_detected].x;
                goal_to_reach.y = object_detected[nb_object_detected].y;
        }
    }
    if ( object_detected )
        ROS_INFO("%d cylinder have been detected.\n", nb_object_detected);

    ROS_INFO("cylinder detected");


}//scanCallback
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop] = scan->ranges[loop];
        else
            range[loop] = range_max;

        //transform the scan in cartesian framework
        current_scan[loop].x = range[loop] * cos(beam_angle);
        current_scan[loop].y = range[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
    previous_robot_moving = current_robot_moving;
    current_robot_moving = state->data;

}//robot_movingCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_moving_persons_detector_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_moving_persons_detector_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "moving_persons_detector");
    object_detector_node bsObject;

    ros::spin();

    return 0;
}
