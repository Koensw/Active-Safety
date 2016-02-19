//init the ROS simulation
bool rosInit(int argc, char **argv){
    Log::info("ActiveSafetyLoader", "Initializing ROS mode...");
    //init ROS
    ros::init(argc, argv, "active_safety_simulation");
    ros::start();
    
    //load BJOS
    if(BJOS::getState() != BJOS::ACTIVE){
        Log::error("ActiveSafetyLoader", "Cannot continue, BJOS not active!");
        return false;
    }
    Process::installSignalHandler();
    BJOS *bjos = BJOS::getOS();
    
    //load the near space near space detector 
    near_space_detector = new NearSpaceDetector();
    
    //retrieve the sonar controller
    sonar_controller = new SonarController();
    if(!bjos->getController("sonar", sonar_controller)){
        Log::error("ActiveSafetyLoader", "Cannot continue, current OS does not have sonar controllor!");
        return false;
    }
    
    //load the sensor model
    std::vector<SonarData> sonar_data = sonar_controller->getData();
    for(size_t i=0; i<sonar_data.size(); ++i){
        //create interface
        BJOSSonarInterface *sonar_interface = new BJOSSonarInterface(sonar_controller, sonar_data[i].id);
        sonar_interfaces.push_back(sonar_interface);
        
        //creates a sensor and register it
        SonarSensor *sonar_sensor = new SonarSensor(sonar_data[i].pose, sonar_interface);
        near_space_detector->registerSensor(sonar_sensor);
    }
    
    //load the controller interface
    controller_interface = new RosControllerInterface("velocity", "iris/vehicle_local_position");
    
    //wait for the controller and sensors interfaces to be available
    ros::Rate wait_init_rate(100);
    /*for(size_t i=0; i<sonar_interfaces.size(); ++i){
        while(Process::isActive() && !sonar_interfaces[i]->isAvailable()) {
            ros::spinOnce();
            wait_init_rate.sleep();
        }
    }*/
    while(Process::isActive() && !controller_interface->isAvailable()){
        ros::spinOnce();
        wait_init_rate.sleep();
    }
    
    //load the active safety interface
    active_safety_interface = new ActiveSafetyInterface();
    
    //load the active safety
    active_safety = new ActiveSafety(near_space_detector, controller_interface, active_safety_interface);
    active_safety->setGlobalMinimumDistance(0.5);
    
    Log::info("ActiveSafetyLoader", "Finished initialization");
    return true;
}

//finalize the ROS simulation
void rosFinalize(){
    Log::info("ActiveSafetyLoader", "Finalizing...");
    //delete interfaces
    for(size_t i=0; i<sonar_interfaces.size(); ++i){
        delete sonar_interfaces[i];
    }
    delete controller_interface;
    delete active_safety_interface;
    
    //delete bjos controllers
    delete sonar_controller;
    
    //delete near space and active safety
    delete near_space_detector;
    delete active_safety;
    
    ros::shutdown();
    Log::info("ActiveSafetyLoader", "Finished finalization");
}

//run the simulation
void rosRun(){
    //set a position for the controller to goto
    //controller_interface->setTargetPosition(Point(0, 0, 4));
    
    //loop until request to die
    ros::Rate rate(50);
    int cnt = 0;
    
    //active_safety_interface->setTargetPosition(Point(8, 0, 3));
    active_safety->setTargetPoint(Point(0, 0, 3));
    
    //->setGlobalRepulsionStrength(5);
    active_safety->setGlobalRepulsionStrength(0);
    active_safety->setTargetAttractionStrength(0.3);
    while(Process::isActive()){
        for(size_t i=0; i<sonar_interfaces.size(); ++i){
            Log::info("ActiveSafetyLoader", "Distance %#1x %f", 0x70+i, sonar_interfaces[i]->getDistance());
        }
        
        //get current position from controller
        Point cur = controller_interface->getPosition();
        Log::info("ActiveSafetyLoader", "Position %f %f %f", cur.x(), cur.y(), cur.z());;
        
        //update active safety
        active_safety->update();
        
        //get direction where flying to
        Vector direction = active_safety->getDirection();
        Log::info("ActiveSafetyLoader", "Direction %f %f %f", direction.x(), direction.y(), direction.z());
        
        rate.sleep();
        ros::spinOnce();
-        
        ++cnt;
    }
}
