#include "../include/kerberos_waypoint_publisher.hpp"
void WayPointManager::StateTransition(){
    //std::cout << current_wp_iter->wp_type << std::endl;
    if(current_wp_iter == wp_array.end()) {
        std::cout << "all waypoint reached" << std::endl;
        exit(0);
    }

    if (current_wp_iter->wp_type == "normal"){
        StateNormal();
    }
    else if (current_wp_iter->wp_type == "back"){
        StateBack();
    }
    else if(current_wp_iter->wp_type == "target"){
        StateFindTarget();
    }
    else if(current_wp_iter->wp_type == "wait"){
        StateWait(); 
    }
    else if(current_wp_iter->wp_type == "turning"){
        StateTurning(); 
    }
    else if(current_wp_iter->wp_type == "line"){
        StateLine(); 
    }
    else if(current_wp_iter->wp_type == "line_before"){
        StateLineBefore(); 
    }
    else if(current_wp_iter->wp_type == "signal"){
        StateSignal(); 
    }
    else if(current_wp_iter->wp_type == "crosswalk"){
        StateCrosswalk();
    }
    else if (current_wp_iter->wp_type == "edr_true"){
        std_msgs::Bool edr;
        edr.data = true;
        target_start_flag_publisher.publish(edr);
        
        std_msgs::String message;
        message.data ="探索対象認識を開始します";
        talk_publisher.publish(message);
        StateNormal();
    }
    else if (current_wp_iter->wp_type == "edr_false"){
        std_msgs::Bool edr;
        edr.data = false;
        target_start_flag_publisher.publish(edr);

        std_msgs::String message;
        message.data ="探索対象認識を終了します";
        talk_publisher.publish(message);
        StateNormal();
    }
    else if (current_wp_iter->wp_type == "amcl"){
        std_msgs::String hnd;
        hnd.data = "amcl";
        hnd_flag_publisher.publish(hnd);

        std_msgs::String message;
        message.data ="地図なし区間を出ました";
        talk_publisher.publish(message);

        StateNormal();
    }
    else if (current_wp_iter->wp_type == "gmapping"){
        std_msgs::String hnd;
        hnd.data = "gmapping";
        hnd_flag_publisher.publish(hnd);

        std_msgs::String message;
        message.data ="地図なしに進入します";
        talk_publisher.publish(message);

        StateNormal();
    }
    else if(current_wp_iter->wp_type == "end_point"){
        //state = "end_point";
        StateEnd(); 
    }else{
        std::cerr << "no such waypoint type: " << current_wp_iter->wp_type << std::endl;;
    }
}

void WayPointManager::StateCrosswalk(){
    dynamic_reconfigure::Config conf, gpconf, lpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;     

    conf.doubles.clear();
    double_param.name="max_range";
    double_param.value=1.0;
    conf.doubles.push_back(double_param);
    srv.request.config = conf;
    laser_fixer_client1.call(srv);    
    laser_fixer_client2.call(srv);

    lpconf.doubles.clear(); 
    double_param.name="max_vel_x";
    //double_param.value=0.7;
    double_param.value=0.9;
    lpconf.doubles.push_back(double_param);
    double_param.name="acceleration_lim_x";
    double_param.value=signal_acceleration;
    lpconf.doubles.push_back(double_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=3000.0;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint " << current_wp_iter->wp_type << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 2; 
    force_ymg_publisher.publish(mode);
    /////////////////////////////////////////////////////////////////////////
    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);

        //check if reached goal
        //if (diff_dis_m < 3.0 ) {
        if (diff_dis_m < goal_tolerance_m/1.5 ) {
            std::cout << "Reached waypoint" << std::endl;
            forward_wp_flag = true;
        }
        
        //timeout
        if ( (ros::Time::now() - wp_update_time) > ros::Duration(skip_time_sec)) {
            std_srvs::Empty srv;
            if (amcl_client.call(srv)){
//                ROS_INFO("Sum: %ld", (long int)srv.response.sum);
            }else{
                ROS_ERROR("Failed to call service add_two_ints");
            }
        } 

        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            if( diff_dis_m < 6) {
                forward_wp_flag =true;
            }else{
                action_client->sendGoal(current_goal);
            }
        }         
        ros::Duration(0.1).sleep();
    }
    if (forward_wp_flag) {
        ++current_wp_iter;
        forward_wp_flag = false;
    }

    conf.doubles.clear();
    double_param.name="max_range";
    double_param.value=30.0;
    conf.doubles.push_back(double_param);
    srv.request.config = conf;
    laser_fixer_client1.call(srv);    
    laser_fixer_client2.call(srv);

    lpconf.doubles.clear(); 
    double_param.name="max_vel_x";
    double_param.value=1.0;
    lpconf.doubles.push_back(double_param);
    double_param.name="acceleration_lim_x";
    double_param.value=normal_acceleration;
    lpconf.doubles.push_back(double_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=10.0;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

}

void WayPointManager::UseBGPCallback(const std_msgs::Bool::ConstPtr &msg){
    std::cerr << "CALLLEDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD" << std::endl;
    /*
    if(msg->data){
        dynamic_reconfigure::Config conf;
        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name="obstacle_stop_margin";
        double_param.value=0.2;
        std::cerr << "margin set to 0.2" << std::endl;
        conf.doubles.push_back(double_param);
        dynamic_reconfigure::Reconfigure srv;
        srv.request.config = conf;
        localplanner_client.call(srv);            
    }else{
        dynamic_reconfigure::Config conf;
        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name="obstacle_stop_margin";
        double_param.value=1.0;
        conf.doubles.push_back(double_param);
        dynamic_reconfigure::Reconfigure srv;
        srv.request.config = conf;
        localplanner_client.call(srv);                    
    }
    */
}

bool WayPointManager::MoveBackward(){
    std::list<WayPoint>::iterator loop_iter = current_wp_iter;
    for ( int i = 0; i < 20 && loop_iter != wp_array.begin(); i++){
        loop_iter--;
        if(loop_iter->wp_type == "target") return true;
    }
    return false;
}

void WayPointManager::StateSignal(){
    //wait until enter is pressed
    dynamic_reconfigure::Config conf, lpconf, gpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;

    lpconf.doubles.clear();
    double_param.name="max_vel_x";
    double_param.value=1.0;
    lpconf.doubles.push_back(double_param);
    double_param.name="acceleration_lim_x";
    double_param.value=normal_acceleration;
    lpconf.doubles.push_back(double_param);   
    srv.request.config = lpconf;
    localplanner_client.call(srv);

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=3000;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    conf.doubles.clear();
    double_param.name="max_range";
    double_param.value=1.0;
    conf.doubles.push_back(double_param);
    srv.request.config = conf;
    laser_fixer_client1.call(srv);    
    laser_fixer_client2.call(srv);    


    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint " <<current_wp_iter->wp_type << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 2; 
    force_ymg_publisher.publish(mode);

    /////////////////////////////////////////////////////////////////////////
    bool waiting_for_signal = false;
    bool goal_reached_flag = false;
    int car_cnt = 0;
    bool first = false;
    ros::Time signal_time = ros::Time::now();
    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);
        double diff_yaw_rad = current_wp_iter->yaw - robot_pose.yaw;
        while(diff_yaw_rad >= M_PI) diff_yaw_rad -=2*M_PI;
        while(diff_yaw_rad <= -M_PI) diff_yaw_rad +=2*M_PI;
        ros::Duration(0.1).sleep();
        // std::cout << "angle " << diff_yaw_rad << std::endl;
        if(diff_dis_m < 0.50 && -20.0*(M_PI/180) < diff_yaw_rad && diff_yaw_rad < 20.0*(M_PI/180)){
            goal_reached_flag = true;
        }
        /*
        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            std_msgs::Empty empty_msg;
            reset_flag_publisher.publish(empty_msg);
            action_client->sendGoal(current_goal);
        }
        */        
        if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED || goal_reached_flag == true){
            action_client->cancelAllGoals();
            if(waiting_for_signal==false){
                std::cout << "waiting for signal" << std::endl;
                std_msgs::String message;
                //message.data ="信号のボタンを押してください";
                message.data ="信号待機中です";
                talk_publisher.publish(message);
                waiting_for_signal =true;
                signal_flag = true;
                std_msgs::String ktm;
                ktm.data = "Start";
                signal_start_flag_publisher.publish(ktm);
            }
        }

        if (signal_ok) {
            if(!first){
                std_msgs::String message;
                message.data ="青信号です";
                talk_publisher.publish(message);
                first = true;
                signal_time = ros::Time::now();
            }
            if(car_flag.data) forward_wp_flag = true;

            //timeout
            //if ( (ros::Time::now() - signal_time) > ros::Duration(20.0)) {
            if ( (ros::Time::now() - signal_time) > ros::Duration(15.0)) {
                signal_time = ros::Time::now();
                signal_ok = false;
                first = false;
                waiting_for_signal = false;
                std_msgs::String message;
                message.data ="横断できませんでした";
                talk_publisher.publish(message);
            } 
        }
        //std::cout << action_client->getState().toString() << " " << std::endl;     
        ros::Duration(0.1).sleep();
    }
    /*
    if (forward_wp_flag) {
        ++current_wp_iter;
        forward_wp_flag = false;
        std_msgs::String message;
        message.data ="横断開始します";
        talk_publisher.publish(message);
    }
    */
    if (forward_wp_flag) {
        std_msgs::String ktm;
        ktm.data = "Stop";
        signal_start_flag_publisher.publish(ktm);

        signal_ok = false;
        signal_flag = false;
            
        ++current_wp_iter;
        forward_wp_flag = false;
        std_msgs::String message;
        message.data ="横断開始します";
        talk_publisher.publish(message);
    }
}

void WayPointManager::StateWait(){
    std::list<WayPoint>::iterator temp_iter = current_wp_iter;
    if(temp_iter != wp_array.begin()) --temp_iter;

    std_msgs::String message;
    //message.data ="リターンキーを押してください";
    message.data ="つぎは一時停止チェックポイントです";
    talk_publisher.publish(message);
    dynamic_reconfigure::Config gpconf, lpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Reconfigure srv;

    if(temp_iter->wp_type == "line"){
        lpconf.bools.clear();
        bool_param.name="line_mode";
        bool_param.value=true;
        lpconf.bools.push_back(bool_param);
        srv.request.config = lpconf;
        localplanner_client.call(srv);  
    }
    else if(temp_iter->wp_type == "back"){
        lpconf.bools.clear();
        bool_param.name="back_mode";
        bool_param.value=true;
        lpconf.bools.push_back(bool_param);
        srv.request.config = lpconf;
        localplanner_client.call(srv);  
    }

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=3000;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    //wait until enter is pressed
    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" << current_wp_iter->wp_type <<std::endl;

    std_msgs::Int32 mode; 
    mode.data = 2; 
    force_ymg_publisher.publish(mode);

    /////////////////////////////////////////////////////////////////////////
    bool ready = false;
    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);
        double diff_yaw_rad = current_wp_iter->yaw - robot_pose.yaw;
        while(diff_yaw_rad >= M_PI) diff_yaw_rad -=2*M_PI;
        while(diff_yaw_rad <= -M_PI) diff_yaw_rad +=2*M_PI;
        ros::Duration(0.1).sleep();
        if(diff_dis_m < 0.5 && -20.0*(M_PI/180) < diff_yaw_rad && diff_yaw_rad < 20.0*(M_PI/180)){
            action_client->cancelAllGoals();
            if(!ready){
                ready =true;
                std_msgs::String message;
                //message.data ="リターンキーを押してください";
                message.data ="エンターキーを押してください";
                talk_publisher.publish(message);
            }
        }

    }
    //ros::Duration(3).sleep();
    ros::Duration(1).sleep();
    if (forward_wp_flag) {
        ++current_wp_iter;
        forward_wp_flag = false;
    }

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=10.0;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    if(temp_iter->wp_type == "line"){
        lpconf.bools.clear();
        bool_param.name="line_mode";
        bool_param.value=false;
        lpconf.bools.push_back(bool_param);
        srv.request.config = lpconf;
        localplanner_client.call(srv);  
    }
    else if(temp_iter->wp_type == "back"){
        lpconf.bools.clear();
        bool_param.name="back_mode";
        bool_param.value=false;
        lpconf.bools.push_back(bool_param);
        srv.request.config = lpconf;
        localplanner_client.call(srv);  
    }

}

void WayPointManager::StateTurning(){
    std::list<WayPoint>::iterator temp_iter = current_wp_iter;
    if(temp_iter != wp_array.begin()) --temp_iter;

    dynamic_reconfigure::Config lpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::BoolParameter bool_param;


    if(temp_iter->wp_type == "line"){
        lpconf.bools.clear();
        bool_param.name="line_mode";
        bool_param.value=true;
        lpconf.bools.push_back(bool_param);
    }
    else if(temp_iter->wp_type == "back"){
        lpconf.bools.clear();
        bool_param.name="back_mode";
        bool_param.value=true;
        lpconf.bools.push_back(bool_param);
    }

    lpconf.doubles.clear();
    double_param.name="acceleration_lim_x";
    double_param.value=normal_acceleration;
    lpconf.doubles.push_back(double_param);
    double_param.name="max_vel_x";
    double_param.value=current_wp_iter->velocity;
    lpconf.doubles.push_back(double_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);  


    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    ros::Time amcl_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" <<current_wp_iter->wp_type << std::endl;
    std::cout << "target_size : " << get_target.size() << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 0;
    force_ymg_publisher.publish(mode);

    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);
        double diff_yaw_rad = current_wp_iter->yaw - robot_pose.yaw;
        while(diff_yaw_rad >= M_PI) diff_yaw_rad -=2*M_PI;
        while(diff_yaw_rad <= -M_PI) diff_yaw_rad +=2*M_PI;

        //check if reached goal
        if(diff_dis_m < 1.0 && -30.0*(M_PI/180) < diff_yaw_rad && diff_yaw_rad < 30.0*(M_PI/180)){
            std::cout << "Reached waypoint" << std::endl;
            forward_wp_flag = true;
            //AmclUpdate(15);
            //AmclUpdate(10);
        }
        
        //timeout
        if ( (ros::Time::now() - wp_update_time) > ros::Duration(skip_time_sec)) {
            std::cout << "Time out" << std::endl;
            forward_wp_flag = true;
        }

        //amcl_update
        /*
        if ( (ros::Time::now() - amcl_update_time) > ros::Duration(12.0)) {
            std::cout << "amcl_updatetime update" << std::endl;
            //AmclUpdate(5);
            AmclUpdate(3);
            amcl_update_time = ros::Time::now();
        }
        */

        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            if( diff_dis_m < 6.0) {
                forward_wp_flag =true;
            }else{
                std_msgs::Empty empty_msg;
                reset_flag_publisher.publish(empty_msg);
                action_client->sendGoal(current_goal);
            }         
        }
        ros::Duration(0.1).sleep();
    }
        
    if (forward_wp_flag) {
        if(MoveBackward()==true) --current_wp_iter;
        else ++current_wp_iter;
        forward_wp_flag = false;
    }

    if(temp_iter->wp_type == "line"){
        lpconf.bools.clear();
        bool_param.name="line_mode";
        bool_param.value=false;
        lpconf.bools.push_back(bool_param);
        srv.request.config = lpconf;
        localplanner_client.call(srv);  
    }
    else if(temp_iter->wp_type == "back"){
        lpconf.bools.clear();
        bool_param.name="back_mode";
        bool_param.value=false;
        lpconf.bools.push_back(bool_param);
        srv.request.config = lpconf;
        localplanner_client.call(srv);  
    }
}

void WayPointManager::StateLine(){
    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint " << current_wp_iter->wp_type << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 2; 
    force_ymg_publisher.publish(mode);

    dynamic_reconfigure::Config gpconf, lpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Reconfigure srv;

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=3000;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    lpconf.doubles.clear(); 
    double_param.name="max_vel_x";
    double_param.value=1.0;
    lpconf.doubles.push_back(double_param);
    /*
    lpconf.ints.clear();
    int_param.name="vth_samples";
    int_param.value=9;
    lpconf.ints.push_back(int_param);
    */
    lpconf.bools.clear();
    bool_param.name="line_mode";
    bool_param.value=true;
    lpconf.bools.push_back(bool_param);
    
    srv.request.config = lpconf;
    localplanner_client.call(srv);

    AmclUpdate(60);

    /////////////////////////////////////////////////////////////////////////
    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);

        //check if reached goal
        if (diff_dis_m < goal_tolerance_m/2 ) {
            std::cout << "Reached waypoint" << std::endl;
            forward_wp_flag = true;
        }
        
        /*
        //timeout
        if ( (ros::Time::now() - wp_update_time) > ros::Duration(skip_time_sec)) {
            std_srvs::Empty srv;
            if (amcl_client.call(srv)){
                //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
            }else{
                ROS_ERROR("Failed to call service add_two_ints");
            }
        }

        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            if( diff_dis_m < 6) {
                forward_wp_flag =true;
            }else{
                action_client->sendGoal(current_goal);
            }
        }
        */
/*
        if(line_flag.data == true){
            action_client->cancelAllGoals();
            stop_return = true;
            //std::cout << "STOP!!!" << std::endl;
        }
        else{
            if(stop_return){
                //std::cout << "RESTART!!!" << std::endl;
                action_client->sendGoal(current_goal);
                stop_return = false;
            }
        }
*/
        /*
        if(cmd_vel_info.linear.x < 0.1){
            //action_client->cancelAllGoals();
            conf.doubles.clear(); 
            double_param.name="dist_param";
            double_param.value=0.2;
            conf.doubles.push_back(double_param);
            double_param.name="curvature_param";
            double_param.value=0.2;
            conf.doubles.push_back(double_param);
            double_param.name="vel_param";
            double_param.value=0.00;
            conf.doubles.push_back(double_param);
            srv.request.config = conf;
            localplanner_client.call(srv);
        }
        else{
            conf.doubles.clear(); 
            double_param.name="dist_param";
            double_param.value=0.05;
            conf.doubles.push_back(double_param);
            double_param.name="curvature_param";
            double_param.value=0.008;
            conf.doubles.push_back(double_param);
            double_param.name="vel_param";
            double_param.value=0.01;
            conf.doubles.push_back(double_param);
            srv.request.config = conf;
            localplanner_client.call(srv);
        }
        */

        ros::Duration(0.1).sleep();
    }
    if (forward_wp_flag) {
        ++current_wp_iter;
        forward_wp_flag = false;
    }

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=10.0;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    /*
    lpconf.ints.clear();
    int_param.name="vth_samples";
    int_param.value=11;
    lpconf.ints.push_back(int_param);
    */
    lpconf.bools.clear();
    bool_param.name="line_mode";
    bool_param.value=false;
    lpconf.bools.push_back(bool_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);

    AmclUpdate(20);
}

void WayPointManager::StateLineBefore(){
    dynamic_reconfigure::Config lpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;

    lpconf.doubles.clear();
    double_param.name="acceleration_lim_x";
    double_param.value=normal_acceleration;
    lpconf.doubles.push_back(double_param);
    double_param.name="max_vel_x";
    double_param.value=current_wp_iter->velocity;
    lpconf.doubles.push_back(double_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);    


    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    ros::Time amcl_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" <<current_wp_iter->wp_type << std::endl;
    std::cout << "target_size : " << get_target.size() << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 0;
    force_ymg_publisher.publish(mode);

    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);

        //check if reached goal
        if (diff_dis_m < 1.5 ) {
            std::cout << "Reached waypoint" << std::endl;
            forward_wp_flag = true;
            //AmclUpdate(15);
            //AmclUpdate(10);
        }
        
        //timeout
        if ( (ros::Time::now() - wp_update_time) > ros::Duration(skip_time_sec*2)) {
            std::cout << "Time out" << std::endl;
            forward_wp_flag = true;
        }

        //amcl_update
        /*
        if ( (ros::Time::now() - amcl_update_time) > ros::Duration(12.0)) {
            std::cout << "amcl_updatetime update" << std::endl;
            //AmclUpdate(5);
            AmclUpdate(3);
            amcl_update_time = ros::Time::now();
        }
        */

        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            if( diff_dis_m < 6.0) {
                forward_wp_flag =true;
            }else{
                std_msgs::Empty empty_msg;
                reset_flag_publisher.publish(empty_msg);
                action_client->sendGoal(current_goal);
            }         
        }
        ros::Duration(0.1).sleep();
    }
        
    if (forward_wp_flag) {
        if(MoveBackward()==true) --current_wp_iter;
        else ++current_wp_iter;
        forward_wp_flag = false;
    }
}


void WayPointManager::StateFindTarget(){     
    const double yaw_resolution_rad = 45.0 / 180 * M_PI;
    const double radius_resolution_m = 0.1;
    const int max_yaw_trial = 2 * M_PI / yaw_resolution_rad;
    const int max_radius_trial = (0.5 / radius_resolution_m); 
    int yaw_trial = 0, radius_trial = 0;  
    double radius_m;
    double yaw_rad;

/*
    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;
    double_param.name="obstacle_stop_margin";
    double_param.value=0.25;
    conf.doubles.push_back(double_param);

    srv.request.config = conf;
    localplanner_client.call(srv);
*/

    /////////////////////initial setup of waypoint//////////////////////////
    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);

    ros::Time wp_update_time = ros::Time::now();
    ros::Time amcl_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" << current_wp_iter->wp_type <<std::endl;

    std_msgs::Int32 mode; 
    mode.data = 2; 
    force_ymg_publisher.publish(mode);

    std_msgs::String message;
    message.data ="アプローチを開始します";
    talk_publisher.publish(message);
    // ros::Duration(3).sleep();

    int n = 1; 
    ////////////////////////////////////////////////////////////////////////
    while(!forward_wp_flag){
        //if goal is unreachable, create goal near target
        //goal will be created in circle 

        WayPoint robot_pose = GetRobotPose(); 
        double diff_dis_m = std::hypot(current_goal.target_pose.pose.position.x - robot_pose.x, current_goal.target_pose.pose.position.y - robot_pose.y);
        double diff_yaw_rad = current_wp_iter->yaw - robot_pose.yaw;
        while(diff_yaw_rad >= M_PI) diff_yaw_rad -=2*M_PI;
        while(diff_yaw_rad <= -M_PI) diff_yaw_rad +=2*M_PI;

        if((ros::Time::now() - wp_update_time) > ros::Duration(40*n)){
            move_base_msgs::MoveBaseGoal current_goal;
            current_goal.target_pose.header.stamp = ros::Time::now();
            current_goal.target_pose.header.frame_id = "map";
            current_goal.target_pose.pose.position.x = current_wp_iter->x - (0.25 * n) * cos(current_wp_iter->yaw);
            current_goal.target_pose.pose.position.y = current_wp_iter->y - (0.25 * n) * sin(current_wp_iter->yaw);
            current_goal.target_pose.pose.position.z = 0.0;
            current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
            action_client->sendGoal(current_goal); 
            n++;
        }

        //time out
        if((ros::Time::now() - wp_update_time) > ros::Duration(50)){
            forward_wp_flag = true;
            message.data ="探索対象に近づけませんでした";
            talk_publisher.publish(message);
            ros::Duration(4).sleep();
        }    
        //check if reached goal
        //if( diff_dis_m < 0.3 || action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ){
        if(diff_dis_m < 0.5 && -20.0*(M_PI/180) < diff_yaw_rad && diff_yaw_rad < 20.0*(M_PI/180)){
            action_client->cancelAllGoals();
            forward_wp_flag = true;
            ros::Duration(3).sleep();
            message.data ="探索対象に接近しました";
            talk_publisher.publish(message);
            ros::Duration(6).sleep();
            //AmclUpdate(15);
        }

        //amcl_update
        /*
        if ( (ros::Time::now() - amcl_update_time) > ros::Duration(25.0)) {
            std::cout << "amcl_updatetime update" << std::endl;
            AmclUpdate(5);
            amcl_update_time = ros::Time::now();
        }
        */

        ros::Duration(0.1).sleep();
    }
    if(forward_wp_flag){ 
        target_already_found = false;
        current_wp_iter = wp_array.erase(current_wp_iter);
        if(MoveBackward()==0) current_wp_iter--;
        else current_wp_iter++;
        forward_wp_flag = false;
    }
    /*
    conf.doubles.clear();
    double_param.name="obstacle_stop_margin";
    double_param.value=0.3;
    conf.doubles.push_back(double_param);
    srv.request.config = conf;
    localplanner_client.call(srv);
    */
}

void WayPointManager::StateNormal(){
    /////////////////////initial setup of waypoint//////////////////////////
    dynamic_reconfigure::Config lpconf, gpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;

    lpconf.doubles.clear();
    double_param.name="acceleration_lim_x";
    double_param.value=normal_acceleration;
    lpconf.doubles.push_back(double_param);
    double_param.name="max_vel_x";
    double_param.value=current_wp_iter->velocity;
    lpconf.doubles.push_back(double_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);    

    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=5.0;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);


    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    ros::Time amcl_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" <<current_wp_iter->wp_type << std::endl;
    std::cout << "target_size : " << get_target.size() << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 0;
    force_ymg_publisher.publish(mode);

    int n = 1;
    //if(current_wp_iter->velocity < 0.8) n = 4; 
    if(current_wp_iter->velocity < 0.8) n = 2; 

    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);

        //check if reached goal
        if (diff_dis_m < goal_tolerance_m/n ) {
            std::cout << "Reached waypoint" << std::endl;
            forward_wp_flag = true;
            //AmclUpdate(15);
            //AmclUpdate(10);
        }
        
        //timeout
        if ( (ros::Time::now() - wp_update_time) > ros::Duration(skip_time_sec)) {
            std::cout << "Time out" << std::endl;
            forward_wp_flag = true;
        }

        //amcl_update
        /*
        if ( (ros::Time::now() - amcl_update_time) > ros::Duration(12.0)) {
            std::cout << "amcl_updatetime update" << std::endl;
            //AmclUpdate(5);
            AmclUpdate(3);
            amcl_update_time = ros::Time::now();
        }
        */

        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            if( diff_dis_m < 6.0) {
                forward_wp_flag =true;
            }else{
                std_msgs::Empty empty_msg;
                reset_flag_publisher.publish(empty_msg);
                action_client->sendGoal(current_goal);
            }         
        }
        ros::Duration(0.1).sleep();
    }
        
    if (forward_wp_flag) {
        if(MoveBackward()==true) --current_wp_iter;
        else ++current_wp_iter;
        forward_wp_flag = false;
    }
}

void WayPointManager::StateBack(){
    /////////////////////initial setup of waypoint//////////////////////////
    dynamic_reconfigure::Config lpconf;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Reconfigure srv;

    lpconf.bools.clear();
    bool_param.name="back_mode";
    bool_param.value=true;
    lpconf.bools.push_back(bool_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);  


    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    ros::Time amcl_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" <<current_wp_iter->wp_type << std::endl;
    std::cout << "target_size : " << get_target.size() << std::endl;

    std_msgs::Int32 mode; 
    mode.data = 0;
    force_ymg_publisher.publish(mode);

    int n = 1;
    //if(current_wp_iter->velocity < 0.8) n = 4; 
    if(current_wp_iter->velocity < 0.8) n = 2; 

    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);

        //check if reached goal
        if (diff_dis_m < goal_tolerance_m/n ) {
            std::cout << "Reached waypoint" << std::endl;
            forward_wp_flag = true;
            //AmclUpdate(15);
            //AmclUpdate(10);
        }
        
        //timeout
        if ( (ros::Time::now() - wp_update_time) > ros::Duration(skip_time_sec)) {
            std::cout << "Time out" << std::endl;
            forward_wp_flag = true;
        }

        //amcl_update
        /*
        if ( (ros::Time::now() - amcl_update_time) > ros::Duration(12.0)) {
            std::cout << "amcl_updatetime update" << std::endl;
            //AmclUpdate(5);
            AmclUpdate(3);
            amcl_update_time = ros::Time::now();
        }
        */

        if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED){
            if( diff_dis_m < 6.0) {
                forward_wp_flag =true;
            }else{
                std_msgs::Empty empty_msg;
                reset_flag_publisher.publish(empty_msg);
                action_client->sendGoal(current_goal);
            }         
        }
        ros::Duration(0.1).sleep();
    }
        
    if (forward_wp_flag) {
        if(MoveBackward()==true) --current_wp_iter;
        else ++current_wp_iter;
        forward_wp_flag = false;
    }

    lpconf.bools.clear();
    bool_param.name="back_mode";
    bool_param.value=false;
    lpconf.bools.push_back(bool_param);
    srv.request.config = lpconf;
    localplanner_client.call(srv);  
}

void WayPointManager::StateEnd(){
    dynamic_reconfigure::Config gpconf;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Reconfigure srv;
    gpconf.doubles.clear();
    double_param.name="stuck_timeout";
    double_param.value=3000;
    gpconf.doubles.push_back(double_param);
    srv.request.config = gpconf;
    globalplanner_client.call(srv);

    //wait until enter is pressed
    move_base_msgs::MoveBaseGoal current_goal;
    current_goal.target_pose.header.stamp = ros::Time::now();
    current_goal.target_pose.header.frame_id = "map";
    current_goal.target_pose.pose.position.x = current_wp_iter->x;
    current_goal.target_pose.pose.position.y = current_wp_iter->y;
    current_goal.target_pose.pose.position.z = 0.0;
    current_goal.target_pose.pose.orientation =tf::createQuaternionMsgFromYaw(current_wp_iter->yaw);
    action_client->sendGoal(current_goal);
      
    ros::Time wp_update_time = ros::Time::now();
    std::cout << "publishing " << current_wp_iter->id << "th waypoint" << current_wp_iter->wp_type <<std::endl;

    std_msgs::Int32 mode; 
    mode.data = 2; 
    force_ymg_publisher.publish(mode);

    /////////////////////////////////////////////////////////////////////////
    bool ready = false;
    while( !forward_wp_flag && ros::ok() ){
        WayPoint robot_pose = GetRobotPose(); 
        const double diff_dis_m = std::hypot(current_wp_iter->x - robot_pose.x, current_wp_iter->y - robot_pose.y);
        double diff_yaw_rad = current_wp_iter->yaw - robot_pose.yaw;
        while(diff_yaw_rad >= M_PI) diff_yaw_rad -=2*M_PI;
        while(diff_yaw_rad <= -M_PI) diff_yaw_rad +=2*M_PI;
        ros::Duration(0.1).sleep();
        if(diff_dis_m < 0.5 && -20.0*(M_PI/180) < diff_yaw_rad && diff_yaw_rad < 20.0*(M_PI/180)){
            action_client->cancelAllGoals();
            if(!ready){
                ready =true;
                std_msgs::String message;
                //message.data ="リターンキーを押してください";
                message.data ="ゴールに到達しました";
                talk_publisher.publish(message);
            }
        }

    }
    //ros::Duration(3).sleep();
    ros::Duration(1).sleep();
    if (forward_wp_flag) {
        ++current_wp_iter;
        forward_wp_flag = false;
    }
}

void WayPointManager::AmclUpdate(int update_time){
    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Reconfigure srv;
    conf.ints.clear();
    int_param.name="update_time";
    int_param.value=update_time;
    conf.ints.push_back(int_param);  
    srv.request.config = conf;
    amcl_update_client.call(srv);
}