
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      // ROS_INFO("Waiting for trigger from RC");

      // while (ros::ok() && (!have_odom_ || !have_trigger_))
      // {
      //   ros::spinOnce();
      //   ros::Duration(0.001).sleep();
      // }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);

      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wpt_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // local target close to the global target
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
        goto force_return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        if (planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
        {
          have_target_ = false;
          have_trigger_ = false;
          ROS_ERROR("The goal is in obstacles, finish the planning.");
          callEmergencyStop(odom_pos_);

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else
        {
          // pass;
        }
      }
      else if (t_cur > replan_thresh_ ||
               (!touch_the_goal && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    // static int last_printed_state = -1, dot_nums = 0;

    // if (exec_state_ != last_printed_state)
    //   dot_nums = 0;
    // else
    //   dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    // last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << endl;

    // cout << string(dot_nums, '.');

    // fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    double height;
    measureGroundHeight(height);

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    // bool close_to_the_end_of_safe_segment = (pts_chk.back().back().first - t_cur) < emergency_time_;
    // // bool close_to_goal = (info->traj.getPos(info->duration) - end_pt_).norm() < 1e-5;
    // if (close_to_the_end_of_safe_segment)
    // {
    //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   return;

    //   // if (!close_to_goal)
    //   // {
    //   //   // ROS_INFO("current position is close to the safe segment end.");
    //   //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //   return;
    //   // }
    //   // else
    //   // {
    //   //   double t_step = map->getResolution() / planner_manager_->pp_.max_vel_;
    //   //   for (double t = pts_chk.back().back().first; t < info->duration; t += t_step)
    //   //   {
    //   //     if (map->getInflateOccupancy(info->traj.getPos(t)))
    //   //     {
    //   //       if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_)
    //   //       {
    //   //         ROS_ERROR("Dense obstacles close to the goal, stop planning.");
    //   //         callEmergencyStop(odom_pos_);
    //   //         have_target_ = false;
    //   //         changeFSMExecState(WAIT_TARGET, "SAFETY");
    //   //         return;
    //   //       }
    //   //       else
    //   //       {
    //   //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //         return;
    //   //       }
    //   //     }
    //   //   }
    //   // }
    // }

    /* ---------- check trajectory ---------- */
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    auto id_ratio = info->traj.locatePieceIdxWithRatio(t_cur);

    // cout << "t_cur=" << t_cur << " info->duration=" << info->duration << endl;

    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());
    if (i_start >= pts_chk.size())
    {
      // ROS_ERROR("i_start >= pts_chk.size()");
      return;
    }
    size_t j_start = 0;
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    // Eigen::Vector3d last_pt = pts_chk[0][0].second;
    // for (size_t i = 0; i < pts_chk.size(); ++i)
    // {
    //   cout << "--------------------" << endl;
    //   for (size_t j = 0; j < pts_chk[i].size(); ++j)
    //   {
    //     cout << pts_chk[i][j].first << " @ " << pts_chk[i][j].second.transpose() << " @ " << (pts_chk[i][j].second - last_pt).transpose() << " @ " << map->getInflateOccupancy(pts_chk[i][j].second) << endl;
    //     last_pt = pts_chk[i][j].second;
    //   }
    // }

    // cout << "pts_chk[i_start][j_start].first - t_cur = " << pts_chk[i_start][j_start].first - t_cur << endl;
    // cout << "devi = " << (pts_chk[i_start][j_start].second - info->traj.getPos(t_cur)).transpose() << endl;

    // cout << "pts_chk.size()=" << pts_chk.size() << " i_start=" << i_start << endl;
    // Eigen::Vector3d p_last = pts_chk[i_start][j_start].second;
    const bool touch_the_end = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;
        // if ( (p - p_last).cwiseAbs().maxCoeff() > planner_manager_->grid_map_->getResolution() * 1.05 )
        // {
        //   ROS_ERROR("BBBBBBBBBBBBBBBBBBBBBBBBBBB");
        //   cout << "p=" << p.transpose() << " p_last=" << p_last.transpose() << " dist=" << (p - p_last).cwiseAbs().maxCoeff() << endl;
        // }
        // p_last = p;

        // cout << "t=" << t << " @ "
        //      << "p=" << p.transpose() << endl;
        // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        // if (t_cur < t_2_3 && t >= t_2_3)
        //   break;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        // cout << "p=" << p.transpose() << endl;

        // if (occ)
        // {
        //   ROS_WARN("AAAAAAAAAAAAAAAAAAA");
        //   cout << "pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
        //   cout << "i=" << i << " j=" << j << " i_start=" << i_start << " j_start=" << j_start << endl;
        //   cout << "pts_chk.size()=" << pts_chk.size() << endl;
        //   cout << "t=" << t << endl;
        //   cout << "from t=" << info->traj.getPos(t).transpose() << endl;
        //   cout << "from rec=" << p.transpose() << endl;
        // }

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t - (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();

            if (dist < CLEARANCE)
            {
              ROS_WARN("Dis. of dro. %d & %d is %.2f after %.3fs.",
                       planner_manager_->pp_.drone_id, (int)id, dist, (t - t_cur));
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {

      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1)
      return;

    ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp))
    {
      have_trigger_ = true;
    }
  }

  // void EGOReplanFSM::planGlobalTrajbyGivenWps()
  // {
  //   std::vector<Eigen::Vector3d> wps(waypoint_num_);
  //   for (int i = 0; i < waypoint_num_; i++)
  //   {
  //     wps[i](0) = waypoints_[i][0];
  //     wps[i](1) = waypoints_[i][1];
  //     wps[i](2) = waypoints_[i][2];

  //     end_pt_ = wps.back();
  //   }
  //   bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero(), wps,
  //                                                            Eigen::Vector3d::Zero(),
  //                                                            Eigen::Vector3d::Zero());

  //   for (size_t i = 0; i < (size_t)waypoint_num_; i++)
  //   {
  //     visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
  //     ros::Duration(0.001).sleep();
  //   }

  //   if (success)
  //   {

  //     /*** display ***/
  //     constexpr double step_size_t = 0.1;
  //     int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
  //     std::vector<Eigen::Vector3d> gloabl_traj(i_end);
  //     for (int i = 0; i < i_end; i++)
  //     {
  //       gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
  //     }

  //     have_target_ = true;
  //     have_new_target_ = true;

  //     // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
  //     ros::Duration(0.001).sleep();
  //     visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  //     ros::Duration(0.001).sleep();
  //   }
  //   else
  //   {
  //     ROS_ERROR("Unable to generate global trajectory!");
  //   }
  // }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }

          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
} // namespace ego_planner
