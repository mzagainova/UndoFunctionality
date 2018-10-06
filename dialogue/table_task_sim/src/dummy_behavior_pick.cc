#include <geometry_msgs/Pose.h>
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <table_task_sim/dummy_behavior_pick.h>

#include <table_task_sim/header.h>
  

bool hold;
bool pick;
std::string hold_object_name;

//int global_x;
namespace task_net {
////////////////////////////////////////////////////////////////////////////////
// DUMMY PLACE BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DummyBehaviorPick::DummyBehaviorPick() {}
DummyBehaviorPick::DummyBehaviorPick(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    ROBOT robot_des,
    bool init_v,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state ,
      object), mut_arm(name.topic.c_str(), "/right_arm_mutex") {

    object_ = object;
    state_.done = false;
    parent_done_ = false;	
    ROS_INFO( "DummyBehaviorPick: [%s] Object: [%s]", name_->topic.c_str(), object_.c_str() );
    robot_des_ = robot_des;
   
    init_v_ = init_v;
    

    // subscribe to simulator state messages
    state_sub_ = local_.subscribe("/state", 1000, &DummyBehaviorPick::StateCallback, this );
}
DummyBehaviorPick::~DummyBehaviorPick() {}
/*void inithold() {

hold = 0;
pick = 0;
hold_object_name="N/A";

 }*/
void DummyBehaviorPick::UpdateActivationPotential() {

  if( state_.done || parent_done_ || state_.peer_active || state_.peer_done )
  {
    ROS_DEBUG_THROTTLE_NAMED( 1, "DummyBehaviorTrace", "[%s]: State/Parent is done, so don't update activationpotential [%d|%d|%d|%d]", object_.c_str(), state_.done, parent_done_, state_.peer_active, state_.peer_done );
    state_.activation_potential = 0;
  
    //hold_s_.hold= false;
   
    return;
  }

  if( !IsActive() )
  {
    ROS_INFO_THROTTLE_NAMED( 1, "DummyBehaviorTrace", "[%s]: Not active, so don't update activationpotential~~~~~~~~~~~~", object_.c_str() );
    state_.activation_potential = 0;
      
   
     //   hold_status_dummy1_.pick= false;
    //old_status_dummy1_.hold= false;
    return;
  }

  ROS_DEBUG_NAMED("DummyBehaviorTrace", "DummyBehavior::UpdateActivationPotential was called: [%s]", object_.c_str() );

  geometry_msgs::Point rpos, opos;

  if( table_state_.robots.size() == 0 || table_state_.objects.size() == 0 )
  {
    ROS_WARN("state has not been populated, yet");
    return;
  }


  // get location of robot
  rpos = table_state_.robots[robot_des_].pose.position;

  // get location of object

    // find object
  int obj_idx = -1;
  for( int i = 0; i < table_state_.objects.size(); i++ )
  {
    //OS_INFO ("%s =?= %s",table_state_.objects[i].name.c_str(), object_.c_str() );
    if( table_state_.objects[i].name.compare(object_) == 0 )
    {
      obj_idx = i;
      break;
    }
  }

  if( obj_idx < 0 )
  {
    ROS_WARN( "could not find object: [%s]", object_.c_str() );
  }
  opos = table_state_.objects[obj_idx].pose.position;


  //double c1 = 1.0; // weight for distance
  //double c2 = 1.0; //weight for suitability
  
  
  //double dist = hypot(rpos.y - opos.y, rpos.x - opos.x);
  if( init_v_ == 0) //at first for the first pick node hold and pick value will be assigned 0
  { hold = 0; 
  	//hold_object_name="N/A";
  	pick=0;
  }
  if(pick==0){
  //if( hold == 0 && hold_object_name.compare("N/A")==0 && fabs(dist)> 0.00001)
  if( hold == 0)
      //state_.activation_potential = ( c1 * (1.0f / dist)) + (c2 * state_.suitability);
      {  
      
      state_.activation_potential = 1;
      //state_.activation_potential = ( c1 * (1.0f / dist)) + (c2 * state_.suitability);
     // hold_object_name="N/aa";
      pick=1;
    
      }
  else state_.activation_potential = 0;
  }
    
  

  	

  ROS_DEBUG_NAMED("DummyBehavior", "%s: activation_potential: [%f]", object_.c_str(), state_.activation_potential );
  //ROS_ERROR("%s: activation_potential_Pick: [%f] DISTANCE %f suit %f", object_.c_str(), state_.activation_potential,fabs(dist),state_.suitability );
}
  

bool DummyBehaviorPick::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  return true;
}

uint32_t DummyBehaviorPick::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  // ControlMessagePtr_t msg(new ControlMessage_t);
  // msg->sender = mask_;
  // msg->activation_level = 1.0f;
  // msg->done = false;
}

void DummyBehaviorPick::Work() {
  ROS_INFO("DummyBehavior::Work: waiting for pause to be done!");
  // boost::this_thread::sleep(boost::posix_time::millisec(10000));
  Pick(object_, robot_des_);
    // while (!PickAndPlaceDone()) {
      //boost::this_thread::sleep(boost::posix_time::millisec(500));
        // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
    // }
  
  mut_arm.Release();
  ROS_INFO("[%s]: DummyBehavior::Work: Done!", name_->topic.c_str());
  //ROS_INFO("\t[%s]: DummyBehavior::MUTEX IS RELEASED!", name_->topic.c_str());
}

void DummyBehaviorPick::Pick(std::string object, ROBOT robot_des) {

  // pick
  table_task_sim::PickUpObject req_pick;
  req_pick.request.robot_id = (int)robot_des;
  req_pick.request.object_name = object;
  // table_task_sim::PickUpObject::Response res_pick; //to know if it failed or not...

  // place
  /*geometry_msgs::Pose pose;
  pose.position.x = -0.45;
  pose.position.y = 0.0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  table_task_sim::PlaceObject req_place;
  req_place.request.robot_id = (int)robot_des;
  req_place.request.goal = pose;*/
  // table_task_sim::PlaceObject::Response res_place; //to know if it failed or not...

  // call the pick service
  if(ros::service::call("pick_service", req_pick)) {

    ROS_INFO("\t\t[%s]: THE PICK SERVICE WAS CALLED!!", name_->topic.c_str());

    // call the place service
    /*if(ros::service::call("place_service", req_place)) {
      ROS_INFO("\t\t[%s]: THE PLACE SERVICE WAS CALLED!!", name_->topic.c_str());
    }*/
  }

  state_.done = true;
  ROS_INFO( "[%s]: PickAndPlace: Pick is done", name_->topic.c_str() );
  hold = 1;
  hold_object_name = object_;

}

bool DummyBehaviorPick::PickDone() {
  // table_setting_demo::pick_and_place msg;
  // msg.request.object = object_;
  // ros::service::call("pick_and_place_check", msg);
  // return msg.response.success;
}

bool DummyBehaviorPick::ActivationPrecondition() {
  ROS_DEBUG_NAMED("DummyBehaviorTrace", "\t[%s]: DummyBehavior::MUTEX IS LOCKING!", name_->topic.c_str());

  // orig
  // return mut_arm.Lock(state_.activation_potential);


  bool lock_okay;

 // first attempt which did not work.....
  // check peer states...?
  if(!state_.peer_active && !state_.peer_done) {
    // return mut_arm.Lock(state_.activation_potential);
    ROS_INFO("\t[%s]: Trying to gain access to mutex!", name_->topic.c_str());
    lock_okay = mut_arm.Lock(state_.activation_potential);
    if(lock_okay){
  // hold = 1;
  // hold_object_name = object_;
    }
    // return true;
  }
  else{
    ROS_INFO("\t[%s]: Peer gained access first, don't activate!", name_->topic.c_str());
    return false;
  }

// second attempt
  // check peer states...?
  if(state_.peer_active || state_.peer_done) {
    ROS_INFO("\t[%s]: Peer gained access first so release mutex and don't activate!", name_->topic.c_str());
    mut_arm.Release();
    
    return false;
  }

  return lock_okay;
}

  void DummyBehaviorPick::StateCallback( table_task_sim::SimState msg)
  {
    table_state_ = msg;
  }

}  // namespace task_net
