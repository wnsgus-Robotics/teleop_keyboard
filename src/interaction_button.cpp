#include "interaction_button.h"

InteractionButton::InteractionButton()
{
  cancel_publisher = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
  elevator_door_publisher = nh.advertise<semantic_navigation_msgs::DoorState>("door_state", 1);
  opened_elevator_publisher = nh.advertise<std_msgs::String>("elevator_choose", 1);
  task_publisher = nh.advertise<semantic_navigation_msgs::Task>("/san_interface_node/set_task", 1 );
}

InteractionButton::~InteractionButton()
{

}
bool InteractionButton::sendTask(char key)
{
  if(key == '5')
  {
    semantic_navigation_msgs::Task task;

    task.task = semantic_navigation_msgs::Task::DELIVER_COFFEE;
    task.robot.name = "c4r01";
    task.goal.name =  "corridor718";

    task_publisher.publish(task);
    return true;
  }
  else
    return false;
}

bool InteractionButton::cancelMovebase(char key)
{
  if(key == '4')
  {
    actionlib_msgs::GoalID cancel;
    cancel_publisher.publish(cancel);
    return true;
  }
  else
    return false;
}

bool InteractionButton::doorSatae(char key)
{
  if(key == '3')
  {
    semantic_navigation_msgs::DoorState msg;
    msg.state = 1;
    elevator_door_publisher.publish(msg);
    return true;
  }
  else
    return false;
}
    
bool InteractionButton::elevatorChoose(char key)
{
  if(key == '1')
  {
    std_msgs::String msg;
    msg.data = "elevator1";
    opened_elevator_publisher.publish(msg);
  }
  else if(key == '2')
  {
    std_msgs::String msg;
    msg.data = "elevator2";
    opened_elevator_publisher.publish(msg);
  }
  else 
    return false;

}

void InteractionButton::in(char key)
{
    cancelMovebase(key);
    elevatorChoose(key);
    doorSatae(key);
}