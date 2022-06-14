#include "teleop_keyboard.h"

TeleopKeyboard::TeleopKeyboard():pnh("~")
{
  init();
}

TeleopKeyboard::~TeleopKeyboard()
{
  delete profile;
  delete mux;
  delete ib;
}

void TeleopKeyboard::init()
{
  getParam();
  pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
  timer1 = nh.createTimer(ros::Duration(0.1), &TeleopKeyboard::t1Callback,this);
  timer2 = nh.createTimer(ros::Duration(1/control_hz), &TeleopKeyboard::t2Callback,this);
  profile = new VelocityProfile(vacc,wacc);
  if(use_mux)
    mux = new TwistMux(topic_name,sub_topic_name);
  ib = new InteractionButton();
}

void TeleopKeyboard::getParam()
{ 
  key = ' ';
  x = 0;
  y = 0;
  z = 0;
  th = 0;
  pnh.param("topic_name",this->topic_name,std::string("joy_vel"));
  pnh.param("sub_topic_name",this->sub_topic_name,std::string("nav_vel"));
  pnh.param("mux",this->use_mux,false);
  pnh.param("speed",this->speed,0.1);
  pnh.param("turn",this->turn,0.25);
  pnh.param("vacc",this->vacc,0.5);
  pnh.param("wacc",this->wacc,0.5);
  pnh.param("control_hz",this->control_hz,10.0);
}
int TeleopKeyboard::getch()
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void TeleopKeyboard::t1Callback(const ros::TimerEvent&)
{
  printf("%s", msg);
  printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);
  // Get the pressed key
  key = getch();
  ib->in(key);
  // If the key corresponds to a key in moveBindings
  if(use_mux)
  {
    if(key == 'a' || key == 'A')
      mux->select(sub_topic_name);
    else 
      mux->select(topic_name);
  }

  if (moveBindings.count(key) == 1)
  {
    // Grab the direction data
    x = moveBindings[key][0];
    y = moveBindings[key][1];
    z = moveBindings[key][2];
    th = moveBindings[key][3];

    printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
  }

  // Otherwise if it corresponds to a key in speedBindings
  else if (speedBindings.count(key) == 1)
  {
    // Grab the speed data
    speed = speed * speedBindings[key][0];
    turn = turn * speedBindings[key][1];

    printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
  }

  // Otherwise, set the robot to stop
  else
  {
    x = 0;
    y = 0;
    z = 0;
    th = 0;

    // If ctrl-C (^C) was pressed, terminate the program
    if (key == '\x03')
    {
      printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
      ros::shutdown();
    }

    printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
  }

  // Update the Twist message
  twist.linear.x = x * speed;
  twist.linear.y = y * speed;
  twist.linear.z = z * speed;

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = th * turn;
}
  
void TeleopKeyboard::t2Callback(const ros::TimerEvent&)
{
  // Publish it and resolve any remaining callbacks
  pub.publish(profile->calc(twist));
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop");
  ROS_INFO("[%s] start node",ros::this_node::getName().c_str());
  TeleopKeyboard x;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
