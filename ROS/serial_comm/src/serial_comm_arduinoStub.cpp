#include "serial_comm/serial_comm_arduinoStub.h"

#include "std_msgs/Float64.h"


void serial_comm_arduinoStub::Prepare(void)
{
	RunPeriod = RUN_PERIOD;

	// Subscriber = Handle.subscribe("name_of_the_topic", length_of_the_queue, &ROSnode::MessageCallback, this);
	/***********************************************************
	 * Uncomment if your node subscribes to the topic called
	 * name_of_the_topic. Substitute length_of_the_queue with an
	 * integer defining how many messages the input queue should
	 * be able to contain.
	 * NOTE: if your node is run periodically, make sure that the
	 * queue is long enough to avoid that useful, but older, 
	 * messages are discarded by ROS because the queue is full.
	 * 
	 * One such instruction is required for each topic that the node
	 * subscribes to.
	 */

	_Publisher = Handle.advertise<std_msgs::Float64>("/arduino", 1);
	/***********************************************************
	 * Uncomment if your node publishes to the topic called
	 * name_of_the_topic. Substitute length_of_the_queue with an
	 * integer defining how many messages the output queue should
	 * be able to contain.
	 * msg_type must be substituted with the name of the type of 
	 * message that your node will publish on the topic; msg_pkg 
	 * must be substituted with the name of the ROS package which
	 * contains the .msg file defining such message type.
	 * A package which defines many common ROS message types is
	 * std_msgs: a list of the message types it provides is here:
	 * http://www.ros.org/doc/api/std_msgs/html/index-msg.html
	 *
	 * One such instruction is required for each topic that the node
	 * publishes to.
	 */

	// Client = Handle.serviceClient<name_of_server_package::name_of_the_srv_file>("name_of_the_service");
	/***********************************************************
	 * Uncomment if your node needs to act as a client of a service
	 * called name_of_the_service, provided by a ROS server defined
	 * by file name_of_the_srv_file.srv which is part of the ROS
	 * package called name_of_the_server_package.
	 *
	 * Add one similar statement for each additional ROS server
	 * that your node needs to access as a client.
	 */

	// Service = Handle.advertiseService("name_of_the_service", &ROSnode::ProvideService, this);
	//ROS_INFO("ROS service %s available (provided by node %s).", "name_of_the_service", ros::this_node::getName().c_str());
	/***********************************************************
	 * Uncomment the first statement if your node acts as a ROS
	 * server, providing a service called name_of_the_service
	 * to clients. The service is implemented by method
	 * ProvideService.
	 * Also uncomment the second statement if you want to highlight
	 * the availability of the service, for instance for debugging
	 * purposes.
	 *
	 * Add similar statements for each additional ROS service that
	 * your node provides.
	 */

	// TimeoutTimer = Handle.createTimer(ros::Duration(duration_of_the_timeout), &ROSnode::TimeoutCallback, this, put_here_true_or_false);
	/***********************************************************
	 * Uncomment if your node requires a timeout. Substitute 
	 * duration_of_the_timeout with the required value, in seconds,
	 * expressed as a float (e.g., 14.0).
	 * put_here_true_or_false should be substituted with true or
	 * false. In the first case, after the timeout expires method
	 * TimeoutCallback will be called only once ("once-only"
	 * timeout). In the second case, once the timeout expires,
	 * TimeoutCallback is called periodically, with period equal to
	 * duration_of_the_timeout.
	 * This statement also starts the timeout.
	 *
	 * You need one such instruction for each TimeoutTimer
	 * attribute that you defined above.
	 */

	/***********************************************************
	 * RETRIEVING PARAMETER VALUES FROM THE ROS PARAMETER SERVER
	 *
	 * Uncomment the following piece of code if you want to
	 * retrieve a parameter named my_param from the ROS parameter
	 * server, and store its value in variable ParamVar (which
	 * you need to have declared as a member variable of class
	 * ROSNode; you should choose whether to make ParamVar a public
	 * or private variable depending on who needs to access it).
	 *
     // FullParamName = ros::this_node::getName()+"/my_param";
     //uncomment this if my_param is a private parameter of the
     //node, i.e. if its full name is 
     // /name_of_the_namespace/NAME_OF_THIS_NODE/my_param

     // FullParamName = Handle.getNamespace()+"my_param";
     //uncomment this if, instead, my_param is a global parameter
     //of the namespace that the node belongs to, i.e. if its
     //full name is /name_of_the_namespace/my_param

     if (true == Handle.getParam(FullParamName, ParamVar))
     {
       ROS_INFO("Node %s: retrieved parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
     else
     {
       ROS_ERROR("Node %s: unable to retrieve parameter %s.",
       ros::this_node::getName().c_str(), FullParamName.c_str());
     }
	 *
	 * You need one piece of code like this for each parameter
	 * value that your node needs to retrieve from the ROS parameter
	 * server.
	 */

	/* Open the serial port */
	int _baudrate = BAUDRATE;
	int _timeout = TIMEOUT;
	std::string _serial_port = std::string(SERIAL_PORT);

	try {
		_serial = new serial::Serial(_serial_port, _baudrate, serial::Timeout::simpleTimeout(_timeout));		
	} catch(std::exception & e)
	{
		ROS_ERROR("Node %s: cannot open serial port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
	}

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void serial_comm_arduinoStub::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  while (ros::ok())
  {
    PeriodicTask();
     
    ros::spinOnce();
    
    LoopRate.sleep();
  }
}


void serial_comm_arduinoStub::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
  
  /* Deleting serial class */
  delete _serial;
}


void serial_comm_arduinoStub::PeriodicTask(void)
{  
	std::string _serial_port = std::string(SERIAL_PORT);

	/* Send a message to odroid */
	size_t bytes_wrote;
	uint8_t out_buffer[13] = {"Hello world!"};

	try {
		bytes_wrote = _serial->write(&(out_buffer[0]), 13*sizeof(uint8_t));
	} catch(std::exception & e)
	{
		ROS_ERROR("Node %s: cannot write a message to port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
	}  

	/* Receive a message from odroid */
	size_t bytes_read;
	uint8_t in_buffer[13];

	try {
		bytes_read = _serial->read(&(in_buffer[0]), 13*sizeof(uint8_t));
	} catch(std::exception & e)
	{
		ROS_ERROR("Node %s: cannot read a message from port %s.", ros::this_node::getName().c_str(), _serial_port.c_str());
	}  
	
	std_msgs::Float64 test_msg;
	test_msg.data = 50.0;
	
	if (bytes_read == 13)
		_Publisher.publish(test_msg);
}

