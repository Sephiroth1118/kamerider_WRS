#include <ros/ros.h>
#include <human_navigation/HumanNaviObjectInfo.h>
#include <human_navigation/HumanNaviDestination.h>
#include <human_navigation/HumanNaviTaskInfo.h>
#include <human_navigation/HumanNaviMsg.h>
#include <human_navigation/HumanNaviGuidanceMsg.h>
#include <human_navigation/HumanNaviAvatarStatus.h>
#include <human_navigation/HumanNaviObjectStatus.h>
//#include <Vector3.h>
#include <math.h>

using namespace std;

//角度计算使用欧拉角
//四元组全部转为Vector3,再转化为数组
//点坐标全部转为数组

double offsetAngel= 3.1415926535/4;//45度
double nearDistance = 1;
double targetDistance ;
double leftAngel;
double rightAngel;
double leftDistance;
double rightDistance;
double headToTarget[2];

bool isOnTheWay = true;
bool isStart = false;

//
bool isSentGetAvatarStatus = false;
//Vector3 towards;
bool turnLeft = false;
bool turnRight = false;
bool turnFront = false;
bool turnBack = false;

double nearTargetDistance = 0.03;
bool moveLeftHand = false;
bool moveRightHand = false;
bool moveHandLeft = false;
bool moveHandRight = false;

bool stopMove = false;

bool LeftHandCautch = false;
bool RightHandCautch = false;

////////////////////////////////////////////////////////////







////////////////////////////////////////////////////////////




class Vector3
{
	public:
	double x, y, z;
	Vector3(){};
	Vector3(const Vector3 &a) :x(a.x), y(a.y), z(a.z){}
    Vector3(double nx, double ny, double nz) :x(nx), y(ny), z(nz){}
};

class player
{
	public:
	double head[3] ;
	double chest[3];
	double lefthand[3] ;
	double righthand[3] ;
	double towards[3];
	double chestToTarget[3];
	bool is_target_object_in_left_hand;
    bool is_target_object_in_right_hand;
    string object_in_left_hand;
    string object_in_right_hand;
};

//player Avatar;

class MoveDirection
{
public:

	enum Step
	{
		ready,
		goRight,
		goWrong,
		moveWhitchHand,
		moveHand,
		canCautch,
		cautchWrong,
		cautchRight
	};

	int step;

	player Avatar;

	bool isNearTheTarget ;

	double targetFuniture[3] ;//0:x   1:y   2:z

	double targetObject[3] ;//0:x   1:y   2:z
	
	human_navigation::HumanNaviTaskInfo taskInfo;
	
	human_navigation::HumanNaviAvatarStatus avatarStatus;

	string targetObjectName;

	string targetFunitureName;


/////////////////////////////////////////////////////////////////////////////

enum class SpeechState
	{
		None,
		WaitingState,
		Speaking,
		Speakable
	};

	// human navigation message from/to the moderator
	const std::string MSG_ARE_YOU_READY      = "Are_you_ready?";
	const std::string MSG_TASK_SUCCEEDED     = "Task_succeeded";
	const std::string MSG_TASK_FAILED        = "Task_failed";
	const std::string MSG_TASK_FINISHED      = "Task_finished";
	const std::string MSG_GO_TO_NEXT_SESSION = "Go_to_next_session";
	const std::string MSG_MISSION_COMPLETE   = "Mission_complete";
	const std::string MSG_REQUEST            = "Guidance_request";
	const std::string MSG_SPEECH_STATE       = "Speech_state";
	const std::string MSG_SPEECH_RESULT      = "Speech_result";

	const std::string MSG_I_AM_READY        = "I_am_ready";
	const std::string MSG_GET_AVATAR_STATUS = "Get_avatar_status";
	const std::string MSG_GET_OBJECT_STATUS = "Get_object_status";
	const std::string MSG_GET_SPEECH_STATE  = "Get_speech_state";

	// display type of guidance message panels for the avatar (test subject)
	const std::string DISPLAY_TYPE_ALL         = "All";
	const std::string DISPLAY_TYPE_ROBOT_ONLY  = "RobotOnly";
	const std::string DISPLAY_TYPE_AVATAR_ONLY = "AvatarOnly";
	const std::string DISPLAY_TYPE_NONE        = "None";

	//int step;
	SpeechState speechState;

	bool isStarted;
	bool isFinished;

	bool isTaskInfoReceived;
	bool isRequestReceived;

	ros::Time timePrevSpeechStateConfirmed;

	bool isSentGetAvatarStatus;
	bool isSentGetObjectStatus;


void reset()
	{
		isStarted             = false;
		isFinished            = false;
		isTaskInfoReceived    = false;
		isRequestReceived     = false;
		isSentGetAvatarStatus = false;
		isSentGetObjectStatus = false;
		//isArrived			  = false;
		//isCatched			  = false;
	}


	// send humanNaviMsg to the moderator (Unity)
	void sendMessage(ros::Publisher &publisher, const std::string &message)
	{
		human_navigation::HumanNaviMsg human_navi_msg;
		human_navi_msg.message = message;
		publisher.publish(human_navi_msg);

		//ROS_INFO("Send message:%s", message.c_str());
	}

	void sendGuidanceMessage(ros::Publisher &publisher, const std::string &message, const std::string displayType)
	{
		human_navigation::HumanNaviGuidanceMsg guidanceMessage;
		guidanceMessage.message = message;
		guidanceMessage.display_type = displayType;
		publisher.publish(guidanceMessage);

		speechState = SpeechState::Speaking;

		ROS_INFO("Send guide message: %s : %s", guidanceMessage.message.c_str(), guidanceMessage.display_type.c_str());
	}


	// receive humanNaviMsg from the moderator (Unity)
	void messageCallback(const human_navigation::HumanNaviMsg::ConstPtr& message)
	{
		ROS_INFO("Subscribe message: %s : %s", message->message.c_str(), message->detail.c_str());

		if(message->message==MSG_ARE_YOU_READY)
		{
			isStarted = true;
		}
		else if(message->message==MSG_REQUEST)
		{
			if(isTaskInfoReceived && !isFinished)
			{
				isRequestReceived = true;
			}
		}
		else if(message->message==MSG_TASK_SUCCEEDED)
		{
		}
		else if(message->message==MSG_TASK_FAILED)
		{
		}
		else if(message->message==MSG_TASK_FINISHED)
		{
			isFinished = true;
		}
		else if(message->message==MSG_GO_TO_NEXT_SESSION)
		{
			ROS_INFO("Go to next session");
			step = ready;
		}
		else if(message->message==MSG_MISSION_COMPLETE)
		{
			//exit(EXIT_SUCCESS);
		}
		else if(message->message==MSG_SPEECH_STATE)
		{
			if(message->detail=="Is_speaking")
			{
				speechState = SpeechState::Speaking;
			}
			else
			{
				speechState = SpeechState::Speakable;
			}
		}
		else if(message->message==MSG_SPEECH_RESULT)
		{
			ROS_INFO("Speech result: %s", message->detail.c_str());
		}
	}

	



////////////////////////////////////////////////////////////////////////////////

	
	void init()
	{
		isNearTheTarget = false;

		turnLeft = false;
		turnRight = false;
		turnFront = false;
		turnBack = false;

		
		moveLeftHand = false;
		moveRightHand = false;
		moveHandLeft = false;
		moveHandRight = false;

		stopMove = false;

		LeftHandCautch = false;
	 	RightHandCautch = false;

	 	step = ready;

	}




	//初始化目标位置
	void taskInfoMessageCallback(const human_navigation::HumanNaviTaskInfo::ConstPtr& message)
	{
		ROS_INFO_STREAM("Session starts!");
		taskInfo = *message;
		targetObject[0] = taskInfo.target_object.position.x;
		targetObject[1] = taskInfo.target_object.position.y;
		targetObject[2] = taskInfo.target_object.position.z;

		targetObjectName = taskInfo.target_object.name;
		isStart=true;
		ROS_INFO_STREAM(step);
	}

	//更新坐标，四元组转欧拉
	void avatarStatusMessageCallback(const human_navigation::HumanNaviAvatarStatus::ConstPtr& message)
	{
		avatarStatus = *message;

		//Vector3 temp ;//= //avatarStatus.head.orientation.eulerAngles();
		double chestToLeft[2];
		double chestToRight[2];

		chestToLeft[0] = avatarStatus.left_hand.position.x - avatarStatus.body.position.x ;
		chestToLeft[1] = avatarStatus.left_hand.position.y - avatarStatus.body.position.y ;

		chestToRight[0] = avatarStatus.right_hand.position.x - avatarStatus.body.position.x ;
		chestToRight[1] = avatarStatus.right_hand.position.y - avatarStatus.body.position.y ;



		Avatar.towards[0] = chestToLeft[0] + chestToRight[0];
		Avatar.towards[1] = chestToLeft[1] + chestToRight[1];
		//Avatar.towards[2] = temp.z;
		
		//head		
		Avatar.head[0] = avatarStatus.head.position.x;
		Avatar.head[1] = avatarStatus.head.position.y;
		Avatar.head[2] = avatarStatus.head.position.z;

		//chest
		Avatar.chest[0] = avatarStatus.body.position.x;
		Avatar.chest[1] = avatarStatus.body.position.y;
		Avatar.chest[2] = avatarStatus.body.position.z;


		//lefthand
		Avatar.lefthand[0] = avatarStatus.left_hand.position.x;
		Avatar.lefthand[1] = avatarStatus.left_hand.position.y;
		Avatar.lefthand[2] = avatarStatus.left_hand.position.z;

		//righthand
		Avatar.righthand[0] = avatarStatus.right_hand.position.x;
		Avatar.righthand[1] = avatarStatus.right_hand.position.y;
		Avatar.righthand[2] = avatarStatus.right_hand.position.z;
		
		Avatar.is_target_object_in_left_hand = avatarStatus.is_target_object_in_left_hand;
		Avatar.is_target_object_in_right_hand = avatarStatus.is_target_object_in_right_hand;
		
		Avatar.object_in_left_hand = avatarStatus.object_in_left_hand;
		Avatar.object_in_right_hand = avatarStatus.object_in_right_hand;
	}


	//判断Avator是否在正确的方向上 偏移角不超过

	bool is_on_the_way()
	{
		Avatar.chestToTarget[0] = targetObject[0] - Avatar.chest[0];
		Avatar.chestToTarget[1] = targetObject[1] - Avatar.chest[1];
		double v1 = sqrt(pow(Avatar.chestToTarget[0],2)+pow(Avatar.chestToTarget[1],2));
		double v2 = sqrt(pow(Avatar.towards[0],2)+pow(Avatar.towards[1],2));
		double offsetAngelNew = acos((Avatar.towards[0]*Avatar.chestToTarget[0]+Avatar.towards[1]*Avatar.chestToTarget[1])/v1*v2);
		if(offsetAngelNew<=offsetAngel)
		{
			isOnTheWay = true;
			step = 	goRight;
		}
		else
		{
			isOnTheWay = false;
			step = goWrong;
		}
		return isOnTheWay;
	}


	//计算头与目标物体的距离（只看XOY平面），距离小于nearDistance判定为“近”
	bool is_near()
	{
		targetDistance = sqrt(pow(Avatar.head[0]-targetObject[0],2)+pow(Avatar.head[1]-targetObject[1],2));
		if(targetDistance<=nearDistance)
		{
			isNearTheTarget = true;
			step = moveWhitchHand;//可以开始移手了
		}
		else//又走错了
		{
			isNearTheTarget = false;
			step = goWrong;
		}	
		return isNearTheTarget;
	}


	//Avatar与目标较远时，给出目标在左还是在右的指示(若is_near()==false,则循环使用这个函数)
	void left_or_right()
	{
		
		//计算距两手距离，判断左右
		leftDistance = sqrt(pow(Avatar.lefthand[0]-targetObject[0],2)+pow(Avatar.lefthand[1]-targetObject[1],2));
		rightDistance = sqrt(pow(Avatar.righthand[0]-targetObject[0],2)+pow(Avatar.righthand[1]-targetObject[1],2));

		if(leftDistance<rightDistance)
		{
			turnLeft = true;
			turnRight = false;
		}
		else
		{
			turnLeft = false;
			turnRight = true;
		}


		//计算向量点积，判断前后
		headToTarget[0] = targetObject[0] - Avatar.head[0];
		headToTarget[1] = targetObject[1] - Avatar.head[1];
		double dianzi = headToTarget[0]*Avatar.towards[0] + headToTarget[1]*Avatar.towards[1];
		if(dianzi>0)
		{
			turnFront = true;
			turnBack = false;
		}
		if(dianzi<0)
		{
			turnFront = false;
			turnBack = true;
		}
		
	}


	//手是否与目标很近,哪只手可以抓取（即是否可以停止调整）
	bool is_stop_move_hand()
	{
		//此处应更新左右手坐标
		leftDistance = sqrt(pow(Avatar.lefthand[0]-targetObject[0],2)+pow(Avatar.lefthand[1]-targetObject[1],2));
		rightDistance = sqrt(pow(Avatar.righthand[0]-targetObject[0],2)+pow(Avatar.righthand[1]-targetObject[1],2));

		//左手可抓取
		if(leftDistance <= nearTargetDistance)
		{
			LeftHandCautch = true;
			RightHandCautch = false;
			step = canCautch;
			return true;
		}

		//右手可抓取
		if(rightDistance <= nearTargetDistance)
		{
			LeftHandCautch = false;
			RightHandCautch = true;
			step = canCautch;
			return true;
		}
		
		return false;
	}


	//Avatar与目标较近时，决定微调哪个手部//(若is_stop_move_hand()==false，则循环使用这个函数)
	void move_whitch_hand()
	{
		//计算距两手距离，判断移哪只手
		//此处应更新左右手坐标
		leftDistance = sqrt(pow(Avatar.lefthand[0]-targetObject[0],2)+pow(Avatar.lefthand[1]-targetObject[1],2));
		rightDistance = sqrt(pow(Avatar.righthand[0]-targetObject[0],2)+pow(Avatar.righthand[1]-targetObject[1],2));

		if(leftDistance<rightDistance)
		{
			moveLeftHand = true;
			moveRightHand = false;

		}
		else
		{
			moveLeftHand = false;
			moveRightHand = true;
		}
		step = moveHand;
	}

	void move_hand()
	{
		double leftToRight[2];//左手点到右手点的向量

		leftToRight[0] = Avatar.righthand[0] - Avatar.lefthand[0];
		leftToRight[1] = Avatar.righthand[1] - Avatar.lefthand[1];

		double handToTarget[2];

		if(moveLeftHand == true)//移左手
		{
			handToTarget[0] = targetObject[0] - Avatar.lefthand[0];
			handToTarget[1] = targetObject[1] - Avatar.lefthand[1];
			double dianzi = handToTarget[0]* leftToRight[0] + handToTarget[1]*leftToRight[1];

			if(dianzi < 0)//向左移
			{
				moveHandLeft = true;
				moveHandRight = false;
			}
			else//向右移
			{
				moveHandLeft = false;
				moveHandRight = true;
			}
		}
		else//移右手
		{
			handToTarget[0] = targetObject[0] - Avatar.righthand[0];
			handToTarget[1] = targetObject[1] - Avatar.righthand[1];
			double dianzi = handToTarget[0]* leftToRight[0] + handToTarget[1]*leftToRight[1];

			if(dianzi > 0)//向左移
			{
				moveHandLeft = true;
				moveHandRight = false;
			}
			else//向右移
			{
				moveHandLeft = false;
				moveHandRight = true;
			}
		}
	}


	//发送指导消息
	void speak()
	{
		/*
		ROS_INFO_STREAM(
			"Subscribe task info message:" << std::endl <<
			"Environment ID: " << taskInfo.environment_id << std::endl <<
			"Target object: " << std::endl << taskInfo.target_object <<
			"Destination: " << std::endl << taskInfo.destination
		);
		*/
		//ROS_INFO_STREAM("speaking"<<std::endl);
		
		switch(step)
			{
				case goRight:
				{
					ROS_INFO_STREAM("step:goright"<<std::endl);
					ROS_INFO_STREAM("You are on the right direction."<<std::endl);
					break;
					//is_on_the_way();
				}
				case goWrong:
				{
					//ROS_INFO_STREAM("You are on the wrong direction."<<std::endl);
					//ROS_INFO_STREAM("The "+targetObjectName+" is ");
					ROS_INFO_STREAM("Please go ");
					if(turnLeft)//左转
					{
						ROS_INFO_STREAM("left ");
					}
					if(turnRight)//右转
					{
						ROS_INFO_STREAM("right ");
					}
					if(turnFront)//左转
					{
						ROS_INFO_STREAM("front ");
					}
					if(turnBack)//右转
					{
						ROS_INFO_STREAM("back ");
					}
					ROS_INFO_STREAM("to find the "+targetObjectName<<std::endl);
					break;

				}
				case moveWhitchHand:
				{
					break;
				}
				case moveHand:
				{
					if(moveLeftHand)//移左手
					{
						ROS_INFO_STREAM("Move your left hand ");
						if(moveHandLeft)
						{
							ROS_INFO_STREAM("left.");
						}
						if(moveHandRight)
						{
							ROS_INFO_STREAM("right.");
						}
					}
					if(moveRightHand)//移右手
					{
						ROS_INFO_STREAM("Move your left hand ");
						if(moveHandLeft)
						{
							ROS_INFO_STREAM("left.");
						}
						if(moveHandRight)
						{
							ROS_INFO_STREAM("right.");
						}
					}
					break;
				}
				case canCautch:
				{
					if(LeftHandCautch)
					{
						ROS_INFO_STREAM("Cautch it with your left hand!");
					}
					if(RightHandCautch)
					{
						ROS_INFO_STREAM("Cautch it with your right hand!");
					}
					break;
				}
				case cautchWrong:
				{
					ROS_INFO_STREAM("You have cautched the wrong thing. Please put it down!");
					break;
				}
				case cautchRight:
				{
					ROS_INFO_STREAM("Good job! You get the target!");
					//isStrat = false;
					break;
				}
			}
		
	}





///////////////////////////////////////////////////////////////////////////
	int run(int argc, char **argv)
	{
		ros::init(argc, argv, "MoveDirection");

		ros::NodeHandle nodeHandle;

		ros::Rate loopRate(1);

		init();

		ROS_INFO("Move Direction Is On!");

		ros::Subscriber subTaskInfoMsg = nodeHandle.subscribe<human_navigation::HumanNaviTaskInfo>("/human_navigation/message/task_info", 1, &MoveDirection::taskInfoMessageCallback, this);
		ros::Subscriber subAvatarStatusMsg = nodeHandle.subscribe<human_navigation::HumanNaviAvatarStatus>("/human_navigation/message/avatar_status", 1, &MoveDirection::avatarStatusMessageCallback, this);
		ros::Publisher pubHumanNaviMsg = nodeHandle.advertise<human_navigation::HumanNaviMsg>("/human_navigation/message/to_moderator", 10);
		ros::Time time;
		/*
		goRight;
		goWrong;
		moveWhitchHand;
		moveHand;
		canCautch;
		cautchWrong;
		cautchRight;
		*/

		/*
		ROS_INFO_STREAM(
			"Subscribe task info message:" << std::endl <<
			"Environment ID: " << taskInfo.environment_id << std::endl <<
			"Target object: " << std::endl << taskInfo.target_object <<
			"Destination: " << std::endl << taskInfo.destination
		);
		*/

		while (ros::ok())
		{ 
			//ROS_INFO_STREAM("    run step:"+step<<std::endl);
			sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
			switch(step)
			{
				case ready:
				{
					if(isStart)
					{
						step = goRight;
					}
					//step = goRight;
					break;
				}
				case goRight:
				{
					if(!is_near())
					{
						is_on_the_way();
					}
					//is_on_the_way();
					ROS_INFO_STREAM("goRight");
					break;
				}
				case goWrong:
				{
					if(!is_near())
					{
						if (!is_on_the_way())
						{
							left_or_right();
						}
					}
					ROS_INFO_STREAM("goWrong");
					break;
				}
				case moveWhitchHand:
				{
					if(is_near())
     					{move_whitch_hand();}
					//move_whitch_hand();
					break;
				}
				case moveHand:
				{
					if(is_near()&&!is_stop_move_hand())
					{
						move_hand();
					}
					//move_hand();
					break;
				}
				case canCautch:
				{
					if(Avatar.object_in_left_hand!=""&&Avatar.object_in_right_hand!="")//抓了东西
					{
						if(Avatar.is_target_object_in_left_hand||Avatar.is_target_object_in_right_hand)
							step = cautchRight;
						else
							step = cautchWrong;	
					}
					ROS_INFO_STREAM("canCautch");
					break;
				}
				case cautchWrong:
				{
					step = moveHand;
					break;
				}
				case cautchRight:
				{
					break;
				}
				//speak();
				/*
				//已经抓住了
				if(Avatar.is_target_object_in_left_hand)//左手对了
				{
					break;
				}
				if(Avatar.is_target_object_in_right_hand)//右手对了
				{
					break;
				}


				//没有走到目标附近时
				while(!is_near())
				{
					left_or_right();
				}

				//已经走到目标附近,但还不可抓取
				while(!is_stop_move_hand())
				{
					move_whitch_hand();
				}

				//已经可以抓取
				if(LeftHandCautch)//左手抓
				{

				}
				else//右手抓
				{

				}
				*/
			}
			
			speak();
			ros::spinOnce();

			loopRate.sleep();
		}

		return 0;
	}



///////////////////////////////////////////////////////////////////////////


	
};




int main(int argc, char **argv)
{
	MoveDirection move;

	move.run(argc, argv);
};
