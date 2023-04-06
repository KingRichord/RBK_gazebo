#include <thread>
#include <deque>
#include <cmath>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/imu.pb.h>
#include <gazebo/msgs/laserscan_stamped.pb.h>

#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/math/Angle.hh>

#include <Eigen/Dense>


#include "../../communication/talker.h"

namespace gazebo
{
    class DiffCartPlugin : public ModelPlugin
    {
        public: DiffCartPlugin(){}
        public: ~DiffCartPlugin(){}
        public: virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            mWheelDiff = 0.26;
            mWheelRadius = 0.1;
            mLeftWheelCmd = 0;
            mRightWheelCmd = 0;

            mWorld = model->GetWorld();
            mModel = model;
            mLeftWheel = model->GetJoint(mModel->GetName() + "::base_2_left_wheel");
            mRightWheel = model->GetJoint(mModel->GetName() + "::base_2_right_wheel");
            mImu = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::get_sensor(mWorld->Name() + "::" + mModel->GetName() + "::base::imu"));
            if (!mImu)
                std::cout << "没找到IMU" << std::endl;

            std::cout << mWorld->Name() << std::endl;
            std::cout << mModel->GetName() << std::endl;

            mSimTime = mWorld->SimTime();
            mLastCmdTime = mSimTime;
            mCmdTimeout = 0.1;

            mXOdom = 0;
            mYOdom = 0;
            mAOdom = 0;
            mOdomCount = 0;
			
            mGazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
            mGazeboNode->Init();
            mImuSub = mGazeboNode->Subscribe("~/" + mModel->GetName() + "/base/imu/imu", &DiffCartPlugin::OnImuMsg, this);
            mLaserScanSub = this->mGazeboNode->Subscribe("~/" + mModel->GetName() + "/lidar/lidar/scan", &DiffCartPlugin::OnLaserScanMsg, this);
            mUpdateEndCnt = event::Events::ConnectWorldUpdateEnd(boost::bind(&DiffCartPlugin::OnWorldUpdateEnd, this));
	        m_talker_ =std::make_shared<Talker>();    //创建与RBK之间的连接关系
	        m_talker_->Init("tcp://127.0.0.1:8002");
        }
        private: void OnImuMsg(ConstIMUPtr & msg)
        {
			nlohmann::json pose_json;
            pose_json["imu"].emplace_back(true);
	        
			pose_json["sec"].emplace_back(  msg->stamp().sec());
	        pose_json["nsec"].emplace_back(  msg->stamp().nsec());
			
	        pose_json["x"].emplace_back(msg->orientation().x());
	        pose_json["y"].emplace_back(msg->orientation().y());
	        pose_json["z"].emplace_back(msg->orientation().z());
	        pose_json["w"].emplace_back(msg->orientation().w());
			
	        pose_json["w_x"].emplace_back(msg->angular_velocity().x());
	        pose_json["w_y"].emplace_back(msg->angular_velocity().y());
	        pose_json["w_z"].emplace_back(msg->angular_velocity().z());
			
	        pose_json["a_x"].emplace_back( msg->linear_acceleration().x());
	        pose_json["a_y"].emplace_back( msg->linear_acceleration().y());
	        pose_json["a_z"].emplace_back( msg->linear_acceleration().z());
			
			m_talker_->Pub(pose_json);
        }

        private: void HandleSingleLaserMsg(ConstLaserScanStampedPtr & msg)
        {
            gazebo::msgs::Pose const & pose_msg = msg->scan().world_pose();
            ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msg);
            Eigen::Quaterniond qua(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

            int count = msg->scan().count();
            int vertical_count = msg->scan().vertical_count();
            double angle_min = msg->scan().angle_min();
            double angle_step = msg->scan().angle_step();
            double range_min = msg->scan().range_min();
            double range_max = msg->scan().range_max();
        }

        private: void HandleMultiLaserMsg(ConstLaserScanStampedPtr & msg)
        {
            gazebo::msgs::Pose const & pose_msg = msg->scan().world_pose();
            ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(pose_msg);
            Eigen::Quaterniond qua(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

            int count = msg->scan().count();
            int vertical_count = msg->scan().vertical_count();
            double angle_min = msg->scan().angle_min();
            double angle_step = msg->scan().angle_step();
            double range_min = msg->scan().range_min();
            double range_max = msg->scan().range_max();
            double vertical_angle_min = msg->scan().vertical_angle_min();
            double vertical_angle_max = msg->scan().vertical_angle_max();
            double vertical_step = msg->scan().vertical_angle_step();
        }
        /*
         * OnLaserScanMsg - 接收到Gazebo中雷达扫描数据消息的回调函数
         * 
         * @msg: 雷达扫描数据
         */
        private: void OnLaserScanMsg(ConstLaserScanStampedPtr & msg)
        {
            if (msg->scan().has_vertical_count() && msg->scan().vertical_count() > 1)
                HandleMultiLaserMsg(msg);
            else
                HandleSingleLaserMsg(msg);
        }


        /*
         * OnWorldUpdateEnd - 每次Gazebo完成仿真更新之后的回调函数
         */
        private: void OnWorldUpdateEnd()
        {
            common::Time lastTime = mSimTime;
            mSimTime = mWorld->SimTime();
            UpdateOdometry(mSimTime - lastTime);
            AdjustVelocity();
        }
 
        private:
            common::Time mSimTime;
            common::Time mLastCmdTime;
            double mCmdTimeout;         // 单位s
			
		private: void OnCmdVelFromRos(const geometry_msgs::TwistConstPtr & msg)
		{
		    mLastCmdTime = mWorld->SimTime();
		    mLeftWheelCmd = msg->linear.x - msg->angular.z * mWheelDiff * 0.5;
		    mRightWheelCmd = msg->linear.x + msg->angular.z * mWheelDiff * 0.5;
		}
		
        private: void AdjustVelocity()
        {
            common::Time td = mSimTime - mLastCmdTime;
            if (td.Double() > mCmdTimeout) {
                mLeftWheelCmd = 0;
                mRightWheelCmd = 0;
            }
            mLeftWheel->SetVelocity(0, mLeftWheelCmd / mWheelRadius);
            mRightWheel->SetVelocity(0, mRightWheelCmd / mWheelRadius);
        }
        private: int mOdomCount;
        private: void UpdateOdometry(common::Time td)
        {
            mLeftWheelVel = mLeftWheel->GetVelocity(0);
            mRightWheelVel = mRightWheel->GetVelocity(0);

            double ds_left = mWheelRadius * mLeftWheelVel * td.Double();
            double ds_right = mWheelRadius * mRightWheelVel * td.Double();
            if (std::isnan(ds_left))
                ds_left = 0;
            if (std::isnan(ds_right))
                ds_right = 0;

            double ds = 0.5 * (ds_left + ds_right);
            //double da = (ds_right - ds_left) / mWheelDiff;
            ignition::math::Vector3d ang_vel = mImu->AngularVelocity();
            double da = ang_vel.Z() * td.Double();

            mXOdom += ds * std::cos(mAOdom);
            mYOdom += ds * std::sin(mAOdom);
            mAOdom += da;
            mOdomCount++;
            if (mOdomCount < 10)
                return;

            mOdomCount = 0;
        }

        private:
	        std::shared_ptr<Talker> m_talker_;
            gazebo::transport::NodePtr mGazeboNode;
            gazebo::transport::SubscriberPtr mImuSub;
            gazebo::transport::SubscriberPtr mLaserScanSub;
	        gazebo::transport::SubscriberPtr mImageSub;
            physics::WorldPtr mWorld;
            physics::ModelPtr mModel;
            physics::JointPtr mLeftWheel;
            physics::JointPtr mRightWheel;
            sensors::ImuSensorPtr mImu;
            event::ConnectionPtr mUpdateEndCnt;
            double mWheelDiff;      // 轮间距
            double mWheelRadius;    // 轮半径

            double mLeftWheelCmd;   // 左轮速度指令
            double mLeftWheelVel;   // 左轮速度状态

            double mRightWheelCmd;  // 右轮速度指令
            double mRightWheelVel;  // 右轮速度状态

            double mXOdom;
            double mYOdom;
            double mAOdom;
    };

    GZ_REGISTER_MODEL_PLUGIN(DiffCartPlugin)
}

