#include <chrono>
#include <fstream>
#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include <ctime>
#include <vector>
#include <mutex>
#include "drdc.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

using namespace std::chrono_literals;

/**
 * @defgroup logger Macros
 * @brief Macros for simplified ROS2 logging.
 *
 * These macros wrap RCLCPP logging functions to allow easy logging,
 * cause I'm lazy.
 *
 * Usage:
 * @code
 * LOG_INFO("This is an info message");
 * LOG_WARN("This is a warning");
 * LOG_ERROR("This is an error");
 * @endcode
 *
 * @note Must be used inside Mothership class, or similar ros node.
 * @{
 */
/** Log an info-level message */
#define LOG_INFO(msg) RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "%s", msg)
/** Log a warning-level message */
#define LOG_WARN(msg) RCLCPP_WARN(rclcpp::get_logger("s7_mothership"), "%s", msg)
/** Log an error-level message */
#define LOG_ERROR(msg) RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "%s", msg)
/** @} */ //

using namespace std::chrono_literals;

/**
 * @defgroup geometry_msgs Type Aliases
 * @brief Again, lazy.
 *
 * These aliases simplify usage of geometry_msgs common in this here code.
 *
 * Usage:
 * @code
 * PoseStamped pose;
 * TwistStamped twist;
 * WrenchStamped wrench;
 * @endcode
 *
 * @note A wise man once said, a good programmer is a lazy programmer.
 * @{
 */
/// Alias for geometry_msgs::msg::PoseStamped
using PoseStamped = geometry_msgs::msg::PoseStamped;
/// Alias for geometry_msgs::msg::TwistStamped
using TwistStamped = geometry_msgs::msg::TwistStamped;
/// Alias for geometry_msgs::msg::WrenchStamped
using WrenchStamped = geometry_msgs::msg::WrenchStamped;
/** @} */ //

/**
 * @defgroup Structs for CSVLogger class
 * @brief creating data types for vectors
 *
 * Probably not optimised
 *
 * @{
 */
/// Struct for logging response time between publishing and subscribing.
struct ResponseTime
{
    u_int id;
    int64_t time1; // Convert from header.time with : rclcpp::Time ros_time(msg.header.stamp);
    int64_t time2;
    // time_t time3;
};
/// Struct for logging operation time between retrieving data, publishing, and controlling
struct OperationTime
{
    int id;
    int64_t sTime;
    int64_t saveTime;
    int64_t pubTime; // Yeaaaah pub time
    int64_t ctrlTime;
    int64_t endTime;
};
/// Struct for logging avg data lock time of the SharedData class
struct LockTime
{
    std::string data;
    std::string lockSource;
    int64_t lockTime;
};

/**@} */

#pragma region subclasses & enums

/**
 * @enum S7Mode
 * @brief State Machine definitions for different modes used by the S7Mothership
 * & the S7Controller.
 */
enum S7Mode
{
    // BRAKING,
    FREE,
    FORCEFEEDBACK //,
                  // SELF_CENTERING
};

/**
 * @class SharedData
 * @brief RAII Class managing data locking between each class using the pose messages m
 */
class SharedData
{
private:
    std::mutex pose_mutex;
    std::unique_ptr<PoseStamped> CurrentPose;

    std::mutex twist_mutex;
    std::unique_ptr<TwistStamped> CurrentTwist;

    std::mutex wrench_mutex;
    std::unique_ptr<WrenchStamped> CurrentWrench;

public:
    // Constructor
    // no need to lock, as the object isn't shared yet
    explicit SharedData()
        : CurrentPose(std::make_unique<PoseStamped>()),
          CurrentTwist(std::make_unique<TwistStamped>()),
          CurrentWrench(std::make_unique<WrenchStamped>())
    {
    }

    std::shared_ptr<PoseStamped> getCurrentPose()
    {
        std::lock_guard<std::mutex> lock_pose(pose_mutex);
        auto tmp = std::make_shared<PoseStamped>(*CurrentPose);
        return tmp;
    }

    void setCurrentPose(PoseStamped &newPose)
    {
        std::lock_guard<std::mutex> lock_pose(pose_mutex);
        *CurrentPose = newPose;
    }

    std::shared_ptr<TwistStamped> getCurrentTwist()
    {
        std::lock_guard<std::mutex> lock_twist(twist_mutex);
        auto tmp = std::make_shared<TwistStamped>(*CurrentTwist);
        return tmp;
    }

    void setCurrentTwist(TwistStamped &newTwist)
    {
        std::lock_guard<std::mutex> lock_twist(twist_mutex);
        *CurrentTwist = newTwist;
    }

    std::shared_ptr<WrenchStamped> getCurrentWrench()
    {
        std::lock_guard<std::mutex> lock_wrench(wrench_mutex);
        auto tmp = std::make_shared<WrenchStamped>(*CurrentWrench);
        return tmp;
    }

    void setCurrentWrench(std::shared_ptr<WrenchStamped> newWrench)
    {
        std::lock_guard<std::mutex> lock_wrench(wrench_mutex);
        *CurrentWrench = *newWrench;
    }
};

/**
 * @class CSVLogger
 * @brief Custom manual logging tool, meant to write timestamps to csv file.
 *
 * Called manually by way of its public member functions, so not automatic.
 * Requires corresponding debug flags on build for each debugging type.
 *
 * DEBUG_RESPONSE_TIME : for response time logging
 * DEBUG_OPERATION_TIME : for main loop time logging
 * DEBUG_LOCK_TIME : for SharedData lock time logging
 */
class CSVLogger
{
public:
    CSVLogger(std::shared_ptr<SharedData> shareddata)
        : rtFile("response_time_log.csv", std::ios::out),
          SharedData_(shareddata)
    {
        if (!rtFile.is_open())
        {
            LOG_ERROR("Failed to open file : response_time_log");
        }
        else
        {
            rtFile << "id_number, publish_time, subscibe_time\n";
        }
    }

    /**
     * @brief Call to write pose time to logger buffer
     *
     * needs to be called before a new pose check is called, to not skip messages.
     */
    void logPose(u_int id, int64_t time)
    {
        // write to latest vector only the pose message time setting corresponding time2 to 0 untill sub message arrives
        ResponseTime R = {id, time, 0};
        rtBuffer.push_back(R);
    }

    /**
     * @brief end of code flush to write all the data in csv files
     */
    void flush()
    {
#ifdef DEBUG_RESPONSE_TIME
        writeResponseTime();
#endif
#ifdef DEBUG_OPERATION_TIME
        writeOperationTime();
#endif
#ifdef DEBUG_LOCK_TIME
        writeLockTime();
#endif
    }

    /**
     * @brief call to write the latest wrench to its corresponding ID in the buffer
     */
    void logWrench(u_int id, int64_t time)
    {
        // write to correct spot in the vector
        // doesn't check if the spot exists in the vector (ideally doesn't need to as a force recieved should always have the same ID as a pose already logged)
        // And also I'm lazy af
        if (id < rtBuffer.size())
        {
            rtBuffer[id].time2 = time;
        }
        else
        {
            LOG_ERROR("Tried to log wrench when insufficent space");
        }
    }

private:
    /**
     * @brief writes all of rtBuffer to csv file
     */
    void writeResponseTime()
    {
        for (std::vector<ResponseTime>::iterator it = rtBuffer.begin(); it != rtBuffer.end(); ++it)
        {
            rtFile << it->id << "," << it->time1 << "," << it->time2 << "\n";
        }
        rtFile.flush();
        LOG_INFO("CSV file for response time flushed");
    }

    // todo
    void writeOperationTime()
    {
    }

    // todo
    void writeLockTime()
    {
    }

private:
    std::ofstream rtFile;
    std::vector<ResponseTime> rtBuffer;
    std::shared_ptr<SharedData> SharedData_;
};

/**
 * @class S7Controller
 * @brief Object in charge of managing the control and data aquisition
 * loop of the Sigma7. Links using same-name functions in S7Mothership.
 *
 */
class S7Controller //: public rclcpp::node
{
public:
    S7Controller(std::shared_ptr<SharedData> sharedData, std::shared_ptr<S7Mode> mode, std::shared_ptr<CSVLogger> csvlogger)
        : CSVLogger_(csvlogger),
          SharedData_(sharedData),
          CurrentMode_(mode)
    {
    }

    ~S7Controller()
    {
        if (stop() < 0)
        {
            LOG_ERROR("failed to stop, shutting down anyways. \nTurn off s7 manually.");
        }

        LOG_INFO("S7 connexion closed.\n");
    }

    /**
     * @brief control loop for the Sigma7
     *
     * Data is aquired here and shipped off to the S7Mothership,
     * FF is applied here also, all at a ~500Hz refresh rate
     */
    int setForce()
    {
        switch (*CurrentMode_)
        {
        case FREE:
            if (dhdSetForceAndTorqueAndGripperForce(0, 0, 0, 0, 0, 0, 0) == (DHD_MOTOR_SATURATED | -1))
            {
                LOG_ERROR("Motor saturated/unable to set force");
                return -1;
            }
            break;

        case FORCEFEEDBACK:
            // TODO
            auto tmpWrench = std::make_shared<WrenchStamped>(*SharedData_->getCurrentWrench());

            // Unnecessary copy, rewrite TODO
            double fx = (*tmpWrench).wrench.force.x;
            double fy = (*tmpWrench).wrench.force.y;
            double fz = (*tmpWrench).wrench.force.z;

            double tx = (*tmpWrench).wrench.torque.x;
            double ty = (*tmpWrench).wrench.torque.y;
            double tz = (*tmpWrench).wrench.torque.z;

            if (dhdSetForceAndTorqueAndGripperForce(fx, fy, fz, tx, ty, tz, 1) == (DHD_MOTOR_SATURATED | -1))
            {
                LOG_ERROR("Motor saturated/unable to set force");
                return -1;
            }
            break;
        }

        return 1;
    }

    int savePose()
    {
        PoseStamped savedPose;
        rclcpp::Time t = rclcpp::Clock().now();
        savedPose.header.stamp = t;
        savedPose.header.frame_id = std::to_string(frame_id_); // set frame_id

        // Obtain all pose data, gets position twice to be able to retrieve velocity
        if (dhdGetPosition(&savedPose.pose.position.x, &savedPose.pose.position.y, &savedPose.pose.position.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate position");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetPosition(&savedPose.pose.position.x, &savedPose.pose.position.y, &savedPose.pose.position.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate position ");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetOrientationRad(&savedPose.pose.orientation.x, &savedPose.pose.orientation.y, &savedPose.pose.orientation.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate angular orientation");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }

        TwistStamped savedTwist;
        savedTwist.header.stamp = t;
        savedTwist.header.frame_id = std::to_string(frame_id_); // set frame_id

        // Obtain twist data
        if (dhdGetLinearVelocity(&savedTwist.twist.linear.x, &savedTwist.twist.linear.y, &savedTwist.twist.linear.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate linear velocity");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetAngularVelocityRad(&savedTwist.twist.angular.x, &savedTwist.twist.angular.y, &savedTwist.twist.angular.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate angular velocity");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }

        SharedData_->setCurrentPose(savedPose);
        SharedData_->setCurrentTwist(savedTwist);

// Log relevant data (before publish to avoid sub messages arriving in the logs too early)
#ifdef DEBUG_RESPONSE_TIME
        // log the pose (remember the static cast here)
        CSVLogger_->logPose(std::abs(frame_id_), static_cast<int64_t>(t.nanoseconds() / 1000000)); // the std::abs() is to
#endif

        // Increment frame_id
        frame_id_++;
        return 1;
    }

    int initialize()
    {
        if (!hapticInit())
        {
            LOG_ERROR("failed to initialize Sigma7, make sure device is on and connected, and Mothership is launched with root privileges.");
            return -1;
        }

        return 1;
    }

    /**
     * @brief tells the control loop to stop
     */
    int stop()
    {
        // Close the connection to the haptic device.
        if (drdClose() < 0)
        {
            LOG_ERROR("failed to close the connection");
            LOG_ERROR(dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }

        return 1;
    }

protected:
    /**
     * @brief configuration function for the Sigma7, sets the encoders
     * and checks connexion before enabling FF on the arm.
     */
    int hapticInit()
    {
        if ((drdOpen() < 0))
        {
            LOG_ERROR("error: failed to open device");
            LOG_ERROR(dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }

        LOG_INFO("Device Detected");
        LOG_WARN("WARNING, do NOT touch Sigma7 during caibration.\n\n");

        if ((drdCheckInit() < 0))
        {
            LOG_ERROR("error: failed to reinitialize device (%s)");
            LOG_ERROR(dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        LOG_INFO("moving to center");
        LOG_INFO("\n");
        if (drdMoveTo(positionCenter) < 0)
        {
            LOG_ERROR("failed to move device");
            LOG_ERROR(dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        // Stop the regulation thread but leaves the forces enabled on the device.
        if (drdStop(true) < 0)
        {
            LOG_ERROR("failed to stop robotic regulation device");
            LOG_ERROR(dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        LOG_INFO("Successfully initialised device");
        return 1;
    }

private:
    std::shared_ptr<CSVLogger> CSVLogger_;
    bool running_ = false;
    double positionCenter[DHD_MAX_DOF] = {};
    std::shared_ptr<SharedData> SharedData_;
    std::shared_ptr<S7Mode> CurrentMode_;
    int frame_id_ = 0;
};

/**
 * @class S7StatePublisher
 * @brief ROS2 node that publishes the current state as a  (TODO)PoseStamped message.
 *
 * This class manages a publisher for the S7Mothership, taking care of publishing the state msgs.
 */
class S7StatePublisher
{
public:
    /**
     * @brief Constructor for S7StatePublisher node.
     *
     * Initializes the ROS2 node with the name "state_publisher"
     * and creates a publisher on the topic "controller_pose_stamped"
     * , which is subscribed to by a component of S7Spaceship.
     */
    S7StatePublisher(std::shared_ptr<SharedData> sharedData, std::shared_ptr<rclcpp::Node> node)
        : SharedData_(sharedData),
          node_(node)
    {
        publisher_ = node_->create_publisher<PoseStamped>("controller_pose_topic", 1);
        publisher2_ = node_->create_publisher<TwistStamped>("controller_twist_topic", 1);
    }

    /**
     * @brief Publish the current state
     *
     * Publishes the current state (current_saved_state_) to controller_pose_stamped if not null
     *
     * @return 1 if published without fail, 0 if current_saved_state_ is null.
     */
    void publish()
    {
        std::shared_ptr<PoseStamped> tmpPose = std::make_shared<PoseStamped>(*SharedData_->getCurrentPose());
        publisher_->publish(*tmpPose);

        std::shared_ptr<TwistStamped> tmpTwist = std::make_shared<TwistStamped>(*SharedData_->getCurrentTwist());
        publisher2_->publish(*tmpTwist);

        // debugPublish(tmpPose); //, tmpTwist);
    }

    // TODO add twist
    /*void debugPublish(std::shared_ptr<PoseStamped> t1) //, std::shared_ptr<TwistStamped> t2)
    {
        LOG_INFO("publishing pos : ");
        LOG_INFO((*t1).pose.position.x);
        LOG_INFO((*t1).pose.position.y);
        LOG_INFO((*t1).pose.position.z);
        LOG_INFO("publishing rot : ");
        LOG_INFO((*t1).pose.orientation.x);
        LOG_INFO((*t1).pose.orientation.y);
        LOG_INFO((*t1).pose.orientation.z);
    }*/

private:
    std::shared_ptr<SharedData> SharedData_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<PoseStamped>> publisher_;
    std::shared_ptr<rclcpp::Publisher<TwistStamped>> publisher2_;
};

/**
 * @class S7ForceSubscriber
 * @brief ROS2 node that subscribes to controller_wrench_topic and recieves F.F. instructions from S7Spaceship
 *
 * This class manages a subscriber for the S7Mothership, taking care of storing the current wrench msg.
 */
class S7ForceSubscriber
{
public:
    /**
     * @brief Constructor for S7ForceSubscriber node.
     *
     * Initializes the ROS2 node with the name "wrench_subscriber"
     * and creates a subscriber on the topic "controller_wrench_topic"
     * containing F.F. instructions for the S7 arm, which is published
     * to by a component of S7Spaceship.
     */
    S7ForceSubscriber(std::shared_ptr<SharedData> sharedData, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CSVLogger> csvlogger)
        : SharedData_(sharedData),
          CSVLogger_(csvlogger),
          node_(node)
    {
        subscriber_ = node_->create_subscription<WrenchStamped>(
            "controller_wrench_topic", 1,
            std::bind(&S7ForceSubscriber::messageCallback, this, std::placeholders::_1));
    }

    ~S7ForceSubscriber() = default;

protected:
    /**
     * @brief Callback function for the subscriber, storing the message
     * in current_saved_wrench_
     */
    void messageCallback(std::shared_ptr<WrenchStamped> newWrench)
    {
        SharedData_->setCurrentWrench(newWrench);

        // Log the response from spaceship
#ifdef DEBUG_RESPONSE_TIME
        std::string id = newWrench->header.frame_id;
        int idValue = std::stoi(id);
        rclcpp::Time t(newWrench->header.stamp);

        CSVLogger_->logWrench(std::abs(idValue), static_cast<int64_t>(t.nanoseconds() / 1000000));
#endif
    }

private:
    std::shared_ptr<SharedData> SharedData_;
    std::shared_ptr<CSVLogger> CSVLogger_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Subscription<WrenchStamped>> subscriber_;
};

/**
 * @class TODO
 */
class S7InterfaceControlPublisher
{
private:
    rclcpp::Node::SharedPtr node_ = nullptr;

public:
    S7InterfaceControlPublisher(const rclcpp::Node::SharedPtr &node) { node_ = node; }
};

/**
 * @class TODO
 */
class S7InterfaceControlsubscriber
{
private:
    rclcpp::Node::SharedPtr node_ = nullptr;

public:
    S7InterfaceControlsubscriber(const rclcpp::Node::SharedPtr &node) { node_ = node; }
};

#pragma endregion

/**
 * @class S7Mothership
 * @brief The ROS node in charge of coordinating the entire system side interface
 */
class S7Mothership // : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for S7Mothership, creates the system side node "s7_mothership"
     */
    S7Mothership()
        //: Node("testi"),
        : node_(std::make_shared<rclcpp::Node>("s7_mothership")),
          SharedData_(std::make_shared<SharedData>()),
          CSVLogger_(std::make_shared<CSVLogger>(SharedData_)),
          CurrentMode_(std::make_shared<S7Mode>()),
          Controller_(std::make_unique<S7Controller>(SharedData_, CurrentMode_, CSVLogger_)),
          StatePublisher_(std::make_unique<S7StatePublisher>(SharedData_, node_)),
          ForceSubscriber_(std::make_unique<S7ForceSubscriber>(SharedData_, node_, CSVLogger_))
    {
        showStartupScreen();

        (*CurrentMode_) = S7Mode::FREE;

        // Controller initialization
        control_thread_ = std::thread(&S7Mothership::initThread, this);

        LOG_INFO("Initializing main thread");
        loop_timer_ = node_->create_wall_timer(2ms, std::bind(&S7Mothership::loop, this));

#ifdef DEBUG_RESPONSE_TIME
        LOG_WARN("Response time logging enabled, expect slightly higher latency on main loop.");
#endif
#ifdef DEBUG_OPERATION_TIME
        LOG_WARN("Operation time logging enabled, expect slightly higher latency on main loop.");
#endif
#ifdef DEBUG_LOCK_TIME
        LOG_WARN("Lock time logging enabled, expect slightly higher latency overall.");
#endif
    }

    /**
     * @brief Main 500Hz loop hosted by the mothership.
     *
     * Calls an iteration of the S7Controller loop, & requests a publish.
     *
     * Important : No ordonancing is taking place, so if one of these hangs, the 500Hz
     * refresh rate may be lost.
     */
    void loop()
    {
        // Timing control
        using namespace std::chrono;
        const auto period = microseconds(2000);
        auto start = steady_clock::now();

        if (running_)
        {
            if (initialized_)
            {
                // save pose
                Controller_->savePose();
                // publish
                StatePublisher_->publish();
                // control step
                Controller_->setForce();
            }

            // get usr input
            getUserInput();
        }
        else
        {
            // deal with shutdown
            Controller_->stop();
            loop_timer_->cancel();
            CSVLogger_->flush();
            // edge case
            while (!initialized_)
            {
                // just wait bro
                std::this_thread::sleep_for(100ms);
            }
            rclcpp::shutdown();
        }

        // Timing control
        auto elapsed = steady_clock::now() - start;
        if (!(elapsed < period))
        {
            // auto overrun = duration_cast<microseconds>(elapsed - period).count();
            LOG_WARN("control thread late");
        }
    }

    /**
     * @brief Function to manage user input during runtime
     *
     * Factorisation from main loop
     */
    void getUserInput()
    {
        if (dhdKbHit())
        {
            switch (dhdKbGet())
            {
            case 'q':
            {
                running_ = false;
                RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "Exit called");
                break;
            }
            case 'm':
            {
                if ((*CurrentMode_) == S7Mode::FREE)
                {
                    (*CurrentMode_) = S7Mode::FORCEFEEDBACK;
                }
                else
                {
                    (*CurrentMode_) = S7Mode::FREE;
                }
                break;
            }
            default:
            {
                break;
            }
            }
        }
    }

    /**
     *
     */
    void initThread()
    {
        LOG_INFO("Initializing connexion");
        if (Controller_->initialize())
        {
            initialized_ = true;
        }
        else
        {
            LOG_ERROR("failed to initialize controller, shutting down");
            running_ = false;
        }
    }

    std::shared_ptr<rclcpp::Node> getNode()
    {
        return node_;
    }

private:
    void showStartupScreen()
    {
        LOG_INFO("Sigma-7 interface mothership, written by Jacob Wallace & Emre Artar - 2025");
        LOG_INFO("______________________________________________________________________________________________________");
        LOG_INFO("'##::::'##::'#######::'########:'##::::'##:'########:'########:::'######::'##::::'##:'####:'########::");
        LOG_INFO(" ###::'###:'##.... ##:... ##..:: ##:::: ##: ##.....:: ##.... ##:'##... ##: ##:::: ##:. ##:: ##.... ##:");
        LOG_INFO(" ####'####: ##:::: ##:::: ##:::: ##:::: ##: ##::::::: ##:::: ##: ##:::..:: ##:::: ##:: ##:: ##:::: ##:");
        LOG_INFO(" ## ### ##: ##:::: ##:::: ##:::: #########: ######::: ########::. ######:: #########:: ##:: ########::");
        LOG_INFO(" ##. #: ##: ##:::: ##:::: ##:::: ##.... ##: ##...:::: ##.. ##::::..... ##: ##.... ##:: ##:: ##.....:::");
        LOG_INFO(" ##:.:: ##: ##:::: ##:::: ##:::: ##:::: ##: ##::::::: ##::. ##::'##::: ##: ##:::: ##:: ##:: ##::::::::");
        LOG_INFO(" ##:::: ##:. #######::::: ##:::: ##:::: ##: ########: ##:::. ##:. ######:: ##:::: ##:'####: ##::::::::");
        LOG_INFO("..:::::..:::.......::::::..:::::..:::::..::........::..:::::..:::......:::..:::::..::....::..:::::::::");
        LOG_INFO("______________________________________________________________________________________________________\n");
        LOG_INFO("Wait for program initialisation and Sigma-7 calibration before interacting with Sigma-7.");
    }

private:
    bool running_ = true;
    bool initialized_ = false;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<SharedData> SharedData_;
    std::shared_ptr<CSVLogger> CSVLogger_;
    std::shared_ptr<S7Mode> CurrentMode_;
    std::unique_ptr<S7Controller> Controller_;
    std::thread control_thread_;
    std::unique_ptr<S7StatePublisher> StatePublisher_;
    std::unique_ptr<S7ForceSubscriber> ForceSubscriber_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    // std::unique_ptr<S7InterfaceControlPublisher> ControlPublisher_;
    // std::unique_ptr<S7InterfaceControlsubscriber> ControlSubscriber_;
};