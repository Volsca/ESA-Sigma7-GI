#include <chrono>
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include <ctime>
#include <vector>
#include <mutex>
// #include <algorithm>
#include "drdc.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// Text color macros
#define COLOR_BLACK "\033[30m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"
#define COLOR_RESET "\033[0m"

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
// using PoseStamped = geometry_msgs::msg::PoseStamped;
/// Alias for geometry_msgs::msg::TwistStamped
// using TwistStamped = geometry_msgs::msg::TwistStamped;
/// Alias for nav_msgs::msg::Odometry;
using Odometry = nav_msgs::msg::Odometry;
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
    uint64_t time1; // Convert from header.time with : rclcpp::Time ros_time(msg.header.stamp);
    uint64_t time2;
    // time_t time3;
};
/// Struct for logging operation time between retrieving data, publishing, and controlling
struct OperationTime
{
    uint16_t id;
    std::chrono::microseconds startTime;
    std::chrono::microseconds saveTime;
    std::chrono::microseconds pubTime;
    std::chrono::microseconds ctrlTime;
    std::chrono::microseconds endTime;
    /*int64_t startTime;
    int64_t saveTime;
    int64_t pubTime;
    int64_t ctrlTime;
    int64_t endTime;*/
    std::string late;
};
/// Struct for logging avg data lock time of the SharedData class
struct LockTime
{
    std::string data;
    std::string lockSource;
    uint64_t lockTime;
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
    FORCEFEEDBACK
};

/**
 * @class SharedData
 * @brief RAII Class managing data locking between each class using the pose messages m
 */
class SharedData
{
private:
    std::mutex odometry_mutex;
    std::unique_ptr<Odometry> CurrentOdometry;

    std::mutex wrench_mutex;
    std::unique_ptr<WrenchStamped> CurrentWrench;

public:
    // Constructor
    // no need to lock, as the object isn't shared yet
    explicit SharedData()
        : CurrentOdometry(std::make_unique<Odometry>()),
          CurrentWrench(std::make_unique<WrenchStamped>())
    {
    }

    std::shared_ptr<Odometry> getCurrentOdometry()
    {
        std::lock_guard<std::mutex> lock_odometry(odometry_mutex);
        auto tmp = std::make_shared<Odometry>(*CurrentOdometry);
        return tmp;
    }

    void setCurrentOdometry(Odometry &newOdometry)
    {
        std::lock_guard<std::mutex> lock_odometry(odometry_mutex);
        *CurrentOdometry = newOdometry;
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
          otFile("operation_time_log.csv", std::ios::out),
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

        if (!otFile.is_open())
        {
            LOG_ERROR("Failed to open file : operation_time_log");
        }
        else
        {
            otFile << "id_number, start_time, save_time, publish_time, control_time, end_time, status\n";
        }
    }

    /**
     * @brief Call to write pose time to logger buffer
     *
     * needs to be called before a new pose check is called, to not skip messages.
     */
    void logPose(u_int id, uint64_t time)
    {
        // write to latest vector only the pose message time setting corresponding time2 to 0 untill sub message arrives
        ResponseTime R = {id, time, 0};
        rtBuffer.push_back(R);
    }

    /**
     * @brief call to write the latest wrench to its corresponding ID in the buffer
     *
     * write to correct spot in the vector
     * doesn't check if the spot exists in the vector (ideally doesn't need to as a force recieved should always have the same ID as a pose already logged)
     * And also I'm lazy af
     *
     * Also update the average response time, max response time and min response time
     */
    void logWrench(u_int id, uint64_t time)
    {

        if (id < rtBuffer.size())
        {
            rtBuffer[id].time2 = time;

            uint16_t rt = time - rtBuffer[id].time1;
            // Update integral and counter
            integral_mutex.lock();
            rtIntegral += rt;
            rtCounter++;
            integral_mutex.unlock();

            // Update max and min
            limits_mutex.lock();
            maxrt = std::max(maxrt, rt);
            minrt = std::min(minrt, rt);
            limits_mutex.unlock();
        }
        else
        {
            LOG_ERROR("Tried to log wrench when insufficent space");
        }
    }

    /**
     * @brief Retrieves the average response time stored in CSVLogger
     */
    void getAvgResponseTime(uint64_t &avgrt)
    {
        std::lock_guard<std::mutex> lock_integrals(integral_mutex);
        if (rtCounter != 0)
        {
            avgrt = (rtIntegral / rtCounter);
        }
        else
        {
            avgrt = 0;
        }
    }

    void getMinMaxResponseTime(uint16_t &min, uint16_t &max)
    {
        std::lock_guard<std::mutex> lock_minmax(limits_mutex);
        min = minrt;
        max = maxrt;
    }

    // TODO, rework logic, is very adhoc, but I'm lazy
    void logOT(uint16_t &counter, std::chrono::microseconds &t1, std::chrono::microseconds &t2, std::chrono::microseconds &t3, std::chrono::microseconds &t4, std::chrono::microseconds &t5, std::string &l8)
    {
        OperationTime ot = {counter, t1, t2, t3, t4, t5, l8};
        otBuffer.push_back(ot);
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

private:
    /**
     * @brief writes all of rtBuffer to csv file
     */
    void writeResponseTime()
    {
        // First print average response time calculated at runtime
        rtFile << "," << "," << "," << (rtIntegral / rtCounter) << "\n";

        // Print the rest of the file
        for (std::vector<ResponseTime>::iterator it = rtBuffer.begin(); it != rtBuffer.end(); ++it)
        {
            rtFile << it->id << "," << it->time1 << "," << it->time2 << "\n";
        }
        rtFile.flush();
        LOG_INFO("CSV file for response time written & flushed");
    }

    // todo
    void writeOperationTime()
    {
        for (std::vector<OperationTime>::iterator it = otBuffer.begin(); it != otBuffer.end(); ++it)
        {
            otFile << it->id << "," << it->startTime.count() << "," << it->saveTime.count() << "," << it->pubTime.count() << "," << it->ctrlTime.count() << "," << it->endTime.count() << "," << it->late << "\n";
        }
        otFile.flush();
        LOG_INFO("CSV file for operation time written & flushed");
    }

    // todo
    void writeLockTime()
    {
    }

private:
    std::ofstream rtFile;
    std::ofstream otFile;
    std::vector<ResponseTime> rtBuffer;
    std::vector<OperationTime> otBuffer;

    // Calculate average response time at runtime
    // as well as max and min rt
    // Variables are thread safe
    std::mutex integral_mutex;
    uint64_t rtIntegral = 0;
    uint32_t rtCounter = 0;
    std::mutex limits_mutex;
    uint16_t maxrt = 0;
    uint16_t minrt = 10000; // Absurd value to start comparison
    std::shared_ptr<SharedData> SharedData_;
};

/**
 * @class S7Controller
 * @brief Object in charge of managing the control and data aquisition
 * loop of the Sigma7. Links using same-name functions in S7Mothership.
 *
 * Takes care of filtering sigma7 movement through derivative control
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
            auto tmpWrench = std::make_shared<WrenchStamped>(*SharedData_->getCurrentWrench());
            auto tmpOdometry = std::make_shared<Odometry>(*SharedData_->getCurrentOdometry());

            // Apply derivative control to dampen movements
            // Do not touch the values without proper understanding of their effects
            float dx = kd * (0.0 - (*tmpOdometry).twist.twist.linear.x);
            float dy = kd * (0.0 - (*tmpOdometry).twist.twist.linear.y);
            float dz = kd * (0.0 - (*tmpOdometry).twist.twist.linear.z);
            float da = kda * (0.0 - (*tmpOdometry).twist.twist.angular.x);
            float db = kdb * (0.0 - (*tmpOdometry).twist.twist.angular.y);
            float dg = kdg * (0.0 - (*tmpOdometry).twist.twist.angular.z);

            if (dhdSetForceAndTorqueAndGripperForce((*tmpWrench).wrench.force.x + dx, (*tmpWrench).wrench.force.y + dy, (*tmpWrench).wrench.force.z + dz,
                                                    (*tmpWrench).wrench.torque.x + da, (*tmpWrench).wrench.torque.y + db, (*tmpWrench).wrench.torque.z + dg, 1) == (DHD_MOTOR_SATURATED | -1))
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
        Odometry savedOdometry;
        rclcpp::Time t = rclcpp::Clock().now();
        savedOdometry.header.stamp = t;
        savedOdometry.header.frame_id = std::to_string(frame_id_); // set frame_id

        // Obtain all pose data, gets position twice to be able to retrieve velocity
        if (dhdGetPosition(NULL, NULL, NULL) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate position");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetOrientationRad(&savedOdometry.pose.pose.orientation.x, &savedOdometry.pose.pose.orientation.y, &savedOdometry.pose.pose.orientation.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate angular orientation");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        // Obtain twist data
        if (dhdGetPosition(&savedOdometry.pose.pose.position.x, &savedOdometry.pose.pose.position.y, &savedOdometry.pose.pose.position.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate position ");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetLinearVelocity(&savedOdometry.twist.twist.linear.x, &savedOdometry.twist.twist.linear.y, &savedOdometry.twist.twist.linear.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate linear velocity");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetAngularVelocityRad(&savedOdometry.twist.twist.angular.x, &savedOdometry.twist.twist.angular.y, &savedOdometry.twist.twist.angular.z) == (DHD_ERROR_TIMEOUT | -1))
        {
            LOG_ERROR("failed to calculate angular velocity");
            LOG_ERROR(dhdErrorGetLastStr());
            return -1;
        }

        SharedData_->setCurrentOdometry(savedOdometry);

// Log relevant data (before publish to avoid sub messages arriving in the logs too early)
#ifdef DEBUG_RESPONSE_TIME
        // log the pose (remember the static cast here)
        CSVLogger_->logPose(std::abs(frame_id_), static_cast<uint64_t>(t.nanoseconds() / 1000)); // the std::abs() is to
#endif

        // Increment frame_id
        frame_id_++;
        return 1;
    }

    int initialise()
    {
        if (!hapticInit())
        {
            LOG_ERROR("failed to initialise Sigma7, make sure device is on and connected, and Mothership is launched with root privileges.");
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
        std::cout << COLOR_RED << "       WARNING, do NOT touch Sigma7 during caibration.\n\n"
                  << COLOR_RESET;

        if ((drdCheckInit() < 0))
        {
            LOG_ERROR("error: failed to reinitialise device (%s)");
            LOG_ERROR(dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        LOG_INFO("moving to center");
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
        std::cout << COLOR_GREEN << "       Successfully initialised device\n"
                  << COLOR_RESET;
        return 1;
    }

private:
    std::shared_ptr<CSVLogger> CSVLogger_;
    bool running_ = false;
    double positionCenter[DHD_MAX_DOF] = {};
    std::shared_ptr<SharedData> SharedData_;
    std::shared_ptr<S7Mode> CurrentMode_;
    int frame_id_ = 0;
    // Derivative control terms (DO NOT TOUCH WITHOUT PROPER UNDERSTAND)
    float kd = 20;
    float kda = 0.04;
    float kdb = 0.05;
    float kdg = 0.02;
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
     * initialises the ROS2 node with the name "state_publisher"
     * and creates a publisher on the topic "controller_pose_stamped"
     * , which is subscribed to by a component of S7Spaceship.
     */
    S7StatePublisher(std::shared_ptr<SharedData> sharedData, std::shared_ptr<rclcpp::Node> node)
        : SharedData_(sharedData),
          node_(node)
    {
        publisher_ = node_->create_publisher<Odometry>("controller_odometry_topic", 10);
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
        publisher_->publish(*SharedData_->getCurrentOdometry());
    }

private:
    std::shared_ptr<SharedData> SharedData_;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::Publisher<Odometry>> publisher_;
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
     * initialises the ROS2 node with the name "wrench_subscriber"
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

        CSVLogger_->logWrench(std::abs(idValue), static_cast<uint64_t>(t.nanoseconds() / 1000));
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
public:
    S7InterfaceControlPublisher(const rclcpp::Node::SharedPtr &node) { node_ = node; }

private:
    rclcpp::Node::SharedPtr node_ = nullptr;
};

/**
 * @class TODO
 */
class S7InterfaceControlsubscriber
{
public:
    S7InterfaceControlsubscriber(const rclcpp::Node::SharedPtr &node) { node_ = node; }

private:
    rclcpp::Node::SharedPtr node_ = nullptr;
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
        showInstructionScreen();

        (*CurrentMode_) = S7Mode::FREE;

        // Controller initialization
        control_thread_ = std::thread(&S7Mothership::initThread, this);

        std::cout << COLOR_GREEN << "       Initializing main thread\n"
                  << COLOR_RESET;
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

    ~S7Mothership()
    {
        std::cout << COLOR_GREEN << "Code exiting, see you next time!\n"
                  << COLOR_RESET;
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
        if (running_)
        {
#ifdef DEBUG_OPERATION_TIME // -------- Debug procedure

            // Timing control & timing
            using namespace std::chrono;
            const auto period = microseconds(2000);
            bool l8 = false;
            auto start = steady_clock::now();
            microseconds startTime = duration_cast<microseconds>(steady_clock::now().time_since_epoch());
            microseconds saveTime;
            microseconds pubTime;
            microseconds ctrlTime;
            microseconds endTime;

            if (initialised_)
            {
                saveTime = duration_cast<microseconds>(steady_clock::now().time_since_epoch());
                Controller_->savePose();

                pubTime = duration_cast<microseconds>(steady_clock::now().time_since_epoch());
                StatePublisher_->publish();

                ctrlTime = duration_cast<microseconds>(steady_clock::now().time_since_epoch());
                Controller_->setForce();
            }

            // Input Management step
            getUserInput();

            // Update status monitor at the top of the terminal
            showStatus();

            // Timing control
            auto elapsed = steady_clock::now() - start;
            if (!(elapsed < period))
            {
                // auto overrun = duration_cast<microseconds>(elapsed - period).count();
                LOG_WARN("control thread late");
                l8 = true;
            }

            // Increase counter and log end of loop
            // Only log if initialised, otherwise makes no sense.
            endTime = duration_cast<microseconds>(steady_clock::now().time_since_epoch());
            if (initialised_)
            {
                if (l8)
                {
                    std::string late = "late";
                    CSVLogger_->logOT(count, startTime, saveTime, pubTime, ctrlTime, endTime, late);
                }
                else
                {
                    std::string nlate = "";
                    CSVLogger_->logOT(count, startTime, saveTime, pubTime, ctrlTime, endTime, nlate);
                }
                count++;
            }

#else // ------------------------------ Normal procedure
      // Timing control
            using namespace std::chrono;
            const auto period = microseconds(2000);
            auto start = steady_clock::now();

            if (initialised_)
            {
                Controller_->savePose();
                StatePublisher_->publish();
                Controller_->setForce();
            }

            // Input Management step
            getUserInput();

            // Update status monitor at the top of the terminal
            showStatus();

            // Timing control
            auto elapsed = steady_clock::now() - start;
            if (!(elapsed < period))
            {
                // auto overrun = duration_cast<microseconds>(elapsed - period).count();
                LOG_WARN("control thread late");
            }

#endif
        }
        // deal with shutdown
        else
        {
            Controller_->stop();
            loop_timer_->cancel();
            CSVLogger_->flush();
            // edge case
            while (!initialised_)
            {
                // just wait bro
                std::this_thread::sleep_for(100ms);
            }
            rclcpp::shutdown();
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
                std::cout << "      Exit called\n";
                running_ = false;
                std::cout << COLOR_RED << "Do not ctrl+c, wait for code to exit normally.\n"
                          << COLOR_RESET;
                break;
            }
            case 'm':
            {
                if ((*CurrentMode_) == S7Mode::FREE)
                {
                    (*CurrentMode_) = S7Mode::FORCEFEEDBACK;
                    // std::cout << "      Force-Feedback activated.\n";
                }
                else
                {
                    (*CurrentMode_) = S7Mode::FREE;
                    // std::cout << "      Force-Feedback deactivated.\n";
                }
                break;
            }
            case 'r':
                showrtvalues = !showrtvalues;
#ifdef DEBUG_RESPONSE_TIME
                if (showrtvalues)
                {
                    // std::cout << "       Response time values shown.\n";
                }
                else
                {
                    // std::cout << "       Response time values hidden.\n";
                }
#else
                std::cout << COLOR_RED << "       Response Time logging not turned on." << COLOR_RESET << "\n";
#endif
                break;
            default:
            {
                std::cout << "       Unrecognised input.\n";
                break;
            }
            }
            std::cout << "\n";
        }
    }

    /**
     *
     */
    void initThread()
    {
        std::cout << COLOR_GREEN << "       Initializing connexion\n"
                  << COLOR_RESET;
        if (Controller_->initialise())
        {
            initialised_ = true;
        }
        else
        {
            LOG_ERROR("failed to initialise controller, shutting down");
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
        std::cout << "       Sigma-7 interface mothership, written by Jacob Wallace & Emre Artar - 2025\n";
        std::cout << "       ______________________________________________________________________________________________________\n";
        std::cout << "       '##::::'##::'#######::'########:'##::::'##:'########:'########:::'######::'##::::'##:'####:'########::\n";
        std::cout << "        ###::'###:'##.... ##:... ##..:: ##:::: ##: ##.....:: ##.... ##:'##... ##: ##:::: ##:. ##:: ##.... ##:\n";
        std::cout << "        ####'####: ##:::: ##:::: ##:::: ##:::: ##: ##::::::: ##:::: ##: ##:::..:: ##:::: ##:: ##:: ##:::: ##:\n";
        std::cout << "        ## ### ##: ##:::: ##:::: ##:::: #########: ######::: ########::. ######:: #########:: ##:: ########::\n";
        std::cout << "        ##. #: ##: ##:::: ##:::: ##:::: ##.... ##: ##...:::: ##.. ##::::..... ##: ##.... ##:: ##:: ##.....:::\n";
        std::cout << "        ##:.:: ##: ##:::: ##:::: ##:::: ##:::: ##: ##::::::: ##::. ##::'##::: ##: ##:::: ##:: ##:: ##::::::::\n";
        std::cout << "        ##:::: ##:. #######::::: ##:::: ##:::: ##: ########: ##:::. ##:. ######:: ##:::: ##:'####: ##::::::::\n";
        std::cout << "       ..:::::..:::.......::::::..:::::..:::::..::........::..:::::..:::......:::..:::::..::....::..:::::::::\n";
        std::cout << "       ______________________________________________________________________________________________________\n\n";
        std::cout << "       Mothership version 1.13.0, for Linux Ubuntu 24.04 using ROS2-Jazzy (2025-12-09)\n";
        std::cout << "           Wait for Mothership initialisation before starting Spaceship.\n";
        std::cout << "           Wait for program initialisation and Sigma-7 calibration before interacting with Sigma-7.\n";
    }

    void showInstructionScreen()
    {
        std::cout << "\n";
        std::cout << "        --- Control legend ---\n";
        std::cout << COLOR_CYAN << "       Toggle Force-Feedback" << COLOR_RESET << " : m\n";
        std::cout << COLOR_WHITE << "       Show response time   " << COLOR_RESET << " : r | (only works while logging response time, and misaligns display)\n";
        std::cout << COLOR_RED << "       EXIT                 " << COLOR_RESET << " : q\n";
        std::cout << "\n";
    }

    void showStatus()
    {
        struct winsize w;
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
        int lastRow = w.ws_row;
        // int lastCol = w.ws_col;

        // Define values regardless of DEBUG_RESPONSE_TIME
        // as otherwise they are out of scope
        uint64_t avgrt = 0;
        uint16_t minrt = 0;
        uint16_t maxrt = 0;

        // ffStatus
        std::string ffStatus;
        if ((*CurrentMode_) == FORCEFEEDBACK)
        {
            ffStatus = "activated";
        }
        else
        {
            ffStatus = "deactivated";
        }

        // rtMonitorStatus & rt values
        std::string rtMonitorStatus = "response time logging deactivated";
#ifdef DEBUG_RESPONSE_TIME
        if (showrtvalues)
        {
            rtMonitorStatus = "activated";

            CSVLogger_->getAvgResponseTime(avgrt);
            CSVLogger_->getMinMaxResponseTime(minrt, maxrt);
        }
        else
        {
            rtMonitorStatus = "deactivated";
        }
#endif

        // Print status
        std::cout << "\033[1;1H\033[2K" << COLOR_MAGENTA;
        std::cout << "       Force-Feedback : " << ffStatus << " | Response time monitoring : " << rtMonitorStatus;
#ifdef DEBUG_RESPONSE_TIME
        if (showrtvalues)
        {
            std::cout << "       Response times | Average : " << avgrt << " | Max : " << maxrt << " | Min : " << minrt << "\n";
            std::cout << "       _________________________________________________________________________________";
        }
#endif
        std::cout << COLOR_RESET << "\033[" << lastRow << ";1H";
    }

private:
    bool running_ = true;
    bool initialised_ = false;
    bool showrtvalues = false;
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<SharedData> SharedData_;
    uint16_t count = 0;
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