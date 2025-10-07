#include <chrono>
#include <memory>
#include <atomic>
#include <thread>
#include "drdc.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#pragma region subclasses & enums
/**
 * @enum S7Mode
 * @brief State Machine definitions for different modes used by the S7Mothership
 * & the S7Controller.
 */
enum S7Mode
{
    BRAKING,
    FREE,
    SELF_CENTERING
};

/**
 *
 */
class LockedPoseMessage
{
private:
    geometry_msgs::msg::PoseStamped::SharedPtr PoseStamped_;
    std::atomic<bool> Semaphore_{false};
    std::atomic<bool> JustChanged_{false};

public:
    LockedPoseMessage()
    {
        PoseStamped_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    };

    geometry_msgs::msg::PoseStamped::SharedPtr getMessage() { return PoseStamped_; }

    void setMessage(geometry_msgs::msg::PoseStamped::SharedPtr m) { *PoseStamped_ = *m; }

    void lock()
    {
        if (Semaphore_)
        {
            RCLCPP_INFO(rclcpp::get_logger("S7Controller"), "ressource locked\n");
        }
        while (Semaphore_)
        {
            // wait while ressource accessed
        }

        Semaphore_ = true;
    }

    void unlock()
    {
        if (!Semaphore_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("S7Controller"), "Ressource not locked\n");
            return;
        }

        Semaphore_ = false;
    }

    bool justChanged()
    {
        return JustChanged_;
    }

    void setJustChanged()
    {
        JustChanged_ = true;
    }

    void setNotJustChanged()
    {
        JustChanged_ = false;
    }
};

/**
 * @class S7StatePublisher
 * @brief ROS2 node that publishes the current state as a  (TODO)PoseStamped message.
 *
 * This class manages a publisher for the S7Mothership, taking care of publishing the state msgs.
 */
class S7StatePublisher
{
private:
    /**
     * @brief ROS2 publisher for (TODO)PoseStamped messages.
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    /**
     * @brief Current state stored as a shared pointer to (TODO)PoseStamped.
     */
    geometry_msgs::msg::PoseStamped::SharedPtr current_saved_state_;

    LockedPoseMessage *CurrentPose_;
    /**
     * @brief Mothership node passed as parameter in constructor
     */
    rclcpp::Node::SharedPtr node_ = nullptr;

public:
    /**
     * @brief Constructor for S7StatePublisher node.
     *
     * Initializes the ROS2 node with the name "state_publisher"
     * and creates a publisher on the topic "controller_pose_stamped"
     * , which is subscribed to by a component of S7Spaceship.
     */
    S7StatePublisher(const rclcpp::Node::SharedPtr &node, LockedPoseMessage *msg)
    {
        this->node_ = node;
        publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("controller_pose_stamped", 1);
        CurrentPose_ = msg;
    }

    /**
     * @brief Setter for current_saved_state_.
     */
    void setCurrentSavedState(geometry_msgs::msg::PoseStamped::SharedPtr newState) { current_saved_state_ = newState; }

    /**
     * @brief Publish the current state
     *
     * Publishes the current state (current_saved_state_) to controller_pose_stamped if not null
     *
     * @return 1 if published without fail, 0 if current_saved_state_ is null.
     */
    int8_t publish()
    {
        if ((*CurrentPose_).justChanged())
        {
            (*CurrentPose_).lock();
            publisher_->publish((*(*CurrentPose_).getMessage()));
            //RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "publishing");
            debugPublish();
            (*CurrentPose_).setNotJustChanged();
            (*CurrentPose_).unlock();
            return 1;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "posemsg unchanged");
            return 0;
        }
    }

    // TO BE USED ONLY IN A LOCKED CONTEXT
    void debugPublish()
    {
        RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "published pos(%f, %f, %f) rot(%f, %f, %f)",
                    (*(*CurrentPose_).getMessage()).pose.position.x,
                    (*(*CurrentPose_).getMessage()).pose.position.y,
                    (*(*CurrentPose_).getMessage()).pose.position.z,
                    (*(*CurrentPose_).getMessage()).pose.orientation.x,
                    (*(*CurrentPose_).getMessage()).pose.orientation.y,
                    (*(*CurrentPose_).getMessage()).pose.orientation.z);
    }
};

/**
 * @class S7ForceSubscriber
 * @brief ROS2 node that subscribes to controller_wrench_topic and recieves F.F. instructions from S7Spaceship
 *
 * This class manages a subscriber for the S7Mothership, taking care of storing the current wrench msg.
 */
class S7ForceSubscriber
{
private:
    /**
     * @brief ROS2 subscriber for WrenchStamped messages.
     */
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber_;

    /**
     * @brief latest recieved F.F. message from S7Spaceship
     */
    geometry_msgs::msg::WrenchStamped::SharedPtr current_saved_wrench_;
    /**
     * @brief Mothership node passed as parameter in constructor
     */
    rclcpp::Node::SharedPtr node_ = nullptr;

public:
    /**
     * @brief Constructor for S7ForceSubscriber node.
     *
     * Initializes the ROS2 node with the name "wrench_subscriber"
     * and creates a subscriber on the topic "controller_wrench_stamped"
     * containing F.F. instructions for the S7 arm, which is published
     * to by a component of S7Spaceship.
     */
    S7ForceSubscriber(const rclcpp::Node::SharedPtr &node)
    {
        this->node_ = node;
        subscriber_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "controller_wrench_stamped", 1,
            std::bind(&S7ForceSubscriber::messageCallback, this, std::placeholders::_1));
    }

    /**
     * @brief Getter for current_saved_wrench_
     *
     * @returns a geometry_msgs::msg::WrenchStamped::SharedPtr
     */
    geometry_msgs::msg::WrenchStamped::SharedPtr getCurrentSavedWrench()
    {
        if (current_saved_wrench_)
        {
            return current_saved_wrench_;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "current_saved_wrench_ null when getting");
            return current_saved_wrench_;
        }
    }

    /**
     * @brief Setter for current_saved_wrench_
     */
    void setCurrentSavedWrench(geometry_msgs::msg::WrenchStamped::SharedPtr newWrench) { current_saved_wrench_ = newWrench; }

private:
    /**
     * @brief Callback function for the subscriber, storing the message
     * in current_saved_wrench_
     */
    void messageCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr message)
    {
        if (message)
        {
            setCurrentSavedWrench(message);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "message in S7ForceSubscriber is null when setting current_saved_wrench_");
            setCurrentSavedWrench(message);
        }
    }
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

/**
 * @class S7Controller
 * @brief Object in charge of managing the control and data aquisition
 * loop of the Sigma7. Links using same-name functions in S7Mothership.
 *
 */
class S7Controller //: public rclcpp::node
{
private:
    /**
     * @brief neutral angles of the encoders, 8 values for the sigma 7
     */
    double positionCenter[DHD_MAX_DOF] = {};

    /**
     * @brief the current active mode of S7Controller.
     */
    S7Mode current_mode_;

    /**
     * @brief if the main loop is running
     */
    bool running_ = false;

    std::thread loop_thread;

    std::atomic<bool> *error;

    LockedPoseMessage *CurrentPose_;

public:
    /**
     * @brief constructor of S7Controller
     */
    S7Controller(std::atomic<bool> *e, LockedPoseMessage *pose)
    {
        error = e;
        CurrentPose_ = pose;
    }

    /**
     * @brief destructor of S7Controller, stops the control loop.
     */
    ~S7Controller() { stop(); }

    /**
     * @brief tells the control loop to start
     */
    int run()
    {
        //RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "running iteration of controller");
        return loop();
    }

    int initialize()
    {
        if (!hapticInit())
        {
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "failed to initialize Sigma7, make sure"
                                                              "device is on and connected, and Mothership"
                                                              "is launched with root privileges.");
            (*error) = true;
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
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "error: failed to close the connection (%s)", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }

        RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "S7 connexion closed.\n");
        return 1;
    }

private:
    /**
     * @brief control loop for the Sigma7
     *
     * Data is aquired here and shipped off to the S7Mothership,
     * FF is applied here also, all at a ~500Hz refresh rate
     */
    int loop()
    {
        // TODO, data aquisition
        if (savePose() == -1)
        {
            return -1;
        }

        // TODO, control section
        if (setForce() == -1)
        {
            return -1;
        }

        return 1;
    }

    int savePose()
    {
        //RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "saving pose");

        // Linear pos/vel
        double px = 0.0; // current positions
        double py = 0.0;
        double pz = 0.0;
        double vx = 0.0; // current velocities
        double vy = 0.0;
        double vz = 0.0;

        // Angular (Alpha Beta Gamma) rot/vel
        double oa = 0.0; // Angular rotations
        double ob = 0.0;
        double og = 0.0;
        double va = 0.0; // current velocities
        double vb = 0.0;
        double vg = 0.0;

        // Obtain all data, gets position twice to be able to retrieve velocity
        if (dhdGetPosition(&px, &py, &pz) == (DHD_ERROR_TIMEOUT | -1))
        {
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "failed to calculate position (%s)", dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetPosition(&px, &py, &pz) == (DHD_ERROR_TIMEOUT | -1))
        {
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "failed to calculate position (%s)", dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetOrientationRad(&oa, &ob, &og) == (DHD_ERROR_TIMEOUT | -1))
        {
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "failed to calculate angular orientation (%s)", dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetLinearVelocity(&vx, &vy, &vz) == (DHD_ERROR_TIMEOUT | -1))
        {
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "failed to calculate linear velocity (%s)", dhdErrorGetLastStr());
            return -1;
        }
        if (dhdGetAngularVelocityRad(&va, &vb, &vg) == (DHD_ERROR_TIMEOUT | -1))
        {
            RCLCPP_ERROR(rclcpp::get_logger("s7_mothership"), "failed to calculate angular velocity (%s)", dhdErrorGetLastStr());
            return -1;
        }

        // save the current pose in the semaphore locked pose msg
        (*CurrentPose_).lock();
        (*(*CurrentPose_).getMessage()).header.frame_id = "";

        (*(*CurrentPose_).getMessage()).pose.position.x = px;
        (*(*CurrentPose_).getMessage()).pose.position.y = py;
        (*(*CurrentPose_).getMessage()).pose.position.z = pz;

        (*(*CurrentPose_).getMessage()).pose.orientation.x = oa;
        (*(*CurrentPose_).getMessage()).pose.orientation.y = ob;
        (*(*CurrentPose_).getMessage()).pose.orientation.z = og;
        (*(*CurrentPose_).getMessage()).pose.orientation.w = 0; // not actually treated like a quaternion so we just ignore the last value
        (*CurrentPose_).setJustChanged();
        (*CurrentPose_).unlock();

        return 1;
    }

    int setForce()
    {
        // set the force

        return 1;
    }

    /**
     * @brief configuration function for the Sigma7, sets the encoders
     * and checks connexion before enabling FF on the arm.
     */
    int hapticInit()
    {
        if ((drdOpen() < 0))
        {
            RCLCPP_ERROR(rclcpp::get_logger("S7Controller"), "error: failed to open device (%s)", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }

        RCLCPP_INFO(rclcpp::get_logger("S7Controller"), "%s device detected.\n\n"
                                                        "WARNING, do NOT touch Sigma7 during caibration.\n\n",
                    dhdGetSystemName());

        if ((drdCheckInit() < 0))
        {
            RCLCPP_ERROR(rclcpp::get_logger("S7Controller"), "error: failed to reinitialize device (%s)", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("S7Controller"), "moving to :");
        for (int i = 0; i < DHD_MAX_DOF; ++i)
        {
            RCLCPP_INFO(rclcpp::get_logger("S7Controller"), "%f", positionCenter[i]);
        }
        RCLCPP_INFO(rclcpp::get_logger("S7Controller"), "\n");
        if (drdMoveTo(positionCenter) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("S7Controller"), "error: failed to move device (%s)", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        // Stop the regulation thread but leaves the forces enabled on the device.
        if (drdStop(true) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("S7Controller"), "error: failed to stop robotic regulation device (%s)", dhdErrorGetLastStr());
            dhdSleep(2.0);
            return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("S7Controller"), "Successfully initialised device : %s", dhdGetSystemName());
        return 1;
    }
};

#pragma endregion

class S7Mothership : public rclcpp::Node
{
private:
    // member objects
    std::unique_ptr<S7StatePublisher> StatePublisher_;
    std::unique_ptr<S7ForceSubscriber> ForceSubscriber_;
    std::unique_ptr<S7Controller> Controller_;
    std::unique_ptr<S7InterfaceControlPublisher> ControlPublisher_;
    std::unique_ptr<S7InterfaceControlsubscriber> ControlSubscriber_;
    S7Mode CurrentMode_;
    S7Mode RequestedMode_;
    std::thread loop_thread;

    std::atomic<bool> error{false};
    LockedPoseMessage CurrentPose_;
    bool running_ = true;

public:
    S7Mothership() : Node("s7_mothership") {}

    void init()
    {
        auto self = shared_from_this();

        StatePublisher_ = std::make_unique<S7StatePublisher>(self, &CurrentPose_);
        ForceSubscriber_ = std::make_unique<S7ForceSubscriber>(self);
        Controller_ = std::make_unique<S7Controller>(&error, &CurrentPose_);
        ControlPublisher_ = std::make_unique<S7InterfaceControlPublisher>(self);
        ControlSubscriber_ = std::make_unique<S7InterfaceControlsubscriber>(self);

        // start controller thread and main thread
        RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "Initializing connexion");
        Controller_->initialize();
        RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "Initializing main thread");
        loop_thread = std::thread(&S7Mothership::loop, this);
    }

    // bodged solution time
    bool getError()
    {
        return error;
    }

    void loop()
    {
        using namespace std::chrono;
        const auto period = microseconds(2000);

        while (running_)
        {
            auto start = steady_clock::now();

            Controller_->run();
            StatePublisher_->publish();

            // Exit condition
            if (dhdKbHit())
            {
                switch (dhdKbGet())
                {
                case 'q':
                {
                    running_ = false;
                    RCLCPP_INFO(rclcpp::get_logger("s7_mothership"), "Initiating Mothership shutdown");
                    break;
                }
                default:
                {
                    break;
                }
                }
            }

            auto elapsed = steady_clock::now() - start;
            if (elapsed < period)
            {
                std::this_thread::sleep_for(period - elapsed);
            }
            else
            {
                auto overrun = duration_cast<microseconds>(elapsed - period).count();
                RCLCPP_WARN(rclcpp::get_logger("s7_mothership"),
                            "main control loop too slow by %ld microseconds.", overrun);
            }
        }
    }
};
