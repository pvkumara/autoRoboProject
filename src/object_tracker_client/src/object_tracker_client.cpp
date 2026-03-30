#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <object_tracker_interfaces/action/track_object.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

using TrackObject = object_tracker_interfaces::action::TrackObject;
using GoalHandleTrackObject = rclcpp_action::ClientGoalHandle<TrackObject>;

// ─────────────────────────────────────────────────────────────────────────────

class ObjectTrackerClient : public rclcpp::Node
{
public:
    ObjectTrackerClient()
    : Node("object_tracker_client")
    {
        action_client_ = rclcpp_action::create_client<TrackObject>(this, "track_object");
    }

    // Blocking call: asks the user for input then sends goals in a loop.
    void run()
    {
        // Wait until the action server is available
        RCLCPP_INFO(get_logger(), "Waiting for action server 'track_object'...");
        while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(get_logger(), "Still waiting...");
        }
        RCLCPP_INFO(get_logger(), "Action server connected.");

        while (rclcpp::ok()) {
            std::string target = promptUser();
            if (!rclcpp::ok()) break;
            if (target.empty()) continue;

            sendGoal(target);

            // Block until this goal is finished (success, fail, cancel)
            std::unique_lock<std::mutex> lock(done_mutex_);
            done_cv_.wait(lock, [this]{ return goal_done_; });
            goal_done_ = false;
        }
    }

private:
    rclcpp_action::Client<TrackObject>::SharedPtr action_client_;

    std::mutex              done_mutex_;
    std::condition_variable done_cv_;
    bool                    goal_done_ = false;

    // ── User prompt ───────────────────────────────────────────────────────────
    static std::string promptUser()
    {
        std::cout << "\n╔══════════════════════════════════════════╗\n";
        std::cout <<   "║  Object Tracker — Enter COCO class name  ║\n";
        std::cout <<   "║  (press Enter for 'cell phone')          ║\n";
        std::cout <<   "╚══════════════════════════════════════════╝\n";
        std::cout << ">>> ";
        std::cout.flush();

        std::string input;
        if (!std::getline(std::cin, input)) {
            return "";  // EOF / pipe closed
        }
        if (input.empty()) {
            input = "cell phone";   // default: track the user's phone
            std::cout << "(defaulting to 'cell phone')\n";
        }
        return input;
    }

    // ── Send goal and wire up callbacks ──────────────────────────────────────
    void sendGoal(const std::string & object_class)
    {
        auto goal_msg = TrackObject::Goal();
        goal_msg.object_class = object_class;

        RCLCPP_INFO(get_logger(), "Sending goal: track '%s'", object_class.c_str());

        auto send_goal_options = rclcpp_action::Client<TrackObject>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this, object_class](const GoalHandleTrackObject::SharedPtr & handle) {
                if (!handle) {
                    RCLCPP_WARN(get_logger(),
                        "Goal REJECTED — '%s' is not a valid COCO class.",
                        object_class.c_str());
                    signalDone();
                } else {
                    RCLCPP_INFO(get_logger(),
                        "Goal ACCEPTED — tracking '%s'.", object_class.c_str());
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleTrackObject::SharedPtr /*handle*/,
                   const std::shared_ptr<const TrackObject::Feedback> feedback) {
                RCLCPP_INFO(get_logger(),
                    "[Feedback] %s  (cx=%.1f  depth=%.3f m)",
                    feedback->status.c_str(),
                    feedback->bbox_cx,
                    feedback->depth_distance);
            };

        send_goal_options.result_callback =
            [this](const GoalHandleTrackObject::WrappedResult & wrapped) {
                switch (wrapped.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "[Result] %s",
                                    wrapped.result->result.c_str());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(get_logger(), "[Result] Goal aborted: %s",
                                    wrapped.result->result.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(get_logger(), "[Result] Goal cancelled.");
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "[Result] Unknown result code.");
                        break;
                }
                signalDone();
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void signalDone()
    {
        std::lock_guard<std::mutex> lock(done_mutex_);
        goal_done_ = true;
        done_cv_.notify_all();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<ObjectTrackerClient>();

    // Spin ROS callbacks in a background thread so the main thread can
    // block on user input (std::getline) without starving the executor.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(client_node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    client_node->run();

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
