#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <object_tracker_interfaces/action/track_object.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

using TrackObject = object_tracker_interfaces::action::TrackObject;
using GoalHandleTrackObject = rclcpp_action::ServerGoalHandle<TrackObject>;

// ── Camera / tracking constants ──────────────────────────────────────────────
static constexpr float  IMAGE_W        = 640.0f;
static constexpr float  IMAGE_H        = 480.0f;
static constexpr float  CENTER_X       = IMAGE_W / 2.0f;   // 320
static constexpr float  CENTER_Y       = IMAGE_H / 2.0f;   // 240
static constexpr float  CENTER_TOL     = 30.0f;            // ±30 px counts as centred
static constexpr float  SUCCESS_DIST_M = 0.20f;            // 20 cm → tracking success

// Number of consecutive 10 Hz ticks the object can be absent before we give up.
// 15 ticks = 1.5 s of grace time for re-searching after a momentary loss.
static constexpr size_t MAX_LOST_FRAMES = 15;

// ── Motor control constants ───────────────────────────────────────────────────
static constexpr float MAX_LINEAR_VEL  = 0.3f;   // m/s  (hard limit: 0.5 per assignment)
static constexpr float MAX_ANGULAR_VEL = 1.0f;   // rad/s
static constexpr float FORWARD_GAIN    = 0.5f;   // linear_x = dep_dist * gain
static constexpr float SEARCH_ANGULAR  = 0.5f;   // slow spin while searching

// All 80 COCO class names (index 0-79)
static const std::array<std::string, 80> COCO_CLASSES = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
};

static bool isValidCocoClass(const std::string & name)
{
    for (const auto & c : COCO_CLASSES) {
        if (c == name) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────

class ObjectTrackerServer : public rclcpp::Node
{
public:
    ObjectTrackerServer()
    : Node("object_tracker_server")
    {
        // Action server
        action_server_ = rclcpp_action::create_server<TrackObject>(
            this,
            "track_object",
            std::bind(&ObjectTrackerServer::handleGoal,    this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ObjectTrackerServer::handleCancel,  this, std::placeholders::_1),
            std::bind(&ObjectTrackerServer::handleAccepted,this, std::placeholders::_1));

        // Subscribe to YOLOv8 detections (Isaac ROS YoloV8DecoderNode output)
        detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
            "/detections_output", 10,
            std::bind(&ObjectTrackerServer::detectionCallback, this, std::placeholders::_1));

        // Subscribe to aligned depth image (RealSense D435i).
        // isaac_ros_examples realsense_mono_rect_depth fragment publishes here
        // (no /camera/ prefix — differs from raw realsense2_camera_node output).
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/aligned_depth_to_color/image_raw", 10,
            std::bind(&ObjectTrackerServer::depthCallback, this, std::placeholders::_1));

        // Publish velocity commands for Part 2 motor control
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(get_logger(), "Object Tracker Action Server ready. "
                    "Default target: 'cell phone' (your phone).");
    }

private:
    // ── Action server handles ─────────────────────────────────────────────────
    rclcpp_action::Server<TrackObject>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & /*uuid*/,
        std::shared_ptr<const TrackObject::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received goal: track '%s'",
                    goal->object_class.c_str());

        if (!isValidCocoClass(goal->object_class)) {
            RCLCPP_WARN(get_logger(),
                        "Rejecting: '%s' is not a COCO class.",
                        goal->object_class.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleTrackObject> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Goal cancel requested.");
        stopRobot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleTrackObject> goal_handle)
    {
        // Run execution in a separate thread to avoid blocking the executor
        std::thread{std::bind(&ObjectTrackerServer::execute, this,
                               std::placeholders::_1), goal_handle}.detach();
    }

    // ── Main tracking execution loop ──────────────────────────────────────────
    void execute(const std::shared_ptr<GoalHandleTrackObject> goal_handle)
    {
        const std::string target = goal_handle->get_goal()->object_class;
        RCLCPP_INFO(get_logger(), "Executing: tracking '%s'", target.c_str());

        // Tell the detection callback to filter for this class only.
        {
            std::lock_guard<std::mutex> lock(detection_mutex_);
            current_target_ = target;
        }

        auto feedback = std::make_shared<TrackObject::Feedback>();
        auto result   = std::make_shared<TrackObject::Result>();

        bool   tracking_started = false;
        size_t lost_frames      = 0;   // consecutive ticks without the target

        rclcpp::Rate rate(10);  // 10 Hz control loop

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->result = "Tracking Cancelled.";
                goal_handle->canceled(result);
                stopRobot();
                clearTarget();
                return;
            }

            // Snapshot current detection state under lock
            float cx, cy, depth;
            bool detected;
            {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                cx       = last_bbox_cx_;
                cy       = last_bbox_cy_;
                depth    = last_depth_m_;
                detected = last_class_ == target;
            }

            if (!detected) {
                ++lost_frames;

                // Give the object MAX_LOST_FRAMES ticks to reappear before giving up.
                if (tracking_started && lost_frames > MAX_LOST_FRAMES) {
                    RCLCPP_WARN(get_logger(),
                                "Object '%s' lost for %.1f s — giving up.",
                                target.c_str(),
                                static_cast<double>(lost_frames) / 10.0);
                    feedback->status         = "Tracking Failed: object lost.";
                    feedback->bbox_cx        = 0;
                    feedback->bbox_cy        = 0;
                    feedback->depth_distance = 0;
                    goal_handle->publish_feedback(feedback);
                    stopRobot();
                    clearTarget();
                    result->result = "Tracking Failed.";
                    goal_handle->succeed(result);
                    return;
                }

                if (tracking_started) {
                    // Briefly lost — spin and try to re-acquire
                    feedback->status = "Re-searching... (" +
                                       std::to_string(lost_frames) + "/" +
                                       std::to_string(MAX_LOST_FRAMES) + ")";
                } else {
                    feedback->status = "Searching.";
                }
                feedback->bbox_cx        = 0;
                feedback->bbox_cy        = 0;
                feedback->depth_distance = 0;
                goal_handle->publish_feedback(feedback);

                // Spin slowly while searching / re-searching
                publishSearchSpin();

            } else {
                // Object detected ─────────────────────────────────────────────
                lost_frames      = 0;
                tracking_started = true;

                std::string position = computePositionLabel(cx, cy);
                feedback->status         = position;
                feedback->bbox_cx        = cx;
                feedback->bbox_cy        = cy;
                feedback->depth_distance = depth;
                goal_handle->publish_feedback(feedback);

                RCLCPP_INFO(get_logger(),
                            "Tracking '%s' | pos=%s  cx=%.1f cy=%.1f depth=%.3fm",
                            target.c_str(), position.c_str(), cx, cy, depth);

                // Check success condition
                float dx = std::abs(cx - CENTER_X);
                if (dx <= CENTER_TOL && depth > 0.0f && depth <= SUCCESS_DIST_M) {
                    stopRobot();
                    RCLCPP_INFO(get_logger(), "Tracking Successful! '%s' is centred "
                                "and %.2f m away.", target.c_str(), depth);
                    result->result = "Tracking Successful!";
                    clearTarget();
                    goal_handle->succeed(result);
                    return;
                }

                // Part 2: drive toward the object
                publishTrackingVelocity(cx, depth);
            }

            rate.sleep();
        }

        stopRobot();
        clearTarget();
        result->result = "Tracking Failed.";
        goal_handle->abort(result);
    }

    // ── Position label helper ─────────────────────────────────────────────────
    std::string computePositionLabel(float cx, float cy) const
    {
        bool left   = cx < CENTER_X - CENTER_TOL;
        bool right  = cx > CENTER_X + CENTER_TOL;
        bool top    = cy < CENTER_Y - CENTER_TOL;
        bool bottom = cy > CENTER_Y + CENTER_TOL;

        if (top    && left)  return "top left";
        if (top    && right) return "top right";
        if (bottom && left)  return "bottom left";
        if (bottom && right) return "bottom right";
        if (top)             return "top";
        if (bottom)          return "bottom";
        if (left)            return "left";
        if (right)           return "right";
        return "center";
    }

    // ── Motor command helpers (Part 2) ────────────────────────────────────────
    void publishTrackingVelocity(float cx, float depth)
    {
        geometry_msgs::msg::Twist cmd;

        // Angular: turn toward the object. Object right → negative angular (turn right).
        float error = (cx - CENTER_X) / CENTER_X;  // normalised [-1, +1]
        cmd.angular.z = -error * MAX_ANGULAR_VEL;

        // Linear: proportional to distance, only advance when roughly centred.
        float dx_norm = std::abs(cx - CENTER_X) / CENTER_X;
        if (depth > SUCCESS_DIST_M && dx_norm < 0.3f) {
            cmd.linear.x = std::min(depth * FORWARD_GAIN, static_cast<float>(MAX_LINEAR_VEL));
        } else {
            cmd.linear.x = 0.0f;
        }

        cmd_vel_pub_->publish(cmd);
    }

    void publishSearchSpin()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = SEARCH_ANGULAR;
        cmd_vel_pub_->publish(cmd);
    }

    void stopRobot()
    {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    }

    void clearTarget()
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        current_target_ = "";
    }

    // ── Class-id resolver ─────────────────────────────────────────────────────
    // Isaac ROS YOLOv8 decoder publishes numeric class_id strings ("67").
    // Convert to the COCO name ("cell phone") so goal matching works.
    static std::string resolveClassName(const std::string & raw_id)
    {
        // Try numeric index first
        try {
            size_t idx = static_cast<size_t>(std::stoul(raw_id));
            if (idx < COCO_CLASSES.size()) {
                return COCO_CLASSES[idx];
            }
        } catch (...) {}

        // Already a name (some decoder versions output the name directly)
        return raw_id;
    }

    // ── Detection callback ────────────────────────────────────────────────────
    void detectionCallback(
        const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);

        // Find the highest-confidence detection that matches the current target.
        // Ignoring higher-confidence detections of other classes ensures a person
        // walking by at 0.9 confidence doesn't cause us to "lose" a phone at 0.6.
        last_class_ = "";
        last_bbox_cx_ = 0.0f;
        last_bbox_cy_ = 0.0f;
        float best_score = 0.0f;

        for (const auto & det : msg->detections) {
            for (const auto & hyp : det.results) {
                std::string name = resolveClassName(hyp.hypothesis.class_id);
                // Only consider detections that match what we're tracking.
                // If no goal is active (current_target_ empty), accept anything.
                bool matches = current_target_.empty() || (name == current_target_);
                if (matches && hyp.hypothesis.score > best_score) {
                    best_score    = hyp.hypothesis.score;
                    last_class_   = name;
                    last_bbox_cx_ = static_cast<float>(det.bbox.center.position.x);
                    last_bbox_cy_ = static_cast<float>(det.bbox.center.position.y);
                }
            }
        }

        // Fetch depth at bbox centre from the latest depth image (16UC1, mm).
        if (!last_class_.empty() && !last_depth_image_.empty()) {
            int u = std::clamp(static_cast<int>(last_bbox_cx_),
                               0, static_cast<int>(IMAGE_W) - 1);
            int v = std::clamp(static_cast<int>(last_bbox_cy_),
                               0, static_cast<int>(IMAGE_H) - 1);

            // Helper: safely read one depth pixel using the cached step.
            auto readDepth = [&](int pu, int pv) -> uint16_t {
                if (pu < 0 || pu >= static_cast<int>(last_depth_step_px_) ||
                    pv < 0 || pv >= static_cast<int>(last_depth_height_px_)) {
                    return 0;
                }
                int pidx = pv * static_cast<int>(last_depth_step_px_) + pu;
                if (pidx < 0 || pidx >= static_cast<int>(last_depth_image_.size())) {
                    return 0;
                }
                return last_depth_image_[pidx];
            };

            uint16_t raw_mm = readDepth(u, v);

            // Phones and reflective surfaces often return 0 at the exact centre.
            // Average valid pixels in a 9×9 neighbourhood as a fallback.
            if (raw_mm == 0) {
                uint32_t sum   = 0;
                int      count = 0;
                for (int dy = -4; dy <= 4; ++dy) {
                    for (int dx = -4; dx <= 4; ++dx) {
                        uint16_t d = readDepth(u + dx, v + dy);
                        if (d > 0) { sum += d; ++count; }
                    }
                }
                raw_mm = (count > 0)
                    ? static_cast<uint16_t>(sum / static_cast<uint32_t>(count))
                    : 0;
            }

            last_depth_m_ = static_cast<float>(raw_mm) / 1000.0f;

            if (raw_mm == 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Depth is 0 at (%d,%d) — surface may be IR-reflective "
                    "or depth stream not aligned.", u, v);
            }
        }
    }

    // ── Depth image callback ──────────────────────────────────────────────────
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        // step is bytes per row; divide by 2 to get uint16_t elements per row.
        last_depth_step_px_   = (msg->step > 0)
            ? msg->step / static_cast<uint32_t>(sizeof(uint16_t))
            : msg->width;
        last_depth_height_px_ = msg->height;
        const uint16_t * ptr  = reinterpret_cast<const uint16_t *>(msg->data.data());
        size_t n = last_depth_step_px_ * msg->height;
        last_depth_image_.assign(ptr, ptr + n);
    }

    // ── Subscribers / publishers ──────────────────────────────────────────────
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            depth_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_vel_pub_;

    // ── Shared state (guarded by detection_mutex_) ────────────────────────────
    std::mutex              detection_mutex_;
    std::string             current_target_;          // class the active goal wants; "" = no filter
    std::string             last_class_;
    float                   last_bbox_cx_       = 0.0f;
    float                   last_bbox_cy_       = 0.0f;
    float                   last_depth_m_       = 0.0f;
    std::vector<uint16_t>   last_depth_image_;
    uint32_t                last_depth_step_px_   = static_cast<uint32_t>(IMAGE_W);
    uint32_t                last_depth_height_px_ = static_cast<uint32_t>(IMAGE_H);
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackerServer>());
    rclcpp::shutdown();
    return 0;
}
