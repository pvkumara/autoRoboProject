#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

// ── Safety clamps (assignment warning: never exceed 0.5 linear) ───────────────
static constexpr float MAX_LINEAR  = 0.3f;   // m/s
static constexpr float MAX_ANGULAR = 1.0f;   // rad/s

// Default USB port for Waveshare robot base on Jetson Orin Nano
static const char * DEFAULT_PORT = "/dev/ttyACM0";

// ─────────────────────────────────────────────────────────────────────────────

class WaveshareMotorDriver : public rclcpp::Node
{
public:
    explicit WaveshareMotorDriver(const std::string & port)
    : Node("waveshare_motor_driver"), fd_(-1)
    {
        fd_ = openSerial(port);
        if (fd_ < 0) {
            RCLCPP_ERROR(get_logger(),
                "Failed to open serial port %s — motor commands will be SIMULATED.",
                port.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Serial port %s opened successfully.", port.c_str());
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WaveshareMotorDriver::cmdVelCallback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Waveshare Motor Driver ready. Subscribed to /cmd_vel.");
    }

    ~WaveshareMotorDriver()
    {
        sendJsonCommand(0.0f, 0.0f);   // stop motors on shutdown
        if (fd_ >= 0) {
            close(fd_);
        }
    }

private:
    int fd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // ── /cmd_vel callback ─────────────────────────────────────────────────────
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float x = static_cast<float>(msg->linear.x);
        float z = static_cast<float>(msg->angular.z);

        // Clamp to safe ranges
        x = std::clamp(x, -MAX_LINEAR,  MAX_LINEAR);
        z = std::clamp(z, -MAX_ANGULAR, MAX_ANGULAR);

        sendJsonCommand(x, z);
    }

    // ── Build and send Waveshare JSON motor command ───────────────────────────
    // Format: {"T":13,"X":<linear>,"Z":<angular>}\n
    void sendJsonCommand(float linear_x, float angular_z)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "{\"T\":13,\"X\":" << linear_x << ",\"Z\":" << angular_z << "}\n";
        std::string cmd = ss.str();

        if (fd_ >= 0) {
            ssize_t written = write(fd_, cmd.c_str(), cmd.size());
            if (written < 0) {
                RCLCPP_ERROR(get_logger(),
                    "Serial write failed: %s", strerror(errno));
            }
        } else {
            // Simulation mode: just log
            RCLCPP_INFO(get_logger(), "[SIM] %s", cmd.c_str());
        }
    }

    // ── Open and configure serial port ───────────────────────────────────────
    static int openSerial(const std::string & port)
    {
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) return -1;

        struct termios tty{};
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            return -1;
        }

        // Waveshare robot base uses 115200 baud, 8N1
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag  = 0;
        tty.c_oflag  = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;  // 0.5-second read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            return -1;
        }

        return fd;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Allow overriding the serial port via command-line argument
    std::string port = DEFAULT_PORT;
    if (argc > 1) {
        port = argv[1];
    }

    rclcpp::spin(std::make_shared<WaveshareMotorDriver>(port));
    rclcpp::shutdown();
    return 0;
}
