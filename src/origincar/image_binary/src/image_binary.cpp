#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>


// ============================================================================
// 调试参数集中定义区域
//
// OpenCV HSV 范围：
// H: 0 ~ 179
// S: 0 ~ 255
// V: 0 ~ 255
//
// 修改这里的值后重新编译即可。
// 同时，这些值也会作为 ROS 参数默认值，可以在 launch 中覆盖。
// ============================================================================

namespace binary_config
{

// 固定输入图像尺寸
static constexpr int IMAGE_WIDTH  = 640;
static constexpr int IMAGE_HEIGHT = 400;

// 只处理图像下方的比例
//
// 0.50：处理下方 50%，即 y=240~479
// 0.60：处理下方 60%，即 y=192~479
// 1.00：处理整张图
static constexpr double ROI_BOTTOM_RATIO = 1.0;


// ==================== 黄色 HSV 阈值 ====================
//
// 黄色是中间通道。
// 当前参数是较宽松的初始范围，需要结合赛场图像继续调整。

static constexpr int GREEN_H_LOW  = 8;
static constexpr int GREEN_H_HIGH = 40;

static constexpr int GREEN_S_LOW  = 15;
static constexpr int GREEN_S_HIGH = 255;

static constexpr int GREEN_V_LOW  = 30;
static constexpr int GREEN_V_HIGH = 255;


// ==================== 绿色 HSV 阈值 ====================
//
// 绿色是赛道两侧区域。
// 绿色范围通常比黄色范围更宽。

static constexpr int YELLOW_H_LOW  = 90;
static constexpr int YELLOW_H_HIGH = 140;

static constexpr int YELLOW_S_LOW  = 15;
static constexpr int YELLOW_S_HIGH = 255;

static constexpr int YELLOW_V_LOW  = 30;
static constexpr int YELLOW_V_HIGH = 255;


// ==================== 图像滤波参数 ====================

// 开运算核大小：去除小噪点
static constexpr int OPEN_KERNEL_SIZE = 3;

// 闭运算核大小：填补孔洞、连接小断裂
static constexpr int CLOSE_KERNEL_SIZE = 7;

// 黄色最小连通域面积
static constexpr int MIN_YELLOW_AREA = 120;

// 绿色最小连通域面积
static constexpr int MIN_GREEN_AREA = 120;


// ==================== 主二值数组输出模式 ====================
//
// 0：binary_array_ 存黄色区域
// 1：binary_array_ 存绿色区域
// 2：binary_array_ 存黄色和绿色的并集
//
// 你的后续任务是识别两侧绿色边界，因此默认使用 1。
static constexpr int BINARY_OUTPUT_MODE = 1;


// 上位机 RGB 调试图是否显示不同颜色
//
// true：
//   绿色区域显示为绿色
//   黄色区域显示为黄色
//   其他区域显示为黑色
//
// false：
//   主二值图转换为黑白 RGB 图
static constexpr bool COLORIZE_RGB_DEBUG = true;

}  // namespace binary_config


class HsvBinaryNode : public rclcpp::Node
{
public:
    using BinaryRow = std::array<
        uint8_t,
        binary_config::IMAGE_WIDTH
    >;

    using BinaryArray = std::array<
        BinaryRow,
        binary_config::IMAGE_HEIGHT
    >;


    HsvBinaryNode()
        : Node("hsv_binary_node")
    {
        // ====================================================================
        // 1. 参数声明
        // ====================================================================

        this->declare_parameter<std::string>(
            "input_hsv_topic",
            "/image_new_hsv"
        );

        this->declare_parameter<std::string>(
            "binary_topic",
            "/image_new_binary"
        );

        this->declare_parameter<std::string>(
            "yellow_binary_topic",
            "/image_yellow_binary"
        );

        this->declare_parameter<std::string>(
            "green_binary_topic",
            "/image_green_binary"
        );

        this->declare_parameter<std::string>(
            "binary_rgb_topic",
            "/image_new_binary_rgb"
        );


        // ROI 参数
        this->declare_parameter<double>(
            "roi_bottom_ratio",
            binary_config::ROI_BOTTOM_RATIO
        );


        // 黄色参数
        this->declare_parameter<int>(
            "yellow_h_low",
            binary_config::YELLOW_H_LOW
        );

        this->declare_parameter<int>(
            "yellow_h_high",
            binary_config::YELLOW_H_HIGH
        );

        this->declare_parameter<int>(
            "yellow_s_low",
            binary_config::YELLOW_S_LOW
        );

        this->declare_parameter<int>(
            "yellow_s_high",
            binary_config::YELLOW_S_HIGH
        );

        this->declare_parameter<int>(
            "yellow_v_low",
            binary_config::YELLOW_V_LOW
        );

        this->declare_parameter<int>(
            "yellow_v_high",
            binary_config::YELLOW_V_HIGH
        );


        // 绿色参数
        this->declare_parameter<int>(
            "green_h_low",
            binary_config::GREEN_H_LOW
        );

        this->declare_parameter<int>(
            "green_h_high",
            binary_config::GREEN_H_HIGH
        );

        this->declare_parameter<int>(
            "green_s_low",
            binary_config::GREEN_S_LOW
        );

        this->declare_parameter<int>(
            "green_s_high",
            binary_config::GREEN_S_HIGH
        );

        this->declare_parameter<int>(
            "green_v_low",
            binary_config::GREEN_V_LOW
        );

        this->declare_parameter<int>(
            "green_v_high",
            binary_config::GREEN_V_HIGH
        );


        // 滤波参数
        this->declare_parameter<int>(
            "open_kernel_size",
            binary_config::OPEN_KERNEL_SIZE
        );

        this->declare_parameter<int>(
            "close_kernel_size",
            binary_config::CLOSE_KERNEL_SIZE
        );

        this->declare_parameter<int>(
            "min_yellow_area",
            binary_config::MIN_YELLOW_AREA
        );

        this->declare_parameter<int>(
            "min_green_area",
            binary_config::MIN_GREEN_AREA
        );


        // 输出模式
        this->declare_parameter<int>(
            "binary_output_mode",
            binary_config::BINARY_OUTPUT_MODE
        );

        this->declare_parameter<bool>(
            "publish_separate_masks",
            true
        );

        this->declare_parameter<bool>(
            "publish_rgb_debug",
            true
        );

        this->declare_parameter<bool>(
            "show_debug_log",
            true
        );


        // ====================================================================
        // 2. 读取参数
        // ====================================================================

        input_hsv_topic_ =
            this->get_parameter("input_hsv_topic").as_string();

        binary_topic_ =
            this->get_parameter("binary_topic").as_string();

        yellow_binary_topic_ =
            this->get_parameter("yellow_binary_topic").as_string();

        green_binary_topic_ =
            this->get_parameter("green_binary_topic").as_string();

        binary_rgb_topic_ =
            this->get_parameter("binary_rgb_topic").as_string();


        roi_bottom_ratio_ =
            this->get_parameter("roi_bottom_ratio").as_double();

        roi_bottom_ratio_ =
            std::clamp(roi_bottom_ratio_, 0.05, 1.0);


        yellow_h_low_ =
            this->get_parameter("yellow_h_low").as_int();

        yellow_h_high_ =
            this->get_parameter("yellow_h_high").as_int();

        yellow_s_low_ =
            this->get_parameter("yellow_s_low").as_int();

        yellow_s_high_ =
            this->get_parameter("yellow_s_high").as_int();

        yellow_v_low_ =
            this->get_parameter("yellow_v_low").as_int();

        yellow_v_high_ =
            this->get_parameter("yellow_v_high").as_int();


        green_h_low_ =
            this->get_parameter("green_h_low").as_int();

        green_h_high_ =
            this->get_parameter("green_h_high").as_int();

        green_s_low_ =
            this->get_parameter("green_s_low").as_int();

        green_s_high_ =
            this->get_parameter("green_s_high").as_int();

        green_v_low_ =
            this->get_parameter("green_v_low").as_int();

        green_v_high_ =
            this->get_parameter("green_v_high").as_int();


        open_kernel_size_ =
            this->get_parameter("open_kernel_size").as_int();

        close_kernel_size_ =
            this->get_parameter("close_kernel_size").as_int();

        min_yellow_area_ =
            this->get_parameter("min_yellow_area").as_int();

        min_green_area_ =
            this->get_parameter("min_green_area").as_int();


        binary_output_mode_ =
            this->get_parameter("binary_output_mode").as_int();

        publish_separate_masks_ =
            this->get_parameter("publish_separate_masks").as_bool();

        publish_rgb_debug_ =
            this->get_parameter("publish_rgb_debug").as_bool();

        show_debug_log_ =
            this->get_parameter("show_debug_log").as_bool();


        // 核大小必须是大于等于 1 的奇数
        normalizeKernelSize(open_kernel_size_);
        normalizeKernelSize(close_kernel_size_);

        // 输出模式限制到 0~2
        binary_output_mode_ =
            std::clamp(binary_output_mode_, 0, 2);


        // ====================================================================
        // 3. 初始化数组
        // ====================================================================

        clearBinaryArrays();


// ====================================================================
// 4. QoS
// ====================================================================

// 摄像头/HSV 输入使用 SensorDataQoS，兼容上游图像发布器
auto sub_qos = rclcpp::SensorDataQoS();

// 输出使用 reliable，确保能够被 hobot_codec 的 reliable 订阅器接收
auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));
pub_qos.reliable();
pub_qos.durability_volatile();


// ====================================================================
// 5. 发布与订阅
// ====================================================================

hsv_sub_ =
    this->create_subscription<sensor_msgs::msg::Image>(
        input_hsv_topic_,
        sub_qos,
        std::bind(
            &HsvBinaryNode::hsvCallback,
            this,
            std::placeholders::_1
        )
    );


binary_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>(
        binary_topic_,
        pub_qos
    );


yellow_binary_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>(
        yellow_binary_topic_,
        pub_qos
    );


green_binary_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>(
        green_binary_topic_,
        pub_qos
    );


binary_rgb_pub_ =
    this->create_publisher<sensor_msgs::msg::Image>(
        binary_rgb_topic_,
        pub_qos
    );
        // ====================================================================
        // 6. 周期日志
        // ====================================================================

        debug_timer_ =
            this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(
                    &HsvBinaryNode::timerDebugCallback,
                    this
                )
            );


        callback_count_ = 0;
        publish_count_ = 0;


        // ====================================================================
        // 7. 启动日志
        // ====================================================================

        RCLCPP_INFO(
            this->get_logger(),
            "hsv_binary_node started."
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribe HSV topic: %s",
            input_hsv_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publish main binary topic: %s",
            binary_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publish yellow topic: %s",
            yellow_binary_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publish green topic: %s",
            green_binary_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Publish RGB debug topic: %s",
            binary_rgb_topic_.c_str()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Yellow HSV: H[%d,%d] S[%d,%d] V[%d,%d]",
            yellow_h_low_,
            yellow_h_high_,
            yellow_s_low_,
            yellow_s_high_,
            yellow_v_low_,
            yellow_v_high_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Green HSV: H[%d,%d] S[%d,%d] V[%d,%d]",
            green_h_low_,
            green_h_high_,
            green_s_low_,
            green_s_high_,
            green_v_low_,
            green_v_high_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "ROI bottom ratio=%.2f, open=%d, close=%d, output_mode=%d",
            roi_bottom_ratio_,
            open_kernel_size_,
            close_kernel_size_,
            binary_output_mode_
        );
    }


private:
    // ========================================================================
    // 图像回调
    // ========================================================================

    void hsvCallback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        callback_count_++;


        // ====================================================================
        // 1. 检查输入格式
        // ====================================================================

        const int width =
            static_cast<int>(msg->width);

        const int height =
            static_cast<int>(msg->height);

        const int step =
            static_cast<int>(msg->step);


        if (width != binary_config::IMAGE_WIDTH ||
            height != binary_config::IMAGE_HEIGHT)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image size mismatch: expected %dx%d, received %dx%d",
                binary_config::IMAGE_WIDTH,
                binary_config::IMAGE_HEIGHT,
                width,
                height
            );

            return;
        }


        if (step < width * 3)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid step: step=%d, required at least %d",
                step,
                width * 3
            );

            return;
        }


        const size_t expected_size =
            static_cast<size_t>(height) *
            static_cast<size_t>(step);


        if (msg->data.size() < expected_size)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image data too small: received=%zu expected=%zu",
                msg->data.size(),
                expected_size
            );

            return;
        }


        // image_bridge_hsv 发布的格式应该是 8UC3。
        // 即使 encoding 名称不同，只要数据确实是 HSV 三通道，仍可继续处理。
        if (msg->encoding != "8UC3" &&
            msg->encoding != "hsv8")
        {
            if (callback_count_ % 60 == 1)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Unexpected encoding '%s'. "
                    "This node assumes data order is H,S,V.",
                    msg->encoding.c_str()
                );
            }
        }


        // ====================================================================
        // 2. 包装 HSV 图像
        //
        // 不复制数据，直接引用当前 msg->data。
        // image_bridge_hsv 输出：
        //
        // data = H,S,V,H,S,V,...
        // ====================================================================

        cv::Mat hsv_image(
            height,
            width,
            CV_8UC3,
            const_cast<uint8_t *>(msg->data.data()),
            static_cast<size_t>(step)
        );


        // ====================================================================
        // 3. 计算下方 ROI
        // ====================================================================

        int roi_start_y =
            static_cast<int>(
                static_cast<double>(height) *
                (1.0 - roi_bottom_ratio_)
            );


        roi_start_y =
            std::clamp(
                roi_start_y,
                0,
                height - 1
            );


        const int roi_height =
            height - roi_start_y;


        const cv::Rect roi_rect(
            0,
            roi_start_y,
            width,
            roi_height
        );


        const cv::Mat hsv_roi =
            hsv_image(roi_rect);


        // ====================================================================
        // 4. HSV 阈值分割
        // ====================================================================

        cv::Mat yellow_roi_raw;
        cv::Mat green_roi_raw;
        cv::Mat black_roi_raw;

        const cv::Scalar yellow_low(
            yellow_h_low_,
            yellow_s_low_,
            yellow_v_low_
        );


        const cv::Scalar yellow_high(
            yellow_h_high_,
            yellow_s_high_,
            yellow_v_high_
        );


        const cv::Scalar green_low(
            green_h_low_,
            green_s_low_,
            green_v_low_
        );


        const cv::Scalar green_high(
            green_h_high_,
            green_s_high_,
            green_v_high_
        );


        cv::inRange(
            hsv_roi,
            yellow_low,
            yellow_high,
            yellow_roi_raw
        );

        // 黑色：主要通过较低的 V 判断
cv::inRange(
    hsv_roi,
    cv::Scalar(0, 0, 0),
    cv::Scalar(179, 255, 29),
    black_roi_raw
);

// 黄色 + 黑色 都变成白色
cv::bitwise_or(
    yellow_roi_raw,
    black_roi_raw,
    yellow_roi_raw
);


        cv::inRange(
            hsv_roi,
            green_low,
            green_high,
            green_roi_raw
        );


        // ====================================================================
        // 5. 开闭运算和连通域过滤
        // ====================================================================

        cv::Mat yellow_roi_clean;
        cv::Mat green_roi_clean;


        cleanMask(
            yellow_roi_raw,
            yellow_roi_clean,
            min_yellow_area_
        );


        cleanMask(
            green_roi_raw,
            green_roi_clean,
            min_green_area_
        );


        // ====================================================================
        // 6. 恢复成完整 640×480 图像
        //
        // ROI 上方区域全部为 0。
        // ====================================================================

        cv::Mat yellow_full =
            cv::Mat::zeros(
                height,
                width,
                CV_8UC1
            );


        cv::Mat green_full =
            cv::Mat::zeros(
                height,
                width,
                CV_8UC1
            );


        yellow_roi_clean.copyTo(
            yellow_full(roi_rect)
        );


        green_roi_clean.copyTo(
            green_full(roi_rect)
        );


        // ====================================================================
        // 7. 生成主二值图
        // ====================================================================

        cv::Mat binary_full;


        if (binary_output_mode_ == 0)
        {
            // 黄色区域
            binary_full =
                yellow_full.clone();
        }
        else if (binary_output_mode_ == 1)
        {
            // 绿色区域
            binary_full =
                green_full.clone();
        }
        else
        {
            // 黄色和绿色并集
            cv::bitwise_or(
                yellow_full,
                green_full,
                binary_full
            );
        }


        // ====================================================================
        // 8. 存入固定 480×640 数组
        //
        // 坐标访问：
        //
        // binary_array_[y][x]
        // yellow_binary_array_[y][x]
        // green_binary_array_[y][x]
        //
        // 值只有：
        // 0   非目标区域
        // 255 目标区域
        // ====================================================================

        copyMatToArray(
            binary_full,
            binary_array_
        );


        copyMatToArray(
            yellow_full,
            yellow_binary_array_
        );


        copyMatToArray(
            green_full,
            green_binary_array_
        );


        // ====================================================================
        // 9. 发布 mono8 二值图
        // ====================================================================

        publishMonoImage(
            binary_pub_,
            binary_full,
            msg->header
        );


        if (publish_separate_masks_)
        {
            publishMonoImage(
                yellow_binary_pub_,
                yellow_full,
                msg->header
            );


            publishMonoImage(
                green_binary_pub_,
                green_full,
                msg->header
            );
        }


        // ====================================================================
        // 10. 发布 rgb8 调试图
        //
        // 此话题可以交给 hobot_codec 编码成 JPEG：
        //
        // /image_new_binary_rgb
        //     -> hobot_codec
        //     -> /image_new_binary_mjpeg
        //     -> websocket
        // ====================================================================

        if (publish_rgb_debug_)
        {
            publishRgbDebugImage(
                binary_full,
                yellow_full,
                green_full,
                msg->header
            );
        }


        publish_count_++;


        // ====================================================================
        // 11. 调试日志
        // ====================================================================

        if (show_debug_log_ &&
            callback_count_ % 30 == 1)
        {
            const int yellow_pixels =
                cv::countNonZero(yellow_full);

            const int green_pixels =
                cv::countNonZero(green_full);

            const int binary_pixels =
                cv::countNonZero(binary_full);


            RCLCPP_INFO(
                this->get_logger(),
                "[BINARY] roi_y=%d roi_height=%d "
                "yellow=%d green=%d main=%d",
                roi_start_y,
                roi_height,
                yellow_pixels,
                green_pixels,
                binary_pixels
            );
        }
    }


    // ========================================================================
    // mask 清理
    // ========================================================================

    void cleanMask(
        const cv::Mat &source,
        cv::Mat &destination,
        int minimum_area)
    {
        cv::Mat cleaned =
            source.clone();


        // 开运算：先腐蚀后膨胀，去掉孤立小白点
        const cv::Mat open_kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(
                    open_kernel_size_,
                    open_kernel_size_
                )
            );


        cv::morphologyEx(
            cleaned,
            cleaned,
            cv::MORPH_OPEN,
            open_kernel
        );


        // 闭运算：先膨胀后腐蚀，连接小断裂并填补孔洞
        const cv::Mat close_kernel =
            cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(
                    close_kernel_size_,
                    close_kernel_size_
                )
            );


        cv::morphologyEx(
            cleaned,
            cleaned,
            cv::MORPH_CLOSE,
            close_kernel
        );


        // 删除小连通区域
        removeSmallComponents(
            cleaned,
            destination,
            minimum_area
        );
    }


    // ========================================================================
    // 连通域过滤
    // ========================================================================

    void removeSmallComponents(
        const cv::Mat &source,
        cv::Mat &destination,
        int minimum_area)
    {
        destination =
            cv::Mat::zeros(
                source.size(),
                CV_8UC1
            );


        cv::Mat labels;
        cv::Mat statistics;
        cv::Mat centroids;


        const int component_count =
            cv::connectedComponentsWithStats(
                source,
                labels,
                statistics,
                centroids,
                8,
                CV_32S
            );


        // label 0 是背景，从 1 开始遍历目标区域
        for (int label = 1;
             label < component_count;
             ++label)
        {
            const int area =
                statistics.at<int>(
                    label,
                    cv::CC_STAT_AREA
                );


            if (area < minimum_area)
            {
                continue;
            }


            const cv::Mat component_mask =
                (labels == label);


            destination.setTo(
                255,
                component_mask
            );
        }
    }


    // ========================================================================
    // cv::Mat 写入固定数组
    // ========================================================================

    void copyMatToArray(
        const cv::Mat &source,
        BinaryArray &destination)
    {
        if (source.rows != binary_config::IMAGE_HEIGHT ||
            source.cols != binary_config::IMAGE_WIDTH ||
            source.type() != CV_8UC1)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "copyMatToArray received invalid Mat."
            );

            return;
        }


        for (int y = 0;
             y < binary_config::IMAGE_HEIGHT;
             ++y)
        {
            const uint8_t *source_row =
                source.ptr<uint8_t>(y);


            std::memcpy(
                destination[y].data(),
                source_row,
                static_cast<size_t>(
                    binary_config::IMAGE_WIDTH
                )
            );
        }
    }


    // ========================================================================
    // 发布 mono8 图像
    // ========================================================================

    void publishMonoImage(
        const rclcpp::Publisher<
            sensor_msgs::msg::Image
        >::SharedPtr &publisher,
        const cv::Mat &image,
        const std_msgs::msg::Header &header)
    {
        if (!publisher ||
            image.empty() ||
            image.type() != CV_8UC1)
        {
            return;
        }


        sensor_msgs::msg::Image output;

        output.header = header;
        output.height =
            static_cast<uint32_t>(image.rows);

        output.width =
            static_cast<uint32_t>(image.cols);

        output.encoding = "mono8";
        output.is_bigendian = false;

        output.step =
            static_cast<uint32_t>(image.cols);


        const size_t data_size =
            static_cast<size_t>(output.step) *
            static_cast<size_t>(output.height);


        output.data.resize(data_size);


        copyMatDataToMessage(
            image,
            output
        );


        publisher->publish(output);
    }


    // ========================================================================
    // 发布供上位机显示的 rgb8 图像
    // ========================================================================

    void publishRgbDebugImage(
        const cv::Mat &binary,
        const cv::Mat &yellow,
        const cv::Mat &green,
        const std_msgs::msg::Header &header)
    {
        cv::Mat rgb_debug;


        // 黄色 mask：黄色区域为 255，其他区域为 0
        // 转成 rgb8，供 hobot_codec 和上位机显示
        cv::cvtColor(
            yellow,
            rgb_debug,
            cv::COLOR_GRAY2RGB
        );


        sensor_msgs::msg::Image output;

        output.header = header;
        output.height =
            static_cast<uint32_t>(rgb_debug.rows);

        output.width =
            static_cast<uint32_t>(rgb_debug.cols);

        output.encoding = "rgb8";
        output.is_bigendian = false;

        output.step =
            static_cast<uint32_t>(
                rgb_debug.cols * 3
            );


        const size_t data_size =
            static_cast<size_t>(output.step) *
            static_cast<size_t>(output.height);


        output.data.resize(data_size);


        copyMatDataToMessage(
            rgb_debug,
            output
        );


        binary_rgb_pub_->publish(output);
    }


    // ========================================================================
    // cv::Mat 数据复制到 ROS Image
    // ========================================================================

    void copyMatDataToMessage(
        const cv::Mat &image,
        sensor_msgs::msg::Image &message)
    {
        const size_t output_row_bytes =
            static_cast<size_t>(message.step);


        if (image.isContinuous() &&
            image.step ==
                output_row_bytes)
        {
            const size_t data_size =
                output_row_bytes *
                static_cast<size_t>(image.rows);


            std::memcpy(
                message.data.data(),
                image.data,
                data_size
            );

            return;
        }


        for (int y = 0;
             y < image.rows;
             ++y)
        {
            const uint8_t *source_row =
                image.ptr<uint8_t>(y);


            uint8_t *destination_row =
                message.data.data() +
                static_cast<size_t>(y) *
                output_row_bytes;


            std::memcpy(
                destination_row,
                source_row,
                output_row_bytes
            );
        }
    }


    // ========================================================================
    // 其他工具函数
    // ========================================================================

    static void normalizeKernelSize(
        int &kernel_size)
    {
        if (kernel_size < 1)
        {
            kernel_size = 1;
        }


        if (kernel_size % 2 == 0)
        {
            kernel_size += 1;
        }
    }


    void clearBinaryArrays()
    {
        for (auto &row : binary_array_)
        {
            row.fill(0);
        }


        for (auto &row : yellow_binary_array_)
        {
            row.fill(0);
        }


        for (auto &row : green_binary_array_)
        {
            row.fill(0);
        }
    }


    void timerDebugCallback()
    {
        if (!show_debug_log_)
        {
            return;
        }


        RCLCPP_INFO(
            this->get_logger(),
            "[TIMER] callback_count=%zu publish_count=%zu",
            callback_count_,
            publish_count_
        );
    }


private:
    // ========================================================================
    // ROS 通信对象
    // ========================================================================

    rclcpp::Subscription<
        sensor_msgs::msg::Image
    >::SharedPtr hsv_sub_;


    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr binary_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr yellow_binary_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr green_binary_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr binary_rgb_pub_;


    rclcpp::TimerBase::SharedPtr debug_timer_;


    // ========================================================================
    // topic 参数
    // ========================================================================

    std::string input_hsv_topic_;
    std::string binary_topic_;
    std::string yellow_binary_topic_;
    std::string green_binary_topic_;
    std::string binary_rgb_topic_;


    // ========================================================================
    // 图像处理参数
    // ========================================================================

    double roi_bottom_ratio_;


    int yellow_h_low_;
    int yellow_h_high_;
    int yellow_s_low_;
    int yellow_s_high_;
    int yellow_v_low_;
    int yellow_v_high_;


    int green_h_low_;
    int green_h_high_;
    int green_s_low_;
    int green_s_high_;
    int green_v_low_;
    int green_v_high_;


    int open_kernel_size_;
    int close_kernel_size_;

    int min_yellow_area_;
    int min_green_area_;

    int binary_output_mode_;

    bool publish_separate_masks_;
    bool publish_rgb_debug_;
    bool show_debug_log_;


    // ========================================================================
    // 固定 480×640 二值数组
    //
    // 访问形式：
    // array[y][x]
    //
    // binary_array_：
    //   由 binary_output_mode 决定内容，默认是绿色 mask
    //
    // yellow_binary_array_：
    //   黄色区域为 255
    //
    // green_binary_array_：
    //   绿色区域为 255
    // ========================================================================

    BinaryArray binary_array_{};
    BinaryArray yellow_binary_array_{};
    BinaryArray green_binary_array_{};


    size_t callback_count_;
    size_t publish_count_;
};


int main(
    int argc,
    char **argv)
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<HsvBinaryNode>();


    rclcpp::spin(node);


    rclcpp::shutdown();

    return 0;
}