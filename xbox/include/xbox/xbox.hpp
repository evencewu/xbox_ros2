#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/joystick.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include <xbox_interfaces/msg/xbox_control.hpp>

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x02
#define XBOX_BUTTON_Y 0x03
#define XBOX_BUTTON_LB 0x04
#define XBOX_BUTTON_RB 0x05
#define XBOX_BUTTON_START 0x06
#define XBOX_BUTTON_BACK 0x07
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x03 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x04 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x02
#define XBOX_AXIS_RT 0x05
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

namespace xbox
{
    typedef struct xbox_map
    {
        int time;
        int a;
        int b;
        int x;
        int y;
        int lb;
        int rb;
        int start;
        int back;
        int home;
        int lo;
        int ro;

        int lx;
        int ly;
        int rx;
        int ry;
        int lt;
        int rt;
        int xx;
        int yy;

    } xbox_map_t;

    class XboxNode : public rclcpp::Node
    {
    public:
        XboxNode();
        ~XboxNode() override;

    private:
        void Init();

        void MainLoop();

        rclcpp::TimerBase::SharedPtr timer_;

        int xbox_fd;
        xbox_map_t map;
        int len, type;
        int axis_value, button_value;
        int number_of_axis, number_of_buttons;

        int xbox_open(char *file_name);
        int xbox_map_read(int xbox_fd, xbox_map_t *map);
        void xbox_close(int xbox_fd);

        rclcpp::Publisher<xbox_interfaces::msg::XboxControl>::SharedPtr xbox_pub_;
        xbox_interfaces::msg::XboxControl xbox_msg_;

        bool map_lt_flag_ = 0;
        bool map_rt_flag_ = 0;
    };
}