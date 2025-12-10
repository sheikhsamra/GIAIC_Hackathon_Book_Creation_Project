# ماڈیول 2: جسمانی مصنوعی ذہانت کے لیے ROS 2 کے بنیادیات - اردو ترجمہ

## جدول کا خلاصہ
- [1. ROS 2 معماری اور تصورات](#1-ros-2-معماری-اور-تصورات)
- [2. نوڈز، پیکجز، اور کمیونیکیشن](#2-نوڈز-پیکجز-اور-کمیونیکیشن)
- [3. خدمات اور ایکشنز](#3-خدمات-اور-ایکشنز)
- [4. پیرامیٹر اور لانچ سسٹم](#4-پیرامیٹر-اور-لانچ-سسٹم)
- [5. اوزار اور حفاظت](#5-اوزار-اور-حفاظت)
- [6. انضمام اور بہترین طریقے](#6-انضمام-اور-بہترین-طریقے)

## 1. ROS 2 معماری اور تصورات

### 1.1 ROS 1 سے ROS 2 کی ترقی

ROS 2، روبوٹ آپریٹنگ سسٹم کی ایک بنیادی دوبارہ ڈیزائن ہے جو ROS 1 کی حدود کو پورا کرنے کے لیے ہے، خاص طور پر پیداوار اور حفاظت سے متعلق ایپلی کیشنز کے لیے۔ کلیدی معماری کی تبدیلیاں شامل ہیں:

#### DDS-مبنی کمیونیکیشن لیئر
- **ڈیٹا ڈسٹری بیوشن سروس (DDS)** بنیادی کمیونیکیشن انفراسٹرکچر فراہم کرتا ہے
- شائع کرنا-سبسکرائب کرنا، درخواست-جواب، اور دیگر کمیونیکیشن پیٹرنز کو لاگو کرتا ہے
- مختلف درخواست کی ضروریات کے لیے معیار کی سروس (QoS) پالیسیز کو سپورٹ کرتا ہے
- مضبوط، ریل ٹائم، اور تقسیم شدہ کمیونیکیشن کے قابل ہے

#### بہتر سیکیورٹی ماڈل
- تصدیق، انکرپشن، اور رسائی کنٹرول سمیت اندرونی سیکیورٹی خصوصیات
- نوڈس کے درمیان محفوظ کمیونیکیشن کے لیے سپورٹ
- فائن گرینڈ اجازت کے لیے اہلیت مبنی سیکیورٹی ماڈل

#### ریل ٹائم سپورٹ
- ریل ٹائم آپریٹنگ سسٹمز اور شیڈولنگ کے ساتھ انضمام
- باقاعدہ تاخیر کے ساتھ قابل اعتماد کمیونیکیشن
- قابل پیش گوئی ٹائم کے ساتھ ریل ٹائم ایپلی کیشنز کے لیے سپورٹ

### 1.2 کور معماری کمپونینٹس

#### نوڈز
نوڈز ROS 2 میں بنیادی انجام دہندہ یونٹس ہیں:
- آزاد عمل کے حساب سے جو کمپیوٹیشن انجام دیتے ہیں
- ٹاپکس، خدمات، اور ایکشنز کے ذریعے دیگر نوڈس کے ساتھ بات چیت کرتے ہیں
- متعدد زبانوں (پائی تھون، سی++، وغیرہ) میں لکھے جا سکتے ہیں
- ROS 2 کلائنٹ لائبریری (rcl) کے ذریعے منظم کیا جاتا ہے

#### ٹاپکس
ٹاپکس نوڈس کے درمیان غیر ہم وقت کمیونیکیشن کو فعال کرتے ہیں:
- ڈیٹا تقسیم کے لیے شائع کرنا-سبسکرائب کرنا پیٹرن
- پیغامات شائع کرنے والے سے سبسکرائبر تک جاتے ہیں
- مختلف QoS پالیسیز کو سپورٹ کرتا ہے (قابل اعتماد، م durable، وغیرہ)
- سینسر ڈیٹا، ریاست کی معلومات، اور دیگر جاری ڈیٹا سٹریمز کے لیے استعمال ہوتا ہے

#### خدمات
خدمات ہم وقت درخواست-جواب کمیونیکیشن فراہم کرتی ہیں:
- کلائنٹ درخواست بھیجتا ہے، سرور جواب کے ساتھ جواب دیتا ہے
- کام کرنے کے لیے ضروری ہے جن کا جواب چاہیے
- بلاکنگ یا غیر-بلاکنگ ایمپلیمنٹس دستیاب ہیں
- کنفیگریشن، ایکٹیویشن، یا ایک بار کے کام کے لیے اچھا ہے

#### ایکشنز
ایکشنز طویل چلنے والے ٹاسکوں کے ساتھ کام کرتے ہیں:
- کلائنٹ مقصد بھیجتا ہے، انجام کے دوران فیڈ بیک حاصل کرتا ہے، اور حتمی نتیجہ حاصل کرتا ہے
- پریمپشن اور منسوخی کو سپورٹ کرتا ہے
- نیویگیشن، مینیپولیشن، اور دیگر پیچیدہ کاموں کے لیے استعمال ہوتا ہے
- ٹاپکس اور خدمات کے مجموعہ کے طور پر لاگو کیا گیا ہے

## 2. نوڈز، پیکجز، اور کمیونیکیشن

### 2.1 ROS 2 پیکج کی تخلیق

ROS 2 پیکج بنانے کے لیے کمانڈ:
```bash
ros2 pkg create --build-type ament_python my_robot_package --dependencies rclpy std_msgs geometry_msgs
```

### 2.2 بنیادی نوڈ کی ساخت

#### 2.2.1 سادہ شائع کرنے والا نوڈ (پائی تھون)

```python
#!/usr/bin/env python3
"""
جسمانی مصنوعی ذہانت کے لیے بنیادی ROS 2 شائع کرنے والا نوڈ
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(String, 'ai_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Physical AI System Status: Operational - Cycle {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    status_publisher = StatusPublisher()
    rclpy.spin(status_publisher)


if __name__ == '__main__':
    main()
```

#### 2.2.2 سادہ سبسکرائب کرنے والا نوڈ (پائی تھون)

```python
#!/usr/bin/env python3
"""
جسمانی مصنوعی ذہانت کے لیے بنیادی ROS 2 سبسکرائب کرنے والا نوڈ
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusSubscriber(Node):
    def __init__(self):
        super().__init__('status_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ai_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    status_subscriber = StatusSubscriber()
    rclpy.spin(status_subscriber)


if __name__ == '__main__':
    main()
```

### 2.3 معیار کی سروس (QoS) پالیسیز

#### قابل اعتماد پالیسی
- **قابل اعتماد**: تمام پیغامات کی ترسیل کی گارنٹی دی جاتی ہے
- **بہترین کوشش**: پیغامات ضائع ہو سکتے ہیں، لیکن ترسیل تیز ہے
- قابل اعتماد کو اہم ڈیٹا (کمانڈز، حفاظتی پیغامات) کے لیے استعمال کریں
- بہترین کوشش کو سینسر ڈیٹا کے لیے استعمال کریں جہاں کچھ نقصان قابل قبول ہے

## 3. خدمات اور ایکشنز

### 3.1 خدمات کا استعمال

#### 3.1.1 سروس سرور (پائی تھون)

```python
#!/usr/bin/env python3
"""
جسمانی مصنوعی ذہانت کے لیے ROS 2 سروس سرور
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger


class SafetyCheckService(Node):
    def __init__(self):
        super().__init__('safety_check_service')
        self.srv = self.create_service(Trigger, 'safety_check', self.safety_check_callback)

    def safety_check_callback(self, request, response):
        # حفاظتی چیک کے لیے منطق یہاں ہو گا
        response.success = True
        response.message = 'System is safe to operate'
        return response


def main(args=None):
    rclpy.init(args=args)
    service = SafetyCheckService()
    rclpy.spin(service)


if __name__ == '__main__':
    main()
```

### 3.2 ایکشنز کا استعمال

#### 3.2.1 ایکشن سرور (پائی تھون)

```python
#!/usr/bin/env python3
"""
جسمانی مصنوعی ذہانت کے لیے ROS 2 ایکشن سرور
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        # نیویگیشن کا کام انجام دیں
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationActionServer()
    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
```

## 4. پیرامیٹر اور لانچ سسٹم

### 4.1 پیرامیٹر مینجمنٹ

#### 4.1.1 پیرامیٹر والے نوڈ (پائی تھون)

```python
#!/usr/bin/env python3
"""
ROS 2 پیرامیٹر مینجمنٹ کا نمونہ نوڈ
"""
import rclpy
from rclpy.node import Node


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # پیرامیٹر کا اعلان کریں
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # پیرامیٹر کی قدر حاصل کریں
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

### 4.2 لانچ فائل (پائی تھون)

```python
#!/usr/bin/env python3
"""
ROS 2 لانچ فائل کا نمونہ
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # نوڈ تیار کریں
    status_publisher = Node(
        package='my_robot_package',
        executable='status_publisher',
        name='status_publisher'
    )

    return LaunchDescription([
        status_publisher
    ])
```

## 5. اوزار اور حفاظت

### 5.1 ROS 2 اوزار

#### 5.1.1 کمانڈ لائن اوزار

- **ros2 topic**: ٹاپکس کی جانچ پڑتال کریں
- **ros2 service**: خدمات کی جانچ پڑتال کریں
- **ros2 action**: ایکشنز کی جانچ پڑتال کریں
- **ros2 node**: نوڈس کا نظم کریں
- **ros2 param**: نوڈ پیرامیٹرز کنفیگر کریں
- **ros2 bag**: ڈیٹا ریکارڈ اور پلے بیک کریں
- **ros2 launch**: پیچیدہ سسٹم شروع کریں

### 5.2 حفاظتی نوڈ (پائی تھون)

```python
#!/usr/bin/env python3
"""
جسمانی مصنوعی ذہانت کے لیے حفاظتی نوڈ
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # سبسکرپشنز
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # پبلیشرز
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # حفاظتی حدیں
        self.max_velocity = 1.0
        self.min_obstacle_distance = 0.5

    def cmd_vel_callback(self, msg):
        # رفتار کی حد کی جانچ
        linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        if linear_speed > self.max_velocity:
            self.trigger_emergency_stop()

    def scan_callback(self, msg):
        # رکاوٹ کی جانچ
        if min(msg.ranges) < self.min_obstacle_distance:
            self.trigger_emergency_stop()

    def trigger_emergency_stop(self):
        # ہنگامی بندش کا پیغام شائع کریں
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)


if __name__ == '__main__':
    main()
```

## 6. انضمام اور بہترین طریقے

### 6.1 بہترین طریقے

#### 6.1.1 ڈیزائن کے اصول

- **ماڈولریٹی**: نوڈس کے لیے واحد ذمہ داری کا اصول
- **واضح انٹرفیس**: اجزاء کے درمیان واضح انٹرفیسز
- **کم کپلنگ**: برقرار رکھنے کے قابل
- **دوبارہ استعمال کے قابل اجزاء**: مختلف پروجیکٹس میں

#### 6.1.2 حفاظت-پہل ڈیزائن

- **ڈیفالٹ طور پر حفاظتی**: غلطی سے محفوظ
- **گہرائی میں دفاع**: متعدد حفاظتی پرتیں
- **گریس فل ڈیگریڈیشن**: خرابی کے بعد بھی محفوظ طور پر کام کرنا
- **جامع خامی کا انتظام**: مناسب خامیوں کا انتظام

### 6.2 کارکردگی کی بہتری

#### 6.2.1 کمیونیکیشن کی کارکردگی

- **مناسب پیغام کی شرح**: ایپلی کیشن کے لیے مناسب
- **کارکردگی کا سریلائزیشن**: ڈیٹا کی کارکردگی
- **QoS پالیسی کا انتخاب**: مناسب QoS پالیسی کا انتخاب
- **نیٹ ورک بینڈ وڈتھ کی بہتری**: بینڈ وڈتھ کا بہتر استعمال

#### 6.2.2 وسائل کا نظم

- **میموری استعمال کی نگرانی**: میموری کا مناسب استعمال
- **CPU استعمال کی کارکردگی**: CPU استعمال کو بہتر بنانا
- **ریل ٹائم تھریڈ کا نظم**: ریل ٹائم کے لیے مناسب تھریڈز
- **پاور استعمال کے تصورات**: پاور کا محفوظ استعمال

---

** نوٹ **: یہ اردو ترجمہ ROS 2 کے بنیادیات کا مکمل ترجمہ ہے جو جسمانی مصنوعی ذہانت کے لیے ضروری ہیں۔ مکمل کتاب کے دیگر ماڈیولز کے لیے مزید ترجمے تیار کیے جائیں گے۔