# ماڈیول 3: گزبو سیمیولیشن اور فزکس انجن - اردو ترجمہ

## جدول کا خلاصہ
- [1. گزبو معماری اور تصورات](#1-گزبو-معماری-اور-تصورات)
- [2. روبوٹ ماڈلنگ: URDF اور SDF](#2-روبوٹ-ماڈلنگ-urdf-اور-sdf)
- [3. فزکس سیمیولیشن اور خصوصیات](#3-فزکس-سیمیولیشن-اور-خصوصیات)
- [4. سینسر سیمیولیشن](#4-سینسر-سیمیولیشن)
- [5. دنیا اور ماحول کی ماڈلنگ](#5-دنیا-اور-ماحول-کی-ماڈلنگ)
- [6. ROS 2 انضمام](#6-ros-2-انضمام)
- [7. سیمیولیشن کی توثیق اور حقیقت کا فرق](#7-سیمیولیشن-کی-توثیق-اور-حقیقت-کا-فرق)

## 1. گزبو معماری اور تصورات

### 1.1 گزبو سسٹم معماری

گزبو ایک ماڈیولر معماری پر مبنی ہے جو فزکس سیمیولیشن، رینڈرنگ، اور صارف انٹرفیس کے اجزاء کو الگ کرتی ہے:

#### سرور (gzserver)
- کور سیمیولیشن انجن جو فزکس، سینسرز، اور ماڈل اپ ڈیٹس چلاتا ہے
- تمام سیمیولیشن اسٹیٹ اور ٹائم کا انتظام کرتا ہے
- پلگ انز اور ورلڈ لوڈنگ کا انتظام کرتا ہے
- کلائنٹس کے ساتھ ٹرانسپورٹ لیئر کے ذریعے رابطہ کرتا ہے

#### کلائنٹ (gzclient)
- وژولائزیشن اور تعامل کے لیے صارف انٹرفیس
- سیمیولیشن ماحول کا ریل ٹائم رینڈرنگ
- سیمیولیشن کنٹرول اور انسپیکشن کے اوزار فراہم کرتا ہے
- سرور کے ساتھ الگ سے چل سکتا ہے

#### ٹرانسپورٹ لیئر
- زیرو ایم کیو پر مبنی کمیونیکیشن سسٹم
- سرور اور کلائنٹس کے درمیان پیغام پاس کرنے کا انتظام کرتا ہے
- ایک سرور سے متعدد کلائنٹس کنکشن کی حمایت کرتا ہے
- بیرونی ایپلی کیشنز کے لیے API فراہم کرتا ہے

### 1.2 فزکس سیمیولیشن کے بنیادی تصورات

#### ٹائم سٹیپنگ
- فزکس کیلکولیشنز کے لیے ڈسکریٹ ٹائم وقفوں
- چھوٹے قدم ایکسلریشن لیکن زیادہ کمپیوٹیشن کی ضرورت ہوتی ہے
- ایکسلریشن اور کارکردگی کے درمیان توازن
- ریل ٹائم فیکٹر (RTF) سیمیولیشن کی رفتار کو پیمائش کرتا ہے

#### کولیژن ڈیٹیکشن
- براڈ فیز: غیر کولیڈنگ والے جوڑوں کو جلدی ہی الگ کرنا
- نیرو فیز: باقی جوڑوں کے لیے مصدقہ کولیژن ڈیٹیکشن
- کونٹیکٹ جنریشن: کولیژن پوائنٹس پر فورسز کا حساب
- باؤنڈنگ والیومز کے ذریعے کارکردگی کی بہتری

### 1.3 کور تصورات

#### ODE (Open Dynamics Engine)
- زیادہ تر ایپلی کیشنز کے لیے تیز اور مضبوط
- زمینی گاڑیوں اور مینیپولیٹرز کے لیے اچھا
- پیچیدہ کونٹیکٹ منظار کی حمایت
- اچھی طرح سے قائم اور وسیع پیمانے پر ٹیسٹ کیا گیا

#### بلیٹ فزکس
- بہتر کونٹیکٹ ماڈلنگ
- پیچیدہ تعاملات والے منظار کے لیے بہتر
- زیادہ پیچیدہ کولیژن شیپس کی حمایت
- ہیومنوڈ روبوٹس کے لیے اکثر استعمال ہوتا ہے

#### DART (Dynamic Animation and Robotics Toolkit)
- اعلی درجے کی پابندی کا انتظام
- پیچیدہ کنیمیٹک چینز کے لیے بہتر
- ملٹی باڈی ڈائنامکس کی حمایت
- پیچیدہ روبوٹ میکنزم کے لیے اچھا

## 2. روبوٹ ماڈلنگ: URDF اور SDF

### 2.1 URDF (Unified Robot Description Format)

URDF روبوٹ ماڈلز کو ظاہر کرنے کے لیے ایک XML فارمیٹ ہے:

```xml
<robot name="my_robot">
  <!-- لنکس ریجڈ باڈیز کی وضاحت کرتے ہیں -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- جوائنٹس لنکس کو جوڑتے ہیں -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### 2.2 SDF (Simulation Description Format)

SDF گزبو کا نیٹیو فارمیٹ ہے جو URDF کی صلاحیتوں کو بڑھاتا ہے:

```xml
<sdf version="1.7">
  <model name="my_robot">
    <pose>0 0 0.5 0 0 0</pose>

    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.4</iyy>
          <iyz>0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.4 0.4 1 1</diffuse>
        </material>
      </visual>
    </link>

    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
    </plugin>
  </model>
</sdf>
```

## 3. فزکس سیمیولیشن اور خصوصیات

### 3.1 ماس اور اناشیا کی خصوصیات

سچی فزکس سیمیولیشن کے لیے ماس اور اناشیا کی خصوصیات اہم ہیں:

#### ماس کی خصوصیات
- ہر لنک کا کل ماس
- مثبت اور حقیقی ہونا چاہیے
- ڈائنامکس اور کنٹرول کے رویے کو متاثر کرتا ہے
- CAD ماڈلز سے ماپا یا گیا جا سکتا ہے

#### اناشیا ٹینسر
- 3x3 میٹرکس جو گھومنے والی اناشیا کی وضاحت کرتا ہے
- ماس کی تقسیم اور شکل پر منحصر ہے
- جسمانی طور پر درست ہونا چاہیے (مثبت دیفینٹ)
- عام شکلوں کے لیے ویسے زیادہ تر ویلیوز

### 3.2 فرکشن اور کونٹیکٹ ماڈلنگ

#### اسٹیٹک فرکشن (mu1, mu2)
- اشیاء کو آپس میں پھسلنے سے روکتا ہے
- زیادہ ویلیوز = پھسلن کے خلاف زیادہ مزاحمت
- پہیوں والے روبوٹس اور مینیپولیشن کے لیے اہم
- عام ویلیوز: 0.5-1.0 ربر پر خشک سطح

#### کونٹیکٹ کی خصوصیات
- kp (spring constant): کونٹیکٹ کی سختی
- kd (damping coefficient): توانائی کا انتظام
- Max_vel: زیادہ سے زیادہ کونٹیکٹ کی رفتار
- Min_depth: کم از کم کونٹیکٹ کی گہرائی

## 4. سینسر سیمیولیشن

### 4.1 کیمرہ سینسرز

کیمرہ سینسرز RGB اور ڈیپتھ ادراک کی نقل کرتے ہیں:

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

#### کیمرہ پیرامیٹرز
- افقی فیلڈ آف ویو (FOV)
- امیج ریزولیوشن (چوڑائی، اونچائی)
- کلپنگ پلینز (قریب، دور)
- اپ ڈیٹ کی شرح (فریم فی سیکنڈ)
- نوائز ماڈلنگ پیرامیٹرز

### 4.2 LIDAR اور رینج سینسرز

LIDAR سینسرز لیزر رینج فائنڈنگ کی نقل کرتے ہیں:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
</sensor>
```

#### LIDAR پیرامیٹرز
- زاویہ ریزولیوشن اور رینج
- کم از کم اور زیادہ سے زیادہ فاصلے
- کتنی ریز/بیم
- اپ ڈیٹ فریکوینسی
- نوائز کی خصوصیات

### 4.3 انرٹیل میزورمینٹ یونٹس (IMU)

IMU سینسرز ایکسلیرومیٹر اور جائیرو اسکوپس کی نقل کرتے ہیں:

```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
    </linear_acceleration>
  </imu>
</sensor>
```

## 5. دنیا اور ماحول کی ماڈلنگ

### 5.1 ورلڈ فائل کی ساخت

SDF ورلڈ فائلز مکمل سیمیولیشن ماحول کی وضاحت کرتی ہیں:

```xml
<sdf version="1.7">
  <world name="default">
    <!-- فزکس خصوصیات -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- لائٹنگ -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.0 -0.9</direction>
    </light>

    <!-- گراؤنڈ پلین -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- ماڈلز -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

    <!-- سٹیٹک اشیاء -->
    <model name="table">
      <pose>1 1 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.5 0.8</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## 6. ROS 2 انضمام

### 6.1 گزبو ROS پیکجز

`gazebo_ros_pkgs` ضروری انٹرفیس فراہم کرتے ہیں:

#### gazebo_ros
- گزبو کے ساتھ لانچ انضمام
- ROS 2 پیرامیٹر اور سروس انٹرفیسز
- ماڈل اسپوننگ اور مینجمنٹ

#### gazebo_plugins
- معیاری سینسرز اور ایکچوایٹرز کے پلگ انز
- ڈفرینشل ڈرائیو اور دیگر کنٹرولرز
- جوائنٹ سٹیٹ پبلشرز

### 6.2 کنٹرول انٹرفیس انضمام

ڈفرینشل ڈرائیو کنٹرولر مثال:

```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.3</wheel_separation>
  <wheel_diameter>0.1</wheel_diameter>
  <max_wheel_torque>20</max_wheel_torque>
  <max_wheel_acceleration>1.0</max_wheel_acceleration>
  <command_topic>cmd_vel</command_topic>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odom_frame>odom</odom_frame>
  <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

## 7. سیمیولیشن کی توثیق اور حقیقت کا فرق

### 7.1 حقیقت کا فرق کے ذرائع

#### فزکس تقریبات
- سادہ کونٹیکٹ ماڈلز
- غلط فرکشن کوائف
- ماس اور اناشیا کی غلطیاں
- سیمیولیشن ٹائم سٹیپنگ

#### سینسر کے فرق
- نوائز کی خصوصیات
- ریزولیوشن کے فرق
- لیٹنسی کے وقفے
- کیلیبریشن کی غلطیاں

#### ماحولیاتی عوامل
- سطح کی خصوصیات
- لائٹنگ کی حالتیں
- بیرونی متغیرات
- پہننے اور کمی کی حالتیں

### 7.2 پلیٹ فارم کی حکمت

#### ڈومین رینڈمائزیشن
- سیمیولیشن پیرامیٹرز کو رینڈم کرنا
- مختلف حالتیں میں تربیت کرنا
- جنرلائزیشن میں بہتری
- سیمیولیشن پر اضافی فٹنگ کو کم کرنا

#### سسٹم آئیڈنٹیفکیشن
- حقیقی روبوٹ پیرامیٹرز کو ناپنا
- سیمیولیشن کو حقیقت کے مطابق کرنا
- ترقی کا عمل
- توثیق ٹیسٹنگ

یہ نظریاتی بنیاد گزبو سیمیولیشن کے تصورات کی اہم سمجھ کو فراہم کرتی ہے جو جسمانی مصنوعی ذہانت کے نظام تیار کرنے کے لیے ضروری ہے۔ بعد کے حصے عملی مثالیں اور ایمپلیمنٹیشن گائیڈنس فراہم کریں گے۔