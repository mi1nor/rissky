`Добавление QR на поле`

Перед тем, как начать писать код, давайте же познакомимся, как положить наш QR-код на поле в симуляторе.

Для начала, нам нужно будем создать сам QR-код, это можно будет сделать на этом сайте. В данном уроке мы будет разбирать, как можно взять слова-цветов и включим нужную подцветку для этого, поэтому, пишите ваши слова цветов на английском языке.

img...

https://www.the-qrcode-generator.com

Для этого, нам нужно перейти в директорию и заменить Aruco метку на наш QR-код. Это нужно перейти в данную (написана ниже) директорию, и заменить наше фото Aruco метки на QR-код. Нужно чтобы наш QR-код был в jpg формате, это обязательно. Так же, вы должны будете назвать ваш QR-код таким же именем, как и сама Aruco метка:

```
catkin_ws/src/clover/clover_simulation/models/aruco_cmit_txt/materials/textures
```

img...

После чего, если у вас была открыта Gazebo, то просто перезапустите ее.

`Написание кода`

Давайте же приступим к ознакомлению, как же мы будем писать код.

Для начала, самым главным, стоит сказать, что мы будем использовать библиотеку pyzbar, которая нам позволяет сразу распознать наш QR-код. То есть, на скрине снизу, вы видите, как идет команда decode и просто фото, тем самым, эта библиотека распознает наш QR-код и дает выданные значения, то есть текст и т.д.

img...

Наша задача будет в том, чтобы использовать эту библиотеку, и сделать так, чтобы он постоянно брал картинку из камеры, то есть распознавал, а когда он распознает, то просто сохранить текст. Давайте же посмотрим, как это сделать:

```
from pyzbar import pyzbar # Библиотека, чтобы распознать наш QR-код
from cv_bridge import CvBridge # Чтобы взять изображение из топика и распознать его
from sensor_msgs.msg import Image # Чтобы брать изображение из топика нашей камеры

bridge = CvBridge() # Чтобы взять изображение из топика и распознать его

qr_code = '' # Иницилизируем чтобы сохранить наш текст
def image_callback(data):
    global qr_code # Делаем ее глобальной, чтобы потом можно было использовать
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8') # Берем скрин из топика и переводим его в изображ.
    barcodes = pyzbar.decode(cv_image) # Детектим что в QR из скрина, есть ли сам QR
    for barcode in barcodes: # Детектим что в QR
        qr_code = barcode.data.decode("utf-8") # Берем параметр самого pyzbar чтобы узнать текст
        print("QR = ", qr_code) # Выводим текст, что распознался в QR


image_sub = rospy.Subscriber("main_camera/image_raw", Image, image_callback, queue_size=1)
# Если у вас включен throttle кадров, то пишем вместо image_raw - image_raw_throttled
```

img...

Если же вы боитесь, что неправильно как-то вставите этот код или напишите, то приложу полный, чтобы вам было более понятно:

```
import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import math

rospy.init_node('flight')

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

def navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        telem_auto = get_telemetry()

        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            rospy.sleep(1)
            print("REACHED | X = {} | Y = {}".format(telem_auto.x, telem_auto.y))
            break

        rospy.sleep(0.2)

qr_code = ''
def image_callback(data):
    global qr_code
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        qr_code = barcode.data.decode("utf-8")
        print("QR = ", qr_code)
        # Если же вы хотите, чтобы он написал 1 раз, qr_code
        # то пишем print("QR = ", qr_code) 
        # уже после while qr_code==''...

image_sub = rospy.Subscriber("main_camera/image_raw", Image, image_callback, queue_size=1)
# Если у вас включен throttle кадров, то пишем вместо image_raw - image_raw_throttled

def main():
    print("WE'RE GOING TO START")
    navigate_wait(z=1, speed=1, frame_id='body', auto_arm=True)
    navigate_wait(x=1) # Тут вы выбираете уже свой путь, какой Aruco вы поменяли
    
    # Можно так же сделать дополнительную проверку, если не находит
    # то добавить i в цикл, по типу:
    # i = 0
    # while i < 50 and qr_code == '' and not rospy.is_shutdown()
    #   rospy.sleep(0.4)
    #   i++
    while qr_code == '' and not rospy.is_shutdown():
        rospy.sleep(0.4)

    land()

if __name__ == "__main__":
    main()
```

`Работа с данными из QR`

В прошлом модуле, мы разобрали, как же задетектить QR-код и вывести данные. Но с этими данными, мы банально ничего не сможем сделать, то есть, как же их использовать.

Я, для большей наглядности, как же их использовать, поместил в QR-код разные названия цветов. Мы будем, считывать эти цвета и включать светодиодную подцветку в тот цвет, который мы распознаем.

Давайте же приступим к написанию кода:

```
    QR_detected = qr_code.split() # Мы будем брать всякое слово, пока оно не достигнет пробела
    # То есть, условно если у нас цвет red green blue, то он будет их помещать, как:
    # "red", "green", "blue" и мы будем считывать каждый отдельный цвет поочередно

    colors = list(map(str, QR_detected)) # берем в массив каждый цвет, то есть если red у нас
    # в qr первый, то colors[0] будет равен red, green стоит вторым, то colors[1] = green...

    for i in range(len(colors)): # Сколько у нас слов, столько раз мы и будем повторять цикл
        if colors[i] == 'red': # Если в слове есть красный, то ставится подсветка
            set_effect(r=255, g=0, b=0) # Меняется подсветка
        if colors[i] == 'yellow':
            set_effect(r=255, g=255, b=0)
        if colors[i] == 'green':
            set_effect(r=0, g=255, b=0)
        if colors[i] == 'blue':
            set_effect(r=0, g=0, b=255)

        print("CHANGING COLOR") # Для откладки

        rospy.sleep(3) # Будет меняться каждые 3 секунды подсветка
```

Сам весь код, должен выглядеть вот так:

```
import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pyzbar import pyzbar
import math

rospy.init_node('flight')

bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

def navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        telem_auto = get_telemetry()

        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            rospy.sleep(1)
            print("REACHED | X = {} | Y = {}".format(telem_auto.x, telem_auto.y))
            break

        rospy.sleep(0.2)

qr_code = ''
def image_callback(data):
    global qr_code
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        qr_code = barcode.data.decode("utf-8")
        print("QR = ", qr_code)
        # Если же вы хотите, чтобы он написал 1 раз, qr_code
        # то пишем print("QR = ", qr_code) 
        # уже после while qr_code==''...

image_sub = rospy.Subscriber("main_camera/image_raw", Image, image_callback, queue_size=1)
# Если у вас включен throttle кадров, то пишем вместо image_raw - image_raw_throttled

def main():
    print("WE'RE GOING TO START")
    navigate_wait(z=1, speed=1, frame_id='body', auto_arm=True)
    navigate_wait(x=1) # Тут вы выбираете уже свой путь, какой Aruco вы поменяли
    
    # Можно так же сделать дополнительную проверку, если не находит
    # то добавить i в цикл, по типу:
    # i = 0
    # while i < 50 and qr_code == '' and not rospy.is_shutdown()
    #   rospy.sleep(0.4)
    #   i++
    while qr_code == '' and not rospy.is_shutdown():
        rospy.sleep(0.4)

    QR_detected = qr_code.split()
    colors = list(map(str, QR_detected))
    for i in range(len(colors)):
        if colors[i] == 'red':
            set_effect(r=255, g=0, b=0)
        if colors[i] == 'yellow':
            set_effect(r=255, g=255, b=0)
        if colors[i] == 'green':
            set_effect(r=0, g=255, b=0)
        if colors[i] == 'blue':
            set_effect(r=0, g=0, b=255)

        print("CHANGING COLOR")

        rospy.sleep(3)


    land()

if __name__ == "__main__":
    main()
```