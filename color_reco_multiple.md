...Распознование нескольких цветов

В прошлом уроке, мы познакомились, как же все-таки определить цвет платформы.

Но данный способ имеет один огромный минус: он узнает цвет только центра камеры, 1 пикселя, то есть, банально цвета, которые находятся рядом, то он их не видит.

img... https://imgur.com/a/rYMQeY5

На первой фотке, это наш код на прошлом уроке, а вторая фотка - в этом уроке. Как вы можете увидеть, на первой фотке берется только центр, а во второй, то весь цвет в целом. Мы будем пробовать с несколькими цветами.

Давайте же в этом уроке, мы научимся различать несколько цветов, а не один.

`Написание кода`

Приступим к написанию кода.

Мы все также будем использовать библиотеку OpenCV. Для начала, нам нужно взять изображение с топика, только потом уже и работать с ним перевести в HSV, чтобы распознать красный или желтый цвет. Создадим массив (dict_flag), где будет храниться HSV значение каждого цвета распознанного. Потом, мы ищем сам этот цвет, поставляя верные значения в inRange. После чего, если цвет распознался, то он должен попытаться найти контуры данного цвета. Потом, мы просто считаем кол-во пикселей с этим цветом, и уже ищем крайние точки по x и y, так же, как и его пиксели. Затем, просто вызываем drawContours, где контуры самого распознанного цвета выведятся в отдельную картинку.

```
import random # Для сохранения скриншота с отрисовкой контура
import cv2 # OpenCV
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Для взятия  скриншота

bridge = CvBridge()

dict_flag, i, detect_flag = [], -1, True
def flag_detect():
    global dict_flag, i, detect_flag
    imgg = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw_throttled', Image), 'bgr8')
    # Берем скриншот из топика камеры, если же у вас нет ноды тротла, то уберите его

    dict_flag.append([[0,0,0], [0,0,0]]) # Для того, чтобы значение отдельного цвета считать в массив
    
    if detect_flag:
        detect_flag = False
        i += 1 # Увеличиваем значение нашего массиваа

    img = cv2.cvtColor(imgg, cv2.COLOR_BGR2HSV)
    
    # Распознавание красного
    frame = cv2.inRange(img, (0, 89, 150), (15, 255, 255)) # Мин/Макс значения по красному
    cnt = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    # Для нахождения конутров самого цвета
    try: 
        for c in cnt:
            moments = cv2.moments(c, 1) # Считаем каждый пиксель с фотки       
            sum_pixel = moments['m00'] # Сумма пикселей
            if sum_pixel > 10: # Если пикселей с красным больше 10, то будет считать, проверка
                dict_flag[i][0] = [sum_pixel, int(moments['m10'] / sum_pixel), int(moments['m01'] / sum_pixel)]
                # Считаем значение самого HSV; m10 - по оси x, где m01 - по оси y
                detect_flag = True
                cv2.drawContours(imgg, [c], 0, (255,255,255), 3) # Рисуем контуры, для откладки на фото
    except:
        pass

    # Распознавание желтого цвета 
    frame = cv2.inRange(img, (25, 150, 200), (35, 255, 255)) # Мин/Макс значения по желтому
    cnt = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    try: 
        for c in cnt:
            moments = cv2.moments(c, 1)       
            sum_pixel = moments['m00']
            if sum_pixel > 10:
                dict_flag[i][1] = [sum_pixel, int(moments['m10'] / sum_pixel), int(moments['m01'] / sum_pixel)]
                detect_flag = True
                cv2.drawContours(imgg, [c], 0, (0,0,0), 3)
    except:
        pass
    
    print(dict_flag) # Вывод итогового HSV значения каждого цвета
    
    cv2.imwrite("/home/clover/image{0}.png".format(random.randint(0,1000)), imgg)
    # Для сохранения фотки где будет храниться контур отрисованный
```

Как итог нашего кода, вы можете увидеть на фотке снизу, где прикреплена отрисовка контуров.

img... https://imgur.com/a/xZfad0z
