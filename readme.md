`1. Intro`

В этой главе мы познакомимся: как писать код, разберемся и обсудим каждую строчку кода, научимся компьютерному зрению, блочному программированию, как вообще использовать платформу Клевер и т.д.

Так как мы уже познакомились с системой позиционирования в качестве теории, то в следующем модуле мы уже познакомимся на практике. Мы будем запускать наш код с системой позиционирования Aruco меток. Конечно, мы могли запустить и в другой системе позиционирования, такую как Optical Flow (с ней мы тоже познакомимся чуть позже). Но банально в симуляторе у нас есть Aruco поле, и почти всегда его все используют. Даже на соревнованиях где требуется запуск дрона в автономном полете, то всегда есть Aruco метки (конечно, есть исключения, но они очень редки).

Весь код мы будем писать на языке Python. Он является самым простым языком в качестве обучения и освоения.

Давайте же заранее запустим симулятор Gazebo и Visual Studio Code:

img...

`2. One more addition`

После того, как вы запустили симулятор, то нам нужно будет создать Python файл в Visual Studio Code чтобы мы могли его запускать.

Для этого:

Перейдите во верхнюю вкладку "File".
Нажмите на "New File".
Выберите посередине сверху "New Python file".
Затем снова перейдите во вкладку "File" и нажмите на "Save as".
Назовите ваш файл и выберите директории где хотите его сохранить ваш файл.

img...

Как вы сохранили файл, то вы можете спокойно приступать к ознакомлению кода.

Давайте же я просто вам прикреплю код и объясню вам его, потом, весь код, мы уже будем писать сами:

```
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
```

Самые первые три строчки, то есть с import до from: это просто подключение библиотек, без которых мы попросту не сможем запустить наш дрон. Они создают ноды и "функции", благодаря которым мы можем понимать в каких координатах находится дрон (get_telemetry), чтобы дрон передвигался в воздухе (navigate) и чтобы садился (land), и другие.

Пятая строчка у нас это просто создание нода, так скажем программы для запуска нашего дрона.

Последущие все строчки, это и есть те самые "функции" благодаря которым, мы можем взлететь, передвигаться и т.д...

`3. Writing the first code`

В прошлом модуле, мы разобрали самые важные строчки кода, но если же просто их скопировать и запустить, то ничего не получится и ничего не взлетит.

Теперь, давайте напишем код, где наш дрон будет взлетать (просто в конце того кода дополните этими строчками):

```
navigate(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)

rospy.sleep(5)

navigate(x=1, y=1, z=1, speed=0.5, frame_id='aruco_map')

rospy.sleep(5)

land()
```

У этого кода есть минусы, но давайте же сначала разберем сам код.

Самая первая строчка означает то, что наш дрон взлетит на высоту 1 метр (z=1), у него включатся моторы (auto_arm=True) и все это произойдет в системе координат 'body', то есть, по значениям которые передаются в get_telemetry. Спросите же, почему не в Aruco метках? Но ответ прост: когда наш дрон не взлетел, то он банально их не видит, соответственно, он не может их распознать. Поэтому, весь взлет мы будем делать по 'body'.

Потом, после того как сразу исполнилась наша программа взлета, то сразу исполняется другая, то есть, rospy.sleep. В этом и есть сам минус этого кода. Мы не знаем, когда наш дрон будет на высоте 1 метр, поэтому, мы используем замедление на 5 секунд до следующей команды, чтобы дрон сразу не исполнил другую. После объяснениям мы познакомимся, как же это исправить.

Потом, наш дрон летит на 1 метку по x, и на 1 метку по y. Он будет лететь это со скоростью 0.5. Но при взлете мы используем скорость 1, ибо нашему дрону будет банально очень сложно взлететь со скоростью 0.5, а вот когда сам дрон уже передвигается, то скорость 0.5 для него является оптимальным значением. Передвигается он по Aruco меткам, соответственно на поле они должны быть.

`4. Correcting the first code`

Давайте же теперь исправим наш код, то есть сделаем так, чтобы пока наш дрон не будет находиться в центре Aruco метки, то другой код не будет исполняться (без rospy.sleep).

Для начала, нам нужно будет создать функцию navigate_wait, где будем писать весь наш код. В саму функцию мы впишем параметры, которые есть у navigate. Это нам нужно чтобы когда мы прописывали navigate_wait, то могли сами ставить скорость передвижения, координаты и т.д...

Конечно же, нам тут понадобится navigate, но мы потом пропишем что другой код не будет исполняться, пока сам дрон не будет в центре Aruco метки.

```
def navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
```

Вы можете использовать сами какую высоту использовать по стандарту, лично для меня, мне всегда удобно использовать по стандарту z=1.5 и систему координат по Aruco (вы сами можете использовать свои значения которые вам будут удобны):

```
def navigate_wait(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
```

Дальше, нам понадобится написать чтобы мы узнавали телеметрию, то есть координаты самой Aruco метки и дрона, и потом мы будем сравнивать значения Aruco метки по x, y, z и самого дрона и если условно разрыв будет больше 0.2 (tolerance), то наш дрон будет дальше двигаться к этой Aruco метки. Нужно учесть, что это действие должно быть всегда активно, а саму проверку, находится ли он на этих координатах будем делать каждые 0.2 секунды. Чтобы узнать значение координат, по которым наш дрон хочет полететь, то это сделаем в с помощью получение телеметрии самой координаты. Учтите, что мы будем использовать библиотеку math, для того, чтобы узнать в центре ли координат находится наш дрон. Давайте же напишем код:

```
# Эту строчку кода в самое начало, где подключение библиотек
import math

def navigate_wait(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown(): # Чтобы было всегда активно
        telem = get_telemetry(frame_id='navigate_target') # Получение координат куда двигается дрон
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break # Если не в центре координат, по которым хочется передвигаться, будет повторяться
        rospy.sleep(0.2) # Каждые 0.2 секунды делать проверку
```

После этого, вы спокойно можете передвигаться с помощью этой функции по координатам. Чтобы вызвать, это будет выглядеть примерно так:

```
navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True)
navigate_wait(x=1, y=1, frame_id='aruco_map')
navigate_wait(x=2, y=1.5, frame_id='aruco_map')
```

А весь код, который будет готов к запуску, должен выглядеть так:

```
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown(): # Чтобы было всегда активно
        telem = get_telemetry(frame_id='navigate_target') # Получение коордиант куда двигается дрон
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break # Если не в центре координат, по которым хочется передвигаться, будет повторяться
        rospy.sleep(0.2) # Каждые 0.2 секунды делать проверку


navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True)
navigate_wait(x=1, y=1, frame_id='aruco_map')
navigate_wait(x=2, y=1.5, frame_id='aruco_map')
land()
```

После этого, вы можете этот код спокойно запускать в симуляторе или на реальном дроне, но лучше всего сначала весь код тестите в симуляторе.

`5. Making our code a little prettier`

Давайте же сделаем наш код немного красивее и более понятливым. Ведь когда наш код будет становиться больше и он будет выглядеть довольно громоздким, то банально наши вызовы функций по типу navigate_wait, мы можем потерять или не увидеть.

Для этого, мы просто создадим функцию main, где весь наш код будет храниться. Потом, эту функцию main мы просто вызовем и она исполнится, это должно выглядеть примерно так:

```
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown(): # Чтобы было всегда активно
        telem = get_telemetry(frame_id='navigate_target') # Получение коордиант куда двигается дрон
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break # Если не в центре координат, по которым хочется передвигаться, будет повторяться
        rospy.sleep(0.2) # Каждые 0.2 секунды делать проверку

def main():
    navigate_wait(z=1.5, speed=1, frame_id='body', auto_arm=True)
    navigate_wait(x=1, y=1, frame_id='aruco_map')
    navigate_wait(x=2, y=1.5, frame_id='aruco_map')
    land()

if __name__ == "__main__":
    main()
```

Это вы должны заменить где у вас вызывался просто запуск navigate_wait.

`6. Running our code`

После того, как мы написали весь наш код, то нам нужно будет его запустить. Сначала мы разберем как его запустить в симуляторе и на дроне.

Давайте приступим сначала как его запустить в симуляторе. Это будет самый легкий способ, но на самом дроне будет немного посложнее, но все равно довольно просто.

В самом симуляторе, вы скорее всего будете писать весь ваш код в Visual Studio Code. Поэтому, после того, как вы написали весь ваш код, то просто справа сверху будет кнопочка запуска вашего кода (как медиаплеер). Вы на нее нажимаете и все.

img...

Теперь, давайте же приступим как это сделать на дроне.

Для начала, вы должны быть подключены к самому Wi-Fi дрона, поэтому он должен быть уже подключен к АКБ.

Потом, вы должны ввести в браузере "192.168.11.1" и открыть раздел "Open Web terminal". Прописать команду nano и название файла с припиской .py - у вас появится окно где не будет ничего написано и сюда вы должны вставить свой код. Это делается клавишами Ctrl + Shift + V. После того, как вы вставили весь ваш код, вы должны его запустить. Это делается с помощью команды python3 (если же у вас образ 0.22 и выше, иначе просто python) и название самого файла. Условно, я назвал файл main.py, и чтобы запустить, я должен написать "python3 main.py".
