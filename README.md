# RoboCV
Проект системы технического зрения для беспилотных ТС.

## Содержание
- [Технологии](#технологии)
- [Установка](#установка)
- [Использование](#использование)
- [Примечание](#примечание)

## Технологии
- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html)
- [Carla 0.9.13](https://carla.readthedocs.io/en/0.9.13/)
- [ROS-Bridge 0.9.12](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)
- [Carla Scenario Runner ](https://carla-scenariorunner.readthedocs.io/en/latest/)

## Уставнока
Для уставноки:
1) Убедитесь в наличии уставноленного ROS2 Foxy.

2) Установите и проверьте установку Carla Simulator, согласно [руководству](https://carla.readthedocs.io/en/0.9.13/start_quickstart/). При установке пакетным менеджером, симулятор будет находиться в /opt/carla-simulator/.

3) Склонируйте репозиторий RoboCV в удобное место командой:
```sh
$ git clone --recurse-submodules https://github.com/aicommunity/RoboCV.git
# После клонирования проверьте, что находитесь в нужной ветке!
```

4) Выполните команды:
```sh
$ pip install --user -r requirements.txt
```		
```sh
$ echo "export CARLA_ROOT=/opt/carla-simulator" >> ~/.bashrc
# Корневая папка Carla Simulator
		
```	
```sh
$ echo "export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/(*ВСТАВИТЬ ИМЯ ФАЙЛА*):$CARLA_ROOT/PythonAPI/carla" >> ~/.bashrc
# Путь внутри корневой папки Carla Simulator. Пример файла: сarla-0.9.13-py3.7-linux-x86_64.egg
		
```	
```sh
$ echo "export SCENARIO_RUNNER_PATH=*ПУТЬ ДО ПАПКИ С РЕПОЗИТОРИЕМ*/RoboCV/scenario_runner-0.9.13" >> ~/.bashrc
# Вставить путь до папки
		
```	
```sh
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r
# При нахождении в папке car_ws репозитория и при настроенной среде ROS2 (source /opt/ros/foxy/setup.bash)
		
```	

## Использование
Для сборки всех пакетов, при нахождении в директории car_ws, выполнить команду:
```sh
$ colcone build
		
```	

## Примечание
При возникновении ошибок при билде, смотрим сообщения в терминале и фиксим. Вероятнее всего, не хватает какого либо модуля, которой позднее будет добавлен в requirements.
В идеале, засунуть все это дело в докер, но это потом.
