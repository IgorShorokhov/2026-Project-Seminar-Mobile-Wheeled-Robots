# Практическая работа 1

Шорохов Игорь, вариант 18

## Что сделано???

ros2 спавнит сначала одну черепашку (дефолтную), затем удаляет ее, и спавнит 2 черепашки в нужных нам координатах для рисования двухзначного числа (в случае этого проекта - 18)


## Сборка
```
cd ~/practices_ws
colcon build --symlink-install --packages-select practice_01_turtlesim
source install/setup.zsh
```

## Запуск
```
ros2 launch practice_01_turtlesim draw_number.launch.py
```

## Диагностика

```
ros2 node list
ros2 topic list
rqt_graph
```