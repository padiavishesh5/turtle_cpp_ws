# Turtlesim Catch Them All (ROS 2, C++)

`turtle1` chases the nearest spawned turtle and kills it. New turtles keep spawning at random positions.

## Packages
- `my_robot_interfaces` - `Turtle`, `TurtleArray` msgs and `CatchTurtle` srv
- `turtlesim_project_cpp` - `spawner` and `controller` nodes
- `my_robot_bringup` - launch file and parameters

## How it works
1. `spawner` calls `/spawn` at a fixed frequency and publishes all alive turtles on `alive_turtles`.
2. `controller` picks the nearest turtle and drives `turtle1` to it with a simple proportional controller.
3. When `turtle1` is within 0.5 of the target, `controller` calls the `catch_turtle` service.
4. `spawner` calls `/kill` and removes that turtle from the list.

## Build and run
```bash
colcon build
source install/setup.bash
ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.xml
```

## Parameters (`my_robot_bringup/config/catch_them_all_config.yaml`)
| Node | Parameter | Meaning |
|---|---|---|
| `turtle_controller` | `catch_closest_turtle_first` | `true`: chase nearest turtle, `false`: oldest first |
| `turtle_spawner` | `turtle_name_prefix` | Name prefix of spawned turtles |
| `turtle_spawner` | `spawn_frequency` | Turtles spawned per second |
