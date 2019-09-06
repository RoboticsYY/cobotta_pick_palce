# cobotta_pick_place

## Terminal 1

Pull docker image

```shell
docker pull xiaotaoqi/denso:kinetic
```

Make ready X windows

```shell
setup_docker_display.sh
```

Run docker image

```shell
docker run -t -i --rm -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw -e XAUTHORITY=/tmp/.docker.xauth -e DISPLAY --network=host --name cobotta xiaotaoqi/denso:kinetic bash
```

Bring up cobotta ros control and moveit

```shell
source devel/setup.bash
roslaunch denso_robot_bringup cobotta_bringup.launch sim:=false
```

## Terminal 2

Open new docker terminal

```shell
docker exec -i -t cobotta bash
```

Bring up cobotta bcap service

```shell
source devel/setup.bash
roslaunch bcap_service bcap_service.launch ip_address:=192.168.0.10
```

## Terminal 3

Open new docker terminal

```shell
docker exec -i -t cobotta bash
```

Bring up cobotta gripper server

```shell
source devel/setup.bash
roslaunch cobotta_pick_place gripper.launch
```
