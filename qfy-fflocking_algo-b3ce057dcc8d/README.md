~~the visualization marker ARROW is static when it is initialized, the start point and end point can not be changed later.~~
Here points attribute of arrow marker are not assigned correctly.

Problem: long distance between uav and target , can move through obstacle; but if short distance between uav and target, the obstacle can be avoided appropriately.
Solution: can try to threshold the attractive potential force.

Problem: marker of uav can not be displayed properly.(track the motion of uav frame).

Solution: can try to change the marker of uav from interactive marker to typical marker.


input:
无人机位置
无人机速度
目标物位置
目标物速度
障碍物位置
障碍物速度

output：
无人机下一时刻位置
