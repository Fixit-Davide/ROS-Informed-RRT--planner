<div id="header" align="center">
  <img src="https://media.giphy.com/media/M9gbBd9nbDrOTu1Mqx/giphy.gif" width="100"/>
</div>
# TODO 

* Change the waypoint generation: Use mavros_msgs/Trajectory -> mavros_msgs/PositionTarget -> geometry_msgs/Point Message to send the point to the drone. 
* Add a check function to see if the drone is already flying, in this case avoid the GUIDED - TAKEOFF procedure. (DONE)

mavros_msgs/PositionTarget.msg: 
https://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html

In order to compute the velocity use the LQR controller in the other repo. 