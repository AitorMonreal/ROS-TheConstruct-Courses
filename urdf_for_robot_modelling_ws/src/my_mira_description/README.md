ALWAYS WHEN YOU SIGN IN AGAIN IN ROBOT IGNITE ACADEMY, you will have to copy this folder AGAIN:

sudo rm -rf /usr/share/gazebo/models/my_mira_description
sudo cp -r /home/user/catkin_ws/src/my_mira_description /usr/share/gazebo/models/


To delete a spawned model from Gazebo:
rosrun my_mira_description delete_mira.py
OR
rosservice call /gazebo/delete_model "model_name: 'mira'"