# Assignment 4


Submitted by:
  Sagar Vishwakarma - svishwa2@binghamton.edu (BU - B00814586)
  Abhilasha Jain - ajain35@binghamton.edu (BU - B00817064)


TurtleBot Follows a Ball Based on Visual Input (RGB-D)

Run Command : roslaunch VishwakarmaS assign6.launch

Youtube : TurtleBot Ball Follower - https://youtu.be/FKLasmc6CHo


Reference :

Ball tracking with OpenCV
https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

Red Ball Image "/others/aster_red.jpg" when ROS topic "/camera/rgb/image_raw" data is saved to jpg file

Note:
    We used the open source tutorial to track a ball using camera frames.
    We then implemented the same using aster data from topic.
    The Red Ball appears blue when we tried to save the numpy data from the ros topic to a jpg image file.
    we made modification in our code to track the ball motion and added motion commands based on ball motion.
