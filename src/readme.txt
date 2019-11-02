# Assignment 3

Part 1:

Navigation In the Lab LNG210

  roslaunch VishwakarmaS assign3_bringup.launch
  roslaunch VishwakarmaS assign3_navigation.launch  # touchdown.py

  Youtube:
    TurtleBot Navigating Through Goal Locations - https://www.youtube.com/watch?v=swwSebC-2J0
    TurtleBot Navigating Through Door (Going through the door only if its open) - https://www.youtube.com/watch?v=JJ5ceWMy5I8


Part 2:

Voice Navigation outside Lab LNG210

  roslaunch VishwakarmaS assign3_bringup.launch
  roslaunch VishwakarmaS assign3_voice.launch  # obeyer.py

  Youtube:
    TurtleBot Voice Navigation - https://www.youtube.com/watch?v=BUEU2VML0ss

  Note:
      We tried using the "google api" to recognize the voice commands.
      It was very slow and api response where taking a lot of time thus resulting in jerking motion of turtlebot.

      We also tried "pocketsphinx", using the https://github.com/shiqizhang6/ros_voice_control package,
      Here the recognition was quick but process was prone to surrounding noices and not recognizing different pronunciations.

      We also implemented a dialog system where it ask 'Yes' or 'No' before performing in commanded motion action.
      To see the dialog system code, Please checkout commit e49c90affe2744f27925e98ea5bc096cca37d6d3
