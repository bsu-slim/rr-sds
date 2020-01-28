import sys
import os
sys.path.append(os.environ['COZMO'])
import cozmo
import asyncio
import time
from cozmo.util import degrees, Angle, distance_mm, speed_mmps
import random
import numpy as np

class CozmoBehaviors():


  

    def __init__(self, robot, max_x=320, max_y=240):
        self.robot = robot
        self.objects = None
        self.robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
        self.robot.set_robot_volume(0.75)
        self.robot.move_lift(5)
        self.pivot_wheel = [25,-25, 40,-40]
        self.turn_wheel = [30, 40, 50, -30, -40, -15]
        self.durations = [1,2,3,4,5]
        self.top_object = None
        self.max_x = max_x
        self.max_y = max_y

    def set_current_objects(self, objects):
        self.objects = objects
        if len(self.objects) > 0:
            self.top_object = objects['object1']
            # self.robot.stop_all_motors()
        else:
            self.top_object = None
    
    def object_in_view(self):
        return self.top_object is not None

    def say(self, text='hello'):
        self.robot.say_text(text)

    def indicate_object(self, num_taps=1):
        '''
        When the robot is near an object, tap to indicate it
        '''
        robot = self.robot
        # Move lift down and tilt the head up
        robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
        robot.set_lift_height(0.0, accel=100.0, max_speed=100.0).wait_for_completed()

        for i in range(num_taps):
            robot.set_lift_height(0.3, accel=100.0, max_speed=100.0).wait_for_completed()
            robot.set_lift_height(0.0, accel=100.0, max_speed=100.0).wait_for_completed()

    def explore(self):
        robot = self.robot
        if random.choice([True,False]):
            l_speed = random.choice(self.pivot_wheel)
            # r_speed = random.choice(self.turn_wheel)
        else: 
            l_speed = random.choice(self.turn_wheel)
            # r_speed = random.choice(self.pivot_wheel)
        dur = random.choice(self.durations)
        robot.drive_wheels(l_wheel_speed=l_speed, r_wheel_speed=-l_speed, duration=dur)

    def find_follow_face(self):
        '''
        Method to rotate looking up and down until a face
        is found, then when a face is found, follow the face.
        '''
        robot = self.robot
        robot.move_lift(-3)
        robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE).wait_for_completed()

        face_to_follow = None

        while True:
            turn_action = None
            if face_to_follow:
                # start turning towards the face
                turn_action = robot.turn_towards_face(face_to_follow, in_parallel=True)

            if not (face_to_follow and face_to_follow.is_visible):
                # find a visible face, timeout if nothing found after a short while
                try:
                    robot.set_head_angle(cozmo.robot.MAX_HEAD_ANGLE, duration=1.0, in_parallel=True)
                    robot.turn_in_place(degrees(20), in_parallel=True) 
                    robot.set_head_angle(degrees(20), duration=1.0, in_parallel=True)
                    
                    #print('face count', robot.world.visible_face_count())
                    if robot.world.visible_face_count() > 0:
                        face_to_follow = next(robot.world.visible_faces, None)
                        robot.wait_for_all_actions_completed()
                    #face_to_follow = robot.world.wait_for_observed_face(timeout=30)
                except asyncio.TimeoutError:
                    print("Didn't find a face - exiting!")
                    return

            if turn_action:
                # Complete the turn action if one was in progress
                turn_action.wait_for_completed()

            time.sleep(.1)

    def object_check(self, go_box):
        xmin = go_box['x1'] / self.max_x # scale to 0-1
        xmax = go_box['x2'] / self.max_x # scale to 0-1
        ymin = go_box['y1'] / self.max_y # scale to 0-1
        ymax = go_box['y2'] / self.max_y # scale to 0-1
        height = ymax - ymin
        width = xmax - xmin
        height = height 
        width = width 
        area = height * width    
        x_center = (width/2) + xmin
        y_center = (height/2) + ymin
        if area > 0.40 or ymin < 0.01 or ymax > 0.60: 
            print('check FAILED', 'ymin: ', ymin, 'ymax: ', ymax, 'obj area: ', area)
            return (False, y_center, x_center)
        else: 
            print('check PASSED', 'ymin: ', ymin, 'ymax: ', ymax, 'obj area: ', area)
            return (True, y_center, x_center)

    def find_coordinates(self, go_box):
        real_obj = self.object_check(go_box)    
        x_center = real_obj[2]
        y_center = real_obj[1]
        if not real_obj[0]: 
            drive_dist = 0 
            turn_angle = 0
        else:
            if x_center > 0.56 or x_center < 0.44:
                x_diff = x_center - 0.5
                turn_angle = (x_diff * (-50)) / 2
            else: turn_angle = 0 
            if y_center > 0.48 or y_center < 0.36:
                y_diff = y_center - 0.42
                base = y_diff * (-10)
                drive_dist = (np.power(base,4)) * 25
            else: drive_dist = 0 
        return real_obj[0], drive_dist, turn_angle   

    def turn_toward_top_object(self):
        # while True:
        real_object, drive_dist, turn_angle = self.find_coordinates(self.top_object)
        print(real_object, ' turn: ', turn_angle, ' drive: ', drive_dist)
        if real_object:
            self.robot.turn_in_place(degrees(turn_angle)).wait_for_completed()

    def go_to_top_object(self):
        real_object, drive_dist, turn_angle = self.find_coordinates(self.top_object)
        if real_object:
            self.robot.drive_straight(distance_mm(drive_dist), 
                                speed_mmps(40), 
                                should_play_anim=False, 
                                in_parallel=True).wait_for_completed()

def test(robot : cozmo.robot.Robot):
    coz = CozmoBehaviors(robot)
    # for i in range(3):
    #     coz.explore()
    objs = {'object1': {'x1': 74, 'x2': 194, 'y1': 52, 'y2': 113, 'label': 'Office supplies'}}
    coz.set_current_objects(objs)
    coz.turn_toward_top_object()
    coz.go_to_top_object()
    

if __name__ == '__main__':
    cozmo.run_program(test, use_viewer=True, force_viewer_on_top=False)