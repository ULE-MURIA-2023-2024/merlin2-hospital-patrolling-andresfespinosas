#!/usr/bin/env python3

import rclpy
import time
from typing import List
from .pddl import room_type, room_patrolled, room_at
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_fsm_action import Merlin2FsmAction, Merlin2BasicStates
from kant_dto import PddlObjectDto, PddlConditionEffectDto
from yasmin import Blackboard, CbState
from yasmin_ros.basic_outcomes import SUCCEED
from geometry_msgs.msg import Twist
 
class Merlin2RoomPatrolFsmAction(Merlin2FsmAction):
     
    def __init__(self)-> None:
         
        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type, "wp")
        
        super().__init__("room_patrol")
        
        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.rotate_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.add_state(
            "PREPARING_TEXT",
            CbState([SUCCEED], self.prepare_text),
            transitions={
                SUCCEED:"SPEAKING"
            }
        )
        self.add_state(
            "SPEAKING",
            tts_state
        )
        
    def rotate(self, blackboard: Blackboard) -> str:
        twist_msg = Twist()
        twist_msg.angular.z = 1.0  # Set the angular velocity to rotate the robot

        # Rotate the robot for 15 seconds
        while time.time() < blackboard.start_time + 15:
            self.rotate_pub.publish(twist_msg)
            time.sleep(0.1)

        # Stop the robot
        twist_msg.angular.z = 0.0
        self.rotate_pub.publish(twist_msg)
        
        return "SUCCEED"
    
    def prepare_text(self, blackboard:Blackboard)->str:
        room_name = blackboard.merlin2_action_goal.objects[0][-1]
        blackboard.text = f"Strecher {room_name} patrolled"
        return SUCCEED
    
    def create_parameters(self) -> List(PddlObjectDto):
        return [self._room, self._wp]
    
    def create_conditions(self)-> List[PddlCondiitonEffectDto]:
        
        cond_1 = PddlCondiitonEffectDto(
            room_patrolled,
            [self._room],
            PddlCondiitonEffectDto=AT_START,
            is_negative= False
        )
        
        cond_2 = PddlCondiitonEffectDto(
            robot_at,
            [self._wp],
            PddlCondiitonEffectDto=AT_START
        )
        
        cond_3 = PddlCondiitonEffectDto(
            room_at,
            [self._room, self._wp],
            PddlCondiitonEffectDto=AT_START,
            is_negative=True
        )
        
        return [cond_1,cond_2,cond_3]
    
    def create_efects(self) -> List(PddlCondiitonEffectDto):
        
        effect_1 = PddlCondiitonEffectDto(
            room_patrolled,
            [self._room],
            time= PddlCondiitonEffectDto.AT_END
        )
        return [effect_1]
    
def main():
    rclpy.init()
    
    node = Merlin2RoomPatrolFsmAction()
    
    node.join_spin()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()