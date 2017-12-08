import datetime
import rospy

import tic_tac_toe.game_agent
import time
from tic_tac_toe.msg import GameState

from threading import Event, Thread
from Queue import Queue

IGNORE_EVENTS_LESS_THAN = 1

class StateEstimator:
    def __init__(self, game_state_topic):
        self.state_change_event = Event()
        self.should_shutdown = Event()
        self.should_listen_for_state = False
        self.observation_queue = Queue()
        self.state_estimate = None
        self.state_estimate_started_stamp = None
        self.candidate_state = None
        self.candidate_state_started_stamp = None
        rospy.Subscriber(game_state_topic, GameState, self.game_state_callback)
        state_estimation_thread = Thread(target=self.process_game_state)
        state_estimation_thread.start()

    def game_state_callback(self, msg):
        self.observation_queue.put(tuple(msg.board_state))

    def process_game_state(self):
        while not rospy.is_shutdown() or self.should_shutdown.is_set():
            observation = self.observation_queue.get(True)
            if not self.should_listen_for_state:
                continue
            if observation != self.state_estimate:
                if observation != self.candidate_state:
                    self.candidate_state = observation
                    self.candidate_state_started_stamp = datetime.datetime.now()
                elif datetime.datetime.now() - self.candidate_state_started_stamp > datetime.timedelta(seconds=IGNORE_EVENTS_LESS_THAN):
                    self.state_estimate = observation
                    self.state_estimate_started_stamp = datetime.datetime.now()
                    rospy.loginfo("Detected transition to configuration {}".format(self.state_estimate))
                    self.state_change_event.set()
                    self.state_change_event.clear()
            else:
                self.candidate_state = None
                self.candidate_state_started_stamp = None
