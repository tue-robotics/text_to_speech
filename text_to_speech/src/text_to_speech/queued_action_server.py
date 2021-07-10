from abc import ABC, abstractmethod
from actionlib import ActionServer
from collections import deque
import rospy
import threading
import traceback


class QueuedActionServer(ABC):
    """
    Action Server wrapper which implements a buffers to handle all requests in
    receiving order
    """
    def __init__(self, ns, action_spec, execute_rate=5, threaded_feedback=True, feedback_rate=5):
        """

        :param ns:
        :param action_spec:
        :param execute_rate: ROS parameter for execute loop rate
        :param threaded_feedback:
        :param feedback_rate:
        """
        self.execute_rate = execute_rate

        self._active_gh = None

        self.buffer = deque()
        self.lock = threading.RLock()
        self.execute_condition = threading.Condition(self.lock)

        self.need_to_terminate = False

        self._server = ActionServer(ns, action_spec, self._goal_cb, self._cancel_cb, auto_start=False)

        self._feedback_type = type(self._server.ActionFeedback().feedback)
        self._result_type = type(self._server.ActionResultType())

        self.execute_thread = threading.Thread(target=self.execute_loop)
        self.execute_thread.start()

        if threaded_feedback:
            self.feedback_rate = feedback_rate
            self.feedback_thread = threading.Thread(target=self.publish_feedback_loop)
            self.feedback_thread.start()

        self._server.start()

    def __del__(self):
        self.need_to_terminate = True

        if hasattr(self, "execute_thread") and self.execute_thread:
            self.execute_thread.join()

        if hasattr(self, "feedback_thread") and self.feedback_thread:
            self.feedback_thread.join()

    def _goal_cb(self, gh):
        with self._server.lock, self.lock:
            rospy.logdebug(f"New goal:\n{gh.get_goal()}")
            self.buffer.append(gh)

    def _cancel_cb(self, gh):
        with self._server.lock, self.lock:
            rospy.logdebug(f"Cancelling goal:\n{gh.get_goal()}")

            if gh != self.active_gh:
                try:
                    self.buffer.remove(gh)
                except ValueError:
                    rospy.logerr(f"Tried to remove goal:\n{gh}\n, which is not in the que:\n{self.buffer}")

            try:
                self.cancel_cb(gh)
            except Exception as e:
                rospy.logerr(f"Failure during cancel_cb of goal: `{gh}`:\n{e}\n{traceback.format_exc()}")

    def publish_feedback_loop(self):
        rate = rospy.Rate(self.feedback_rate)

        while not rospy.is_shutdown():
            if self.need_to_terminate:
                break

            if self.active:
                with self._server.lock, self.lock:
                    goal = self.active_gh

                try:
                    feedback = self.generate_feedback(goal)
                except Exception as e:
                    rospy.logerr(f"Exception in your generate_feedback: {e}\n{traceback.format_exc()}")
                else:
                    if feedback is not None:
                        assert isinstance(feedback, self._feedback_type),\
                            f"Return of generate_feedback should be a {self._feedback_type}"
                        goal.publish_feedback(feedback)

            rate.sleep()

    def get_new_goal(self):
        with self.lock:
            try:
                return self.buffer.popleft()
            except IndexError:
                return None

    def execute_loop(self):
        rate = rospy.Rate(self.execute_rate)

        while not rospy.is_shutdown():
            if self.need_to_terminate:
                break

            assert not self.active, "There shouldn't be an active goal at a new iteration of the execution loop"

            self.active_gh = self.get_new_goal()
            if self.active_gh:
                with self._server.lock, self.lock:
                    gh = self.active_gh
                    gh.set_accepted()

                try:
                    result = self.execute_cb(gh)
                except Exception as e:
                    rospy.logerr(f"Exception in your execute callback: {e}\n{traceback.format_exc()}")
                    with self._server.lock, self.lock:
                        gh.set_aborted(None, f"Exception in execute callback: {e}")
                        self.active_gh = None
                else:
                    rospy.loginfo(f"Succeeded: {gh.get_goal_id()}", )
                    if result is not None:
                        assert isinstance(result, self._result_type),\
                            f"Return of execute_cb should be a {self._result_type}"
                    with self._server.lock, self.lock:
                        gh.set_succeeded(result)
                        self.active_gh = None

            rate.sleep()

    @property
    def active_gh(self):
        return self._active_gh

    @active_gh.setter
    def active_gh(self, gh):
        self._active_gh = gh

    @property
    def active(self):
        return bool(self.active_gh)

    @abstractmethod
    def execute_cb(self, gh):
        raise NotImplementedError

    @abstractmethod
    def cancel_cb(self, gh):
        raise NotImplementedError

    @abstractmethod
    def generate_feedback(self, gh):
        raise NotImplementedError
