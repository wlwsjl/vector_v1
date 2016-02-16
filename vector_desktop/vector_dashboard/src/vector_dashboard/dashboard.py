import rospy
from vector_msgs.msg import *

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize
from .battery_widget import BatteryWidget

class SegwayDashboard(Dashboard):
    def setup(self, context):
        self.message = None

        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        self._bat = BatteryWidget("Battery Power")
        self._bat_sub = rospy.Subscriber('/vector/feedback/battery', Battery, self._battery_callback)
        self._last_dashboard_message_time = rospy.get_time()
        self._system_charging = False

    def get_widgets(self):
        return [[MonitorDashWidget(self.context), ConsoleDashWidget(self.context)], [self._bat, self._propulsion_bat]]

    def _battery_callback(self,msg):
        self._bat.update_perc(msg.battery_soc)
        self._bat.update_time(msg.battery_soc)
        if (0x1000 == (msg.battery_status & 0x1000)):
            self._bat.set_charging(True)
            self._system_charging = True 
        else:
            self._bat.set_charging(False)
            self._system_charging = False
        
    def shutdown_dashboard(self):
        self._bat_sub.unregister()
