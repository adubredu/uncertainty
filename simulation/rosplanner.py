import requests
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
from fd import Fast_Downward
import rospy
from std_msgs.msg import Bool, String

class ROS_Planner:
	def __init__(self):
		rospy.init_node('Planner')
		self.plan_pub = rospy.Publisher('/planned', String, queue_size=1)
		rospy.Subscriber('/should_plan', Bool, self.plan_callback)
		rospy.spin()

	def plan_callback(self,data):
		if data.data:
			f = Fast_Downward()
			plan = f.plan('belief_domain.pddl', 'newprob.pddl')
			if plan is not None and len(plan)!=0:
				p=''
				for action in plan:
					p+=str(action)
					p+='\n'
			else:
				p='Fail'
			p = p[:-1]
			f=open('fdplan','w')
			f.write(p)
			f.close()
			# pd = String()
			# pd.data = p[:-1]
			# self.plan_pub.publish(pd)

if __name__=='__main__':
	r = ROS_Planner()