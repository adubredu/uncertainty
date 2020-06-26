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
		self.planning_time = 0
		rospy.Subscriber('/should_plan', Bool, self.plan_callback)
		rospy.spin()

	def plan_callback(self,data):
		if data.data:
			f = Fast_Downward()
			start = time.time()
			plan = f.plan('belief_domain.pddl', 'newprob.pddl')
			self.planning_time+= time.time() - start
			if plan is not None and len(plan)!=0:
				p=''
				for action in plan:
					p+=str(action)
					p+='\n'
			else:
				p='Faill'
			p = p[:-1]
			f=open('fdplan','w')
			f.write(p)
			f.close()
			print("\n PLANNING TIME: "+str(self.planning_time))
			# pd = String()
			# pd.data = p[:-1]
			# self.plan_pub.publish(pd)

if __name__=='__main__':
	r = ROS_Planner()