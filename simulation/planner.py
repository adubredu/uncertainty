import requests
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
from fd import Fast_Downward

class Planner:
	def __init__(self):
		self.url = 'http://solver.planning.domains/solve'


	def plan(self, domain_path, problem_path):
		data = {
				'domain': open(domain_path, 'r').read(),
				'problem': open(problem_path,'r').read()
				}

		resp = requests.post(self.url,verify=False, json=data).json()
		
		if resp['status'] == 'ok':
			plan = [tuple(act['name'].replace(')','').replace('(','').split(' ')) for act in resp['result']['plan']]
			return plan
		else:
			print(resp)
			return None




if __name__=='__main__':
	# for i in range(10):
		# p = Planner()
		# prob = open('newprob.pddl','r').read()
		# open('newprob.pddl','w').write(prob)
	f = Planner()
	print(f.plan('belief_domain.pddl', 'newprob.pddl'))
	time.sleep(2)