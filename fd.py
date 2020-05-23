from __future__ import print_function 

import os
import sys 
import subprocess
import time

class Fast_Downward:
    def __init__(self):
        self.search_options = {
            # Optimal
            'dijkstra': '--heuristic "h=blind(transform=adapt_costs(cost_type=NORMAL))" '
                        '--search "astar(h,cost_type=NORMAL,max_time=30)"',
            'max-astar': '--heuristic "h=hmax(transform=adapt_costs(cost_type=NORMAL))"'
                         ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
            'cerberus':  '--heuristic "h=hmax(transform=adapt_costs(cost_type=NORMAL))"'
                         ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',
            'lmcut-astar': '--heuristic "h=lmcut(transform=adapt_costs(cost_type=NORMAL))"'
                         ' --search "astar(h,cost_type=NORMAL,max_time=%s,bound=%s)"',

            # Suboptimal
            'ff-astar': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                        '--search "astar(h,cost_type=NORMAL,max_time=30)"',
            'ff-eager': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                        '--search "eager_greedy([h],max_time=%s,bound=%s)"',
            'ff-eager-pref': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                             '--search "eager_greedy([h],preferred=[h],max_time=%s,bound=%s)"',
            'ff-lazy': '--heuristic "h=ff(transform=adapt_costs(cost_type=PLUSONE))" '
                       '--search "lazy_greedy([h],preferred=[h],max_time=%s,bound=%s)"',
            'goal-lazy': '--heuristic "h=goalcount(transform=no_transform())" '
                         '--search "lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)"',
            'add-random-lazy': '--heuristic "h=add(transform=adapt_costs(cost_type=PLUSONE))" '
                               '--search "lazy_greedy([h],randomize_successors=True,max_time=%s,bound=%s)"',

            'ff-eager-tiebreak': '--heuristic "h=ff(transform=no_transform())" '
                                 '--search "eager(tiebreaking([h, g()]),reopen_closed=false,'
                                 'cost_type=NORMAL,max_time=%s,bound=%s, f_eval=sum([g(), h]))"', # preferred=[h],
            'ff-lazy-tiebreak': '--heuristic "h=ff(transform=no_transform())" '
                                 '--search "lazy(tiebreaking([h, g()]),reopen_closed=false,'
                                 'randomize_successors=True,cost_type=NORMAL,max_time=%s,bound=%s)"',  # preferred=[h],

            'ff-ehc': '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" '
                      '--search "ehc(h,preferred=[h],preferred_usage=RANK_PREFERRED_FIRST,'
                      'cost_type=NORMAL,max_time=%s,bound=%s)"',
            'astar' : '--search "astar(blind())" '
            # The key difference is that ehc resets the open list upon finding an improvement

        }
        for w in range(1, 1+5):
            self.search_options['ff-wastar{}'.format(w)] = '--heuristic "h=ff(transform=adapt_costs(cost_type=NORMAL))" ' \
                          '--search "lazy_wastar([h],preferred=[h],reopen_closed=true,boost=100,w={},' \
                          'preferred_successors_first=true,cost_type=NORMAL,max_time=%s,bound=%s)"'.format(w)
            self.search_options['cea-wastar{}'.format(w)] = '--heuristic "h=cea(transform=adapt_costs(cost_type=PLUSONE))" ' \
                           '--search "lazy_wastar([h],preferred=[h],reopen_closed=false,boost=1000,w={},' \
                           'preferred_successors_first=true,cost_type=PLUSONE,max_time=%s,bound=%s)"'.format(w)

        self.default_max_time = 30 # INF
        self.default_planner = 'dijkstra'
        self.max_fd_cost = 1e8
        self.plan_file = '/home/developer/uncertainty/simulation/sas_plan'
        self.fd_path = '/home/developer/garage/FastDownward/fast-downward.py'


    def plan(self,domain_path, problem_path,debug=True):
        start_time = time.time()
        command = self.fd_path+" "+domain_path+" "+problem_path+" "+self.search_options[self.default_planner]
        print(command)
        proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, cwd=None, close_fds=True)
        output, error = proc.communicate()
        if debug:
            print(output[:-1])
            print('Search runtime:', time.time() - start_time)
        time.sleep(3)
        plan = self.read(self.plan_file)
        return plan


    def read(self, filename):
      try:
        with open(filename, 'r') as f:
            plan = f.read()
      except:
        return None
      p = plan.split('\n')[:-2]
      retplan = []
      for act in p:
        tup = act.replace(')','').replace('(','').split(' ')
        tup = tuple(tup)
        retplan.append(tup)
      return retplan



if __name__ == '__main__':
    f = Fast_Downward()
    print(f.plan('pddl/dom.pddl', 'pddl/prob.pddl'))
