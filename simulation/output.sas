begin_version
3
end_version
begin_metric
0
end_metric
14
begin_variable
var0
-1
8
Atom holding(h0)
Atom inbox(h0)
Atom inclutter(h0)
Atom on(h0, m0)
Atom on(h0, m1)
Atom on(h0, m2)
Atom on(h0, m3)
Atom on(h0, m4)
end_variable
begin_variable
var1
-1
8
Atom holding(m0)
Atom inbox(m0)
Atom inclutter(m0)
Atom on(m0, h0)
Atom on(m0, m1)
Atom on(m0, m2)
Atom on(m0, m3)
Atom on(m0, m4)
end_variable
begin_variable
var2
-1
8
Atom holding(m1)
Atom inbox(m1)
Atom inclutter(m1)
Atom on(m1, h0)
Atom on(m1, m0)
Atom on(m1, m2)
Atom on(m1, m3)
Atom on(m1, m4)
end_variable
begin_variable
var3
-1
8
Atom holding(m2)
Atom inbox(m2)
Atom inclutter(m2)
Atom on(m2, h0)
Atom on(m2, m0)
Atom on(m2, m1)
Atom on(m2, m3)
Atom on(m2, m4)
end_variable
begin_variable
var4
-1
8
Atom holding(m3)
Atom inbox(m3)
Atom inclutter(m3)
Atom on(m3, h0)
Atom on(m3, m0)
Atom on(m3, m1)
Atom on(m3, m2)
Atom on(m3, m4)
end_variable
begin_variable
var5
-1
8
Atom holding(m4)
Atom inbox(m4)
Atom inclutter(m4)
Atom on(m4, h0)
Atom on(m4, m0)
Atom on(m4, m1)
Atom on(m4, m2)
Atom on(m4, m3)
end_variable
begin_variable
var6
-1
2
Atom topfree(h0)
NegatedAtom topfree(h0)
end_variable
begin_variable
var7
-1
2
Atom topfree(m0)
NegatedAtom topfree(m0)
end_variable
begin_variable
var8
-1
2
Atom topfree(m1)
NegatedAtom topfree(m1)
end_variable
begin_variable
var9
-1
2
Atom topfree(m2)
NegatedAtom topfree(m2)
end_variable
begin_variable
var10
-1
2
Atom topfree(m3)
NegatedAtom topfree(m3)
end_variable
begin_variable
var11
-1
2
Atom handempty()
NegatedAtom handempty()
end_variable
begin_variable
var12
-1
2
Atom topfree(m4)
NegatedAtom topfree(m4)
end_variable
begin_variable
var13
0
2
Atom new-axiom@0()
NegatedAtom new-axiom@0()
end_variable
7
begin_mutex_group
7
11 0
0 0
1 0
2 0
3 0
4 0
5 0
end_mutex_group
begin_mutex_group
7
0 0
1 3
2 3
3 3
4 3
5 3
6 0
end_mutex_group
begin_mutex_group
7
0 3
1 0
2 4
3 4
4 4
5 4
7 0
end_mutex_group
begin_mutex_group
7
0 4
1 4
2 0
3 5
4 5
5 5
8 0
end_mutex_group
begin_mutex_group
7
0 5
1 5
2 5
3 0
4 6
5 6
9 0
end_mutex_group
begin_mutex_group
7
0 6
1 6
2 6
3 6
4 0
5 7
10 0
end_mutex_group
begin_mutex_group
7
0 7
1 7
2 7
3 7
4 7
5 0
12 0
end_mutex_group
begin_state
2
2
2
2
2
2
0
0
0
0
0
0
0
1
end_state
begin_goal
1
13 0
end_goal
84
begin_operator
pick-from h0 m0
0
4
0 11 0 1
0 0 3 0
0 6 0 1
0 7 -1 0
1
end_operator
begin_operator
pick-from h0 m1
0
4
0 11 0 1
0 0 4 0
0 6 0 1
0 8 -1 0
1
end_operator
begin_operator
pick-from h0 m2
0
4
0 11 0 1
0 0 5 0
0 6 0 1
0 9 -1 0
1
end_operator
begin_operator
pick-from h0 m3
0
4
0 11 0 1
0 0 6 0
0 6 0 1
0 10 -1 0
1
end_operator
begin_operator
pick-from h0 m4
0
4
0 11 0 1
0 0 7 0
0 6 0 1
0 12 -1 0
1
end_operator
begin_operator
pick-from m0 h0
0
4
0 11 0 1
0 1 3 0
0 6 -1 0
0 7 0 1
1
end_operator
begin_operator
pick-from m0 m1
0
4
0 11 0 1
0 1 4 0
0 7 0 1
0 8 -1 0
1
end_operator
begin_operator
pick-from m0 m2
0
4
0 11 0 1
0 1 5 0
0 7 0 1
0 9 -1 0
1
end_operator
begin_operator
pick-from m0 m3
0
4
0 11 0 1
0 1 6 0
0 7 0 1
0 10 -1 0
1
end_operator
begin_operator
pick-from m0 m4
0
4
0 11 0 1
0 1 7 0
0 7 0 1
0 12 -1 0
1
end_operator
begin_operator
pick-from m1 h0
0
4
0 11 0 1
0 2 3 0
0 6 -1 0
0 8 0 1
1
end_operator
begin_operator
pick-from m1 m0
0
4
0 11 0 1
0 2 4 0
0 7 -1 0
0 8 0 1
1
end_operator
begin_operator
pick-from m1 m2
0
4
0 11 0 1
0 2 5 0
0 8 0 1
0 9 -1 0
1
end_operator
begin_operator
pick-from m1 m3
0
4
0 11 0 1
0 2 6 0
0 8 0 1
0 10 -1 0
1
end_operator
begin_operator
pick-from m1 m4
0
4
0 11 0 1
0 2 7 0
0 8 0 1
0 12 -1 0
1
end_operator
begin_operator
pick-from m2 h0
0
4
0 11 0 1
0 3 3 0
0 6 -1 0
0 9 0 1
1
end_operator
begin_operator
pick-from m2 m0
0
4
0 11 0 1
0 3 4 0
0 7 -1 0
0 9 0 1
1
end_operator
begin_operator
pick-from m2 m1
0
4
0 11 0 1
0 3 5 0
0 8 -1 0
0 9 0 1
1
end_operator
begin_operator
pick-from m2 m3
0
4
0 11 0 1
0 3 6 0
0 9 0 1
0 10 -1 0
1
end_operator
begin_operator
pick-from m2 m4
0
4
0 11 0 1
0 3 7 0
0 9 0 1
0 12 -1 0
1
end_operator
begin_operator
pick-from m3 h0
0
4
0 11 0 1
0 4 3 0
0 6 -1 0
0 10 0 1
1
end_operator
begin_operator
pick-from m3 m0
0
4
0 11 0 1
0 4 4 0
0 7 -1 0
0 10 0 1
1
end_operator
begin_operator
pick-from m3 m1
0
4
0 11 0 1
0 4 5 0
0 8 -1 0
0 10 0 1
1
end_operator
begin_operator
pick-from m3 m2
0
4
0 11 0 1
0 4 6 0
0 9 -1 0
0 10 0 1
1
end_operator
begin_operator
pick-from m3 m4
0
4
0 11 0 1
0 4 7 0
0 10 0 1
0 12 -1 0
1
end_operator
begin_operator
pick-from m4 h0
0
4
0 11 0 1
0 5 3 0
0 6 -1 0
0 12 0 1
1
end_operator
begin_operator
pick-from m4 m0
0
4
0 11 0 1
0 5 4 0
0 7 -1 0
0 12 0 1
1
end_operator
begin_operator
pick-from m4 m1
0
4
0 11 0 1
0 5 5 0
0 8 -1 0
0 12 0 1
1
end_operator
begin_operator
pick-from m4 m2
0
4
0 11 0 1
0 5 6 0
0 9 -1 0
0 12 0 1
1
end_operator
begin_operator
pick-from m4 m3
0
4
0 11 0 1
0 5 7 0
0 10 -1 0
0 12 0 1
1
end_operator
begin_operator
pick-from-box h0
0
3
0 11 0 1
0 0 1 0
0 6 0 1
1
end_operator
begin_operator
pick-from-box m0
0
3
0 11 0 1
0 1 1 0
0 7 0 1
1
end_operator
begin_operator
pick-from-box m1
0
3
0 11 0 1
0 2 1 0
0 8 0 1
1
end_operator
begin_operator
pick-from-box m2
0
3
0 11 0 1
0 3 1 0
0 9 0 1
1
end_operator
begin_operator
pick-from-box m3
0
3
0 11 0 1
0 4 1 0
0 10 0 1
1
end_operator
begin_operator
pick-from-box m4
0
3
0 11 0 1
0 5 1 0
0 12 0 1
1
end_operator
begin_operator
pick-from-clutter h0
0
3
0 11 0 1
0 0 2 0
0 6 0 1
1
end_operator
begin_operator
pick-from-clutter m0
0
3
0 11 0 1
0 1 2 0
0 7 0 1
1
end_operator
begin_operator
pick-from-clutter m1
0
3
0 11 0 1
0 2 2 0
0 8 0 1
1
end_operator
begin_operator
pick-from-clutter m2
0
3
0 11 0 1
0 3 2 0
0 9 0 1
1
end_operator
begin_operator
pick-from-clutter m3
0
3
0 11 0 1
0 4 2 0
0 10 0 1
1
end_operator
begin_operator
pick-from-clutter m4
0
3
0 11 0 1
0 5 2 0
0 12 0 1
1
end_operator
begin_operator
put-in-box h0
0
3
0 11 -1 0
0 0 0 1
0 6 -1 0
1
end_operator
begin_operator
put-in-box m0
0
3
0 11 -1 0
0 1 0 1
0 7 -1 0
1
end_operator
begin_operator
put-in-box m1
0
3
0 11 -1 0
0 2 0 1
0 8 -1 0
1
end_operator
begin_operator
put-in-box m2
0
3
0 11 -1 0
0 3 0 1
0 9 -1 0
1
end_operator
begin_operator
put-in-box m3
0
3
0 11 -1 0
0 4 0 1
0 10 -1 0
1
end_operator
begin_operator
put-in-box m4
0
3
0 11 -1 0
0 5 0 1
0 12 -1 0
1
end_operator
begin_operator
put-in-clutter h0
0
3
0 11 -1 0
0 0 0 2
0 6 -1 0
1
end_operator
begin_operator
put-in-clutter m0
0
3
0 11 -1 0
0 1 0 2
0 7 -1 0
1
end_operator
begin_operator
put-in-clutter m1
0
3
0 11 -1 0
0 2 0 2
0 8 -1 0
1
end_operator
begin_operator
put-in-clutter m2
0
3
0 11 -1 0
0 3 0 2
0 9 -1 0
1
end_operator
begin_operator
put-in-clutter m3
0
3
0 11 -1 0
0 4 0 2
0 10 -1 0
1
end_operator
begin_operator
put-in-clutter m4
0
3
0 11 -1 0
0 5 0 2
0 12 -1 0
1
end_operator
begin_operator
put-on h0 m0
0
4
0 11 -1 0
0 0 0 3
0 6 -1 0
0 7 0 1
1
end_operator
begin_operator
put-on h0 m1
0
4
0 11 -1 0
0 0 0 4
0 6 -1 0
0 8 0 1
1
end_operator
begin_operator
put-on h0 m2
0
4
0 11 -1 0
0 0 0 5
0 6 -1 0
0 9 0 1
1
end_operator
begin_operator
put-on h0 m3
0
4
0 11 -1 0
0 0 0 6
0 6 -1 0
0 10 0 1
1
end_operator
begin_operator
put-on h0 m4
0
4
0 11 -1 0
0 0 0 7
0 6 -1 0
0 12 0 1
1
end_operator
begin_operator
put-on m0 h0
0
4
0 11 -1 0
0 1 0 3
0 6 0 1
0 7 -1 0
1
end_operator
begin_operator
put-on m0 m1
0
4
0 11 -1 0
0 1 0 4
0 7 -1 0
0 8 0 1
1
end_operator
begin_operator
put-on m0 m2
0
4
0 11 -1 0
0 1 0 5
0 7 -1 0
0 9 0 1
1
end_operator
begin_operator
put-on m0 m3
0
4
0 11 -1 0
0 1 0 6
0 7 -1 0
0 10 0 1
1
end_operator
begin_operator
put-on m0 m4
0
4
0 11 -1 0
0 1 0 7
0 7 -1 0
0 12 0 1
1
end_operator
begin_operator
put-on m1 h0
0
4
0 11 -1 0
0 2 0 3
0 6 0 1
0 8 -1 0
1
end_operator
begin_operator
put-on m1 m0
0
4
0 11 -1 0
0 2 0 4
0 7 0 1
0 8 -1 0
1
end_operator
begin_operator
put-on m1 m2
0
4
0 11 -1 0
0 2 0 5
0 8 -1 0
0 9 0 1
1
end_operator
begin_operator
put-on m1 m3
0
4
0 11 -1 0
0 2 0 6
0 8 -1 0
0 10 0 1
1
end_operator
begin_operator
put-on m1 m4
0
4
0 11 -1 0
0 2 0 7
0 8 -1 0
0 12 0 1
1
end_operator
begin_operator
put-on m2 h0
0
4
0 11 -1 0
0 3 0 3
0 6 0 1
0 9 -1 0
1
end_operator
begin_operator
put-on m2 m0
0
4
0 11 -1 0
0 3 0 4
0 7 0 1
0 9 -1 0
1
end_operator
begin_operator
put-on m2 m1
0
4
0 11 -1 0
0 3 0 5
0 8 0 1
0 9 -1 0
1
end_operator
begin_operator
put-on m2 m3
0
4
0 11 -1 0
0 3 0 6
0 9 -1 0
0 10 0 1
1
end_operator
begin_operator
put-on m2 m4
0
4
0 11 -1 0
0 3 0 7
0 9 -1 0
0 12 0 1
1
end_operator
begin_operator
put-on m3 h0
0
4
0 11 -1 0
0 4 0 3
0 6 0 1
0 10 -1 0
1
end_operator
begin_operator
put-on m3 m0
0
4
0 11 -1 0
0 4 0 4
0 7 0 1
0 10 -1 0
1
end_operator
begin_operator
put-on m3 m1
0
4
0 11 -1 0
0 4 0 5
0 8 0 1
0 10 -1 0
1
end_operator
begin_operator
put-on m3 m2
0
4
0 11 -1 0
0 4 0 6
0 9 0 1
0 10 -1 0
1
end_operator
begin_operator
put-on m3 m4
0
4
0 11 -1 0
0 4 0 7
0 10 -1 0
0 12 0 1
1
end_operator
begin_operator
put-on m4 h0
0
4
0 11 -1 0
0 5 0 3
0 6 0 1
0 12 -1 0
1
end_operator
begin_operator
put-on m4 m0
0
4
0 11 -1 0
0 5 0 4
0 7 0 1
0 12 -1 0
1
end_operator
begin_operator
put-on m4 m1
0
4
0 11 -1 0
0 5 0 5
0 8 0 1
0 12 -1 0
1
end_operator
begin_operator
put-on m4 m2
0
4
0 11 -1 0
0 5 0 6
0 9 0 1
0 12 -1 0
1
end_operator
begin_operator
put-on m4 m3
0
4
0 11 -1 0
0 5 0 7
0 10 0 1
0 12 -1 0
1
end_operator
12
begin_rule
6
0 1
1 1
2 1
3 1
4 3
5 4
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 3
5 5
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 3
5 6
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 4
5 3
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 4
5 5
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 4
5 6
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 5
5 3
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 5
5 4
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 5
5 6
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 6
5 3
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 6
5 4
13 1 0
end_rule
begin_rule
6
0 1
1 1
2 1
3 1
4 6
5 5
13 1 0
end_rule
