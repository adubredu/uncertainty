begin_version
3
end_version
begin_metric
0
end_metric
13
begin_variable
var0
-1
2
Atom boxfull()
NegatedAtom boxfull()
end_variable
begin_variable
var1
-1
7
Atom holding(h0)
Atom inbox(h0)
Atom inclutter(h0)
Atom on(h0, h1)
Atom on(h0, h2)
Atom on(h0, h3)
Atom on(h0, m0)
end_variable
begin_variable
var2
-1
7
Atom holding(h1)
Atom inbox(h1)
Atom inclutter(h1)
Atom on(h1, h0)
Atom on(h1, h2)
Atom on(h1, h3)
Atom on(h1, m0)
end_variable
begin_variable
var3
-1
7
Atom holding(h2)
Atom inbox(h2)
Atom inclutter(h2)
Atom on(h2, h0)
Atom on(h2, h1)
Atom on(h2, h3)
Atom on(h2, m0)
end_variable
begin_variable
var4
-1
7
Atom holding(h3)
Atom inbox(h3)
Atom inclutter(h3)
Atom on(h3, h0)
Atom on(h3, h1)
Atom on(h3, h2)
Atom on(h3, m0)
end_variable
begin_variable
var5
-1
7
Atom holding(m0)
Atom inbox(m0)
Atom inclutter(m0)
Atom on(m0, h0)
Atom on(m0, h1)
Atom on(m0, h2)
Atom on(m0, h3)
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
Atom topfree(h1)
NegatedAtom topfree(h1)
end_variable
begin_variable
var8
-1
2
Atom topfree(h2)
NegatedAtom topfree(h2)
end_variable
begin_variable
var9
-1
2
Atom topfree(h3)
NegatedAtom topfree(h3)
end_variable
begin_variable
var10
-1
2
Atom handempty()
NegatedAtom handempty()
end_variable
begin_variable
var11
-1
2
Atom topfree(m0)
NegatedAtom topfree(m0)
end_variable
begin_variable
var12
0
2
Atom new-axiom@0()
NegatedAtom new-axiom@0()
end_variable
6
begin_mutex_group
6
10 0
1 0
2 0
3 0
4 0
5 0
end_mutex_group
begin_mutex_group
6
1 0
2 3
3 3
4 3
5 3
6 0
end_mutex_group
begin_mutex_group
6
1 3
2 0
3 4
4 4
5 4
7 0
end_mutex_group
begin_mutex_group
6
1 4
2 4
3 0
4 5
5 5
8 0
end_mutex_group
begin_mutex_group
6
1 5
2 5
3 5
4 0
5 6
9 0
end_mutex_group
begin_mutex_group
6
1 6
2 6
3 6
4 6
5 0
11 0
end_mutex_group
begin_state
0
1
1
1
1
2
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
12 0
end_goal
60
begin_operator
pick-from h0 h1
0
4
0 10 0 1
0 1 3 0
0 6 0 1
0 7 -1 0
1
end_operator
begin_operator
pick-from h0 h2
0
4
0 10 0 1
0 1 4 0
0 6 0 1
0 8 -1 0
1
end_operator
begin_operator
pick-from h0 h3
0
4
0 10 0 1
0 1 5 0
0 6 0 1
0 9 -1 0
1
end_operator
begin_operator
pick-from h0 m0
0
4
0 10 0 1
0 1 6 0
0 6 0 1
0 11 -1 0
1
end_operator
begin_operator
pick-from h1 h0
0
4
0 10 0 1
0 2 3 0
0 6 -1 0
0 7 0 1
1
end_operator
begin_operator
pick-from h1 h2
0
4
0 10 0 1
0 2 4 0
0 7 0 1
0 8 -1 0
1
end_operator
begin_operator
pick-from h1 h3
0
4
0 10 0 1
0 2 5 0
0 7 0 1
0 9 -1 0
1
end_operator
begin_operator
pick-from h1 m0
0
4
0 10 0 1
0 2 6 0
0 7 0 1
0 11 -1 0
1
end_operator
begin_operator
pick-from h2 h0
0
4
0 10 0 1
0 3 3 0
0 6 -1 0
0 8 0 1
1
end_operator
begin_operator
pick-from h2 h1
0
4
0 10 0 1
0 3 4 0
0 7 -1 0
0 8 0 1
1
end_operator
begin_operator
pick-from h2 h3
0
4
0 10 0 1
0 3 5 0
0 8 0 1
0 9 -1 0
1
end_operator
begin_operator
pick-from h2 m0
0
4
0 10 0 1
0 3 6 0
0 8 0 1
0 11 -1 0
1
end_operator
begin_operator
pick-from h3 h0
0
4
0 10 0 1
0 4 3 0
0 6 -1 0
0 9 0 1
1
end_operator
begin_operator
pick-from h3 h1
0
4
0 10 0 1
0 4 4 0
0 7 -1 0
0 9 0 1
1
end_operator
begin_operator
pick-from h3 h2
0
4
0 10 0 1
0 4 5 0
0 8 -1 0
0 9 0 1
1
end_operator
begin_operator
pick-from h3 m0
0
4
0 10 0 1
0 4 6 0
0 9 0 1
0 11 -1 0
1
end_operator
begin_operator
pick-from m0 h0
0
4
0 10 0 1
0 5 3 0
0 6 -1 0
0 11 0 1
1
end_operator
begin_operator
pick-from m0 h1
0
4
0 10 0 1
0 5 4 0
0 7 -1 0
0 11 0 1
1
end_operator
begin_operator
pick-from m0 h2
0
4
0 10 0 1
0 5 5 0
0 8 -1 0
0 11 0 1
1
end_operator
begin_operator
pick-from m0 h3
0
4
0 10 0 1
0 5 6 0
0 9 -1 0
0 11 0 1
1
end_operator
begin_operator
pick-from-box h0
0
4
0 0 -1 1
0 10 0 1
0 1 1 0
0 6 0 1
1
end_operator
begin_operator
pick-from-box h1
0
4
0 0 -1 1
0 10 0 1
0 2 1 0
0 7 0 1
1
end_operator
begin_operator
pick-from-box h2
0
4
0 0 -1 1
0 10 0 1
0 3 1 0
0 8 0 1
1
end_operator
begin_operator
pick-from-box h3
0
4
0 0 -1 1
0 10 0 1
0 4 1 0
0 9 0 1
1
end_operator
begin_operator
pick-from-box m0
0
4
0 0 -1 1
0 10 0 1
0 5 1 0
0 11 0 1
1
end_operator
begin_operator
pick-from-clutter h0
0
3
0 10 0 1
0 1 2 0
0 6 0 1
1
end_operator
begin_operator
pick-from-clutter h1
0
3
0 10 0 1
0 2 2 0
0 7 0 1
1
end_operator
begin_operator
pick-from-clutter h2
0
3
0 10 0 1
0 3 2 0
0 8 0 1
1
end_operator
begin_operator
pick-from-clutter h3
0
3
0 10 0 1
0 4 2 0
0 9 0 1
1
end_operator
begin_operator
pick-from-clutter m0
0
3
0 10 0 1
0 5 2 0
0 11 0 1
1
end_operator
begin_operator
put-in-box h0
1
0 1
3
0 10 -1 0
0 1 0 1
0 6 -1 0
1
end_operator
begin_operator
put-in-box h1
1
0 1
3
0 10 -1 0
0 2 0 1
0 7 -1 0
1
end_operator
begin_operator
put-in-box h2
1
0 1
3
0 10 -1 0
0 3 0 1
0 8 -1 0
1
end_operator
begin_operator
put-in-box h3
1
0 1
3
0 10 -1 0
0 4 0 1
0 9 -1 0
1
end_operator
begin_operator
put-in-box m0
1
0 1
3
0 10 -1 0
0 5 0 1
0 11 -1 0
1
end_operator
begin_operator
put-in-clutter h0
0
3
0 10 -1 0
0 1 0 2
0 6 -1 0
1
end_operator
begin_operator
put-in-clutter h1
0
3
0 10 -1 0
0 2 0 2
0 7 -1 0
1
end_operator
begin_operator
put-in-clutter h2
0
3
0 10 -1 0
0 3 0 2
0 8 -1 0
1
end_operator
begin_operator
put-in-clutter h3
0
3
0 10 -1 0
0 4 0 2
0 9 -1 0
1
end_operator
begin_operator
put-in-clutter m0
0
3
0 10 -1 0
0 5 0 2
0 11 -1 0
1
end_operator
begin_operator
put-on h0 h1
0
4
0 10 -1 0
0 1 0 3
0 6 -1 0
0 7 0 1
1
end_operator
begin_operator
put-on h0 h2
0
4
0 10 -1 0
0 1 0 4
0 6 -1 0
0 8 0 1
1
end_operator
begin_operator
put-on h0 h3
0
4
0 10 -1 0
0 1 0 5
0 6 -1 0
0 9 0 1
1
end_operator
begin_operator
put-on h0 m0
0
4
0 10 -1 0
0 1 0 6
0 6 -1 0
0 11 0 1
1
end_operator
begin_operator
put-on h1 h0
0
4
0 10 -1 0
0 2 0 3
0 6 0 1
0 7 -1 0
1
end_operator
begin_operator
put-on h1 h2
0
4
0 10 -1 0
0 2 0 4
0 7 -1 0
0 8 0 1
1
end_operator
begin_operator
put-on h1 h3
0
4
0 10 -1 0
0 2 0 5
0 7 -1 0
0 9 0 1
1
end_operator
begin_operator
put-on h1 m0
0
4
0 10 -1 0
0 2 0 6
0 7 -1 0
0 11 0 1
1
end_operator
begin_operator
put-on h2 h0
0
4
0 10 -1 0
0 3 0 3
0 6 0 1
0 8 -1 0
1
end_operator
begin_operator
put-on h2 h1
0
4
0 10 -1 0
0 3 0 4
0 7 0 1
0 8 -1 0
1
end_operator
begin_operator
put-on h2 h3
0
4
0 10 -1 0
0 3 0 5
0 8 -1 0
0 9 0 1
1
end_operator
begin_operator
put-on h2 m0
0
4
0 10 -1 0
0 3 0 6
0 8 -1 0
0 11 0 1
1
end_operator
begin_operator
put-on h3 h0
0
4
0 10 -1 0
0 4 0 3
0 6 0 1
0 9 -1 0
1
end_operator
begin_operator
put-on h3 h1
0
4
0 10 -1 0
0 4 0 4
0 7 0 1
0 9 -1 0
1
end_operator
begin_operator
put-on h3 h2
0
4
0 10 -1 0
0 4 0 5
0 8 0 1
0 9 -1 0
1
end_operator
begin_operator
put-on h3 m0
0
4
0 10 -1 0
0 4 0 6
0 9 -1 0
0 11 0 1
1
end_operator
begin_operator
put-on m0 h0
0
4
0 10 -1 0
0 5 0 3
0 6 0 1
0 11 -1 0
1
end_operator
begin_operator
put-on m0 h1
0
4
0 10 -1 0
0 5 0 4
0 7 0 1
0 11 -1 0
1
end_operator
begin_operator
put-on m0 h2
0
4
0 10 -1 0
0 5 0 5
0 8 0 1
0 11 -1 0
1
end_operator
begin_operator
put-on m0 h3
0
4
0 10 -1 0
0 5 0 6
0 9 0 1
0 11 -1 0
1
end_operator
4
begin_rule
5
1 1
2 1
3 1
4 1
5 3
12 1 0
end_rule
begin_rule
5
1 1
2 1
3 1
4 1
5 4
12 1 0
end_rule
begin_rule
5
1 1
2 1
3 1
4 1
5 5
12 1 0
end_rule
begin_rule
5
1 1
2 1
3 1
4 1
5 6
12 1 0
end_rule
