begin_version
3
end_version
begin_metric
0
end_metric
17
begin_variable
var0
-1
2
Atom on(h4, h4)
NegatedAtom on(h4, h4)
end_variable
begin_variable
var1
-1
2
Atom boxfull()
NegatedAtom boxfull()
end_variable
begin_variable
var2
-1
8
Atom holding(h0)
Atom inbox(h0)
Atom inclutter(h0)
Atom on(h0, h1)
Atom on(h0, h2)
Atom on(h0, h4)
Atom on(h0, m0)
Atom on(h0, m1)
end_variable
begin_variable
var3
-1
8
Atom holding(h1)
Atom inbox(h1)
Atom inclutter(h1)
Atom on(h1, h0)
Atom on(h1, h2)
Atom on(h1, h4)
Atom on(h1, m0)
Atom on(h1, m1)
end_variable
begin_variable
var4
-1
8
Atom holding(h2)
Atom inbox(h2)
Atom inclutter(h2)
Atom on(h2, h0)
Atom on(h2, h1)
Atom on(h2, h4)
Atom on(h2, m0)
Atom on(h2, m1)
end_variable
begin_variable
var5
-1
8
Atom holding(m0)
Atom inbox(m0)
Atom inclutter(m0)
Atom on(m0, h0)
Atom on(m0, h1)
Atom on(m0, h2)
Atom on(m0, h4)
Atom on(m0, m1)
end_variable
begin_variable
var6
-1
8
Atom holding(m1)
Atom inbox(m1)
Atom inclutter(m1)
Atom on(m1, h0)
Atom on(m1, h1)
Atom on(m1, h2)
Atom on(m1, h4)
Atom on(m1, m0)
end_variable
begin_variable
var7
-1
2
Atom inclutter(h4)
NegatedAtom inclutter(h4)
end_variable
begin_variable
var8
-1
2
Atom inbox(h4)
NegatedAtom inbox(h4)
end_variable
begin_variable
var9
-1
3
Atom on(h4, h0)
Atom topfree(h0)
<none of those>
end_variable
begin_variable
var10
-1
3
Atom on(h4, h1)
Atom topfree(h1)
<none of those>
end_variable
begin_variable
var11
-1
3
Atom on(h4, h2)
Atom topfree(h2)
<none of those>
end_variable
begin_variable
var12
-1
3
Atom on(h4, m0)
Atom topfree(m0)
<none of those>
end_variable
begin_variable
var13
-1
3
Atom on(h4, m1)
Atom topfree(m1)
<none of those>
end_variable
begin_variable
var14
-1
3
Atom handempty()
Atom holding(h4)
<none of those>
end_variable
begin_variable
var15
-1
2
Atom topfree(h4)
NegatedAtom topfree(h4)
end_variable
begin_variable
var16
0
2
Atom new-axiom@0()
NegatedAtom new-axiom@0()
end_variable
6
begin_mutex_group
7
14 0
14 1
2 0
3 0
4 0
5 0
6 0
end_mutex_group
begin_mutex_group
7
2 0
3 3
4 3
5 3
6 3
9 0
9 1
end_mutex_group
begin_mutex_group
7
2 3
3 0
4 4
5 4
6 4
10 0
10 1
end_mutex_group
begin_mutex_group
7
2 4
3 4
4 0
5 5
6 5
11 0
11 1
end_mutex_group
begin_mutex_group
7
2 6
3 6
4 6
5 0
6 7
12 0
12 1
end_mutex_group
begin_mutex_group
7
2 7
3 7
4 7
5 7
6 0
13 0
13 1
end_mutex_group
begin_state
1
0
1
1
1
1
2
0
1
1
1
1
1
1
1
0
1
end_state
begin_goal
1
16 0
end_goal
86
begin_operator
pick-from h0 h1
0
4
0 14 0 2
0 2 3 0
0 9 1 2
0 10 -1 1
1
end_operator
begin_operator
pick-from h0 h2
0
4
0 14 0 2
0 2 4 0
0 9 1 2
0 11 -1 1
1
end_operator
begin_operator
pick-from h0 h4
0
4
0 14 0 2
0 2 5 0
0 9 1 2
0 15 -1 0
1
end_operator
begin_operator
pick-from h0 m0
0
4
0 14 0 2
0 2 6 0
0 9 1 2
0 12 -1 1
1
end_operator
begin_operator
pick-from h0 m1
0
4
0 14 0 2
0 2 7 0
0 9 1 2
0 13 -1 1
1
end_operator
begin_operator
pick-from h1 h0
0
4
0 14 0 2
0 3 3 0
0 9 -1 1
0 10 1 2
1
end_operator
begin_operator
pick-from h1 h2
0
4
0 14 0 2
0 3 4 0
0 10 1 2
0 11 -1 1
1
end_operator
begin_operator
pick-from h1 h4
0
4
0 14 0 2
0 3 5 0
0 10 1 2
0 15 -1 0
1
end_operator
begin_operator
pick-from h1 m0
0
4
0 14 0 2
0 3 6 0
0 10 1 2
0 12 -1 1
1
end_operator
begin_operator
pick-from h1 m1
0
4
0 14 0 2
0 3 7 0
0 10 1 2
0 13 -1 1
1
end_operator
begin_operator
pick-from h2 h0
0
4
0 14 0 2
0 4 3 0
0 9 -1 1
0 11 1 2
1
end_operator
begin_operator
pick-from h2 h1
0
4
0 14 0 2
0 4 4 0
0 10 -1 1
0 11 1 2
1
end_operator
begin_operator
pick-from h2 h4
0
4
0 14 0 2
0 4 5 0
0 11 1 2
0 15 -1 0
1
end_operator
begin_operator
pick-from h2 m0
0
4
0 14 0 2
0 4 6 0
0 11 1 2
0 12 -1 1
1
end_operator
begin_operator
pick-from h2 m1
0
4
0 14 0 2
0 4 7 0
0 11 1 2
0 13 -1 1
1
end_operator
begin_operator
pick-from h4 h0
0
5
0 14 0 1
0 8 -1 1
0 7 -1 1
0 9 0 1
0 15 0 1
1
end_operator
begin_operator
pick-from h4 h1
0
5
0 14 0 1
0 8 -1 1
0 7 -1 1
0 10 0 1
0 15 0 1
1
end_operator
begin_operator
pick-from h4 h2
0
5
0 14 0 1
0 8 -1 1
0 7 -1 1
0 11 0 1
0 15 0 1
1
end_operator
begin_operator
pick-from h4 h4
1
15 0
4
0 14 0 1
0 8 -1 1
0 7 -1 1
0 0 0 1
1
end_operator
begin_operator
pick-from h4 m0
0
5
0 14 0 1
0 8 -1 1
0 7 -1 1
0 12 0 1
0 15 0 1
1
end_operator
begin_operator
pick-from h4 m1
0
5
0 14 0 1
0 8 -1 1
0 7 -1 1
0 13 0 1
0 15 0 1
1
end_operator
begin_operator
pick-from m0 h0
0
4
0 14 0 2
0 5 3 0
0 9 -1 1
0 12 1 2
1
end_operator
begin_operator
pick-from m0 h1
0
4
0 14 0 2
0 5 4 0
0 10 -1 1
0 12 1 2
1
end_operator
begin_operator
pick-from m0 h2
0
4
0 14 0 2
0 5 5 0
0 11 -1 1
0 12 1 2
1
end_operator
begin_operator
pick-from m0 h4
0
4
0 14 0 2
0 5 6 0
0 12 1 2
0 15 -1 0
1
end_operator
begin_operator
pick-from m0 m1
0
4
0 14 0 2
0 5 7 0
0 12 1 2
0 13 -1 1
1
end_operator
begin_operator
pick-from m1 h0
0
4
0 14 0 2
0 6 3 0
0 9 -1 1
0 13 1 2
1
end_operator
begin_operator
pick-from m1 h1
0
4
0 14 0 2
0 6 4 0
0 10 -1 1
0 13 1 2
1
end_operator
begin_operator
pick-from m1 h2
0
4
0 14 0 2
0 6 5 0
0 11 -1 1
0 13 1 2
1
end_operator
begin_operator
pick-from m1 h4
0
4
0 14 0 2
0 6 6 0
0 13 1 2
0 15 -1 0
1
end_operator
begin_operator
pick-from m1 m0
0
4
0 14 0 2
0 6 7 0
0 12 -1 1
0 13 1 2
1
end_operator
begin_operator
pick-from-box h0
0
4
0 1 -1 1
0 14 0 2
0 2 1 0
0 9 1 2
1
end_operator
begin_operator
pick-from-box h1
0
4
0 1 -1 1
0 14 0 2
0 3 1 0
0 10 1 2
1
end_operator
begin_operator
pick-from-box h2
0
4
0 1 -1 1
0 14 0 2
0 4 1 0
0 11 1 2
1
end_operator
begin_operator
pick-from-box h4
0
5
0 1 -1 1
0 14 0 1
0 8 0 1
0 7 -1 1
0 15 0 1
1
end_operator
begin_operator
pick-from-box m0
0
4
0 1 -1 1
0 14 0 2
0 5 1 0
0 12 1 2
1
end_operator
begin_operator
pick-from-box m1
0
4
0 1 -1 1
0 14 0 2
0 6 1 0
0 13 1 2
1
end_operator
begin_operator
pick-from-clutter h0
0
3
0 14 0 2
0 2 2 0
0 9 1 2
1
end_operator
begin_operator
pick-from-clutter h1
0
3
0 14 0 2
0 3 2 0
0 10 1 2
1
end_operator
begin_operator
pick-from-clutter h2
0
3
0 14 0 2
0 4 2 0
0 11 1 2
1
end_operator
begin_operator
pick-from-clutter h4
0
4
0 14 0 1
0 8 -1 1
0 7 0 1
0 15 0 1
1
end_operator
begin_operator
pick-from-clutter m0
0
3
0 14 0 2
0 5 2 0
0 12 1 2
1
end_operator
begin_operator
pick-from-clutter m1
0
3
0 14 0 2
0 6 2 0
0 13 1 2
1
end_operator
begin_operator
put-in-box h0
1
1 1
3
0 14 -1 0
0 2 0 1
0 9 -1 1
1
end_operator
begin_operator
put-in-box h1
1
1 1
3
0 14 -1 0
0 3 0 1
0 10 -1 1
1
end_operator
begin_operator
put-in-box h2
1
1 1
3
0 14 -1 0
0 4 0 1
0 11 -1 1
1
end_operator
begin_operator
put-in-box h4
1
1 1
3
0 14 1 0
0 8 -1 0
0 15 -1 0
1
end_operator
begin_operator
put-in-box m0
1
1 1
3
0 14 -1 0
0 5 0 1
0 12 -1 1
1
end_operator
begin_operator
put-in-box m1
1
1 1
3
0 14 -1 0
0 6 0 1
0 13 -1 1
1
end_operator
begin_operator
put-in-clutter h0
0
3
0 14 -1 0
0 2 0 2
0 9 -1 1
1
end_operator
begin_operator
put-in-clutter h1
0
3
0 14 -1 0
0 3 0 2
0 10 -1 1
1
end_operator
begin_operator
put-in-clutter h2
0
3
0 14 -1 0
0 4 0 2
0 11 -1 1
1
end_operator
begin_operator
put-in-clutter h4
0
3
0 14 1 0
0 7 -1 0
0 15 -1 0
1
end_operator
begin_operator
put-in-clutter m0
0
3
0 14 -1 0
0 5 0 2
0 12 -1 1
1
end_operator
begin_operator
put-in-clutter m1
0
3
0 14 -1 0
0 6 0 2
0 13 -1 1
1
end_operator
begin_operator
put-on h0 h1
0
4
0 14 -1 0
0 2 0 3
0 9 -1 1
0 10 1 2
1
end_operator
begin_operator
put-on h0 h2
0
4
0 14 -1 0
0 2 0 4
0 9 -1 1
0 11 1 2
1
end_operator
begin_operator
put-on h0 h4
0
4
0 14 -1 0
0 2 0 5
0 9 -1 1
0 15 0 1
1
end_operator
begin_operator
put-on h0 m0
0
4
0 14 -1 0
0 2 0 6
0 9 -1 1
0 12 1 2
1
end_operator
begin_operator
put-on h0 m1
0
4
0 14 -1 0
0 2 0 7
0 9 -1 1
0 13 1 2
1
end_operator
begin_operator
put-on h1 h0
0
4
0 14 -1 0
0 3 0 3
0 9 1 2
0 10 -1 1
1
end_operator
begin_operator
put-on h1 h2
0
4
0 14 -1 0
0 3 0 4
0 10 -1 1
0 11 1 2
1
end_operator
begin_operator
put-on h1 h4
0
4
0 14 -1 0
0 3 0 5
0 10 -1 1
0 15 0 1
1
end_operator
begin_operator
put-on h1 m0
0
4
0 14 -1 0
0 3 0 6
0 10 -1 1
0 12 1 2
1
end_operator
begin_operator
put-on h1 m1
0
4
0 14 -1 0
0 3 0 7
0 10 -1 1
0 13 1 2
1
end_operator
begin_operator
put-on h2 h0
0
4
0 14 -1 0
0 4 0 3
0 9 1 2
0 11 -1 1
1
end_operator
begin_operator
put-on h2 h1
0
4
0 14 -1 0
0 4 0 4
0 10 1 2
0 11 -1 1
1
end_operator
begin_operator
put-on h2 h4
0
4
0 14 -1 0
0 4 0 5
0 11 -1 1
0 15 0 1
1
end_operator
begin_operator
put-on h2 m0
0
4
0 14 -1 0
0 4 0 6
0 11 -1 1
0 12 1 2
1
end_operator
begin_operator
put-on h2 m1
0
4
0 14 -1 0
0 4 0 7
0 11 -1 1
0 13 1 2
1
end_operator
begin_operator
put-on h4 h0
0
3
0 14 1 0
0 9 1 0
0 15 -1 0
1
end_operator
begin_operator
put-on h4 h1
0
3
0 14 1 0
0 10 1 0
0 15 -1 0
1
end_operator
begin_operator
put-on h4 h2
0
3
0 14 1 0
0 11 1 0
0 15 -1 0
1
end_operator
begin_operator
put-on h4 h4
1
15 0
2
0 14 1 0
0 0 -1 0
1
end_operator
begin_operator
put-on h4 m0
0
3
0 14 1 0
0 12 1 0
0 15 -1 0
1
end_operator
begin_operator
put-on h4 m1
0
3
0 14 1 0
0 13 1 0
0 15 -1 0
1
end_operator
begin_operator
put-on m0 h0
0
4
0 14 -1 0
0 5 0 3
0 9 1 2
0 12 -1 1
1
end_operator
begin_operator
put-on m0 h1
0
4
0 14 -1 0
0 5 0 4
0 10 1 2
0 12 -1 1
1
end_operator
begin_operator
put-on m0 h2
0
4
0 14 -1 0
0 5 0 5
0 11 1 2
0 12 -1 1
1
end_operator
begin_operator
put-on m0 h4
0
4
0 14 -1 0
0 5 0 6
0 12 -1 1
0 15 0 1
1
end_operator
begin_operator
put-on m0 m1
0
4
0 14 -1 0
0 5 0 7
0 12 -1 1
0 13 1 2
1
end_operator
begin_operator
put-on m1 h0
0
4
0 14 -1 0
0 6 0 3
0 9 1 2
0 13 -1 1
1
end_operator
begin_operator
put-on m1 h1
0
4
0 14 -1 0
0 6 0 4
0 10 1 2
0 13 -1 1
1
end_operator
begin_operator
put-on m1 h2
0
4
0 14 -1 0
0 6 0 5
0 11 1 2
0 13 -1 1
1
end_operator
begin_operator
put-on m1 h4
0
4
0 14 -1 0
0 6 0 6
0 13 -1 1
0 15 0 1
1
end_operator
begin_operator
put-on m1 m0
0
4
0 14 -1 0
0 6 0 7
0 12 1 2
0 13 -1 1
1
end_operator
13
begin_rule
6
2 1
3 1
4 1
5 3
6 4
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 3
6 5
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 3
6 6
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 4
6 3
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 4
6 5
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 4
6 6
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 5
6 3
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 5
6 4
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 5
6 6
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 6
6 3
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 6
6 4
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 6
6 5
8 0
16 1 0
end_rule
begin_rule
6
2 1
3 1
4 1
5 6
6 6
8 0
16 1 0
end_rule
