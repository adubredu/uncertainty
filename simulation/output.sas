begin_version
3
end_version
begin_metric
0
end_metric
5
begin_variable
var0
-1
2
Atom topfree(m0)
NegatedAtom topfree(m0)
end_variable
begin_variable
var1
-1
2
Atom topfree(m1)
NegatedAtom topfree(m1)
end_variable
begin_variable
var2
-1
2
Atom handempty()
NegatedAtom handempty()
end_variable
begin_variable
var3
-1
4
Atom holding(m0)
Atom inbox(m0)
Atom inclutter(m0)
Atom on(m0, m1)
end_variable
begin_variable
var4
-1
4
Atom holding(m1)
Atom inbox(m1)
Atom inclutter(m1)
Atom on(m1, m0)
end_variable
3
begin_mutex_group
3
2 0
3 0
4 0
end_mutex_group
begin_mutex_group
3
3 0
4 3
0 0
end_mutex_group
begin_mutex_group
3
3 3
4 0
1 0
end_mutex_group
begin_state
0
0
0
2
2
end_state
begin_goal
2
3 1
4 1
end_goal
12
begin_operator
pick-from m0 m1
0
4
0 2 0 1
0 3 3 0
0 0 0 1
0 1 -1 0
1
end_operator
begin_operator
pick-from m1 m0
0
4
0 2 0 1
0 4 3 0
0 0 -1 0
0 1 0 1
1
end_operator
begin_operator
pick-from-box m0
0
3
0 2 0 1
0 3 1 0
0 0 0 1
1
end_operator
begin_operator
pick-from-box m1
0
3
0 2 0 1
0 4 1 0
0 1 0 1
1
end_operator
begin_operator
pick-from-clutter m0
0
3
0 2 0 1
0 3 2 0
0 0 0 1
1
end_operator
begin_operator
pick-from-clutter m1
0
3
0 2 0 1
0 4 2 0
0 1 0 1
1
end_operator
begin_operator
put-in-box m0
0
3
0 2 -1 0
0 3 0 1
0 0 -1 0
1
end_operator
begin_operator
put-in-box m1
0
3
0 2 -1 0
0 4 0 1
0 1 -1 0
1
end_operator
begin_operator
put-in-clutter m0
0
3
0 2 -1 0
0 3 0 2
0 0 -1 0
1
end_operator
begin_operator
put-in-clutter m1
0
3
0 2 -1 0
0 4 0 2
0 1 -1 0
1
end_operator
begin_operator
put-on m0 m1
0
4
0 2 -1 0
0 3 0 3
0 0 -1 0
0 1 0 1
1
end_operator
begin_operator
put-on m1 m0
0
4
0 2 -1 0
0 4 0 3
0 0 0 1
0 1 -1 0
1
end_operator
0
